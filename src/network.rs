use core::str::FromStr as _;

use byteorder::ByteOrder as _;
use chrono::{DateTime, NaiveDateTime, TimeDelta};
use embassy_net::{tcp::TcpSocket, Ipv4Address};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Timer};
use embedded_svc::io::asynch::{Read as _, Write as _};
use enumset::*;
use esp_hal::rtc_cntl::Rtc;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};
use micromath::F32Ext as _;

use crate::{flag::Flag, State};

const STREAM_ADDR: (Ipv4Address, u16) = (Ipv4Address::new(192, 168, 178, 30), 3123);
const SSID: &str = "5G fuer alte!";
const PASSWORD: &str = "59663346713734943174";

#[embassy_executor::task]
pub(crate) async fn net_task(
    stack: &'static embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>,
) {
    stack.run().await
}

#[embassy_executor::task]
pub(crate) async fn connection(
    mut controller: WifiController<'static>,
    enabled_flag: &'static Flag<NoopRawMutex>,
) {
    log::info!(
        "[WIFI] Device capabilities: {:?}",
        controller.capabilities()
    );
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                loop {
                    let events = controller
                        .wait_for_events(
                            enum_set!(
                                WifiEvent::WifiReady
                                    | WifiEvent::ScanDone
                                    | WifiEvent::StaStart
                                    | WifiEvent::StaStop
                                    | WifiEvent::StaConnected
                                    | WifiEvent::StaDisconnected
                                    | WifiEvent::StaAuthmodeChange
                                    | WifiEvent::StaWpsErSuccess
                                    | WifiEvent::StaWpsErFailed
                                    | WifiEvent::StaWpsErTimeout
                                    | WifiEvent::StaWpsErPin
                                    | WifiEvent::StaWpsErPbcOverlap
                                    | WifiEvent::ApStart
                                    | WifiEvent::ApStop
                                    | WifiEvent::ApStaconnected
                                    | WifiEvent::ApStadisconnected
                                    | WifiEvent::ApProbereqrecved
                                    | WifiEvent::FtmReport
                                    | WifiEvent::StaBssRssiLow
                                    | WifiEvent::ActionTxStatus
                                    | WifiEvent::RocDone
                                    | WifiEvent::StaBeaconTimeout
                                    | WifiEvent::ConnectionlessModuleWakeIntervalStart
                                    | WifiEvent::ApWpsRgSuccess
                                    | WifiEvent::ApWpsRgFailed
                                    | WifiEvent::ApWpsRgTimeout
                                    | WifiEvent::ApWpsRgPin
                                    | WifiEvent::ApWpsRgPbcOverlap
                                    | WifiEvent::ItwtSetup
                                    | WifiEvent::ItwtTeardown
                                    | WifiEvent::ItwtProbe
                                    | WifiEvent::ItwtSuspend
                                    | WifiEvent::TwtWakeup
                                    | WifiEvent::BtwtSetup
                                    | WifiEvent::BtwtTeardown
                                    | WifiEvent::NanStarted
                                    | WifiEvent::NanStopped
                                    | WifiEvent::NanSvcMatch
                                    | WifiEvent::NanReplied
                                    | WifiEvent::NanReceive
                                    | WifiEvent::NdpIndication
                                    | WifiEvent::NdpConfirm
                                    | WifiEvent::NdpTerminated
                                    | WifiEvent::HomeChannelChange
                                    | WifiEvent::StaNeighborRep
                            ),
                            false,
                        )
                        .await;
                    let mut disconnected = false;
                    for event in events {
                        log::info!("[WIFI] [EVENT] {event:?}");
                        if event == WifiEvent::StaDisconnected {
                            disconnected = true;
                            enabled_flag.reset();
                        }
                    }
                    if disconnected {
                        break;
                    }
                }

                // controller.wait_for_event(WifiEvent::StaDisconnected).await;
                // enabled_flag.reset();
                log::warn!("[WIFI] Disconnected! Retrying in 30 seconds...");
                embassy_time::Timer::after(Duration::from_secs(30)).await;
            }
            _ => {
                log::info!("[WIFI] Current state: {:?}", esp_wifi::wifi::wifi_state());
            }
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config =
                esp_wifi::wifi::Configuration::Client(esp_wifi::wifi::ClientConfiguration {
                    ssid: heapless::String::from_str(SSID).expect("SSID should be valid"),
                    password: heapless::String::from_str(PASSWORD)
                        .expect("Password should be valid"),
                    ..Default::default()
                });
            controller.set_configuration(&client_config).unwrap();
            log::info!("[WIFI] Starting controller...");
            controller.start().unwrap();
            log::info!("[WIFI] Controller started!");
        }
        log::info!("[WIFI] About to connect...");

        match controller.connect() {
            Ok(_) => {
                log::info!("[WIFI] Controller connect returned successfully.");
                controller.wait_for_event(WifiEvent::StaConnected).await;
                log::info!("[WIFI] Controller reached connected state!");
                enabled_flag.flag();
            }
            Err(e) => {
                log::error!("[WIFI] Failed to connect! {e:?}");
                embassy_time::Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

pub(crate) async fn start_stream(
    stream_socket: &mut TcpSocket<'_>,
    state: &mut crate::State,
    frame_duration_micros: &mut u64,
) {
    if let Err(stream_connection_error) = stream_socket.connect(STREAM_ADDR).await {
        log::warn!("[Stream] Connection error: {stream_connection_error:?}");
        *state = crate::State::default();
        return;
    }

    log::info!("[Stream] Connected!");
    let mut fps_decoder_buf = [0_u8; 4];
    stream_socket
        .read_exact(&mut fps_decoder_buf)
        .await
        .unwrap();
    let fps = byteorder::LE::read_f32(&fps_decoder_buf);
    log::info!("[Stream] Playing at {fps:.2}");
    *frame_duration_micros = (1_000_000.0 / fps).round() as u64;
}

#[repr(u8)]
enum IncomingPacketType {
    TimeSync,
    TimeFollowUp,
    TimeDelayResp,
    StateChange,
}

impl TryFrom<u8> for IncomingPacketType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::TimeSync),
            1 => Ok(Self::TimeFollowUp),
            2 => Ok(Self::TimeDelayResp),
            3 => Ok(Self::StateChange),
            _ => Err(()),
        }
    }
}

enum OutgoingPacketType {
    Keepalive,
    TimeDelayReq,
}

const PACKET_FILLER: u8 = 0b01010110;
async fn send_tcp_packet(
    socket: &mut TcpSocket<'_>,
    packet: OutgoingPacketType,
    last_packet_sent: &mut NaiveDateTime,
    rtc: &Rtc<'_>,
) -> Option<()> {
    static mut TCP_PACKET: Mutex<NoopRawMutex, [u8; 256]> = Mutex::new([PACKET_FILLER; 256]);

    let packet_id: u8 = match packet {
        OutgoingPacketType::Keepalive => 0,
        OutgoingPacketType::TimeDelayReq => 1,
    };
    let mut tcp_packet = unsafe { TCP_PACKET.lock() }.await;

    tcp_packet[0] = packet_id;
    socket.write_all(&*tcp_packet).await.is_ok().then(|| {
        *last_packet_sent = rtc.current_time();
    })
}

pub(crate) async fn setup_tcp_socket(socket: &mut TcpSocket<'_>) {
    socket.set_timeout(Some(embassy_time::Duration::from_secs(5)));
    let endpoint = (Ipv4Address::new(192, 168, 178, 30), 3124);
    log::info!("[MAIN] (TCP Socket) Trying to connect...");
    while let Err(e) = socket.connect(endpoint).await {
        log::warn!("[MAIN] (TCP Socket) Connection failed, retrying. {e:?}");
        Timer::after_secs(10).await;
    }
    log::info!("[MAIN] (TCP Socket) Connected!");
    socket.set_timeout(Some(embassy_time::Duration::from_secs(90)));
    Timer::after_secs(5).await;
}

pub(crate) async fn reset_tcp_socket(
    socket: &mut TcpSocket<'_>,
    wifi_enabled_flag: &Flag<NoopRawMutex>,
) {
    socket.abort();
    wifi_enabled_flag.wait_peek().await;
    setup_tcp_socket(socket).await;
}

fn read_naive_datetime(receive_buffer: &[u8]) -> Option<NaiveDateTime> {
    DateTime::from_timestamp(
        byteorder::LE::read_i64(&receive_buffer[0..8]),
        byteorder::LE::read_u32(&receive_buffer[8..12]),
    )
    .map(|dt| DateTime::<chrono::Utc>::naive_utc(&dt))
}

enum PTPStateMachine {
    Idle,
    Sync(NaiveDateTime),
    DelayReq(NaiveDateTime),
}

impl PTPStateMachine {
    fn new() -> Self {
        Self::Idle
    }
}

/// Read state updates and time sync from network, handle read/write of keepalive messages
#[embassy_executor::task]
pub async fn tcp_socket_task(
    mut tcp_socket: TcpSocket<'static>,
    rtc: &'static Rtc<'static>,
    wifi_enabled_flag: &'static Flag<NoopRawMutex>,
    state_signal: &'static Signal<NoopRawMutex, State>,
) {
    let mut tcp_receive_buffer = [0_u8; 64];
    let mut ptp_state_machine = PTPStateMachine::new();
    let mut last_packet_sent = NaiveDateTime::MIN;
    let mut last_packet_received = NaiveDateTime::MAX;
    let mut socket_invalid = false;
    loop {
        if tcp_socket.can_recv() && !socket_invalid {
            socket_invalid = 'read_packet: {
                if let Err(e) = tcp_socket.read_exact(&mut tcp_receive_buffer).await {
                    log::error!("[MAIN] (TCP Socket) Error receiving full packet! {e}");
                    break 'read_packet true;
                };

                let Ok(packet_type) = IncomingPacketType::try_from(tcp_receive_buffer[0]) else {
                    log::error!("[MAIN] (TCP Socket) Error parsing packet type!");
                    break 'read_packet true;
                };

                match packet_type {
                    IncomingPacketType::StateChange => {
                        let Ok(new_state_network) = State::try_from(tcp_receive_buffer[1]) else {
                            log::error!("[MAIN] (TCP Socket) Error parsing state type!");
                            break 'read_packet true;
                        };
                        state_signal.signal(new_state_network);
                    }
                    IncomingPacketType::TimeSync => {
                        ptp_state_machine = PTPStateMachine::Sync(rtc.current_time());
                    }
                    IncomingPacketType::TimeFollowUp => {
                        if let PTPStateMachine::Sync(sync_received) = ptp_state_machine {
                            let Some(sync_sent) = read_naive_datetime(&tcp_receive_buffer[1..13])
                            else {
                                log::error!("[MAIN] (TCP Socket) Error reading PTP FollowUp!");
                                break 'read_packet true;
                            };

                            rtc.set_current_time(
                                rtc.current_time()
                                    .checked_add_signed(sync_sent - sync_received)
                                    .unwrap(),
                            );

                            if send_tcp_packet(
                                &mut tcp_socket,
                                OutgoingPacketType::TimeDelayReq,
                                &mut last_packet_sent,
                                rtc,
                            )
                            .await
                            .is_none()
                            {
                                log::error!("[MAIN] (TCP Socket) Error sending PTP DelayReq!");
                                break 'read_packet true;
                            };

                            ptp_state_machine = PTPStateMachine::DelayReq(rtc.current_time());
                        } else {
                            log::warn!("[MAIN] (TCP Socket) PTP FollowUp received in wrong state!");
                            ptp_state_machine = PTPStateMachine::new();
                        }
                    }
                    IncomingPacketType::TimeDelayResp => {
                        if let PTPStateMachine::DelayReq(delay_req_sent_adjusted) =
                            ptp_state_machine
                        {
                            let Some(delay_resp_sent) =
                                read_naive_datetime(&tcp_receive_buffer[1..13])
                            else {
                                log::error!("[MAIN] (TCP Socket) Error reading PTP DelayResp!");
                                break 'read_packet true;
                            };
                            rtc.set_current_time(
                                rtc.current_time()
                                    .checked_add_signed(
                                        (delay_resp_sent - delay_req_sent_adjusted) / 2,
                                    )
                                    .unwrap(),
                            );
                        } else {
                            log::warn!(
                                "[MAIN] (TCP Socket) PTP DelayResp received in wrong state!"
                            );
                        }
                        ptp_state_machine = PTPStateMachine::new();
                    }
                }

                last_packet_received = rtc.current_time();
                false
            };
        } else {
            // Ensure the task yields
            Timer::after_millis(5).await;
        }

        if socket_invalid
            || (rtc.current_time() - last_packet_received >= TimeDelta::seconds(90))
            || (rtc.current_time() - last_packet_sent >= TimeDelta::seconds(10)
                && send_tcp_packet(
                    &mut tcp_socket,
                    OutgoingPacketType::Keepalive,
                    &mut last_packet_sent,
                    rtc,
                )
                .await
                .is_none())
        {
            reset_tcp_socket(&mut tcp_socket, wifi_enabled_flag).await;

            let new_socket_invalid = send_tcp_packet(
                &mut tcp_socket,
                OutgoingPacketType::Keepalive,
                &mut last_packet_sent,
                rtc,
            )
            .await
            .is_none();

            if !new_socket_invalid && socket_invalid {
                last_packet_received = NaiveDateTime::MAX;
            }

            socket_invalid = new_socket_invalid;
        }
    }
}
