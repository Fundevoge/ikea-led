use core::str::FromStr as _;

use byteorder::ByteOrder as _;
use embassy_net::{tcp::TcpSocket, Ipv4Address};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Duration;
use embedded_svc::io::asynch::Read as _;
use enumset::*;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};
use micromath::F32Ext as _;

use crate::flag::Flag;

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
