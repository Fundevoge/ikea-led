use core::{ptr::addr_of_mut, str::FromStr as _};

use byteorder::ByteOrder as _;
use embassy_executor::task;
use embassy_net::{tcp::TcpSocket, Ipv4Address};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer};
use embedded_svc::io::asynch::{Read as _, Write};
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
    log::trace!("start connection task");
    log::info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        if esp_wifi::wifi::wifi_state() == WifiState::StaConnected {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            enabled_flag.reset();
            log::warn!("[WIFI] DISCONNECTED");
            embassy_time::Timer::after(Duration::from_millis(5000)).await
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
            log::info!("Starting wifi");
            controller.start().unwrap();
            log::info!("Wifi started!");
        }
        log::trace!("About to connect...");

        match controller.connect() {
            Ok(_) => {
                log::info!("Wifi connected!");
                enabled_flag.flag();
            }
            Err(e) => {
                log::error!("Failed to connect to wifi: {e:?}");
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
        log::warn!("Connection error: {stream_connection_error:?}");
        *state = crate::State::default();
        return;
    }

    log::info!("Stream Connected!");
    let mut fps_decoder_buf = [0_u8; 4];
    stream_socket
        .read_exact(&mut fps_decoder_buf)
        .await
        .unwrap();
    let fps = byteorder::LE::read_f32(&fps_decoder_buf);
    log::info!("Playing at {fps:.2}");
    *frame_duration_micros = (1_000_000.0 / fps).round() as u64;
}

// Additional memory of keep alive task to reduce stack size
const RX_BUFFER_SIZE_KEEPALIVE: usize = 64;
const TX_BUFFER_SIZE_KEEPALIVE: usize = 1024;
static mut KEEPALIVE_RX_BUFFER: [u8; RX_BUFFER_SIZE_KEEPALIVE] = [0; RX_BUFFER_SIZE_KEEPALIVE];
static mut KEEPALIVE_TX_BUFFER: [u8; TX_BUFFER_SIZE_KEEPALIVE] = [0; TX_BUFFER_SIZE_KEEPALIVE];
static KEEPALIVE_PACKET: [u8; 256] = [0b01010110; 256];

#[task]
pub(crate) async fn keep_alive(
    wifi_program_stack: &'static embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>,
    keepalive_established_flag: &'static Flag<NoopRawMutex>,
    wifi_enabled_flag: &'static Flag<NoopRawMutex>,
) {
    let mut keepalive_socket = TcpSocket::new(
        wifi_program_stack,
        unsafe { &mut *addr_of_mut!(KEEPALIVE_RX_BUFFER) },
        unsafe { &mut *addr_of_mut!(KEEPALIVE_TX_BUFFER) },
    );

    keepalive_socket.set_timeout(Some(Duration::from_secs(20)));
    let endpoint = (Ipv4Address::new(192, 168, 178, 30), 3126);

    while let Err(e) = keepalive_socket.connect(endpoint).await {
        log::warn!("Keepalive connection failed {e:?}, retrying...");
        Timer::after_secs(10).await;
    }
    log::info!("Keepalive connected!");
    keepalive_established_flag.flag();

    loop {
        match keepalive_socket.write_all(&KEEPALIVE_PACKET).await {
            Ok(()) => {
                Timer::after_secs(10).await;
            }
            Err(e) => {
                keepalive_established_flag.reset();
                log::error!("Error in keepalive: {:?}", e);
                keepalive_socket.close();
                wifi_enabled_flag.wait_peek().await;
                while let Err(e) = keepalive_socket.connect(endpoint).await {
                    log::warn!("Keepalive reconnection failed {e:?}, retrying...");
                    Timer::after_secs(10).await;
                }
                log::info!("Keepalive reconnected!");
                keepalive_established_flag.flag();
            }
        }
        Timer::after_secs(1).await;
    }
}
