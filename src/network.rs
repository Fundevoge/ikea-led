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
    log::info!(
        "[WIFI] Device capabilities: {:?}",
        controller.capabilities()
    );
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                enabled_flag.reset();
                log::warn!("[WIFI] Disconnected! Retrying in 5 seconds...");
                embassy_time::Timer::after(Duration::from_millis(5000)).await;
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

    let endpoint = (Ipv4Address::new(192, 168, 178, 30), 3126);

    log::info!("[Keepalive] Trying to connect...");
    keepalive_socket.set_timeout(Some(Duration::from_secs(5)));
    while let Err(e) = keepalive_socket.connect(endpoint).await {
        log::warn!("[Keepalive] Initial connection failed, retrying. {e:?}");
        Timer::after_secs(10).await;
    }
    keepalive_socket.set_timeout(Some(Duration::from_secs(20)));
    log::info!("[Keepalive] Initially connected!");
    keepalive_established_flag.flag();

    loop {
        // Wait for message
        match keepalive_socket.write_all(&KEEPALIVE_PACKET).await {
            Ok(()) => {
                Timer::after_secs(10).await;
            }
            Err(e) => {
                keepalive_established_flag.reset();
                log::error!("[Keepalive] Error writing keepalive: {:?}", e);
                keepalive_socket.close();
                wifi_enabled_flag.wait_peek().await;
                keepalive_socket.set_timeout(Some(Duration::from_secs(5)));
                while let Err(e) = keepalive_socket.connect(endpoint).await {
                    log::warn!("[Keepalive] Reconnection failed {e:?}, retrying...");
                    Timer::after_secs(5).await;
                }
                keepalive_socket.set_timeout(Some(Duration::from_secs(20)));
                log::info!("[Keepalive] Reconnected!");
                keepalive_established_flag.flag();
            }
        }
    }
}
