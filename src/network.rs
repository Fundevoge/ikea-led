use core::str::FromStr as _;

use byteorder::ByteOrder as _;
use embassy_executor::task;
use embassy_net::{tcp::TcpSocket, Ipv4Address};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use embedded_svc::io::asynch::{Read as _, Write};
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};
use micromath::F32Ext as _;

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
    enabled_signal: &'static Signal<NoopRawMutex, ()>,
) {
    log::trace!("start connection task");
    log::info!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        if esp_wifi::wifi::get_wifi_state() == WifiState::StaConnected {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            enabled_signal.reset();
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
            controller.start().await.unwrap();
            log::info!("Wifi started!");
        }
        log::trace!("About to connect...");

        match controller.connect().await {
            Ok(_) => {
                log::info!("Wifi connected!");
                enabled_signal.signal(());
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
        *state = crate::State::Clock;
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

#[task]
pub(crate) async fn keep_alive(
    wifi_program_stack: &'static embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>,
    keepalive_established_signal: &'static Signal<NoopRawMutex, ()>,
) {
    let mut keepalive_rx_buffer = [0; 64];
    let mut keepalive_tx_buffer = [0; 1024];

    let mut keepalive_socket = TcpSocket::new(
        wifi_program_stack,
        &mut keepalive_rx_buffer,
        &mut keepalive_tx_buffer,
    );

    keepalive_socket.set_timeout(Some(Duration::from_secs(20)));

    while let Err(e) = keepalive_socket
        .connect((Ipv4Address::new(192, 168, 178, 30), 3126))
        .await
    {
        log::warn!("Keepalive connection failed {e:?}, retrying...");
        Timer::after_secs(10).await;
    }
    log::info!("Keepalive connected!");
    keepalive_established_signal.signal(());

    let keepalive_packet: [u8; 256] = [0b01010110; 256];

    loop {
        keepalive_socket.write_all(&keepalive_packet).await.unwrap();
        Timer::after_secs(10).await;
    }
}
