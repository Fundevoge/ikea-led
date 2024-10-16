use chrono::{DateTime, TimeZone as _};
use embassy_net::{tcp::TcpSocket, Ipv4Address};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embedded_svc::io::asynch::Read as _;
use esp_hal::rtc_cntl::Rtc;
use esp_wifi::wifi::{WifiDevice, WifiStaDevice};

use crate::tz_de::TzDe;

fn set_rtc_offset(rtc: &Rtc, unix_time: u64) {
    rtc.set_current_time(
        DateTime::from_timestamp(unix_time as i64, 0)
            .unwrap()
            .naive_utc(),
    );
}

pub(crate) fn get_german_datetime(rtc: &Rtc) -> DateTime<TzDe> {
    TzDe.from_utc_datetime(&rtc.current_time())
}

const RX_BUFFER_SIZE_TIME: usize = 1024;
const TX_BUFFER_SIZE_TIME: usize = 1024;
// static mut TIME_RX_BUFFER: [u8; RX_BUFFER_SIZE_TIME] = [0; RX_BUFFER_SIZE_TIME];
// static mut TIME_TX_BUFFER: [u8; TX_BUFFER_SIZE_TIME] = [0; TX_BUFFER_SIZE_TIME];

#[embassy_executor::task]
pub(crate) async fn rtc_adjust_task(
    wifi_program_stack: &'static embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>,
    rtc: &'static Rtc<'_>,
    rtc_offset_signal: &'static Signal<NoopRawMutex, ()>,
) {
    let mut time_rx_buffer = [0; RX_BUFFER_SIZE_TIME];
    let mut time_tx_buffer = [0; TX_BUFFER_SIZE_TIME];
    let mut time_socket =
        TcpSocket::new(wifi_program_stack, &mut time_rx_buffer, &mut time_tx_buffer);
    // time_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
    if let Err(time_connection_error) = time_socket
        .connect((Ipv4Address::new(192, 168, 178, 30), 3125))
        .await
    {
        log::warn!("Connection error: {time_connection_error:?}");
    } else {
        log::info!("Time Control Connected!");
    }

    let mut unix_time_buffer = [0_u8; 8];
    time_socket.read_exact(&mut unix_time_buffer).await.unwrap();
    set_rtc_offset(rtc, u64::from_le_bytes(unix_time_buffer));
    rtc_offset_signal.signal(());

    loop {
        time_socket.read_exact(&mut unix_time_buffer).await.unwrap();
        set_rtc_offset(rtc, u64::from_le_bytes(unix_time_buffer));
        rtc_offset_signal.signal(());
    }
}
