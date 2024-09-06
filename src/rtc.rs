use chrono::{DateTime, TimeZone as _, Utc};
use embassy_net::{tcp::TcpSocket, Ipv4Address};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Timer;
use embedded_svc::io::asynch::{Read as _, Write as _};
use esp_hal::rtc_cntl::Rtc;
use esp_wifi::wifi::{WifiDevice, WifiStaDevice};

use crate::tz_de::TzDe;

#[derive(Copy, Clone, Debug)]
pub(crate) struct RtcOffset {
    pub(crate) secs: u64,
}

fn get_rtc_offset(rtc: &Rtc, unix_time: u64) -> RtcOffset {
    let rtc_secs = rtc.get_time_ms() / 1_000;
    RtcOffset {
        secs: unix_time - rtc_secs,
    }
}

pub(crate) fn get_german_datetime(rtc: &Rtc, rtc_offset: &RtcOffset) -> DateTime<TzDe> {
    let micros = rtc.get_time_us();
    let unix_secs = micros / 1_000_000 + rtc_offset.secs;
    let dt_utc =
        DateTime::<Utc>::from_timestamp(unix_secs as i64, ((micros % 1_000_000) * 1_000) as u32)
            .unwrap();
    TzDe.from_utc_datetime(&dt_utc.naive_utc())
}

const RX_BUFFER_SIZE_TIME: usize = 1024;
const TX_BUFFER_SIZE_TIME: usize = 1024;
// static mut TIME_RX_BUFFER: [u8; RX_BUFFER_SIZE_TIME] = [0; RX_BUFFER_SIZE_TIME];
// static mut TIME_TX_BUFFER: [u8; TX_BUFFER_SIZE_TIME] = [0; TX_BUFFER_SIZE_TIME];

#[embassy_executor::task]
pub(crate) async fn rtc_adjust_task(
    wifi_program_stack: &'static embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>,
    rtc: &'static Rtc<'_>,
    rtc_offset_signal: &'static Signal<NoopRawMutex, RtcOffset>,
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
    let rtc_offset = get_rtc_offset(rtc, u64::from_le_bytes(unix_time_buffer));
    rtc_offset_signal.signal(rtc_offset);

    loop {
        time_socket.read_exact(&mut unix_time_buffer).await.unwrap();
        let rtc_offset = get_rtc_offset(rtc, u64::from_le_bytes(unix_time_buffer));
        rtc_offset_signal.signal(rtc_offset);
    }
}
