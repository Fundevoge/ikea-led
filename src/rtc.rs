use core::ptr::addr_of_mut;

use chrono::{DateTime, TimeZone};
use embassy_net::{tcp::TcpSocket, Ipv4Address};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use embedded_svc::io::asynch::Read;
use esp_hal::rtc_cntl::Rtc;
use esp_wifi::wifi::{WifiDevice, WifiStaDevice};

use crate::{flag::Flag, tz_de::TzDe};

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

// Additional memory of rtc adjust task to reduce stack size
const RX_BUFFER_SIZE_TIME: usize = 1024;
const TX_BUFFER_SIZE_TIME: usize = 1024;
static mut TIME_RX_BUFFER: [u8; RX_BUFFER_SIZE_TIME] = [0; RX_BUFFER_SIZE_TIME];
static mut TIME_TX_BUFFER: [u8; TX_BUFFER_SIZE_TIME] = [0; TX_BUFFER_SIZE_TIME];

#[embassy_executor::task]
pub(crate) async fn rtc_adjust_task(
    wifi_program_stack: &'static embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>,
    rtc: &'static Rtc<'static>,
    rtc_offset_flag: &'static Flag<NoopRawMutex>,
    wifi_enabled_flag: &'static Flag<NoopRawMutex>,
) {
    let mut time_socket = TcpSocket::new(
        wifi_program_stack,
        unsafe { &mut *addr_of_mut!(TIME_RX_BUFFER) },
        unsafe { &mut *addr_of_mut!(TIME_TX_BUFFER) },
    );
    // time_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
    let endpoint = (Ipv4Address::new(192, 168, 178, 30), 3125);

    while let Err(e) = time_socket.connect(endpoint).await {
        log::warn!("[Rtc Offset] Initial connection failed, retrying. {e:?}");
        Timer::after_secs(10).await;
    }

    let mut unix_time_buffer = [0_u8; 8];
    time_socket.read_exact(&mut unix_time_buffer).await.unwrap();
    set_rtc_offset(rtc, u64::from_le_bytes(unix_time_buffer));
    log::info!("[Rtc Offset] Initially connected!");
    rtc_offset_flag.flag();

    loop {
        // Yield control immediately
        Timer::after_secs(1).await;
        // Wait for message
        match time_socket.read_exact(&mut unix_time_buffer).await {
            Ok(()) => {
                set_rtc_offset(rtc, u64::from_le_bytes(unix_time_buffer));
                rtc_offset_flag.flag();
            }
            Err(e) => {
                rtc_offset_flag.reset();
                log::error!("[Rtc Offset] Error reading timestamp: {:?}", e);
                time_socket.close();
                wifi_enabled_flag.wait_peek().await;
                while let Err(e) = time_socket.connect(endpoint).await {
                    log::warn!("[Rtc Offset] Reconnection failed {e:?}, retrying...");
                    Timer::after_secs(10).await;
                }
                log::info!("[Rtc Offset] Reconnected!");
                rtc_offset_flag.flag();
            }
        }
    }
}
