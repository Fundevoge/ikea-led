use core::{
    convert::Infallible,
    fmt::{Debug, Write},
};

use crate::{tz_de::TzDe, RTC2 as RTC_REF};
use chrono::{Datelike, TimeZone as _, Timelike};
use embedded_sdmmc::{File, SdCard, TimeSource, Timestamp, VolumeManager};
use esp_hal::{
    delay::{self, Delay},
    gpio::Output,
    spi::master::Spi,
};
use esp_println::println;
use heapless::String;
struct Clock;

impl TimeSource for Clock {
    fn get_timestamp(&self) -> Timestamp {
        let current_time = unsafe { RTC_REF.unwrap() }.current_time();
        Timestamp {
            year_since_1970: (current_time.year() - 1970) as u8,
            zero_indexed_month: current_time.month0() as u8,
            zero_indexed_day: current_time.day0() as u8,
            hours: current_time.hour() as u8,
            minutes: current_time.minute() as u8,
            seconds: current_time.second() as u8,
        }
    }
}

pub(crate) fn is_enabled(_level: log::Level, _target: &str) -> bool {
    true
}

struct SdSpiDevice {
    spi: Spi<'static, esp_hal::Blocking>,
    cs: Output<'static>,
}

impl SdSpiDevice {
    fn new(spi: Spi<'static, esp_hal::Blocking>, mut cs: Output<'static>) -> SdSpiDevice {
        // IMPORTANT, otherwise MCP23S17 might not work
        cs.set_high();

        SdSpiDevice { spi, cs }
    }
}

impl embedded_hal::spi::SpiDevice for SdSpiDevice {
    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        for operation in operations {
            match operation {
                embedded_hal::spi::Operation::DelayNs(_) => {}
                _ => {
                    self.cs.set_low();
                }
            }
            match operation {
                embedded_hal::spi::Operation::Read(words) => {
                    let _ = embedded_hal::spi::SpiBus::read(&mut self.spi, words);
                }
                embedded_hal::spi::Operation::Write(words) => {
                    let _ = embedded_hal::spi::SpiBus::write(&mut self.spi, words);
                }
                embedded_hal::spi::Operation::Transfer(read, write) => {
                    let _ = embedded_hal::spi::SpiBus::transfer(&mut self.spi, read, write);
                }
                embedded_hal::spi::Operation::TransferInPlace(words) => {
                    let _ = embedded_hal::spi::SpiBus::transfer_in_place(&mut self.spi, words);
                }
                embedded_hal::spi::Operation::DelayNs(ns) => {
                    delay::Delay::new().delay_nanos(*ns);
                }
            }
            match operation {
                embedded_hal::spi::Operation::DelayNs(_) => {}
                _ => {
                    self.cs.set_high();
                }
            }
        }
        Ok(())
    }
}

impl embedded_hal::spi::ErrorType for SdSpiDevice {
    type Error = Infallible;
}

impl core::fmt::Debug for SdSpiDevice {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MySpi")
    }
}

type VolumeMgr = VolumeManager<SdCard<SdSpiDevice, Delay>, Clock>;
static mut SD_LOGGER_VOLUME_MANAGER: Option<VolumeMgr> = None;
static mut LOG_FILE_NAME: String<12> = String::new();

/// Initialize the logger with the given maximum log level.
pub fn init_logger(
    level: log::LevelFilter,
    spi: Spi<'static, esp_hal::Blocking>,
    cs: Output<'static>,
) {
    let sdcard = SdCard::new(SdSpiDevice::new(spi, cs), Delay::new());
    let mut volume_mgr = VolumeManager::new(sdcard, Clock);

    let mut volume0 = volume_mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .my_expect("Failed to open volume 0");
    let mut root_dir = volume0
        .open_root_dir()
        .my_expect("Failed to open root directory");
    let mut max_file_num: u32 = 0;
    root_dir
        .iterate_dir(|entry| {
            if entry.attributes.is_directory()
                || entry.attributes.is_hidden()
                || entry.attributes.is_read_only()
                || entry.attributes.is_system()
                || entry.attributes.is_volume()
                || !entry.name.base_name().iter().all(u8::is_ascii_digit)
            {
                println!("INFO - Skipping SD Card entry '{}'", entry.name);
                return;
            }

            let num = entry
                .name
                .base_name()
                .as_ascii()
                .my_expect("File name should be ascii")
                .as_str()
                .parse::<u32>()
                .my_expect("File name should be parseable number");
            max_file_num = max_file_num.max(num);
        })
        .my_expect("Failed to iterate through directory");

    let mut log_file_name = heapless::String::<12>::new(); // 32 byte string buffer

    write!(log_file_name, "{}", max_file_num + 1)
        .my_expect("New file name should fit into 12 bytes");

    let mut log_file: File<'_, SdCard<SdSpiDevice, Delay>, Clock, 4, 4, 1> = root_dir
        .open_file_in_dir(
            log_file_name.as_str(),
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        )
        .my_expect("Failed to open or create log file");

    log_file
        .write(b"BEGIN OF LOG\r\n")
        .my_expect("Failed to write to log file initially");

    log_file.close().my_expect("Failed to close file");
    root_dir.close().my_expect("Failed to root directory");
    volume0.close().my_expect("Failed to close volume");

    unsafe {
        SD_LOGGER_VOLUME_MANAGER = Some(volume_mgr);
        LOG_FILE_NAME = log_file_name;
        log::set_logger_racy(&SdLogger).unwrap();
        log::set_max_level_racy(level);
    }
}

// pub(crate) const FILTER_MAX: log::LevelFilter = log::LevelFilter::Off;
// /// Initialize the logger from the `ESP_LOG` environment variable.
// pub fn init_logger_from_env(spi: Spi<'static, esp_hal::Async>) {
//     let sd_logger: &'static SdLogger = SD_LOGGER_SPI.init(SdLogger { spi });
//     unsafe {
//         log::set_logger_racy(sd_logger).unwrap();
//         log::set_max_level_racy(FILTER_MAX);
//     }
// }

// Example SPI wrapper
pub struct SdFileWriter<'a> {
    log_file: File<'a, SdCard<SdSpiDevice, Delay>, Clock, 4, 4, 1>,
}

impl<'a> SdFileWriter<'a> {
    fn destruct(self) -> File<'a, SdCard<SdSpiDevice, Delay>, Clock, 4, 4, 1> {
        self.log_file
    }
}

// Implement the Write trait for the wrapper
impl Write for SdFileWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.log_file
            .write(s.as_bytes())
            .map(|_| ())
            .map_err(|_| core::fmt::Error)
    }
}

struct SdLogger;

impl log::Log for SdLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        let level = metadata.level();
        let target = metadata.target();
        is_enabled(level, target)
    }

    #[allow(unused)]
    fn log(&self, record: &log::Record) {
        let mut formatted = heapless::String::<32>::new();
        let _ = write!(formatted, "{}", record.args());
        if formatted.starts_with("esp_wifi_internal_tx 1229") {
            panic!("Received esp_wifi_internal_tx 12294");
        }

        if !self.enabled(record.metadata()) {
            return;
        }
        const RESET: &str = "\u{001B}[0m";
        const RED: &str = "\u{001B}[31m";
        const GREEN: &str = "\u{001B}[32m";
        const YELLOW: &str = "\u{001B}[33m";
        const BLUE: &str = "\u{001B}[34m";
        const CYAN: &str = "\u{001B}[35m";

        let color = match record.level() {
            log::Level::Error => RED,
            log::Level::Warn => YELLOW,
            log::Level::Info => GREEN,
            log::Level::Debug => BLUE,
            log::Level::Trace => CYAN,
        };
        let reset = RESET;

        if unsafe { !crate::panic_reboot::FIRST_REBOOT } {
            println!("{}{} - {}{}", color, record.level(), record.args(), reset);
        }

        let current_time = TzDe.from_utc_datetime(&unsafe { RTC_REF.unwrap() }.current_time());

        println!(
            "{}{} - {} - {}{}",
            color,
            record.level(),
            current_time,
            record.args(),
            reset
        );

        // Safety: Only used on one core
        let mut volume_mgr = unsafe { SD_LOGGER_VOLUME_MANAGER.as_mut().unwrap() };
        let mut volume0 = volume_mgr
            .open_volume(embedded_sdmmc::VolumeIdx(0))
            .my_expect("Failed to open volume 0");
        let mut root_dir = volume0
            .open_root_dir()
            .my_expect("Failed to open root directory");

        let mut log_file = root_dir
            .open_file_in_dir(
                unsafe { LOG_FILE_NAME.as_str() },
                embedded_sdmmc::Mode::ReadWriteAppend,
            )
            .my_expect("Failed to open log file");

        let mut sd_writer = SdFileWriter { log_file };
        writeln!(
            sd_writer,
            "{} - {} - {}",
            record.level(),
            current_time,
            record.args()
        );
        let mut log_file = sd_writer.destruct();

        log_file.close().my_expect("Failed to close file");
        root_dir.close().my_expect("Failed to root directory");
        volume0.close().my_expect("Failed to close volume");
    }

    fn flush(&self) {}
}

// Define the trait
pub trait MyExpect<T> {
    fn my_expect(self, msg: &str) -> T;
}

// Implement the trait for Result
impl<T, E: Debug> MyExpect<T> for Result<T, E> {
    fn my_expect(self, msg: &str) -> T {
        match self {
            Ok(val) => val,
            Err(err) => {
                println!("{}: {:?}", msg, err);
                panic!("{}: {:?}", msg, err);
            }
        }
    }
}

// Implement the trait for Result
impl<T> MyExpect<T> for Option<T> {
    fn my_expect(self, msg: &str) -> T {
        match self {
            Some(val) => val,
            None => {
                println!("{}", msg);
                panic!("{}", msg);
            }
        }
    }
}
