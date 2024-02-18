#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]

use core::{
    ops::{Deref, DerefMut},
    str::FromStr,
};

use chrono::{DateTime, NaiveDateTime, TimeZone};
use embassy_net::{
    udp::{PacketMetadata, UdpSocket},
    Ipv4Address, Ipv4Cidr, StackResources, StaticConfigV4,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
};
use embassy_time::{Duration, Ticker};

use esp32s3_hal::{
    clock::ClockControl,
    cpu_control::{self, CpuControl},
    dma::DmaPriority,
    dma_descriptors,
    embassy::{self, executor::Executor},
    gdma::*,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::RtcClock,
    spi::{
        master::{prelude::*, Spi},
        FullDuplexMode, SpiMode,
    },
    timer::TimerGroup,
    Rng, Rtc, IO,
};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};

use embedded_svc::wifi::Wifi;
use heapless::Vec;
use micromath::F32Ext;
use sntp::UnixTimeStamp;
use tz_de::TzDe;

mod sntp;
mod tz_de;

const ROWS: usize = 16;
const COLS: usize = 16;

const GRAY_LEVELS: u16 = 256;
const BRIGHTNESS_STEP: u16 = 256_u16 / GRAY_LEVELS;

const POSITIONS: [u8; COLS * ROWS] = [
    0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21, 0x20, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    0x2f, 0x2e, 0x2d, 0x2c, 0x2b, 0x2a, 0x29, 0x28, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
    0x4f, 0x4e, 0x4d, 0x4c, 0x4b, 0x4a, 0x49, 0x48, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
    0x47, 0x46, 0x45, 0x44, 0x43, 0x42, 0x41, 0x40, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
    0x67, 0x66, 0x65, 0x64, 0x63, 0x62, 0x61, 0x60, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77,
    0x6f, 0x6e, 0x6d, 0x6c, 0x6b, 0x6a, 0x69, 0x68, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
    0x8f, 0x8e, 0x8d, 0x8c, 0x8b, 0x8a, 0x89, 0x88, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f,
    0x87, 0x86, 0x85, 0x84, 0x83, 0x82, 0x81, 0x80, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97,
    0xa7, 0xa6, 0xa5, 0xa4, 0xa3, 0xa2, 0xa1, 0xa0, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7,
    0xaf, 0xae, 0xad, 0xac, 0xab, 0xaa, 0xa9, 0xa8, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf,
    0xcf, 0xce, 0xcd, 0xcc, 0xcb, 0xca, 0xc9, 0xc8, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf,
    0xc7, 0xc6, 0xc5, 0xc4, 0xc3, 0xc2, 0xc1, 0xc0, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7,
    0xe7, 0xe6, 0xe5, 0xe4, 0xe3, 0xe2, 0xe1, 0xe0, 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
    0xef, 0xee, 0xed, 0xec, 0xeb, 0xea, 0xe9, 0xe8, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff,
];

static mut APP_CORE_STACK: cpu_control::Stack<8192> = cpu_control::Stack::new();
static mut RENDER_BUF_1: RenderBuffer = RenderBuffer::empty();
static mut RENDER_BUF_2: RenderBuffer = RenderBuffer::empty();

static mut RENDER_BUF_PTR_USED: Mutex<CriticalSectionRawMutex, *mut RenderBuffer> =
    Mutex::new(unsafe { &mut RENDER_BUF_1 } as *mut RenderBuffer);

#[derive(Copy, Clone, Debug)]
struct RenderBuffer([u8; COLS * ROWS]);

impl Default for RenderBuffer {
    fn default() -> Self {
        Self::empty()
    }
}

impl RenderBuffer {
    const fn empty() -> Self {
        Self([0; COLS * ROWS])
    }

    fn invert(mut self) -> Self {
        for value in self.0.iter_mut() {
            *value = 255 - *value;
        }
        self
    }

    fn even_on() -> Self {
        let mut buf = Self::default();
        for value in buf.0.iter_mut().step_by(2) {
            *value = 255;
        }
        buf
    }

    fn odd_on() -> Self {
        Self::even_on().invert()
    }

    fn first_row_inc() -> Self {
        let mut buf = Self::default();
        for (i, value) in buf.0.iter_mut().take(COLS).enumerate() {
            *value = i as u8 * COLS as u8 + 15;
        }
        buf
    }

    fn corner_distance() -> Self {
        const NORM: f32 = 1.0 / (16.0 * core::f32::consts::SQRT_2);
        let mut buf = Self::default();
        for (idx, value) in buf.0.iter_mut().enumerate() {
            let x = idx % 16;
            let y = idx / 16;
            *value = (255.0 * (1.0 - ((x.pow(2) + y.pow(2)) as f32).sqrt() * NORM)) as u8;
        }
        buf
    }
}

#[embassy_executor::task]
async fn render_loop(
    spi: Spi<'static, esp32s3_hal::peripherals::SPI2, FullDuplexMode>,
    dma_channel: ChannelCreator0,
) {
    let mut min_brightness = 0_u16;
    let mut pixel_bits: [u8; ROWS * COLS / 8] = [0_u8; ROWS * COLS / 8];
    let mut ticker = Ticker::every(Duration::from_micros(100));

    let (mut descriptors, mut rx_descriptors) = dma_descriptors!(32000);
    let mut spi = spi.with_dma(dma_channel.configure(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    ));

    loop {
        let mb = min_brightness;
        let mb_1 = mb + 1;
        let buffer_guard = unsafe { RENDER_BUF_PTR_USED.lock() }.await;
        let buffer = unsafe { &**buffer_guard.deref() };
        for idx in 0..ROWS * COLS {
            let b_val = buffer.0[POSITIONS[idx] as usize] as u16;

            //if b_val >= *min_brightness {
            if (b_val * mb) / (GRAY_LEVELS - 1) != (b_val * mb_1) / (GRAY_LEVELS - 1) {
                pixel_bits[idx / 8] |= 0b1000_0000 >> (idx % 8);
            } else {
                pixel_bits[idx / 8] &= !(0b1000_0000 >> (idx % 8));
            };
        }
        drop(buffer_guard);
        min_brightness = (min_brightness + BRIGHTNESS_STEP) % 256;
        embedded_hal_async::spi::SpiBus::write(&mut spi, &pixel_bits)
            .await
            .unwrap();

        ticker.next().await;
    }
}

async fn render(editable_buffer: &mut &mut RenderBuffer) {
    let mut render_buf_used_guard = unsafe { RENDER_BUF_PTR_USED.lock() }.await;
    let render_buf_used = render_buf_used_guard.deref_mut();
    let new_free_render_buf = *render_buf_used;
    *render_buf_used = *editable_buffer;
    drop(render_buf_used_guard);
    *editable_buffer = unsafe { &mut *new_free_render_buf };
}

const SSID: &str = "5G fuer alte!";
const PASSWORD: &str = "59663346713734943174";

const RX_BUFFER_SIZE_UDP: usize = 1024;
const TX_BUFFER_SIZE_UDP: usize = 1024;

#[embassy_executor::task]
async fn connection(
    mut controller: WifiController<'static>,
    enabled_signal: &'static embassy_sync::signal::Signal<NoopRawMutex, ()>,
) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        if esp_wifi::wifi::get_wifi_state() == WifiState::StaConnected {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            enabled_signal.reset();
            embassy_time::Timer::after(Duration::from_millis(5000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = embedded_svc::wifi::Configuration::Client(
                embedded_svc::wifi::ClientConfiguration {
                    ssid: heapless::String::from_str(SSID).expect("SSID should be valid"),
                    password: heapless::String::from_str(PASSWORD)
                        .expect("Password should be valid"),
                    ..Default::default()
                },
            );
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start().await.unwrap();
            println!("Wifi started!");
        }
        println!("About to connect...");

        match controller.connect().await {
            Ok(_) => {
                println!("Wifi connected!");
                enabled_signal.signal(());
            }
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                embassy_time::Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}

struct RtcOffset {
    secs: u64,
    micros: u32,
}

fn adjust_rtc(rtc: &Rtc, rtc_offset: &mut RtcOffset, unix_time: &UnixTimeStamp) {
    // Convert Unix timestamp to DateTime<Utc>
    let rtc_time_raw = rtc.get_time_raw();
    let rtc_time_micros =
        rtc_time_raw * 1_000_000 / RtcClock::get_slow_freq().frequency().to_Hz() as u64;
    let rtc_micros = (rtc_time_micros % 1_000_000) as u32;
    let rtc_secs = rtc_time_micros / 1_000_000;
    rtc_offset.secs = (unix_time.secs - rtc_secs as i64) as u64;
    let unix_micros = unix_time.nanos / 1000;
    if unix_micros >= rtc_micros {
        rtc_offset.micros = unix_micros - rtc_micros;
    } else {
        rtc_offset.secs -= 1;
        rtc_offset.micros = (1_000_000 + unix_micros) - rtc_micros;
    }
}

fn get_german_datetime(rtc: &Rtc, rtc_offset: &RtcOffset) -> DateTime<TzDe> {
    // Convert Unix timestamp to DateTime<Utc>
    let micros = rtc.get_time_us() + rtc_offset.micros as u64;
    let unix_secs = micros / 1_000_000 + rtc_offset.secs;
    let naive_dt =
        NaiveDateTime::from_timestamp_opt(unix_secs as i64, ((micros % 1_000_000) * 1_000) as u32)
            .unwrap();
    TzDe.from_utc_datetime(&naive_dt)
}

#[main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut cpu_control = CpuControl::new(system.cpu_control);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    println!("Hello world!");
    println!(
        "App core stack from {:?} to {:?}",
        unsafe { APP_CORE_STACK.bottom() },
        unsafe { APP_CORE_STACK.top() }
    );

    let rtc = Rtc::new(peripherals.LPWR);
    let mut rtc_offset = RtcOffset { secs: 0, micros: 0 };

    let _btn = io.pins.gpio1.into_pull_down_input();
    let mut screen_override = io.pins.gpio2.into_push_pull_output();
    screen_override.set_low().unwrap();

    let sclk = io.pins.gpio40;
    let miso = io.pins.gpio26;
    let mosi = io.pins.gpio41;
    let cs = io.pins.gpio42;

    let dma = Gdma::new(peripherals.DMA);
    let dma_channel: ChannelCreator0 = dma.channel0;

    let spi = Spi::new(peripherals.SPI2, 12u32.MHz(), SpiMode::Mode0, &clocks).with_pins(
        Some(sclk),
        Some(mosi),
        Some(miso),
        Some(cs),
    );
    println!("Created spi!");

    let timer = esp32s3_hal::timer::TimerGroup::new(peripherals.TIMG1, &clocks).timer0;
    let wifi_init = esp_wifi::initialize(
        esp_wifi::EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let (wifi_interface, wifi_controller) =
        esp_wifi::wifi::new_with_mode(&wifi_init, wifi, WifiStaDevice).unwrap();
    let dns_servers = Vec::from_slice(&[
        Ipv4Address([192, 168, 178, 30]),
        Ipv4Address([192, 168, 178, 30]),
        Ipv4Address([192, 168, 178, 30]),
    ])
    .unwrap();

    let config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Address([192, 168, 178, 40]), 24),
        gateway: Some(Ipv4Address([192, 168, 178, 1])),
        dns_servers,
    });

    let seed = 1234; // very random, very secure seed

    // Init network stack
    let wifi_program_stack = &*static_cell::make_static!(embassy_net::Stack::new(
        wifi_interface,
        config,
        static_cell::make_static!(StackResources::<3>::new()),
        seed
    ));

    let wifi_enabled_signal: &embassy_sync::signal::Signal<NoopRawMutex, ()> =
        static_cell::make_static!(embassy_sync::signal::Signal::new());

    spawner
        .spawn(connection(wifi_controller, wifi_enabled_signal))
        .unwrap();
    spawner.spawn(net_task(wifi_program_stack)).unwrap();
    wifi_enabled_signal.wait().await;

    let mut rx_buffer = [0; RX_BUFFER_SIZE_UDP];
    let mut tx_buffer = [0; TX_BUFFER_SIZE_UDP];

    let mut rx_meta_buffer = [PacketMetadata::EMPTY; RX_BUFFER_SIZE_UDP];
    let mut tx_meta_buffer = [PacketMetadata::EMPTY; TX_BUFFER_SIZE_UDP];
    let mut socket = UdpSocket::new(
        wifi_program_stack,
        &mut rx_meta_buffer,
        &mut rx_buffer,
        &mut tx_meta_buffer,
        &mut tx_buffer,
    );
    socket.bind(34254).unwrap();
    let unix_timestamp = crate::sntp::get_timestamp_ntp(&mut socket).await;
    println!("Time stamp: {unix_timestamp:?}");
    adjust_rtc(&rtc, &mut rtc_offset, &unix_timestamp);
    println!("In germany: {}", get_german_datetime(&rtc, &rtc_offset));

    let cpu1_fnctn = move || {
        let executor = static_cell::make_static!(Executor::new());
        executor.run(|spawner| {
            spawner.spawn(render_loop(spi, dma_channel)).ok();
        });
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut APP_CORE_STACK }, cpu1_fnctn)
        .unwrap();

    let mut editable_render_buf = unsafe { &mut RENDER_BUF_2 };
    let mut ticker = Ticker::every(Duration::from_secs(3));

    loop {
        println!("Corner dist");
        *editable_render_buf = RenderBuffer::corner_distance();
        render(&mut editable_render_buf).await;
        ticker.next().await;

        println!("First_row_inc");
        *editable_render_buf = RenderBuffer::first_row_inc();
        render(&mut editable_render_buf).await;
        ticker.next().await;
    }
}
