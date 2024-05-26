#![no_std]
#![no_main]
#![feature(const_mut_refs)]
#![feature(const_slice_from_raw_parts_mut)]

mod network;
mod patterns;
mod render;
mod rtc;
mod tz_de;

use core::ptr::addr_of_mut;

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    cpu_control::{self, CpuControl},
    dma::Dma,
    embassy::{self, executor::Executor},
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    rtc_cntl::Rtc,
    spi::{master::Spi, SpiMode},
    timer::TimerGroup,
};
use esp_wifi::wifi::{WifiDevice, WifiStaDevice};

use embassy_net::{tcp::TcpSocket, Ipv4Address, Ipv4Cidr, StackResources, StaticConfigV4};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    signal::Signal,
};
use embassy_time::{Duration, Timer};

use embedded_svc::io::asynch::Read;

use heapless::Vec;

use render::RenderBuffer;

use crate::rtc::RtcOffset;

macro_rules! mk_static {
    ($t:path,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum ButtonPress {
    Short,
    Long,
}

#[derive(Debug, PartialEq, Eq)]
enum State {
    Stream,
    Clock,
    Off,
}

impl State {
    fn next(&mut self) {
        *self = match self {
            State::Stream => State::Clock,
            State::Clock => State::Stream,
            State::Off => State::Clock,
        };
    }

    fn toggle_on_off(&mut self) {
        *self = match self {
            State::Off => State::Clock,
            _ => State::Off,
        };
    }
}

static mut APP_CORE_STACK: cpu_control::Stack<8192> = cpu_control::Stack::new();
static mut RENDER_BUF_1: RenderBuffer = RenderBuffer::empty();
static mut RENDER_BUF_2: RenderBuffer = RenderBuffer::empty();

static mut RENDER_BUF_PTR_USED: Mutex<CriticalSectionRawMutex, *mut RenderBuffer> =
    Mutex::new(unsafe { addr_of_mut!(RENDER_BUF_1) });

const RX_BUFFER_SIZE_STREAM: usize = 2048;
const TX_BUFFER_SIZE_STREAM: usize = 64;
static mut STREAM_RX_BUFFER: [u8; RX_BUFFER_SIZE_STREAM] = [0; RX_BUFFER_SIZE_STREAM];
static mut STREAM_TX_BUFFER: [u8; TX_BUFFER_SIZE_STREAM] = [0; TX_BUFFER_SIZE_STREAM];

const RX_BUFFER_SIZE_CONTROL: usize = 1024;
const TX_BUFFER_SIZE_CONTROL: usize = 64;
static mut CONTROL_RX_BUFFER: [u8; RX_BUFFER_SIZE_CONTROL] = [0; RX_BUFFER_SIZE_CONTROL];
static mut CONTROL_TX_BUFFER: [u8; TX_BUFFER_SIZE_CONTROL] = [0; TX_BUFFER_SIZE_CONTROL];

#[embassy_executor::task]
async fn button_read_task(
    mut btn: esp_hal::gpio::GpioPin<esp_hal::gpio::Input<esp_hal::gpio::PullUp>, 1>,
    signal: &'static Signal<NoopRawMutex, ButtonPress>,
) {
    loop {
        btn.wait_for_falling_edge().await;
        // False Positive Detection
        Timer::after(Duration::from_millis(80)).await;
        if btn.is_high() {
            // Debounce
            Timer::after(Duration::from_millis(80)).await;
            continue;
        }
        // Short Pulse detection; < 400ms
        Timer::after(Duration::from_millis(320)).await;
        if btn.is_high() {
            signal.signal(ButtonPress::Short);
        }
        // Long Pulse detection; > 400ms
        else {
            signal.signal(ButtonPress::Long);
            btn.wait_for_rising_edge().await;
        }
        // Debounce
        Timer::after(Duration::from_millis(400)).await;
    }
}

#[main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = Peripherals::take();
    let system: esp_hal::system::SystemParts<'_> = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut cpu_control = CpuControl::new(system.cpu_control);

    let timer_group0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    log::info!("Hello world!");

    let mut screen_override = io.pins.gpio2.into_push_pull_output();
    screen_override.set_low();

    let wifi_timer = TimerGroup::new(peripherals.TIMG1, &clocks, None).timer0;
    let wifi_init = esp_wifi::initialize(
        esp_wifi::EspWifiInitFor::Wifi,
        wifi_timer,
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
    let seed = 666722646956068949;
    let wifi_program_stack = &*mk_static!(
        embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>,
        embassy_net::Stack::new(
            wifi_interface,
            config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed
        )
    );
    spawner
        .spawn(network::net_task(wifi_program_stack))
        .unwrap();

    let wifi_enabled_signal: &Signal<NoopRawMutex, ()> =
        mk_static!(Signal<NoopRawMutex, ()>, Signal::new());
    spawner
        .spawn(network::connection(wifi_controller, wifi_enabled_signal))
        .unwrap();
    wifi_enabled_signal.wait().await;

    let rtc = &*mk_static!(Rtc, Rtc::new(peripherals.LPWR, None));
    let rtc_offset_signal: &Signal<NoopRawMutex, RtcOffset> =
        mk_static!(Signal<NoopRawMutex, RtcOffset>, Signal::new());
    spawner
        .spawn(rtc::rtc_adjust_task(
            wifi_program_stack,
            rtc,
            rtc_offset_signal,
        ))
        .unwrap();
    let mut rtc_offset = rtc_offset_signal.wait().await;

    let mut control_socket = TcpSocket::new(
        wifi_program_stack,
        unsafe { &mut *addr_of_mut!(CONTROL_RX_BUFFER) },
        unsafe { &mut *addr_of_mut!(CONTROL_TX_BUFFER) },
    );
    if let Err(control_connection_error) = control_socket
        .connect((Ipv4Address::new(192, 168, 178, 30), 3124))
        .await
    {
        log::warn!("Connection error: {control_connection_error:?}");
    } else {
        log::info!("State Control Connected!");
    }

    let mut stream_socket = TcpSocket::new(
        wifi_program_stack,
        unsafe { &mut *addr_of_mut!(STREAM_RX_BUFFER) },
        unsafe { &mut *addr_of_mut!(STREAM_TX_BUFFER) },
    );

    let sclk = io.pins.gpio40;
    let miso = io.pins.gpio26;
    let mosi = io.pins.gpio41;
    let cs = io.pins.gpio42;
    let spi = Spi::new(peripherals.SPI2, 4u32.MHz(), SpiMode::Mode0, &clocks).with_pins(
        Some(sclk),
        Some(mosi),
        Some(miso),
        Some(cs),
    );
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;
    let cpu1_fnctn = move || {
        let executor = mk_static!(Executor, Executor::new());
        executor.run(|spawner| {
            spawner.spawn(render::render_loop(spi, dma_channel)).ok();
        });
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, cpu1_fnctn)
        .unwrap();
    log::info!("Started Spi renderer!");

    let btn = io.pins.gpio1.into_pull_up_input();
    let button_signal: &Signal<NoopRawMutex, ButtonPress> =
        mk_static!(Signal<NoopRawMutex, ButtonPress>, Signal::new());
    spawner.spawn(button_read_task(btn, button_signal)).unwrap();

    let mut editable_render_buf = unsafe { &mut *addr_of_mut!(RENDER_BUF_2) };

    let mut state_decode_buf = [0_u8; 16];
    let mut frame_duration_micros: u64 = 200_000;
    log::info!("Entering main loop");

    let mut frame = RenderBuffer::empty();
    let mut state = State::Clock;

    loop {
        if control_socket.can_recv() {
            let n_bytes = control_socket.read(&mut state_decode_buf).await.unwrap();
            if n_bytes != 0 && state != State::Stream && state != State::Off {
                network::start_stream(&mut stream_socket, &mut state, &mut frame_duration_micros)
                    .await;
            }
        }
        if button_signal.signaled() {
            match state {
                State::Clock => {}
                State::Stream => {
                    stream_socket.abort();
                    stream_socket.flush().await.unwrap();
                }
                State::Off => {
                    screen_override.set_low();
                }
            }
            match button_signal.wait().await {
                ButtonPress::Short => {
                    log::info!("Got short button input!");
                    state.next();
                }
                ButtonPress::Long => {
                    log::info!("Got long button input!");
                    state.toggle_on_off();
                }
            }
            match state {
                State::Clock => {
                    frame_duration_micros = 300_000;
                    frame.reset();
                }
                State::Stream => {
                    network::start_stream(
                        &mut stream_socket,
                        &mut state,
                        &mut frame_duration_micros,
                    )
                    .await;
                }
                State::Off => {
                    screen_override.set_high();
                    frame_duration_micros = 1_000_000;
                }
            }
        }

        let frame_timer = Timer::after(Duration::from_micros(frame_duration_micros));
        match state {
            State::Stream => {
                stream_socket
                    .read_exact(frame.as_continuous_mut())
                    .await
                    .unwrap();
            }
            State::Clock => {
                if rtc_offset_signal.signaled() {
                    rtc_offset = rtc_offset_signal.wait().await;
                }
                let time = rtc::get_german_datetime(rtc, &rtc_offset);
                frame.show_time(&time);
            }
            State::Off => {}
        }

        frame_timer.await;
        *editable_render_buf = frame;
        render::render(&mut editable_render_buf).await;
    }
}
