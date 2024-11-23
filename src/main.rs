#![no_std]
#![no_main]
#![feature(const_mut_refs)]
#![feature(const_slice_from_raw_parts_mut)]
#![feature(asm_experimental_arch)]
#![feature(slice_index_methods)]

mod flag;
mod network;
mod panic_reboot;
mod patterns;
mod render;
mod rtc;
mod sd_logger;
mod tz_de;

use core::ptr::addr_of_mut;
use esp_backtrace as _;
use esp_hal::{
    cpu_control::{self, CpuControl},
    dma::Dma,
    gpio::{self, GpioPin},
    prelude::*,
    rng::Rng,
    rtc_cntl::Rtc,
    spi::{master::Spi, SpiMode},
    timer::timg::TimerGroup,
};
use esp_hal_embassy::{self, Executor};
use esp_wifi::{
    wifi::{WifiDevice, WifiStaDevice},
    EspWifiController,
};

use embassy_net::{tcp::TcpSocket, Ipv4Address, Ipv4Cidr, StackResources, StaticConfigV4};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    signal::Signal,
};
use embassy_time::{Duration, Timer};

use embedded_svc::io::asynch::Read;

use heapless::Vec;

use flag::Flag;
use render::RenderBuffer;
use static_cell::StaticCell;

#[derive(defmt::Format, PartialEq, Eq, Clone, Copy)]
enum ButtonPress {
    Short,
    Long,
}

#[derive(defmt::Format, PartialEq, Eq, Clone, Copy)]
enum State {
    Stream,
    Clock(ClockType),
    Off,
}

impl Default for State {
    fn default() -> Self {
        State::Clock(ClockType::Large)
    }
}

#[derive(defmt::Format, PartialEq, Eq, Clone, Copy)]
enum ClockType {
    Small,
    Large,
}

impl State {
    fn next(&mut self) {
        *self = match self {
            State::Stream => State::Clock(ClockType::Large),
            State::Clock(ClockType::Large) => State::Clock(ClockType::Small),
            State::Clock(ClockType::Small) => State::Stream,
            State::Off => Self::default(),
        };
    }

    fn toggle_on_off(&mut self) {
        *self = match self {
            State::Off => Self::default(),
            _ => State::Off,
        };
    }
}

// Inter task communication
static WIFI_ENABLED_FLAG: StaticCell<Flag<NoopRawMutex>> = StaticCell::new();
static RTC_OFFSET_FLAG: StaticCell<Flag<NoopRawMutex>> = StaticCell::new();
static KEEPALIVE_ESTABLISHED_FLAG: StaticCell<Flag<NoopRawMutex>> = StaticCell::new();
static BUTTON_SIGNAL: StaticCell<Signal<NoopRawMutex, ButtonPress>> = StaticCell::new();
static mut RENDER_BUF_PTR_USED: Mutex<CriticalSectionRawMutex, *mut RenderBuffer> =
    Mutex::new(addr_of_mut!(RENDER_BUF_1));

// Static Resources
static RTC: StaticCell<Rtc> = StaticCell::new();
static WIFI_CONTROLLER: StaticCell<EspWifiController<'static>> = StaticCell::new();
static WIFI_STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
static WIFI_PROGRAM_STACK: StaticCell<embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>> =
    StaticCell::new();
static mut APP_CORE_STACK: cpu_control::Stack<8192> = cpu_control::Stack::new();

// Shared mutable memory
static mut RENDER_BUF_1: RenderBuffer = RenderBuffer::empty();
static mut RENDER_BUF_2: RenderBuffer = RenderBuffer::empty();

#[embassy_executor::task]
async fn button_read_task(
    mut btn: esp_hal::gpio::Input<'static, GpioPin<1>>,
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

// Additional memory of main task to reduce stack size
const RX_BUFFER_SIZE_STREAM: usize = 2048;
const TX_BUFFER_SIZE_STREAM: usize = 64;
static mut STREAM_RX_BUFFER: [u8; RX_BUFFER_SIZE_STREAM] = [0; RX_BUFFER_SIZE_STREAM];
static mut STREAM_TX_BUFFER: [u8; TX_BUFFER_SIZE_STREAM] = [0; TX_BUFFER_SIZE_STREAM];

const RX_BUFFER_SIZE_CONTROL: usize = 1024;
const TX_BUFFER_SIZE_CONTROL: usize = 64;
static mut CONTROL_RX_BUFFER: [u8; RX_BUFFER_SIZE_CONTROL] = [0; RX_BUFFER_SIZE_CONTROL];
static mut CONTROL_TX_BUFFER: [u8; TX_BUFFER_SIZE_CONTROL] = [0; TX_BUFFER_SIZE_CONTROL];

#[main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timg1 = TimerGroup::new(peripherals.TIMG1);

    esp_hal_embassy::init(timg1.timer0);

    log::info!("Hello world!");

    let mut screen_override = gpio::Output::new(peripherals.GPIO2, gpio::Level::Low);

    let rtc: &'static Rtc = RTC.init(Rtc::new(peripherals.LPWR));

    let wifi_controller: &'static EspWifiController<'static> = WIFI_CONTROLLER.uninit().write(
        esp_wifi::init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap(),
    );

    let wifi = peripherals.WIFI;
    let (wifi_interface, wifi_controller) =
        esp_wifi::wifi::new_with_mode(wifi_controller, wifi, WifiStaDevice).unwrap();

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

    let wifi_stack_resources: &'static mut StackResources<4> =
        &mut *WIFI_STACK_RESOURCES.init(StackResources::<4>::new());

    let wifi_program_stack: &'static embassy_net::Stack<WifiDevice<'_, WifiStaDevice>> =
        &*WIFI_PROGRAM_STACK.init(embassy_net::Stack::new(
            wifi_interface,
            config,
            wifi_stack_resources,
            seed,
        ));

    spawner
        .spawn(network::net_task(wifi_program_stack))
        .unwrap();

    let wifi_enabled_flag: &'static Flag<NoopRawMutex> = &*WIFI_ENABLED_FLAG.init(Flag::new());

    spawner
        .spawn(network::connection(wifi_controller, wifi_enabled_flag))
        .unwrap();
    wifi_enabled_flag.wait_peek().await;

    let keepalive_established_flag: &'static Flag<NoopRawMutex> =
        &*KEEPALIVE_ESTABLISHED_FLAG.init(Flag::new());

    spawner
        .spawn(network::keep_alive(
            wifi_program_stack,
            keepalive_established_flag,
            wifi_enabled_flag,
        ))
        .unwrap();
    keepalive_established_flag.wait_peek().await;

    let rtc_offset_flag: &'static Flag<NoopRawMutex> = &*RTC_OFFSET_FLAG.init(Flag::new());

    spawner
        .spawn(rtc::rtc_adjust_task(
            wifi_program_stack,
            rtc,
            rtc_offset_flag,
            wifi_enabled_flag,
        ))
        .unwrap();
    rtc_offset_flag.wait_peek().await;

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

    let sclk = peripherals.GPIO40;
    let miso = peripherals.GPIO26;
    let mosi = peripherals.GPIO41;
    let cs = peripherals.GPIO42;
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let spi: Spi<'static, esp_hal::Async> = Spi::new_with_config(
        peripherals.SPI2,
        esp_hal::spi::master::Config {
            frequency: 4.MHz(),
            mode: SpiMode::Mode0,
            ..esp_hal::spi::master::Config::default()
        },
    )
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_cs(cs)
    .into_async();

    let cpu1_fnctn = move || {
        static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor: &'static mut Executor = &mut *EXECUTOR.init(Executor::new());
        executor.run(|spawner| {
            spawner.spawn(render::render_loop(spi, dma_channel)).ok();
        });
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, cpu1_fnctn)
        .unwrap();
    log::info!("Started Spi renderer!");

    let btn = gpio::Input::new_typed(peripherals.GPIO1, gpio::Pull::Up);

    let button_signal: &'static Signal<NoopRawMutex, ButtonPress> =
        &*BUTTON_SIGNAL.init(Signal::new());

    spawner.spawn(button_read_task(btn, button_signal)).unwrap();

    let mut editable_render_buf = unsafe { &mut *addr_of_mut!(RENDER_BUF_2) };

    let mut state_decode_buf = [0_u8; 16];
    let mut frame_duration_micros: u64 = 200_000;
    log::info!("Entering main loop");

    let mut frame = RenderBuffer::empty();
    let mut state = State::default();

    loop {
        if control_socket.can_recv() {
            let n_bytes = control_socket.read(&mut state_decode_buf).await.unwrap();
            if n_bytes != 0 && state != State::Stream && state != State::Off {
                network::start_stream(&mut stream_socket, &mut state, &mut frame_duration_micros)
                    .await;
            }
        }
        if button_signal.signaled() {
            // Cleanup previous state
            match state {
                State::Clock(_) => {}
                State::Stream => {
                    stream_socket.abort();
                    stream_socket.flush().await.unwrap();
                }
                State::Off => {
                    screen_override.set_low();
                }
            }
            // Move to next State
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
            // Initialize next State
            match state {
                State::Clock(_) => {
                    frame_duration_micros = 300_000;
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
            State::Clock(clock_type) => {
                if rtc_offset_flag.flagged() {
                    rtc_offset_flag.wait_take().await;
                }
                let time = rtc::get_german_datetime(rtc);

                frame.show_time(&time, clock_type);
            }
            State::Off => {}
        }

        frame_timer.await;
        *editable_render_buf = frame;
        render::render(&mut editable_render_buf).await;
    }
}
