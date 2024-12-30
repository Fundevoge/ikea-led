#![no_std]
#![no_main]
#![feature(const_mut_refs)]
#![feature(const_slice_from_raw_parts_mut)]
#![feature(asm_experimental_arch)]
#![feature(slice_index_methods)]
#![feature(ascii_char)]

mod flag;
mod network;
mod panic_reboot;
mod patterns;
mod render;
mod sd_logger;
mod tz_de;
mod xtensa;

use chrono::TimeZone as _;
use core::ptr::addr_of_mut;
use tz_de::TzDe;
// use esp_backtrace as _;
use esp_hal::{
    cpu_control::{self, CpuControl},
    dma::Dma,
    gpio::{self, GpioPin, Output},
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
use panic_reboot::{FIRST_REBOOT, REBOOT_IMMINENT};
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

impl TryFrom<u8> for State {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(State::Off),
            1 => Ok(State::Stream),
            2..=3 => Ok(State::Clock(ClockType::try_from(value - 2)?)),
            _ => Err(()),
        }
    }
}

#[derive(defmt::Format, PartialEq, Eq, Clone, Copy)]
enum ClockType {
    Small,
    Large,
}

impl TryFrom<u8> for ClockType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ClockType::Small),
            1 => Ok(ClockType::Large),
            _ => Err(()),
        }
    }
}

impl State {
    fn next(&self) -> Self {
        match self {
            State::Stream => State::Clock(ClockType::Large),
            State::Clock(ClockType::Large) => State::Clock(ClockType::Small),
            State::Clock(ClockType::Small) => State::Stream,
            State::Off => Self::default(),
        }
    }

    fn toggle_on_off(&self) -> Self {
        match self {
            State::Off => Self::default(),
            _ => State::Off,
        }
    }

    const fn frame_duration(&self) -> Option<u64> {
        match self {
            State::Stream => None,
            State::Clock(_) => Some(10_000),
            State::Off => Some(10_000),
        }
    }
}

// Inter task communication
static WIFI_ENABLED_FLAG: StaticCell<Flag<NoopRawMutex>> = StaticCell::new();
static BUTTON_SIGNAL: StaticCell<Signal<NoopRawMutex, ButtonPress>> = StaticCell::new();
static STATE_SIGNAL: StaticCell<Signal<NoopRawMutex, State>> = StaticCell::new();
static mut RENDER_BUF_PTR_USED: Mutex<CriticalSectionRawMutex, *mut RenderBuffer> =
    Mutex::new(addr_of_mut!(RENDER_BUF_1));

// Static Resources
static RTC: StaticCell<Rtc> = StaticCell::new();
static mut RTC2: Option<&'static Rtc> = None;
static WIFI_CONTROLLER: StaticCell<EspWifiController<'static>> = StaticCell::new();
static WIFI_STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
static WIFI_PROGRAM_STACK: StaticCell<embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>> =
    StaticCell::new();
static mut APP_CORE_STACK: cpu_control::Stack<8192> = cpu_control::Stack::new();
static CPU1_EXECUTOR: StaticCell<Executor> = StaticCell::new();

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

const WIFI_RX_BUFFER_SIZE: usize = 2048;
const WIFI_TX_BUFFER_SIZE: usize = 256;
static mut WIFI_RX_BUFFER: [u8; WIFI_RX_BUFFER_SIZE] = [0; WIFI_RX_BUFFER_SIZE];
static mut WIFI_TX_BUFFER: [u8; WIFI_TX_BUFFER_SIZE] = [0; WIFI_TX_BUFFER_SIZE];

#[main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    unsafe {
        FIRST_REBOOT = true;
        REBOOT_IMMINENT = false;
    }

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let dma = Dma::new(peripherals.DMA);
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    let rng = Rng::new(peripherals.RNG);

    let rtc: &'static Rtc = RTC.init(Rtc::new(peripherals.LPWR));
    unsafe { RTC2 = Some(rtc) };

    let wifi = peripherals.WIFI;
    let radio_clk = peripherals.RADIO_CLK;
    let timg_peripheral_wifi = peripherals.TIMG0;
    let timg_peripheral_embassy = peripherals.TIMG1;

    let screen_override_pin = peripherals.GPIO2;
    let button_pin = peripherals.GPIO1;

    let sclk_sd = peripherals.GPIO35;
    let miso_sd = peripherals.GPIO36;
    let mosi_sd = peripherals.GPIO37;
    let cs_sd = peripherals.GPIO38;

    let sclk_display = peripherals.GPIO40;
    let _miso_display = peripherals.GPIO26;
    let mosi_display = peripherals.GPIO41;
    let cs_display = peripherals.GPIO42;

    let dma_channel_display = dma.channel0;

    let spi_peripheral_sd = peripherals.SPI3;
    let spi_peripheral_display = peripherals.SPI2;

    let spi_sd: Spi<'static, esp_hal::Blocking> = Spi::new_with_config(
        spi_peripheral_sd,
        esp_hal::spi::master::Config {
            frequency: 4.MHz(),
            mode: SpiMode::Mode0,
            ..esp_hal::spi::master::Config::default()
        },
    )
    .with_sck(sclk_sd)
    .with_mosi(mosi_sd)
    .with_miso(miso_sd);
    let cs_pin_sd = Output::new(cs_sd, gpio::Level::High);
    sd_logger::init_logger(log::LevelFilter::Info, spi_sd, cs_pin_sd);

    // esp_println::logger::init_logger(log::LevelFilter::Info);

    esp_alloc::heap_allocator!(72 * 1024);

    let timg_wifi = TimerGroup::new(timg_peripheral_wifi);
    let timg_embassy = TimerGroup::new(timg_peripheral_embassy);

    esp_hal_embassy::init(timg_embassy.timer0);

    log::info!("[MAIN] Hello world!");

    let mut screen_override = gpio::Output::new(screen_override_pin, gpio::Level::Low);

    let wifi_controller: &'static EspWifiController<'static> =
        WIFI_CONTROLLER.init(esp_wifi::init(timg_wifi.timer0, rng, radio_clk).unwrap());

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

    let mut stream_socket = TcpSocket::new(
        wifi_program_stack,
        unsafe { &mut *addr_of_mut!(STREAM_RX_BUFFER) },
        unsafe { &mut *addr_of_mut!(STREAM_TX_BUFFER) },
    );

    // Setting up the TCP Socket
    let mut tcp_socket = TcpSocket::new(
        wifi_program_stack,
        unsafe { &mut *addr_of_mut!(WIFI_RX_BUFFER) },
        unsafe { &mut *addr_of_mut!(WIFI_TX_BUFFER) },
    );
    network::setup_tcp_socket(&mut tcp_socket).await;

    let spi: Spi<'static, esp_hal::Async> = Spi::new_with_config(
        spi_peripheral_display,
        esp_hal::spi::master::Config {
            frequency: 4.MHz(),
            mode: SpiMode::Mode0,
            ..esp_hal::spi::master::Config::default()
        },
    )
    .with_sck(sclk_display)
    .with_mosi(mosi_display)
    .with_miso(_miso_display)
    .with_cs(cs_display)
    .into_async();

    let cpu1_fnctn = move || {
        let executor: &'static mut Executor = &mut *CPU1_EXECUTOR.init(Executor::new());
        executor.run(|spawner| {
            spawner
                .spawn(render::render_loop(spi, dma_channel_display))
                .ok();
        });
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, cpu1_fnctn)
        .unwrap();
    log::info!("[MAIN] Started Spi renderer!");

    let button_input = gpio::Input::new_typed(button_pin, gpio::Pull::Up);

    let button_signal: &'static Signal<NoopRawMutex, ButtonPress> =
        &*BUTTON_SIGNAL.init(Signal::new());

    spawner
        .spawn(button_read_task(button_input, button_signal))
        .unwrap();

    let state_signal: &'static Signal<NoopRawMutex, State> = &*STATE_SIGNAL.init(Signal::new());
    spawner
        .spawn(network::tcp_socket_task(
            tcp_socket,
            rtc,
            wifi_enabled_flag,
            state_signal,
        ))
        .unwrap();

    let mut editable_render_buf = unsafe { &mut *addr_of_mut!(RENDER_BUF_2) };

    let mut frame_duration_micros: u64 = 200_000;
    log::info!("[MAIN] Entering main loop");

    let mut frame = RenderBuffer::empty();
    let mut state = State::default();
    loop {
        let frame_timer = Timer::after(Duration::from_micros(frame_duration_micros));

        // Render frame depending on the state
        match state {
            State::Stream => {
                if let Err(e) = stream_socket.read_exact(frame.as_continuous_mut()).await {
                    log::error!("[MAIN] (Stream Socket) Error receiving packet {e}!");
                    stream_socket.abort();
                    stream_socket.flush().await.unwrap();

                    state = State::default();
                    frame_duration_micros = state.frame_duration().unwrap();

                    // network::reset_tcp_socket(&mut tcp_socket, wifi_enabled_flag).await;
                }
            }
            State::Clock(clock_type) => {
                let time = TzDe.from_utc_datetime(&rtc.current_time());
                frame.show_time(&time, clock_type);
            }
            State::Off => {}
        }
        *editable_render_buf = frame;
        render::render(&mut editable_render_buf).await;

        // Checking for new state from Network or Button
        let mut new_state: Option<State> = state_signal.try_take();

        if button_signal.signaled() {
            match button_signal.wait().await {
                ButtonPress::Short => {
                    log::info!("[MAIN] (Button) Got short button input!");
                    new_state = Some(state.next());
                }
                ButtonPress::Long => {
                    log::info!("[MAIN] (Button) Got long button input!");
                    new_state = Some(state.toggle_on_off());
                }
            }
        }

        if let Some(new_state) = new_state {
            if state != new_state {
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

                // Initialize next State
                match new_state {
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
                    }
                    _ => {}
                }
                if let Some(frame_duration) = new_state.frame_duration() {
                    frame_duration_micros = frame_duration;
                }

                // Move to next State
                state = new_state;
            }
        }

        frame_timer.await;
    }
}
