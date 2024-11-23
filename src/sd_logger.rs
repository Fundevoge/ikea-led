// use cortex_m::interrupt::{self, Mutex};
// use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeManager};
// use panic_halt as _;
// use stm32f3xx_hal::{delay::Delay, pac, prelude::*, rcc::RccExt, spi::Spi};

// use core::{
//     cell::RefCell,
//     fmt::{Debug, Write},
// };

// use cortex_m_rt::entry;
// use cortex_m_semihosting::hio;
// struct Clock;

// impl TimeSource for Clock {
//     fn get_timestamp(&self) -> Timestamp {
//         Timestamp {
//             year_since_1970: 0,
//             zero_indexed_month: 0,
//             zero_indexed_day: 0,
//             hours: 0,
//             minutes: 0,
//             seconds: 0,
//         }
//     }
// }

// static mut HSTDOUT: Option<hio::HostStream> = None;

// macro_rules! println {
//     ($($t:tt)*) => {
//         unsafe {
//             if let Some(console) = HSTDOUT.as_mut() {
//                 let _ = writeln!(console, $($t)*);
//             }
//         }
//     };
// }

// macro_rules! print {
//     ($($t:tt)*) => {
//         unsafe {
//             if let Some(console) = HSTDOUT.as_mut() {
//                 let _ = write!(console, $($t)*);
//             }
//         }
//     };
// }

// // Define the trait
// pub trait MyExpect<T> {
//     fn my_expect(self, msg: &str) -> T;
// }

// // Implement the trait for Result
// impl<T, E: Debug> MyExpect<T> for Result<T, E> {
//     fn my_expect(self, msg: &str) -> T {
//         match self {
//             Ok(val) => val,
//             Err(err) => {
//                 println!("{}: {:?}", msg, err);
//                 panic!("{}: {:?}", msg, err);
//             }
//         }
//     }
// }

// // Implement the trait for Result
// impl<T> MyExpect<T> for Option<T> {
//     fn my_expect(self, msg: &str) -> T {
//         match self {
//             Some(val) => val,
//             None => {
//                 println!("{}", msg);
//                 panic!("{}", msg);
//             }
//         }
//     }
// }

// #[entry]
// fn main() -> ! {
//     let hstdout = hio::hstdout().my_expect("Failed to initialize host stdout");
//     unsafe { HSTDOUT = Some(hstdout) };
//     println!("Hello World!");

//     let p = cortex_m::Peripherals::take().my_expect("Failed to take Cortex-M peripherals");
//     let dp = pac::Peripherals::take().my_expect("Failed to take device peripherals");
//     let mut flash = dp.FLASH.constrain();
//     let mut rcc = dp.RCC.constrain();
//     let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
//     let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

//     let clocks = rcc
//         .cfgr
//         .use_hse(8.MHz())
//         .sysclk(48.MHz())
//         .pclk1(24.MHz())
//         .freeze(&mut flash.acr);

//     // Configure pins for SPI
//     let sck = gpioa
//         .pa5
//         .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
//     let miso = gpioa
//         .pa6
//         .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
//     let mosi = gpioa
//         .pa7
//         .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

//     let spi = Spi::new(dp.SPI1, (sck, miso, mosi), 4.MHz(), clocks, &mut rcc.apb2);

//     // Build an SD Card interface out of an SPI device, a chip-select pin and a delay object
//     let sdcard = SdCard::new(
//         spi,
//         gpiob
//             .pb6
//             .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper),
//         Delay::new(p.SYST, clocks),
//     );
//     println!("INIT SD CARD!");
//     let mut volume_mgr = VolumeManager::new(sdcard, Clock);
//     println!(
//         "Card size is {} bytes",
//         volume_mgr
//             .device()
//             .num_bytes()
//             .my_expect("Failed to get card size")
//     );
//     let volume0 = volume_mgr
//         .open_volume(embedded_sdmmc::VolumeIdx(0))
//         .my_expect("Failed to open volume 0");
//     println!("Volume 0: {:?}", volume0);
//     // Open the root directory (passing in the volume we're using).
//     let root_dir = volume_mgr
//         .open_root_dir(volume0)
//         .my_expect("Failed to open root directory");
//     let my_file = volume_mgr
//         .open_file_in_dir(
//             root_dir,
//             "0",
//             embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
//         )
//         .my_expect("Failed to open or create file '0'");
//     volume_mgr
//         .write(my_file, b"Hello, this is a new file on disk\r\n")
//         .my_expect("Failed to write to file");
//     volume_mgr
//         .close_file(my_file)
//         .my_expect("Failed to close the file");

//     println!("Listing /");
//     let mut max_file_num = -1;
//     volume_mgr
//         .iterate_dir(root_dir, |entry| {
//             if entry.attributes.is_directory()
//                 || entry.attributes.is_hidden()
//                 || entry.attributes.is_read_only()
//                 || entry.attributes.is_system()
//                 || entry.attributes.is_volume()
//             {
//                 return;
//             }
//             print!("{:12} {:9} {}", entry.name, entry.size, entry.mtime,);
//             if entry.attributes.is_archive() {
//                 print!("<ARC>");
//             }
//             if entry.attributes.is_lfn() {
//                 print!("<LFF>");
//             }
//             println!();
//             if !entry
//                 .name
//                 .base_name()
//                 .iter()
//                 .all(|b| b'0' <= *b && *b <= b'9')
//             {
//                 return;
//             }
//             let num = entry
//                 .name
//                 .base_name()
//                 .as_ascii()
//                 .expect("File name should be ascii")
//                 .as_str()
//                 .parse::<i32>()
//                 .expect("Number should be parseable");
//             println!("Number {num}");
//             max_file_num = max_file_num.max(num);
//         })
//         .my_expect("Failed to iterate through directory");
//     println!("Highest file number: {max_file_num}");

//     volume_mgr
//         .close_dir(root_dir)
//         .my_expect("Failed to close root directory");

//     panic!()
// }
