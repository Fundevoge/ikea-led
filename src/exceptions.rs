#![allow(rustdoc::bare_urls, unused_macros)]

use esp_println as _;

macro_rules! println {
    ($($arg:tt)*) => {
        esp_println::println!($($arg)*);
    };
}

#[cfg_attr(target_arch = "xtensa", path = "xtensa.rs")]
pub mod arch;

#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    println!("");
    println!("====================== PANIC ======================");

    println!("{}", info);

    println!("");
    println!("Backtrace:");
    println!("");

    let backtrace = arch::backtrace();
    for addr in backtrace.into_iter().flatten() {
        println!("0x{:x}", addr - arch::RA_OFFSET);
    }

    halt();
}

#[no_mangle]
#[link_section = ".rwtext"]
unsafe fn __user_exception(cause: arch::ExceptionCause, context: arch::Context) {
    // Unfortunately, a different formatter string is used
    esp_println::println!("\n\nException occured '{:?}'", cause);

    println!("{:?}", context);

    let backtrace = arch::backtrace_internal(context.A1, 0);
    for addr in backtrace.into_iter().flatten() {
        println!("0x{:x}", addr);
    }
    println!("");
    println!("");
    println!("");
    halt();
}

#[allow(unused)]
fn halt() -> ! {
    loop {
        continue;
    }
}
