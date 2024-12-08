use defmt::unreachable;
use esp_hal::reset;

static mut REBOOT_IMMINENT: bool = false;
pub(crate) static mut FIRST_REBOOT: bool = true;

fn halt() -> ! {
    reset::software_reset();
    unreachable!();
}

use defmt as _;

pub(crate) const MAX_BACKTRACE_ADDRESSES: usize = 10;

#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    pre_backtrace();

    log::error!(
        "\n\n\n====================== PANIC ======================\n{}",
        defmt::Display2Format(info)
    );

    log::error!("\nBacktrace:\n");

    let backtrace = crate::xtensa::backtrace();
    for addr in backtrace.into_iter().flatten() {
        log::error!("0x{:x}", addr - crate::xtensa::RA_OFFSET);
    }
    log::error!("\n\n");

    halt();
}

#[no_mangle]
#[link_section = ".rwtext"]
unsafe fn __user_exception(cause: crate::xtensa::ExceptionCause, context: crate::xtensa::Context) {
    pre_backtrace();

    log::error!("\n\nException occurred '{}'\n{:?}", cause, context);

    let backtrace = crate::xtensa::backtrace_internal(context.A1, 0);
    for addr in backtrace.iter().flatten() {
        log::error!("0x{:x}", addr);
    }
    log::error!("\n\n");

    halt();
}

// Ensure that the address is in DRAM and that it is 16-byte aligned.
//
// Based loosely on the `esp_stack_ptr_in_dram` function from
// `components/esp_hw_support/include/esp_memory_utils.h` in ESP-IDF.
//
// Address ranges can be found in `components/soc/$CHIP/include/soc/soc.h` as
// `SOC_DRAM_LOW` and `SOC_DRAM_HIGH`.
pub(crate) fn is_valid_ram_address(address: u32) -> bool {
    if (address & 0xF) != 0 {
        return false;
    }

    if !(0x3FC8_8000..=0x3FD0_0000).contains(&address) {
        return false;
    }

    true
}

fn pre_backtrace() {
    unsafe {
        if REBOOT_IMMINENT {
            FIRST_REBOOT = false;
        }
        REBOOT_IMMINENT = true;
    }
}
