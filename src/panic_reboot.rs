use defmt::unreachable;
use esp_hal::reset;

#[no_mangle]
pub extern "Rust" fn custom_halt() -> ! {
    reset::software_reset();
    unreachable!();
}
