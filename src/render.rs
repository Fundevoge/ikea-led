use core::{
    mem,
    ops::{Deref as _, DerefMut as _},
    ptr::addr_of_mut,
};

use chrono::{DateTime, Timelike as _};
use embassy_time::{Duration, Ticker};
use esp_hal::{
    dma::{ChannelCreator0, DmaPriority, DmaTxBuf},
    dma_descriptors,
    spi::{master::Spi, FullDuplexMode},
};
use micromath::F32Ext;

use crate::patterns::DIGITS_3_5;

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
static mut FRAME_COUNTER: u32 = 0;

#[derive(Copy, Clone, Debug)]
pub(crate) struct RenderBuffer([[u8; COLS]; ROWS]);

impl Default for RenderBuffer {
    fn default() -> Self {
        Self::empty()
    }
}

#[allow(unused)]
impl RenderBuffer {
    pub(crate) const fn empty() -> Self {
        Self([[0; COLS]; ROWS])
    }

    pub(crate) const fn as_continuous(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts((&self.0) as *const _ as *const u8, ROWS * COLS) }
    }
    pub(crate) const fn as_continuous_mut(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut((&mut self.0) as *mut _ as *mut u8, ROWS * COLS) }
    }

    pub(crate) fn reset(&mut self) {
        for v in self.as_continuous_mut() {
            *v = 0;
        }
    }

    pub(crate) fn invert(mut self) -> Self {
        for value in self.as_continuous_mut() {
            *value = 255 - *value;
        }
        self
    }

    pub(crate) fn even_on() -> Self {
        let mut buf = Self::default();
        for value in buf.as_continuous_mut().iter_mut().step_by(2) {
            *value = 255;
        }
        buf
    }

    pub(crate) fn odd_on() -> Self {
        Self::even_on().invert()
    }

    pub(crate) fn first_row_inc() -> Self {
        let mut buf = Self::default();
        for (i, value) in buf.0[0].iter_mut().enumerate() {
            *value = i as u8 * COLS as u8 + 15;
        }
        buf
    }

    pub(crate) fn corner_distance() -> Self {
        const NORM: f32 = 1.0 / (16.0 * core::f32::consts::SQRT_2);
        let mut buf = Self::default();
        for (y, row) in buf.0.iter_mut().enumerate() {
            for (x, value) in row.iter_mut().enumerate() {
                *value = (255.0 * (1.0 - ((x.pow(2) + y.pow(2)) as f32).sqrt() * NORM)) as u8;
            }
        }
        buf
    }

    pub(crate) fn test_pattern() -> Self {
        let mut buf = Self::default();
        for (i, line) in buf.0.iter_mut().enumerate() {
            for (j, pixel) in line.iter_mut().enumerate() {
                *pixel = (16 * i + j) as u8;
            }
        }
        buf
    }

    pub(crate) fn show_time<T: chrono::TimeZone>(&mut self, time: &DateTime<T>) {
        let hours = time.hour();
        let minutes = time.minute();
        self.show_number_3_5(hours as u8 / 10, 3, 1);
        self.show_number_3_5(hours as u8 % 10, 7, 1);
        self.show_number_3_5(minutes as u8 / 10, 3, 8);
        self.show_number_3_5(minutes as u8 % 10, 7, 8);
    }

    pub(crate) fn show_number_3_5(&mut self, digit: u8, offset_x: usize, offset_y: usize) {
        for (image_row, digit_row) in self.0[offset_y..]
            .iter_mut()
            .zip(&DIGITS_3_5[digit as usize])
        {
            for (image_pixel, digit_pixel) in image_row[offset_x..].iter_mut().zip(digit_row) {
                *image_pixel = *digit_pixel;
            }
        }
    }
}

const RENDER_LOOP_DURATION_MICROS: u64 = 50;
#[embassy_executor::task]
pub(crate) async fn render_loop(
    spi: Spi<'static, esp_hal::peripherals::SPI2, FullDuplexMode>,
    dma_channel: ChannelCreator0,
) {
    let mut spi = spi.with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority9));
    let (tx_descriptors, _rx_descriptors) = dma_descriptors!(4096);
    let mut tx_descriptors: &mut [esp_hal::dma::DmaDescriptor] = tx_descriptors;

    let mut min_brightness = 0_u16;
    static mut PIXEL_BITS_BUFFER_1: [u8; ROWS * COLS / 8] = [0_u8; ROWS * COLS / 8];
    static mut PIXEL_BITS_BUFFER_2: [u8; ROWS * COLS / 8] = [0_u8; ROWS * COLS / 8];
    let mut pixel_bits_buffer_writable = unsafe { addr_of_mut!(PIXEL_BITS_BUFFER_1) };
    let mut pixel_bits_buffer_in_transfer = unsafe { addr_of_mut!(PIXEL_BITS_BUFFER_2) };
    let mut ticker = Ticker::every(Duration::from_micros(RENDER_LOOP_DURATION_MICROS));

    loop {
        let buffer_in_transfer = unsafe { &mut *(pixel_bits_buffer_in_transfer as *mut _) };
        let dma_buffer_in_transfer = DmaTxBuf::new(tx_descriptors, buffer_in_transfer).unwrap();
        let spi_transfer = spi.dma_write(dma_buffer_in_transfer).unwrap();

        let mb = min_brightness;
        let mb_1 = mb + 1;
        let buffer_guard = unsafe { crate::RENDER_BUF_PTR_USED.lock() }.await;
        let buffer = unsafe { &**buffer_guard.deref() };
        for (idx, pixel_position) in POSITIONS.iter().copied().enumerate() {
            let b_val = buffer.as_continuous()[pixel_position as usize];
            // if b_val == 1 {
            //     b_val += 1;
            // }
            let b_val = b_val as u16;

            //if b_val >= *min_brightness {
            if (b_val * mb) / (GRAY_LEVELS - 1) != (b_val * mb_1) / (GRAY_LEVELS - 1) {
                unsafe {
                    (*pixel_bits_buffer_writable)[idx / 8] |= 0b1000_0000 >> (idx % 8);
                }
            } else {
                unsafe {
                    (*pixel_bits_buffer_writable)[idx / 8] &= !(0b1000_0000 >> (idx % 8));
                }
            };
        }
        unsafe {
            FRAME_COUNTER += 1;
        }
        drop(buffer_guard);
        min_brightness = (min_brightness + BRIGHTNESS_STEP) % 256;

        let (spi_, buf) = spi_transfer.wait();
        spi = spi_;
        let (tx_desc, _) = buf.split();
        tx_descriptors = tx_desc;

        mem::swap(
            &mut pixel_bits_buffer_writable,
            &mut pixel_bits_buffer_in_transfer,
        );

        ticker.next().await;
    }
}

pub(crate) async fn render(editable_buffer: &mut &mut RenderBuffer) {
    let mut render_buf_used_guard = unsafe { crate::RENDER_BUF_PTR_USED.lock() }.await;
    let render_buf_used = render_buf_used_guard.deref_mut();
    let new_free_render_buf = *render_buf_used;
    *render_buf_used = *editable_buffer;
    // let frame_count;
    unsafe {
        // frame_count = FRAME_COUNTER;
        FRAME_COUNTER = 0;
    }
    drop(render_buf_used_guard);
    *editable_buffer = unsafe { &mut *new_free_render_buf };
    // log::info!(
    //     "Displayed frame {:.3} times [{frame_count} repetitions]",
    //     (frame_count as f32) / 256.0
    // );
}
