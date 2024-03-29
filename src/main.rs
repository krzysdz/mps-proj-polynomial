// make `std` available when testing
// #![cfg_attr(not(test), no_std)]
// #![cfg_attr(not(test), no_main)]
#![no_std]
#![no_main]

#[macro_use]
extern crate static_assertions;

assert_cfg!(
    not(all(feature = "panic-semihosting", feature = "panic-itm")),
    "Only one panic handler may be specified"
);

// mod complex;
mod display_buffer;
mod io;
mod polynomial_2nd_degree;
mod rtc_source;

use chrono::{DateTime, NaiveDate, Utc};
use core::cell::RefCell;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use cortex_m::asm::wfi;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use display_buffer::DisplayBuffer;
use embedded_hal::spi::{Mode as SPIMode, Phase, Polarity};
use embedded_sdmmc as sdmmc;
use io::InputPorts;
use num::complex::Complex32;
use num::Complex;
#[cfg(feature = "analytical")]
use num::Float;
use num::Zero;
#[cfg(not(any(feature = "panic-semihosting", feature = "panic-itm")))]
use panic_halt as _;
#[cfg(feature = "panic-itm")]
use panic_itm as _;
#[cfg(feature = "panic-semihosting")]
use panic_semihosting as _;
use polynomial_2nd_degree::Polynomial2ndDegree;
use rtc_source::RTCSource;
use sdmmc::{BlockDevice, TimeSource, Volume};
use stm32f1xx_hal::rcc::Clocks;
use stm32f1xx_hal::{
    device::{AFIO, GPIOA},
    gpio::{
        self,
        gpioa::PA4,
        gpiob::{PB0, PB1},
        PushPull,
    },
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    rtc::{Rtc, RtcClkLsi},
    spi::{self, Spi, Spi1NoRemap},
};

const MODE: SPIMode = SPIMode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};
const FILE_NAME: &str = "MPS_RES.TXT";

struct LEDStruct {
    led1: PB0<gpio::Output<PushPull>>,
    led2: PB1<gpio::Output<PushPull>>,
}

// https://github.com/rust-embedded/not-yet-awesome-embedded-rust#sharing-data-with-interrupts
static DATA_READY: AtomicBool = AtomicBool::new(false);
static A_RAW_VAL: AtomicU32 = AtomicU32::new(0);
static B_RAW_VAL: AtomicU32 = AtomicU32::new(0);
static C_RAW_VAL: AtomicU32 = AtomicU32::new(0);
static CURRENT_BYTE: AtomicU8 = AtomicU8::new(0);
// https://therealprof.github.io/blog/interrupt-comparison/#cortex-m-rt-with-interrupts-and-moved-resources
static INPUT_PORTS: Mutex<RefCell<Option<InputPorts>>> = Mutex::new(RefCell::new(None));
static LEDS: Mutex<RefCell<Option<LEDStruct>>> = Mutex::new(RefCell::new(None));

#[exception]
fn SysTick() {
    static mut PORTS: Option<InputPorts> = None;
    static mut LED: Option<LEDStruct> = None;
    // if initialized
    if let Some(inp) = PORTS {
        let last_byte = inp.read_ports(&A_RAW_VAL, &B_RAW_VAL, &C_RAW_VAL, &CURRENT_BYTE);
        if last_byte {
            DATA_READY.store(last_byte, Ordering::Relaxed);
        }
        if let Some(leds) = LED {
            leds.led1.toggle(); // toggle LED1 on each read
            if last_byte {
                leds.led2.set_low(); // turn LED2 on after the last byte was read
            } else {
                leds.led2.set_high();
            }
        }
    } else {
        cortex_m::interrupt::free(|cs| {
            PORTS.replace(INPUT_PORTS.borrow(cs).replace(None).unwrap());
            if LED.is_none() {
                LED.replace(LEDS.borrow(cs).replace(None).unwrap());
            }
        });
    }
}

// MISO, CLK, MOSI = PA{5,6,7} (SPI1); CS = PA4
#[allow(non_snake_case)]
fn setup_spi(
    AFIO: AFIO,
    GPIOA: GPIOA,
    SPI1: SPI1,
    clocks: Clocks,
) -> (
    Spi<SPI1, Spi1NoRemap, impl spi::Pins<Spi1NoRemap>, u8>,
    PA4<gpio::Output<PushPull>>,
) {
    let mut afio = AFIO.constrain();
    let mut gpioa = GPIOA.split();

    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let cs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    let spi = Spi::spi1(
        SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        MODE,
        1_u32.mhz(),
        clocks,
    );

    (spi, cs)
}

#[entry]
// #[cfg(not(test))]
fn main() -> ! {
    let mut buffer = DisplayBuffer::default();
    let dp = Peripherals::take().unwrap();
    let p = CorePeripherals::take().unwrap();
    let mut syst = p.SYST;
    let mut pwr = dp.PWR;
    let rcc = dp.RCC.constrain();
    let mut backup_domain = rcc.bkp.constrain(dp.BKP, &mut pwr);
    let rtc = Rtc::<RtcClkLsi>::rtc(dp.RTC, &mut backup_domain);
    let mut flash = dp.FLASH.constrain();
    let cfgr = rcc.cfgr;
    let clocks = cfgr.freeze(&mut flash.acr);

    let mut gpiob = dp.GPIOB.split();
    let led1 = gpiob.pb0;
    let led2 = gpiob.pb1;
    let led1 = led1.into_push_pull_output_with_state(&mut gpiob.crl, gpio::PinState::High);
    let led2 = led2.into_push_pull_output_with_state(&mut gpiob.crl, gpio::PinState::High);

    cortex_m::interrupt::free(|cs| {
        *INPUT_PORTS.borrow(cs).borrow_mut() = Some(InputPorts::new(dp.GPIOD, dp.GPIOE));
        *LEDS.borrow(cs).borrow_mut() = Some(LEDStruct { led1, led2 });
    });

    let mut timesource = RTCSource::new(rtc);
    if cfg!(feature = "set-date") {
        let datetime =
            DateTime::<Utc>::from_utc(NaiveDate::from_ymd(2022, 1, 17).and_hms(2, 21, 35), Utc);
        timesource.set_date(datetime);
    }
    let (spi, cs) = setup_spi(dp.AFIO, dp.GPIOA, dp.SPI1, clocks);
    let mut spi_dev = sdmmc::SdMmcSpi::new(spi, cs);
    spi_dev.init().unwrap();
    let mut cont = sdmmc::Controller::new(spi_dev, timesource); // get FS controller
    let mut volume = cont.get_volume(sdmmc::VolumeIdx(0)).unwrap(); // halt on error
    let root_dir = cont.open_root_dir(&volume).unwrap();
    let mut file = cont
        .open_file_in_dir(
            &mut volume,
            &root_dir,
            FILE_NAME,
            sdmmc::Mode::ReadWriteCreateOrAppend,
        )
        .unwrap();

    // configure the system timer to wrap around every second
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(clocks.sysclk().0); // 1s (core clock is 8 MHz)
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    loop {
        // Make sure that no interrupts occur when reading the data - CRITICAL SECTION
        let p = cortex_m::interrupt::free(|_| {
            let status =
                DATA_READY.compare_exchange(true, false, Ordering::Relaxed, Ordering::Relaxed);
            if let Ok(true) = status {
                let a = A_RAW_VAL.load(Ordering::Relaxed);
                let a = Complex::from(f32::from_ne_bytes(a.to_ne_bytes()));
                let b = B_RAW_VAL.load(Ordering::Relaxed);
                let b = Complex::from(f32::from_ne_bytes(b.to_ne_bytes()));
                let c = C_RAW_VAL.load(Ordering::Relaxed);
                let c = Complex::from(f32::from_ne_bytes(c.to_ne_bytes()));
                Some(Polynomial2ndDegree::new(a, b, c))
            } else {
                None
            }
        });
        if let Some(p) = p {
            // solve the equation and print result
            // fortuantely the conversions can be done without transmute (equivalent of reinterpret_cast)
            let results = solve(p);
            write_result(results, &mut buffer);
            write_output(&buffer, &mut cont, &mut volume, &mut file);
        }
        wfi(); // wait for interrupt (put to sleep)
    }
}

enum SolveResult {
    NoSolutions,
    InfinitelyMany,
    SolvedLinear(f32),
    SolvedQuadratic(Complex32, Complex32),
}

#[cfg(not(feature = "analytical"))]
fn solve(p: Polynomial2ndDegree<Complex32>) -> SolveResult {
    const MAX_ITER: u32 = 100;
    const TOLERANCE_SQ: f32 = 1e-10; // square of the tolerance
    const STARTING_POINT: Complex32 = Complex { re: 1.0, im: 1.0 };
    let derivative = p.derive();
    let mut solved = false;
    let mut xi = STARTING_POINT;
    let mut x_prev = xi;
    if p.infinitely_many_solutions() {
        return SolveResult::InfinitelyMany;
    }
    if !p.equation_unsolvable() {
        for _i in 0..MAX_ITER {
            xi -= p.eval(xi) / derivative.eval(xi);
            if (xi - x_prev).norm_sqr() < TOLERANCE_SQ {
                solved = true;
                break;
            }
            x_prev = xi;
        }
    }
    if solved {
        let x1 = xi;
        let remaining_poly = p.reduce(x1);
        let Polynomial2ndDegree { b: a, c: b, .. } = remaining_poly;
        if a.is_zero() {
            // if a is zero the equation was b(x - x1) - a linear one
            // because inputs must be real x1 is real too
            SolveResult::SolvedLinear(x1.re)
        } else {
            let x2 = -b / a;
            SolveResult::SolvedQuadratic(x1, x2)
        }
    } else {
        SolveResult::NoSolutions
    }
}

#[cfg(feature = "analytical")]
fn solve(p: Polynomial2ndDegree<Complex32>) -> SolveResult {
    if p.infinitely_many_solutions() {
        return SolveResult::InfinitelyMany;
    }
    if p.equation_unsolvable() {
        return SolveResult::NoSolutions;
    }
    if p.a.is_zero() {
        return SolveResult::SolvedLinear(-p.b.re / p.a.re);
    }
    let delta = p.b.powu(2) - p.a * p.c * 4.0;
    assert!(delta.im == 0.0);
    let delta_sq = if delta.re >= 0.0 {
        delta.re.sqrt().into()
    } else {
        Complex32::new(0.0, (-delta.re).sqrt())
    };
    let x1 = (-p.b + delta_sq) / (p.a * 2.0);
    let x2 = (-p.b - delta_sq) / (p.a * 2.0);
    SolveResult::SolvedQuadratic(x1, x2)
}

fn write_result(result: SolveResult, buffer: &mut DisplayBuffer) {
    use SolveResult::*;

    buffer.clear();
    match result {
        InfinitelyMany => write!(buffer, "Infinitely many solutions").unwrap(), // by pure luck the words wrap perfectly
        NoSolutions => write!(buffer, "No solutions").unwrap(),
        SolvedLinear(x) => write!(buffer, "{:.2}", x).unwrap(),
        SolvedQuadratic(x1, x2) => {
            write!(buffer, "{:.2}", x1).unwrap();
            // Width is not supported without std https://github.com/rust-num/num-complex/blob/master/src/lib.rs#L1195-L1209
            buffer.set_cursor(16);
            write!(buffer, "{:.2}", x2).unwrap();
        }
    }
}

fn write_output<D, T>(
    buffer: &DisplayBuffer,
    cont: &mut sdmmc::Controller<D, T>,
    volume: &mut Volume,
    file: &mut sdmmc::File,
) where
    D: BlockDevice,
    T: TimeSource,
    <D as BlockDevice>::Error: core::fmt::Debug,
{
    let mut to_write = [0_u8; 35];
    let first_line = buffer.first_line();
    let second_line = buffer.second_line();
    let first_line_len = first_line.len();
    to_write[0..first_line_len].copy_from_slice(first_line);
    to_write[first_line_len] = 0xA; // 0xA = 10 = '\n'
    let second_line_start = first_line_len + 1;
    let second_line_end = second_line_start + second_line.len();
    let msg_end = second_line_end + 2;
    to_write[second_line_start..second_line_end].copy_from_slice(second_line);
    to_write[second_line_end..msg_end].copy_from_slice(b"\n\n");
    cont.write(volume, file, &to_write[0..msg_end]).unwrap(); // write at once
}

#[cfg(test)]
fn main() {}

#[cfg(test)]
mod tests {
    use num::{Complex, One, Zero};
    use std::println;

    use crate::{display_buffer::DisplayBuffer, polynomial_2nd_degree::Polynomial2ndDegree, solve};

    #[test]
    fn solve_x2_m1() {
        let mut buff = DisplayBuffer::default();
        let poly = Polynomial2ndDegree::<Complex<f32>>::new(
            Complex::<f32>::one(),
            Complex::<f32>::zero(),
            -Complex::<f32>::one(),
        );
        let results = solve(poly);
        write_result(results, &mut buff);
        println!(
            "{:?}\n{}\n{}",
            buff,
            std::str::from_utf8(buff.first_line()).unwrap(),
            std::str::from_utf8(buff.second_line()).unwrap()
        );
    }
}
