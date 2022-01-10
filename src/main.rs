// make `std` available when testing
// #![cfg_attr(not(test), no_std)]
// #![cfg_attr(not(test), no_main)]
#![no_std]
#![no_main]

// mod complex;
mod display_buffer;
mod io;
mod polynomial_2nd_order;

use core::cell::RefCell;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
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
use panic_halt as _;
use polynomial_2nd_order::Polynomial2ndOrder;
use sdmmc::{BlockDevice, TimeSource, Timestamp, Volume};
use stm32f1xx_hal::device::{AFIO, FLASH, GPIOA, RCC};
use stm32f1xx_hal::gpio::gpiob::{PB0, PB1};
use stm32f1xx_hal::{
    gpio::{self, gpioa::PA4, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{self, Spi, Spi1NoRemap},
};

const MODE: SPIMode = SPIMode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};
const FILE_NAME: &'static str = "MPS_RES.TXT";

struct Clock;
impl TimeSource for Clock {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 52,
            zero_indexed_month: 0,
            zero_indexed_day: 9,
            hours: 12,
            minutes: 30,
            seconds: 0,
        }
    }
}

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
            if let None = LED {
                LED.replace(LEDS.borrow(cs).replace(None).unwrap());
            }
        });
    }
}

// MISO, CLK, MOSI = PA{5,6,7} (SPI1); CS = PA4
#[allow(non_snake_case)]
fn setup_spi(
    FLASH: FLASH,
    RCC: RCC,
    AFIO: AFIO,
    GPIOA: GPIOA,
    SPI1: SPI1,
) -> (
    Spi<SPI1, Spi1NoRemap, impl spi::Pins<Spi1NoRemap>, u8>,
    PA4<gpio::Output<PushPull>>,
) {
    let mut flash = FLASH.constrain();
    let rcc = RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

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

    let mut gpiob = dp.GPIOB.split();
    let led1 = gpiob.pb0;
    let led2 = gpiob.pb1;
    let led1 = led1.into_push_pull_output_with_state(&mut gpiob.crl, gpio::PinState::High);
    let led2 = led2.into_push_pull_output_with_state(&mut gpiob.crl, gpio::PinState::High);

    cortex_m::interrupt::free(|cs| {
        *INPUT_PORTS.borrow(cs).borrow_mut() = Some(InputPorts::new(dp.GPIOD, dp.GPIOE));
        *LEDS.borrow(cs).borrow_mut() = Some(LEDStruct { led1, led2 });
    });

    // configure the system timer to wrap around every second
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(8_000_000); // 1s
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    let (spi, cs) = setup_spi(dp.FLASH, dp.RCC, dp.AFIO, dp.GPIOA, dp.SPI1);
    let mut cont = sdmmc::Controller::new(sdmmc::SdMmcSpi::new(spi, cs), Clock); // get FS controller
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
                Some(Polynomial2ndOrder::new(a, b, c))
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
    }
}

enum SolveResult {
    NoSolutions,
    InfinitelyMany,
    Solved(Complex32, Complex32),
}

#[cfg(not(feature = "analytical"))]
fn solve(p: Polynomial2ndOrder<Complex32>) -> SolveResult {
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
        let Polynomial2ndOrder { b: a, c: b, .. } = remaining_poly;
        let x2 = -b / a;
        SolveResult::Solved(x1, x2)
    } else {
        SolveResult::NoSolutions
    }
}

#[cfg(feature = "analytical")]
fn solve(p: Polynomial2ndOrder<Complex32>) -> SolveResult {
    if p.infinitely_many_solutions() {
        return SolveResult::InfinitelyMany;
    }
    if p.equation_unsolvable() {
        return SolveResult::NoSolutions;
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
    SolveResult::Solved(x1, x2)
}

fn write_result(result: SolveResult, buffer: &mut DisplayBuffer) {
    use SolveResult::*;

    buffer.clear();
    match result {
        InfinitelyMany => write!(buffer, "Infinitely many solutions").unwrap(), // by pure luck the words wrap perfectly
        NoSolutions => write!(buffer, "No solutions").unwrap(),
        Solved(x1, x2) => {
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
    cont.write(volume, file, buffer.first_line()).unwrap();
    cont.write(volume, file, b"\n").unwrap();
    cont.write(volume, file, buffer.second_line()).unwrap();
    cont.write(volume, file, b"\n\n").unwrap();
}

#[cfg(test)]
fn main() {}

#[cfg(test)]
mod tests {
    use num::{Complex, One, Zero};
    use std::println;

    use crate::{display_buffer::DisplayBuffer, polynomial_2nd_order::Polynomial2ndOrder, solve};

    #[test]
    fn solve_x2_m1() {
        let mut buff = DisplayBuffer::default();
        let poly = Polynomial2ndOrder::<Complex<f32>>::new(
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
