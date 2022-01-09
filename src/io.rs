use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use stm32f1xx_hal::device::{GPIOD, GPIOE};

// a = PD0-7
// b = PE0-7
// c = PE8-15

#[allow(dead_code)]
pub struct InputPorts {
    gpiod: GPIOD,
    gpioe: GPIOE,
}

impl InputPorts {
    /// Create a port reader
    pub fn new(gpiod: GPIOD, gpioe: GPIOE) -> Self {
        // a very "unrusty" way to set up pins by direclty accessing registers in an unsafe way
        unsafe {
            // pull-up PD0-7 (set bits 0-7 of GPIOD_ODR to 1 by setting BSSR)
            (*GPIOD::ptr()).bsrr.write(|w| w.bits(0x0000_00FF));
            (*GPIOD::ptr()).crl.write(|w| w.bits(0x8888_8888));
            (*GPIOE::ptr()).bsrr.write(|w| w.bits(0x0000_FFFF));
            (*GPIOE::ptr()).crl.write(|w| w.bits(0x8888_8888));
            (*GPIOE::ptr()).crh.write(|w| w.bits(0x8888_8888));
        };
        // the proper way to set up pins is:
        // let pinsd = gpiod.split();
        // let pinse = gpioe.split();
        // pinsd.pd0.into_pull_up_input(&mut gpiod.crl)
        // pinsd.pd1.into_pull_up_input(&mut gpiod.crl)
        // pinsd.pd2.into_pull_up_input(&mut gpiod.crl)
        // pinsd.pd3.into_pull_up_input(&mut gpiod.crl)
        // pinsd.pd4.into_pull_up_input(&mut gpiod.crl)
        // pinsd.pd5.into_pull_up_input(&mut gpiod.crl)
        // pinsd.pd6.into_pull_up_input(&mut gpiod.crl)
        // pinsd.pd7.into_pull_up_input(&mut gpiod.crl)
        // pinse.pe0.into_pull_up_input(&mut gpioe.crl)
        // pinse.pe1.into_pull_up_input(&mut gpioe.crl)
        // pinse.pe2.into_pull_up_input(&mut gpioe.crl)
        // pinse.pe3.into_pull_up_input(&mut gpioe.crl)
        // pinse.pe4.into_pull_up_input(&mut gpioe.crl)
        // pinse.pe5.into_pull_up_input(&mut gpioe.crl)
        // pinse.pe6.into_pull_up_input(&mut gpioe.crl)
        // pinse.pe7.into_pull_up_input(&mut gpioe.crl)
        // pinse.pe8.into_pull_up_input(&mut gpioe.crh)
        // pinse.pe9.into_pull_up_input(&mut gpioe.crh)
        // pinse.pe10.into_pull_up_input(&mut gpioe.crh)
        // pinse.pe11.into_pull_up_input(&mut gpioe.crh)
        // pinse.pe12.into_pull_up_input(&mut gpioe.crh)
        // pinse.pe13.into_pull_up_input(&mut gpioe.crh)
        // pinse.pe14.into_pull_up_input(&mut gpioe.crh)
        // pinse.pe15.into_pull_up_input(&mut gpioe.crh)
        InputPorts { gpiod, gpioe }
    }
    /// Read bytes from ports. Returns `true` if the fourth byte was read.
    pub fn read_ports(
        &self,
        a: &AtomicU32,
        b: &AtomicU32,
        c: &AtomicU32,
        current_byte: &AtomicU8,
    ) -> bool {
        // This again is against Rust rules, but unlike above, the correct Rust option would be quite complicated
        // as each pin would have to be read separately and shifted
        // According to reference manual:
        // > These bits are read only and can be accessed in Word mode only. They contain the input
        // > value of the corresponding I/O port.
        let portd = unsafe { (*GPIOD::ptr()).idr.read().bits() };
        let porte = unsafe { (*GPIOE::ptr()).idr.read().bits() };
        let a_byte = portd & 0xFF;
        let b_byte = porte & 0xFF;
        let c_byte = (porte >> 8) & 0xFF;
        // let byte = current_byte.compare_exchange(4, 0, Ordering::Relaxed, Ordering::Relaxed);
        let byte =
            current_byte.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |x| Some((x + 1) % 4));
        match byte {
            Ok(0) => {
                a.store(a_byte, Ordering::Relaxed);
                b.store(b_byte, Ordering::Relaxed);
                c.store(c_byte, Ordering::Relaxed);
            }
            Ok(bn) => {
                a.fetch_or(a_byte << (bn * 8), Ordering::Release);
                b.fetch_or(b_byte << (bn * 8), Ordering::Release);
                c.fetch_or(b_byte << (bn * 8), Ordering::Release);
            }
            _ => panic!(), // Some is always returned, so Err should not be possible
        };
        byte == Ok(3)
    }
}

// fn read_ports(a: AtomicU32, b: AtomicU32, c: AtomicU32, current_byte: AtomicU8) {
//     let dp = Peripherals::take().unwrap();
//     let gpioa = dp.GPIOA.split();
// }
