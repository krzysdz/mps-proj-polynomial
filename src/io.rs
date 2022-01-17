use cfg_if::cfg_if;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
#[allow(unused_imports)]
use stm32f1xx_hal::{
    device::{GPIOD, GPIOE, RCC},
    gpio::{gpiod, gpioe, Cr, Floating, GpioExt, Input, PullUp, CRH, CRL},
};

// a = PD0-7
// b = PE0-7
// c = PE8-15

cfg_if! {
    if #[cfg(feature = "rust-gpio")] {
        use gpiod::*;
        pub struct PartsD {
            /// Opaque CRL register
            pub crl: Cr<CRL, 'D'>,
            /// Opaque CRH register
            pub crh: Cr<CRH, 'D'>,
            /// Pin
            pub pd0: PD0<Input<PullUp>>,
            pub pd1: PD1<Input<PullUp>>,
            pub pd2: PD2<Input<PullUp>>,
            pub pd3: PD3<Input<PullUp>>,
            pub pd4: PD4<Input<PullUp>>,
            pub pd5: PD5<Input<PullUp>>,
            pub pd6: PD6<Input<PullUp>>,
            pub pd7: PD7<Input<PullUp>>,
            pub pd8: PD8<Input<Floating>>,
            pub pd9: PD9<Input<Floating>>,
            pub pd10: PD10<Input<Floating>>,
            pub pd11: PD11<Input<Floating>>,
            pub pd12: PD12<Input<Floating>>,
            pub pd13: PD13<Input<Floating>>,
            pub pd14: PD14<Input<Floating>>,
            pub pd15: PD15<Input<Floating>>,
        }
        use gpioe::*;
        pub struct PartsE {
            /// Opaque CRL register
            pub crl: Cr<CRL, 'E'>,
            /// Opaque CRH register
            pub crh: Cr<CRH, 'E'>,
            /// Pin
            pub pe0: PE0<Input<PullUp>>,
            pub pe1: PE1<Input<PullUp>>,
            pub pe2: PE2<Input<PullUp>>,
            pub pe3: PE3<Input<PullUp>>,
            pub pe4: PE4<Input<PullUp>>,
            pub pe5: PE5<Input<PullUp>>,
            pub pe6: PE6<Input<PullUp>>,
            pub pe7: PE7<Input<PullUp>>,
            pub pe8: PE8<Input<PullUp>>,
            pub pe9: PE9<Input<PullUp>>,
            pub pe10: PE10<Input<PullUp>>,
            pub pe11: PE11<Input<PullUp>>,
            pub pe12: PE12<Input<PullUp>>,
            pub pe13: PE13<Input<PullUp>>,
            pub pe14: PE14<Input<PullUp>>,
            pub pe15: PE15<Input<PullUp>>,
        }

        #[allow(dead_code)]
        pub struct InputPorts {
            partsd: PartsD,
            partse: PartsE,
        }
    } else {
        pub struct InputPorts {}
    }
}

impl InputPorts {
    /// Create a port reader
    #[allow(unused_variables)]
    pub fn new(gpiod: GPIOD, gpioe: GPIOE) -> Self {
        cfg_if! {
            if #[cfg(not(feature = "rust-gpio"))] {
                // a very "unrusty" way to set up pins by direclty accessing registers in an unsafe way
                cortex_m::interrupt::free(|_| {
                    unsafe {
                        // pull-up PD0-7 (set bits 0-7 of GPIOD_ODR to 1 by setting BSSR)
                        (*RCC::PTR)
                            .apb2enr
                            .modify(|r, w| w.bits(r.bits() | 0b11 << 5));
                        (*RCC::PTR)
                            .apb2rstr
                            .modify(|r, w| w.bits(r.bits() | 0b11 << 5));
                        (*RCC::PTR)
                            .apb2rstr
                            .modify(|r, w| w.bits(r.bits() & !(0b11 << 5)));
                        (*GPIOD::PTR).bsrr.write(|w| w.bits(0x0000_00FF));
                        (*GPIOD::PTR).crl.write(|w| w.bits(0x8888_8888));
                        (*GPIOE::PTR).bsrr.write(|w| w.bits(0x0000_FFFF));
                        (*GPIOE::PTR).crl.write(|w| w.bits(0x8888_8888));
                        (*GPIOE::PTR).crh.write(|w| w.bits(0x8888_8888));
                    };
                });
                InputPorts{}
            } else {
                // the proper way to set up pins is:
                let mut pinsd = gpiod.split();
                let mut pinse = gpioe.split();
                let pd0 = pinsd.pd0.into_pull_up_input(&mut pinsd.crl);
                let pd1 = pinsd.pd1.into_pull_up_input(&mut pinsd.crl);
                let pd2 = pinsd.pd2.into_pull_up_input(&mut pinsd.crl);
                let pd3 = pinsd.pd3.into_pull_up_input(&mut pinsd.crl);
                let pd4 = pinsd.pd4.into_pull_up_input(&mut pinsd.crl);
                let pd5 = pinsd.pd5.into_pull_up_input(&mut pinsd.crl);
                let pd6 = pinsd.pd6.into_pull_up_input(&mut pinsd.crl);
                let pd7 = pinsd.pd7.into_pull_up_input(&mut pinsd.crl);
                let pe0 = pinse.pe0.into_pull_up_input(&mut pinse.crl);
                let pe1 = pinse.pe1.into_pull_up_input(&mut pinse.crl);
                let pe2 = pinse.pe2.into_pull_up_input(&mut pinse.crl);
                let pe3 = pinse.pe3.into_pull_up_input(&mut pinse.crl);
                let pe4 = pinse.pe4.into_pull_up_input(&mut pinse.crl);
                let pe5 = pinse.pe5.into_pull_up_input(&mut pinse.crl);
                let pe6 = pinse.pe6.into_pull_up_input(&mut pinse.crl);
                let pe7 = pinse.pe7.into_pull_up_input(&mut pinse.crl);
                let pe8 = pinse.pe8.into_pull_up_input(&mut pinse.crh);
                let pe9 = pinse.pe9.into_pull_up_input(&mut pinse.crh);
                let pe10 = pinse.pe10.into_pull_up_input(&mut pinse.crh);
                let pe11 = pinse.pe11.into_pull_up_input(&mut pinse.crh);
                let pe12 = pinse.pe12.into_pull_up_input(&mut pinse.crh);
                let pe13 = pinse.pe13.into_pull_up_input(&mut pinse.crh);
                let pe14 = pinse.pe14.into_pull_up_input(&mut pinse.crh);
                let pe15 = pinse.pe15.into_pull_up_input(&mut pinse.crh);
                InputPorts {
                    partsd: PartsD {
                        pd0,
                        pd1,
                        pd2,
                        pd3,
                        pd4,
                        pd5,
                        pd6,
                        pd7,
                        pd8: pinsd.pd8,
                        pd9: pinsd.pd9,
                        pd10: pinsd.pd10,
                        pd11: pinsd.pd11,
                        pd12: pinsd.pd12,
                        pd13: pinsd.pd13,
                        pd14: pinsd.pd14,
                        pd15: pinsd.pd15,
                        crl: pinsd.crl,
                        crh: pinsd.crh,
                    },
                    partse: PartsE {
                        pe0,
                        pe1,
                        pe2,
                        pe3,
                        pe4,
                        pe5,
                        pe6,
                        pe7,
                        pe8,
                        pe9,
                        pe10,
                        pe11,
                        pe12,
                        pe13,
                        pe14,
                        pe15,
                        crl: pinse.crl,
                        crh: pinse.crh,
                    },
                }
            }
        }
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
        let portd = unsafe { (*GPIOD::PTR).idr.read().bits() };
        let porte = unsafe { (*GPIOE::PTR).idr.read().bits() };
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
