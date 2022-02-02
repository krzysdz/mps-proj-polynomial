---
# Metadata
title: Microprocessor Systems Project Report
subtitle: Polynomial solver (2nd degree)
author: Krzysztof Dziembała
date: "2022-01-29"

# Pandoc document settings
standalone: true
lang: en-GB
# Pandoc LaTeX variables
geometry: [a4paper, bindingoffset=0mm, inner=30mm, outer=30mm, top=30mm, bottom=30mm]
documentclass: report
fontsize: 12pt
colorlinks: true
numbersections: true
toc: true
lof: false # List of figures

header-includes:
  # Remove "Chapter N" from the line above chapter name in report class document
  # I could not include a file for some reason, but this works
  - |
    ````{=latex}
	\usepackage{apptools}
    \usepackage{titlesec}
    \titleformat{\chapter}
      {\normalfont\LARGE\bfseries}{\IfAppendix{\appendixname\ }{}\thechapter.}{1em}{}
    \titlespacing*{\chapter}{0pt}{3.5ex plus 1ex minus .2ex}{2.3ex plus .2ex}
    ````
  # Keep footnote numbering
  - |
    ````{=latex}
    \counterwithout{footnote}{chapter}
    ````
  # Wrap long source code lines
  - |
    ````{=latex}
    \usepackage{fvextra}
    \DefineVerbatimEnvironment{Highlighting}{Verbatim}{breaklines,commandchars=\\\{\}}
    ````
  # Set background colour of blockquotes
  - |
    ````{=latex}
    \usepackage{framed}
    \usepackage{xcolor}
    \let\oldquote=\quote
    \let\endoldquote=\endquote
    \colorlet{shadecolor}{CadetBlue!15}
    \renewenvironment{quote}{\begin{shaded*}\begin{oldquote}}{\end{oldquote}\end{shaded*}}
    ````
---

<!-- Allow multiple top-level headers (interpreted as chapters by pandoc) -->
<!-- markdownlint-disable MD025 -->
# Project description

The project is based on the following suggestion from the projects list:

> **4\. Polynomial calculator**
>
> Solving for x such as Port0 = A, Port1 = B, Port2 = C, Port3 = result for x

## Changes in the project assumptions

Operations on integers do not make sense in case of a polynomial solver, so the basic type must be floating point. Because single precision is enough, it was chosen as the base numeric type.\
This decision has some drawbacks which make implementation of the project in the original version impossible.

The following problems occurred during the design process of the solution:

- single precision IEEE 754 floating point requires 32-bits, while ports on AVR are 8-bit wide and 16-bit on STM32 line of microcontrollers
- ATmega328P based Arduino boards which I could use do not have enough GPIO pins
- second degree polynomial equations have two complex solutions and presenting between 0 and 2 complex solutions with 32-bit real and imaginary parts on a single port would require some kind of a protocol to communicate
- variable number of solutions is a problem too, for an equation $ax^2 + bx + c = 0$ the following cases must be considered:
  - $a \ne 0$ - quadratic equation with two complex solutions (may be double)
  - $a = 0 \And b \ne 0$ - linear equation with a single real solution
  - $a = 0 \And b = 0$ - there is no variable for which the equation can be solved, this leaves two possibilities:
    - $c = 0$ - the equality holds true for any value of $x$, because it does not depend on it
    - $c \ne 0$ - the equality is not satisfied regardless of the value of $x$

The problem with number of pins on Arduino boards was avoided by using an [STM32F107](https://www.st.com/en/microcontrollers-microprocessors/stm32f107vc.html) based [STM32Butterfly2](https://kamami.pl/zestawy-uruchomieniowe-stm32/178507-stm32butterfly2.html) board.
Although it is a huge improvement over Arduino boards, it still does not allow a parallel transfer of three 32-bit inputs. Theoretically with this MCU it should be possible to transfer three half precision floating point numbers using separate ports, but the board layout and built-in features make such solution impossible as there are *only* four 8-bin GPIO headers (10-pin, with GND and +3.3V). Finally, the input is transferred in four reads. Every second 8 of the 32 bits of each of the three inputs are read from the respective pins (see [the next section](#solution-details) for details).

The initial solution to the problem with possible outputs was to use an LCD display (16x2 was assumed). Parts of the code were written with a 16x2 display in mind, but, unfortunately, such display was not available during prototyping. Fortunately, the board had a feature which could be used instead - an SD card slot. Although the output couldn't be read immediately, the SD card was a good idea, since huge[^huge] amounts of data can be written to it and stored for a long time.

[^huge]: Compared to 32 characters displayed on a 16x2 display even 512 MB is huge, as it allows hundreds of millions characters to be stored.

# Solution details

Input:

- read byte by byte, LE order
- reads occur in 1 second intervals
- connectors are mapped to the input variables as follows:
  - Con1 (PD0-PD7) - $a$
  - Con2 (PE0-PE7) - $b$
  - Con3 (PE8-PE15) - $c$
- pin with the highest number is the most significant bit of a byte
- helper script is described in [Appendix A](#python-helper)
- pins are in a pull-up mode - logical 1 when floating, 0 shorted to ground

Signalling:

- reads are signalised by toggling the LED D1 (the rightmost diode)
- after four reads the D2 LED turns on and is turned off after the first byte of the next polynomial is read

Output:

- the output is written to the SD card (the card must be formatted using FAT filesystem and use MBR)
- SD card is required for operation, if it is missing or does not work with SPI the core halts and requires reset
- the output is formatted as if it was limited to two sixteen-character lines
- each solution spans two lines, one of which might be blank
- solutions are separated with a blank line
- the file date may be wrong, because the board is not equipped with a backup battery to power RTC when power is disconnected (V\textsubscript{BAT} is connected to +3.3 V line)

The project is written in Rust, because I wanted to practise programming in Rust and I have never used Rust for embedded systems.

![Photo of the STM32Butterfly2 board with a ZL30PRGv2-1 programmer and wires attached to pins used as input shorted to ground on a breadboard.](img/board_with_wires.jpg)

## Maths

Quadratic equations can be solved analytically, but the analytical solution requires calculating square root, which is not implemented in the `core` crate. `f32::sqrt` software implementation is available in the [`libm`](https://github.com/rust-lang/libm) crate, which is used by the `num` crate if the project is built with the `analytical` feature.

By default (without the `analytical` feature) the solver uses the Newton's method to find the first solution. To allow complex solutions, the starting point must have an imaginary component, so the algorithm starts with $x_0 = 1 + 1i$. Stop criterion is defined as $(|x_n - x_{n-1}|) < \epsilon^2$ with $\epsilon^2 = 1 \times 10^{-10}$. To make sure that the computations don't take too long, the loop is executed at most 100 times. Because of such numerical solution, the stop criterion might not be satisfied within the the allowed number of iterations and a solution might not be found.

The Newton's method is defined as:

$$x_{n+1} = x_n - \frac{f(x_n)}{f'(x_n)}$$

Because the project deals exclusively with polynomials the derivative is easy to calculate and is a polynomial too.

After the first solution is found numerically, the polynomial can be divided by $(x-x_i)$. The division is done using Horner's method. If the input was a quadratic equation, the result is a linear equation in form of $ax + b = 0$ and the second zero of the polynomial can be calculated as $\frac{-b}{a}$. In case of a linear equation $a$ is 0 and such division is illegal, so the algorithm must not try to find a second non-existent solution of a linear equation.

## Overview

### Initialisation

The first step after starting the code is initialisation. During this stage:

- peripheral clocks are initialised,
- RTC is configured with the on-chip oscillator, if the feature `set-date` is enabled the RTC timestamp is set to the date and time specified in the code
- pins are set to appropriate modes
  - LED pins (PB0 and PB1) are configured as push-pull outputs with high initial state (turned off)
  - SPI is configured in SPI1 configuration with 1 MHz clock
  - Pins used for $a$, $b$ and $c$ parameters are initialised by writing to appropriate registers - see [details](#register-level-gpio-access)
- file `MPS_RES.txt` from the first volume is opened in append mode
- SysTick is configured to reload every second

### Main loop

Main loop deals with the most important part of the project - actually solving the polynomial equations.

During every loop execution the code checks if the input data is ready. If it is, the polynomial roots are found as described in the [Maths](#maths) section. The results are then written as a string to a buffer which has two 16-character lines. The buffer is then copied with lines separated by a newline character to another buffer and two '\\n' characters are appended. The buffer with newline characters is then written to the SD card. After the content of the loop has been executed (or skipped if the data was not ready), the [`wfi`](https://developer.arm.com/documentation/dui0552/a/the-cortex-m3-instruction-set/miscellaneous-instructions/wfi) instruction is executed to put the core into (light) sleep until an interrupt occurs.

### SysTick

Data is read from the input pins every time the SysTick timer reloads. On reload the SysTick exception handler is executed. The handler reads data from the GPIO pins and stores the bytes with an appropriate shift in the target variables. After reading the data, handler sets the LED state as described in [Details](#solution-details).

**Note:** One second is too short interval to manually rewire the input. To increase the available time, a breakpoint in the handler can be set, or the interval can be changed to *n* second by changing the line:

```rust
syst.set_reload(clocks.sysclk().0 * n); // n seconds (core clock is 8 MHz)
```

### Sharing data between main and interrupt

Sharing the data such as the inputs and pins (pins are treated as data) in Rust is more tricky than in C, because of Rust's ownership rules.\
This problem is solved in the project in the three following ways:

#### Atomic variables

*Simple* data, namely integer types (including `bool`, which really is `u8`), can be shared using static atomic variables. This assures that no modification can occur when another thread (in this case exception occurring during read) is reading/modifying the data. Atomic variables may also enforce memory ordering, but [`Relaxed`](https://en.cppreference.com/w/cpp/atomic/memory_order#Relaxed_ordering) is used, since in this case it probably does not matter.

This method is used for the `DATA_READY` flag, current byte number and the values collected so far (stored as u32, later reinterpreted as f32).

#### `Mutex` guarded `Option`

This method is used for one way moving of pins, but appropriate for sharing any data. Unlike atomic variables, it does not allow specifying memory ordering. The type of the used `static` variable can be *decomposed* as:

1. [`cortex_m::Mutex`](https://docs.rs/cortex-m/latest/cortex_m/interrupt/struct.Mutex.html) - a re-exported [`bare_metal::Mutex`](https://docs.rs/bare-metal/latest/bare_metal/struct.Mutex.html), allows exclusive access to its content for the duration of a [`CriticalSection`](https://docs.rs/bare-metal/latest/bare_metal/struct.CriticalSection.html). A `CriticalSection` can be obtained as a parameter of a closure executed by [`cortex_m::interrupt::free`](https://docs.rs/cortex-m/latest/cortex_m/interrupt/fn.free.html). Such mutex guarantees can be assured only on single-core systems, as `CriticalSection`s *protect* only from interrupts.
2. [`RefCell`](https://doc.rust-lang.org/core/cell/struct.RefCell.html) - allows mutable borrows checked at runtime. This is necessary, because access from interrupts cannot be checked at compile time. More details are available in the [Rust Book](https://doc.rust-lang.org/book/ch15-05-interior-mutability.html).
3. [`Option`](https://doc.rust-lang.org/core/option/) - allows moving a value from and into it.
4. The type which is moved between main context and interrupts.

In this project the static variables are created with `None` inside the `Option`. During the initialisation, structures owning pins configured in the main context are moved into the `Option`. The `SysTick` exception handler has static `Option` variables, which are initially `None`. If the `Option` is `None` the handler tries to replace its value inside a critical section with the one stored in the global static variable, which hopefully is `Some`.

#### Unchecked register access (sharing hardware)

This approach is used only for the GPIO pins (`GPIOE` & `PD0-7`, see [Register-level GPIO access](#register-level-gpio-access)). It is impossible[^ptr-mem] to share real data this way, as it applies only to memory mapped hardware. It requires dereferencing pointers, therefore `unsafe` code, and is the most *unrusty* way of obtaining its goal.

Instead of respecting the ownership rules, one may simply grab the relevant register block, dereference its pointer and read/write directly from/to registers or any other memory.

[^ptr-mem]: It is possible to select a constant value and use it as a pointer in multiple places, but this is a very bad idea. One must somehow make sure that the pointer points to a memory which is always unused and in case of libraries the side effects of such operation would make it unusable for any bigger project and exclude the use of allocators. Because registers' addresses can be obtained from peripheral access crates ([PACs](https://docs.rust-embedded.org/book/start/registers.html#using-a-peripheral-access-crate-pac)), it is safer (but **not safe**) than using a *random* number as a pointer to shared memory.

## Register-level GPIO access

This is the *"bare metal"*[^bare-metal] part requested by MSc. Eng. Walichiewicz. Only pins used for $a$, $b$ and $c$ inputs (Port E and PD0-7) are handled this way. It can be divided into two sections - [Initialisation](#pin-initialisation) and [Reading](#reading-pins).

[^bare-metal]: "Bare metal" phrase is avoided throughout this report in favour of more precise descriptions, as, especially in the world of embedded Rust, "bare metal" usually means no operating system.

### Pin initialisation

If the `rust-gpio` feature is not enabled, pins are set up by writing appropriate values to registers.

Before GPIO can be used, the ports must be enabled. First, `1` must be written to bits 5 (`IOPDEN`) and 6 (`IOPEEN`) of the APB2 peripheral clock enable register (`RCC_APB2ENR`) to enable peripheral clocks. If peripheral clock is disabled, peripheral registers are read as `0x0`.

Next the appropriate peripherals must be reset, because otherwise the port configuration may be locked. Reset is performed by writing `1` and then `0` to bits 5 (`IOPDRST`) and 6 (`IOPERST`) of the APB2 peripheral reset register (`RCC_APB2RSTR`).

After the ports are enabled and reset, pins can be configured as desired. To configure the pins in pull-up input mode, the port output data registers (`GPIOx_ODR`, `x` = `D`/`E`) must contain `1` for each of the input pins and port configuration registers low (`GPIOx_CRL`) and high (`GPIOx_CRH`) must have `CNFy[1:0]` set to `0b10` (input with pull-up / pull-down) and `MODEy[1:0]` set to `0b00` (input mode). Combined 4 bits of the configuration registers for each pin are `0b1000 = 0x8`.\
The `GPIOx_ODR` registers cannot be modified directly and can be set through the port bit set/reset registers (`GPIOx_BSRR`). The lower 16 bits of these registers set respective values in the `GPIOx_ODR` registers and the higher 16 bits reset those bits in `GPIOx_ODR`.

All things combined, the initialisation in code is:

```rust
cortex_m::interrupt::free(|_| {
    unsafe {
        // enable peripheral clocks for ports D and E
        (*RCC::PTR)
            .apb2enr
            .modify(|r, w| w.bits(r.bits() | 0b11 << 5));
        // reset ports D and E
        (*RCC::PTR)
            .apb2rstr
            .modify(|r, w| w.bits(r.bits() | 0b11 << 5));
        (*RCC::PTR)
            .apb2rstr
            .modify(|r, w| w.bits(r.bits() & !(0b11 << 5)));
        // Using the GPIOD_BSRR register set 1 on bits 0-7 of GPIOD_ODR
        (*GPIOD::PTR).bsrr.write(|w| w.bits(0x0000_00FF));
        // Set pins 0-7 of port D to push-pull input mode
        (*GPIOD::PTR).crl.write(|w| w.bits(0x8888_8888));
        // Set bits 0-15 of GPIOE_ODR through GPIOE_BSRR
        (*GPIOE::PTR).bsrr.write(|w| w.bits(0x0000_FFFF));
        // Set pins 0-7 of port E to push-pull input mode
        (*GPIOE::PTR).crl.write(|w| w.bits(0x8888_8888));
        // Set pins 8-15 of port E to push-pull input mode
        (*GPIOE::PTR).crh.write(|w| w.bits(0x8888_8888));
    };
});
```

It is also possible to use `core::ptr::read_volatile` and `core::ptr::write_volatile` to read and write registers. In such case one would write

```rust
core::ptr::write_volatile((*GPIOE::PTR).crh.as_ptr(), 0x8888_8888);
// instead of
(*GPIOE::PTR).crh.write(|w| w.bits(0x8888_8888));
```

![`GPIOx_CRL` register layout and description from the [Reference manual](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf). `GPIOx_CRL` registers have the same layout, but apply to pins 8-15 instead of 0-7.](./img/CRL.png)

![`GPIOx_BSRR` register layout and description from the [Reference manual](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).](./img/BSRR.png)

### Reading pins

Reading the value of pins configured in input mode is very easy. STM32F10xxx microcontrollers have port input data registers `GPIOx_IDR`. The `GPIOx_IDR` registers can be accessed in Rust under the `idr` field of respective data register blocks. The registers are 32-bit, the most significant 16 bits are 0. For example, reading the whole Port E looks like this:

```rust
let porte = unsafe { (*GPIOE::PTR).idr.read().bits() };
```

# Code

The code is based on the [`cortex-m-quickstart` template](https://github.com/rust-embedded/cortex-m-quickstart) and contains multiple (unnecessary) comments with examples for other Cortex-M boards/MCUs.
Files which are not strictly necessary for the project to build (debugging configuration, etc.) are not included in the report.

Full project can be found on GitHub [krzysdz/mps-proj-polynomial](https://github.com/krzysdz/mps-proj-polynomial). The code presented in this report is from commit [c0360807af](https://github.com/krzysdz/mps-proj-polynomial/tree/c0360807af33fb1fa73f96a6309f3f4d443bf74b).

## `.cargo/config.toml`

```toml
[target.thumbv7m-none-eabi]
# uncomment this to make `cargo run` execute programs on QEMU
# runner = "qemu-system-arm -cpu cortex-m3 -machine lm3s6965evb -nographic -semihosting-config enable=on,target=native -kernel"

[target.'cfg(all(target_arch = "arm", target_os = "none"))']
# uncomment ONE of these three option to make `cargo run` start a GDB session
# which option to pick depends on your system
# runner = "arm-none-eabi-gdb -q -x openocd.gdb"
# runner = "gdb-multiarch -q -x openocd.gdb"
# runner = "gdb -q -x openocd.gdb"

rustflags = [
  # This is needed if your flash or ram addresses are not aligned to 0x10000 in memory.x
  # See https://github.com/rust-embedded/cortex-m-quickstart/pull/95
  "-C", "link-arg=--nmagic",

  # LLD (shipped with the Rust toolchain) is used as the default linker
  "-C", "link-arg=-Tlink.x",

  # if you run into problems with LLD switch to the GNU linker by commenting out
  # this line
  # "-C", "linker=arm-none-eabi-ld",

  # if you need to link to pre-compiled C libraries provided by a C toolchain
  # use GCC as the linker by commenting out both lines above and then
  # uncommenting the three lines below
  # "-C", "linker=arm-none-eabi-gcc",
  # "-C", "link-arg=-Wl,-Tlink.x",
  # "-C", "link-arg=-nostartfiles",
]

[build]
# Pick ONE of these compilation targets
# target = "thumbv6m-none-eabi"        # Cortex-M0 and Cortex-M0+
target = "thumbv7m-none-eabi"        # Cortex-M3
# target = "thumbv7em-none-eabi"       # Cortex-M4 and Cortex-M7 (no FPU)
# target = "thumbv7em-none-eabihf"     # Cortex-M4F and Cortex-M7F (with FPU)
# target = "thumbv8m.base-none-eabi"   # Cortex-M23
# target = "thumbv8m.main-none-eabi"   # Cortex-M33 (no FPU)
# target = "thumbv8m.main-none-eabihf" # Cortex-M33 (with FPU)
```

## `src/display_buffer.rs`

```rs
#![allow(dead_code)]
use core::fmt::Write;

#[derive(Debug, Default)]
pub struct DisplayBuffer {
    buff: [u8; 32],
    offset: usize,
}

// Modified https://stackoverflow.com/a/39491059
impl Write for DisplayBuffer {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        assert!(s.is_ascii());
        let bytes = s.as_bytes();
        let remainder = &mut self.buff[self.offset..];
        if remainder.len() < bytes.len() {
            return Err(core::fmt::Error);
        }
        let remainder = &mut remainder[..bytes.len()];
        remainder.copy_from_slice(bytes);
        self.offset += bytes.len();
        Ok(())
    }
}

impl DisplayBuffer {
    pub fn new(buff: [u8; 32], offset: usize) -> Self {
        assert!(offset < 32);
        DisplayBuffer { buff, offset }
    }

    pub fn set_cursor(&mut self, pos: usize) {
        assert!(pos < 32);
        self.offset = pos;
    }

    fn get_line(&self, lineno: usize) -> &[u8] {
        let startpos = 16 * lineno;
        let buff = &self.buff[startpos..];
        let pos = buff.iter().position(|&c| c == 0);
        let pos = match pos {
            Some(p) => {
                if p > 16 {
                    16
                } else {
                    p
                }
            }
            None => 16,
        };
        &buff[0..pos]
    }

    pub fn first_line(&self) -> &[u8] {
        self.get_line(0)
    }

    pub fn second_line(&self) -> &[u8] {
        self.get_line(1)
    }

    pub fn clear(&mut self) {
        self.buff.fill(0);
        self.offset = 0;
    }
}
```

## `src/io.rs`

```rs
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
                c.fetch_or(c_byte << (bn * 8), Ordering::Release);
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
```

## `src/main.rs`

```rs
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
```

## `src/polynomial_2nd_degree.rs`

```rs
#![allow(dead_code)]
use core::ops::Mul;

use num::{Complex, Num, Zero};

#[derive(Debug, Clone, Copy)]
pub struct Polynomial2ndDegree<T: Num + Copy> {
    pub a: T,
    pub b: T,
    pub c: T,
}

impl<T: Num + Copy> Polynomial2ndDegree<T> {
    pub fn new(a: T, b: T, c: T) -> Self {
        Polynomial2ndDegree { a, b, c }
    }
}

impl<T: Num + Mul<f32, Output = T> + Copy> Polynomial2ndDegree<T> {
    pub fn derive(self) -> Self {
        Polynomial2ndDegree {
            a: T::zero(),
            b: T::mul(self.a, 2.0), /* self.a * 2.0*/
            c: self.b,
        }
    }

    pub fn eval(&self, x: T) -> T {
        self.a * x * x + self.b * x + self.c
    }

    pub fn equation_unsolvable(&self) -> bool {
        self.a.is_zero() && self.b.is_zero() && !self.c.is_zero()
    }

    pub fn infinitely_many_solutions(&self) -> bool {
        self.a.is_zero() && self.b.is_zero() && self.c.is_zero()
    }
}

impl<T: Num + Copy + PartialOrd + From<f32>> Polynomial2ndDegree<Complex<T>> {
    pub fn reduce(self, known_solution: Complex<T>) -> Self {
        let a = self.a;
        let b = self.b + a * known_solution;
        let r = self.c + b * known_solution;
        assert!(
            r.norm_sqr() < 1e-6.into(),
            "known_solution is not a solution"
        );
        Polynomial2ndDegree {
            a: Complex::<T>::zero(),
            b: a,
            c: b,
        }
    }
}

// impl<T> Polynomial2ndOrder<T>
// where
//     T: Num + Signed + Copy + PartialOrd + From<f32>,
// {
//     pub fn reduce(self, known_solution: T) -> Self {
//         let a = self.a;
//         let b = self.b - a * known_solution;
//         let r = self.c - b * known_solution;
//         assert!(abs(r) < 1e-4.into(), "known_solution is not a solution");
//         Polynomial2ndOrder {
//             a: T::zero(),
//             b: a,
//             c: b,
//         }
//     }
// }
```

## `src/rtc_source.rs`

```rs
use chrono::{DateTime, Datelike, TimeZone, Timelike, Utc};
use embedded_sdmmc::{TimeSource, Timestamp};
use stm32f1xx_hal::rtc::Rtc;

pub struct RTCSource<T> {
    rtc: Rtc<T>,
}

impl<T> RTCSource<T> {
    pub fn new(rtc: Rtc<T>) -> Self {
        RTCSource { rtc }
    }
    pub fn set_date<Tz: TimeZone>(&mut self, datetime: DateTime<Tz>) {
        let timestamp: u32 = datetime.timestamp().try_into().unwrap_or(0);
        self.rtc.set_time(timestamp)
    }
}

impl<T> TimeSource for RTCSource<T> {
    fn get_timestamp(&self) -> Timestamp {
        let rtc_time = self.rtc.current_time();
        let datetime = Utc.timestamp(rtc_time as i64, 0);
        let date = datetime.date();
        let year = date.year();
        let time = datetime.time();
        Timestamp {
            year_since_1970: (year - 1970).try_into().unwrap_or(0),
            zero_indexed_month: date.month0() as u8,
            zero_indexed_day: date.day0() as u8,
            hours: time.hour() as u8,
            minutes: time.minute() as u8,
            seconds: time.second() as u8,
        }
    }
}
```

## `build.rs`

```rs
//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");
}
```

## `Cargo.toml`

```toml
[package]
authors = ["krzysdz <krzysdz@users.noreply.github.com>"]
edition = "2021"
readme = "README.md"
name = "mps-proj-polynomial"
version = "0.1.0"

[features]
# analytical solution requires sqrt, which is not available in core, use libm
# https://github.com/rust-lang/rfcs/issues/2505
analytical = ["num/libm"]
rust-gpio = []
set-date = []

[dependencies]
cortex-m = "0.7.4"
cortex-m-rt = "0.7.1"
panic-halt = "0.2.0"
panic-semihosting = { version = "0.5.6", optional = true }
panic-itm = { version = "0.4.2", optional = true }
embedded-sdmmc = "0.3.0"
embedded-hal = "0.2.6"
static_assertions = "1.1.0"
cfg-if = "1.0"

# Uncomment for the panic example.
# panic-itm = "0.4.1"

# Uncomment for the allocator example.
# alloc-cortex-m = "0.4.0"

# Reexported as stm32f1xx_hal::pac
# [dependencies.stm32f1]
# features = ["stm32f107", "rt"]
# version = "0.14.0"

[dependencies.stm32f1xx-hal]
version = "0.8.0"
features = ["rt", "stm32f107"]

[dependencies.num]
version = "0.4"
default-features = false

[dependencies.chrono]
version = "0.4"
default-features = false

# this lets you use `cargo fix`!
[[bin]]
name = "mps-proj-polynomial"
test = false
bench = false

[profile.release]
codegen-units = 1 # better optimizations
debug = true # symbols are nice and they don't increase the size on Flash
lto = true # better optimizations
```

## `memory.x`

```x
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* TODO Adjust these memory regions to match your device memory layout */
  /* These values correspond to the LM3S6965, one of the few devices QEMU can emulate */
  FLASH : ORIGIN = 0x08000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* You may want to use this variable to locate the call stack and static
   variables in different memory regions. Below is shown the default value */
/* _stack_start = ORIGIN(RAM) + LENGTH(RAM); */

/* You can use this symbol to customize the location of the .text section */
/* If omitted the .text section will be placed right after the .vector_table
   section */
/* This is required only on microcontrollers that store some configuration right
   after the vector table */
/* _stext = ORIGIN(FLASH) + 0x400; */

/* Example of putting non-initialized variables into custom RAM locations. */
/* This assumes you have defined a region RAM2 above, and in the Rust
   sources added the attribute `#[link_section = ".ram2bss"]` to the data
   you want to place there. */
/* Note that the section will not be zero-initialized by the runtime! */
/* SECTIONS {
     .ram2bss (NOLOAD) : ALIGN(4) {
       *(.ram2bss);
       . = ALIGN(4);
     } > RAM2
   } INSERT AFTER .bss;
*/
```

\appendix

# Python helper

The following python script ([`as_bytes.py`](https://github.com/krzysdz/mps-proj-polynomial/blob/c0360807af33fb1fa73f96a6309f3f4d443bf74b/as_bytes.py) in the repository) can be used to *translate* a floating point numbers into bytes (in the correct order) and displays them in binary format (LTR MSB-LSB).

<!-- markdownlint-disable MD010 -->
```python
import struct
import sys

def as_bytes(num: float):
	packed_float = struct.pack("<f", num)
	# print(packed_float.hex())
	print(f"{num}: bytes to pass (in order):")
	for b in packed_float:
		print(f"{b:08b}")
	a = packed_float[0] | (packed_float[1] << 8) | (packed_float[2] << 16) | (packed_float[3] << 24)
	# print(hex(a))
	packed_uint = struct.pack("=L", a)
	# print(packed_uint.hex())
	unpacked_float = struct.unpack("=f", packed_uint)
	# print(unpacked_float)
	# check if the packed data correctly represents the number converted to a (32-bit) float
	assert unpacked_float[0] == struct.unpack("f", struct.pack("f", num))[0]

if __name__ == "__main__":
	if len(sys.argv) > 1:
		for arg in sys.argv[1:]:
			as_bytes(float(arg))
	else:
		num = float(input("type a float:"))
		as_bytes(num)
```
<!-- markdownlint-enable MD010 -->

# Used libraries

The following libraries were used in this project:

- [`cortex-m`](https://github.com/rust-embedded/cortex-m) - dual licensed under [Apache-2.0](https://github.com/rust-embedded/cortex-m/blob/master/LICENSE-APACHE) and [MIT](https://github.com/rust-embedded/cortex-m/blob/master/LICENSE-MIT)
- [`cortex-m-rt`](https://github.com/rust-embedded/cortex-m) - included in the `cortex-m` repository
- [`panic-halt`](https://github.com/korken89/panic-halt) - dual licensed under [Apache-2.0](https://github.com/korken89/panic-halt/blob/master/LICENSE-APACHE) and [MIT](https://github.com/korken89/panic-halt/blob/master/LICENSE-MIT)
- [`panic-semihosting`](https://github.com/rust-embedded/cortex-m) - included in the `cortex-m` repository
- [`panic-itm`](https://github.com/rust-embedded/cortex-m) - included in the `cortex-m` repository
- [`embedded-sdmmc`](https://github.com/rust-embedded-community/embedded-sdmmc-rs) - dual licensed under [Apache-2.0](https://github.com/rust-embedded-community/embedded-sdmmc-rs/blob/develop/LICENSE-APACHE) and [MIT](https://github.com/rust-embedded-community/embedded-sdmmc-rs/blob/develop/LICENSE-MIT)
- [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) - dual licensed under [Apache-2.0](https://github.com/rust-embedded/embedded-hal/blob/master/LICENSE-APACHE) and [MIT](https://github.com/rust-embedded/embedded-hal/blob/master/LICENSE-MIT)
- [`static_assertions`](https://github.com/nvzqz/static-assertions-rs) - dual licensed under [Apache-2.0](https://github.com/nvzqz/static-assertions-rs/blob/master/LICENSE-APACHE) and [MIT](https://github.com/nvzqz/static-assertions-rs/blob/master/LICENSE-MIT)
- [`cfg-if`](https://github.com/alexcrichton/cfg-if) - dual licensed under [Apache-2.0](https://github.com/alexcrichton/cfg-if/blob/main/LICENSE-APACHE) and [MIT](https://github.com/alexcrichton/cfg-if/blob/main/LICENSE-MIT)
- [`stm32f1xx-hal`](https://github.com/stm32-rs/stm32f1xx-hal) - dual licensed under [Apache-2.0](https://github.com/stm32-rs/stm32f1xx-hal/blob/master/LICENSE-APACHE) and [MIT](https://github.com/stm32-rs/stm32f1xx-hal/blob/master/LICENSE-MIT)
- [`num`](https://github.com/rust-num/num) - dual licensed under [Apache-2.0](https://github.com/rust-num/num/blob/master/LICENSE-APACHE) and [MIT](https://github.com/rust-num/num/blob/master/LICENSE-MIT)
- [`chrono`](https://github.com/chronotope/chrono) - dual [licensed](https://github.com/chronotope/chrono/blob/main/LICENSE.txt) under Apache-2.0 and MIT
