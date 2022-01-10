#![allow(dead_code)]
use core::ops::Mul;

use num::{Complex, Num, Zero};

#[derive(Debug, Clone, Copy)]
pub struct Polynomial2ndOrder<T: Num + Copy> {
    pub a: T,
    pub b: T,
    pub c: T,
}

impl<T: Num + Copy> Polynomial2ndOrder<T> {
    pub fn new(a: T, b: T, c: T) -> Self {
        Polynomial2ndOrder { a, b, c }
    }
}

impl<T: Num + Mul<f32, Output = T> + Copy> Polynomial2ndOrder<T> {
    pub fn derive(self) -> Self {
        Polynomial2ndOrder {
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

impl<T: Num + Copy + PartialOrd + From<f32>> Polynomial2ndOrder<Complex<T>> {
    pub fn reduce(self, known_solution: Complex<T>) -> Self {
        let a = self.a;
        let b = self.b + a * known_solution;
        let r = self.c + b * known_solution;
        assert!(
            r.norm_sqr() < 1e-6.into(),
            "known_solution is not a solution"
        );
        Polynomial2ndOrder {
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
