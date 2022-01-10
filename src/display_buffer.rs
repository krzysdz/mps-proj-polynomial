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
