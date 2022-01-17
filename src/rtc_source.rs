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
