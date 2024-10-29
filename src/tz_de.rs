use core::cmp::Ordering;
use core::fmt::{Debug, Display, Error, Formatter};

use chrono::{FixedOffset, LocalResult, NaiveDate, NaiveDateTime, NaiveTime, Offset, TimeZone};

#[derive(defmt::Format, Debug, Clone, Copy)]
pub struct TzDe;

/// An Offset that applies for a period of time
///
/// For example, [`::US::Eastern`] is composed of at least two
/// `FixedTimespan`s: `EST` and `EDT`, that are variously in effect.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct FixedTimespan {
    /// The base offset from UTC; this usually doesn't change unless the government changes something
    pub utc_offset: i32,
    /// The additional offset from UTC for this timespan; typically for daylight saving time
    pub dst_offset: i32,
    /// The name of this timezone, for example the difference between `EDT`/`EST`
    pub name: &'static str,
}

impl Offset for FixedTimespan {
    fn fix(&self) -> FixedOffset {
        FixedOffset::east_opt(self.utc_offset + self.dst_offset).unwrap()
    }
}

impl Display for FixedTimespan {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), Error> {
        write!(f, "{}", self.name)
    }
}

impl defmt::Format for FixedTimespan {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", self.name)
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct TzOffset {
    offset: FixedTimespan,
}

impl TzOffset {
    fn new(offset: FixedTimespan) -> Self {
        TzOffset { offset }
    }

    fn map_localresult(result: LocalResult<FixedTimespan>) -> LocalResult<Self> {
        match result {
            LocalResult::None => LocalResult::None,
            LocalResult::Single(s) => LocalResult::Single(TzOffset::new(s)),
            LocalResult::Ambiguous(a, b) => {
                LocalResult::Ambiguous(TzOffset::new(a), TzOffset::new(b))
            }
        }
    }
}

impl Offset for TzOffset {
    fn fix(&self) -> FixedOffset {
        self.offset.fix()
    }
}

impl Display for TzOffset {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), Error> {
        Display::fmt(&self.offset, f)
    }
}

impl Debug for TzOffset {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), Error> {
        Debug::fmt(&self.offset, f)
    }
}

impl defmt::Format for TzOffset {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", &self.offset)
    }
}

/// Represents the span of time that a given rule is valid for.
/// Note that I have made the assumption that all ranges are
/// left-inclusive and right-exclusive - that is to say,
/// if the clocks go forward by 1 hour at 1am, the time 1am
/// does not exist in local time (the clock goes from 00:59:59
/// to 02:00:00). Likewise, if the clocks go back by one hour
/// at 2am, the clock goes from 01:59:59 to 01:00:00. This is
/// an arbitrary choice, and I could not find a source to
/// confirm whether or not this is correct.
struct Span {
    begin: Option<i64>,
    end: Option<i64>,
}

impl Span {
    fn contains(&self, x: i64) -> bool {
        match (self.begin, self.end) {
            (Some(a), Some(b)) if a <= x && x < b => true,
            (Some(a), None) if a <= x => true,
            (None, Some(b)) if b > x => true,
            (None, None) => true,
            _ => false,
        }
    }

    fn cmp(&self, x: i64) -> Ordering {
        match (self.begin, self.end) {
            (Some(a), Some(b)) if a <= x && x < b => Ordering::Equal,
            (Some(a), Some(b)) if a <= x && b <= x => Ordering::Less,
            (Some(_), Some(_)) => Ordering::Greater,
            (Some(a), None) if a <= x => Ordering::Equal,
            (Some(_), None) => Ordering::Greater,
            (None, Some(b)) if b <= x => Ordering::Less,
            (None, Some(_)) => Ordering::Equal,
            (None, None) => Ordering::Equal,
        }
    }
}

#[derive(Copy, Clone)]
pub struct FixedTimespanSet {
    pub first: FixedTimespan,
    pub rest: &'static [(i64, FixedTimespan)],
}

impl FixedTimespanSet {
    fn len(&self) -> usize {
        1 + self.rest.len()
    }

    fn utc_span(&self, index: usize) -> Span {
        debug_assert!(index < self.len());
        Span {
            begin: if index == 0 {
                None
            } else {
                Some(self.rest[index - 1].0)
            },
            end: if index == self.rest.len() {
                None
            } else {
                Some(self.rest[index].0)
            },
        }
    }

    fn local_span(&self, index: usize) -> Span {
        debug_assert!(index < self.len());
        Span {
            begin: if index == 0 {
                None
            } else {
                let span = self.rest[index - 1];
                Some(span.0 + span.1.utc_offset as i64 + span.1.dst_offset as i64)
            },
            end: if index == self.rest.len() {
                None
            } else if index == 0 {
                Some(
                    self.rest[index].0
                        + self.first.utc_offset as i64
                        + self.first.dst_offset as i64,
                )
            } else {
                Some(
                    self.rest[index].0
                        + self.rest[index - 1].1.utc_offset as i64
                        + self.rest[index - 1].1.dst_offset as i64,
                )
            },
        }
    }

    fn get(&self, index: usize) -> FixedTimespan {
        debug_assert!(index < self.len());
        if index == 0 {
            self.first
        } else {
            self.rest[index - 1].1
        }
    }
}

pub trait TimeSpans {
    fn timespans(&self) -> FixedTimespanSet;
}

impl TimeZone for TzDe {
    type Offset = TzOffset;

    fn from_offset(_offset: &Self::Offset) -> Self {
        Self
    }

    #[allow(deprecated)]
    fn offset_from_local_date(&self, local: &NaiveDate) -> LocalResult<Self::Offset> {
        let earliest = self.offset_from_local_datetime(&local.and_time(NaiveTime::MIN));
        let latest = self.offset_from_local_datetime(&local.and_hms_opt(23, 59, 59).unwrap());
        // From the chrono docs:
        //
        // > This type should be considered ambiguous at best, due to the inherent lack of
        // > precision required for the time zone resolution. There are some guarantees on the usage
        // > of `Date<Tz>`:
        // > - If properly constructed via `TimeZone::ymd` and others without an error,
        // >   the corresponding local date should exist for at least a moment.
        // >   (It may still have a gap from the offset changes.)
        //
        // > - The `TimeZone` is free to assign *any* `Offset` to the local date,
        // >   as long as that offset did occur in given day.
        // >   For example, if `2015-03-08T01:59-08:00` is followed by `2015-03-08T03:00-07:00`,
        // >   it may produce either `2015-03-08-08:00` or `2015-03-08-07:00`
        // >   but *not* `2015-03-08+00:00` and others.
        //
        // > - Once constructed as a full `DateTime`,
        // >   `DateTime::date` and other associated methods should return those for the original `Date`.
        // >   For example, if `dt = tz.ymd(y,m,d).hms(h,n,s)` were valid, `dt.date() == tz.ymd(y,m,d)`.
        //
        // > - The date is timezone-agnostic up to one day (i.e. practically always),
        // >   so the local date and UTC date should be equal for most cases
        // >   even though the raw calculation between `NaiveDate` and `Duration` may not.
        //
        // For these reasons we return always a single offset here if we can, rather than being
        // technically correct and returning Ambiguous(_,_) on days when the clock changes. The
        // alternative is painful errors when computing unambiguous times such as
        // `TimeZone.ymd(ambiguous_date).hms(unambiguous_time)`.
        use chrono::LocalResult::*;
        match (earliest, latest) {
            (result @ Single(_), _) => result,
            (_, result @ Single(_)) => result,
            (Ambiguous(offset, _), _) => Single(offset),
            (_, Ambiguous(offset, _)) => Single(offset),
            (None, None) => None,
        }
    }

    // First search for a timespan that the local datetime falls into, then, if it exists,
    // check the two surrounding timespans (if they exist) to see if there is any ambiguity.
    fn offset_from_local_datetime(&self, local: &NaiveDateTime) -> LocalResult<Self::Offset> {
        let timestamp = local.and_utc().timestamp();
        let timespans = self.timespans();
        let index = binary_search(0, timespans.len(), |i| {
            timespans.local_span(i).cmp(timestamp)
        });
        TzOffset::map_localresult(match index {
            Ok(0) if timespans.len() == 1 => LocalResult::Single(timespans.get(0)),
            Ok(0) if timespans.local_span(1).contains(timestamp) => {
                LocalResult::Ambiguous(timespans.get(0), timespans.get(1))
            }
            Ok(0) => LocalResult::Single(timespans.get(0)),
            Ok(i) if timespans.local_span(i - 1).contains(timestamp) => {
                LocalResult::Ambiguous(timespans.get(i - 1), timespans.get(i))
            }
            Ok(i) if i == timespans.len() - 1 => LocalResult::Single(timespans.get(i)),
            Ok(i) if timespans.local_span(i + 1).contains(timestamp) => {
                LocalResult::Ambiguous(timespans.get(i), timespans.get(i + 1))
            }
            Ok(i) => LocalResult::Single(timespans.get(i)),
            Err(_) => LocalResult::None,
        })
    }

    #[allow(deprecated)]
    fn offset_from_utc_date(&self, utc: &NaiveDate) -> Self::Offset {
        // See comment above for why it is OK to just take any arbitrary time in the day
        self.offset_from_utc_datetime(&utc.and_time(NaiveTime::MIN))
    }

    // Binary search for the required timespan. Any i64 is guaranteed to fall within
    // exactly one timespan, no matter what (so the `unwrap` is safe).
    fn offset_from_utc_datetime(&self, utc: &NaiveDateTime) -> Self::Offset {
        let timestamp = utc.and_utc().timestamp();
        let timespans = self.timespans();
        let index =
            binary_search(0, timespans.len(), |i| timespans.utc_span(i).cmp(timestamp)).unwrap();
        TzOffset::new(timespans.get(index))
    }
}

/// An implementation of binary search on indices only
/// that does not require slices to be constructed. Mirrors
/// the semantics of binary_search_by in the standard library.
pub fn binary_search<F>(mut start: usize, mut end: usize, mut f: F) -> Result<usize, usize>
where
    F: FnMut(usize) -> Ordering,
{
    loop {
        let mid = start + (end - start) / 2;
        if mid == end {
            return Err(start);
        }
        match f(mid) {
            Ordering::Less => start = mid + 1,
            Ordering::Greater => end = mid,
            Ordering::Equal => return Ok(mid),
        }
    }
}

impl TimeSpans for TzDe {
    fn timespans(&self) -> FixedTimespanSet {
        const REST: &[(i64, FixedTimespan)] = &[
            (
                -2422054408,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                -1693706400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -1680483600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                -1663455600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -1650150000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                -1632006000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -1618700400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                -938905200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -857257200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                -844556400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -828226800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                -812502000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -796777200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                -781052400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -776563200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 7200,
                    name: "CEMT",
                },
            ),
            (
                -765936000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -761180400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                -748479600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -733273200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                -717631200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -714610800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 7200,
                    name: "CEMT",
                },
            ),
            (
                -710380800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -701910000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                -684975600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -670460400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                -654130800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                -639010800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                323830800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                338950800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                354675600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                370400400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                386125200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                401850000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                417574800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                433299600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                449024400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                465354000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                481078800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                496803600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                512528400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                528253200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                543978000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                559702800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                575427600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                591152400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                606877200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                622602000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                638326800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                654656400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                670381200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                686106000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                701830800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                717555600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                733280400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                749005200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                764730000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                780454800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                796179600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                811904400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                828234000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                846378000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                859683600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                877827600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                891133200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                909277200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                922582800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                941331600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                954032400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                972781200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                985482000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1004230800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1017536400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1035680400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1048986000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1067130000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1080435600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1099184400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1111885200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1130634000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1143334800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1162083600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1174784400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1193533200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1206838800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1224982800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1238288400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1256432400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1269738000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1288486800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1301187600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1319936400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1332637200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1351386000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1364691600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1382835600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1396141200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1414285200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1427590800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1445734800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1459040400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1477789200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1490490000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1509238800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1521939600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1540688400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1553994000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1572138000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1585443600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1603587600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1616893200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1635642000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1648342800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1667091600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1679792400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1698541200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1711846800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1729990800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1743296400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1761440400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1774746000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1792890000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1806195600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1824944400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1837645200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1856394000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1869094800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1887843600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1901149200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1919293200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1932598800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1950742800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1964048400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                1982797200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                1995498000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2014246800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2026947600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2045696400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2058397200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2077146000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2090451600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2108595600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2121901200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2140045200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2153350800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2172099600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2184800400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2203549200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2216250000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2234998800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2248304400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2266448400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2279754000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2297898000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2311203600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2329347600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2342653200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2361402000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2374102800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2392851600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2405552400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2424301200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2437606800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2455750800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2469056400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2487200400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2500506000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2519254800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2531955600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2550704400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2563405200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2582154000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2595459600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2613603600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2626909200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2645053200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2658358800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2676502800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2689808400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2708557200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2721258000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2740006800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2752707600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2771456400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2784762000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2802906000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2816211600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2834355600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2847661200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2866410000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2879110800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2897859600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2910560400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2929309200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2942010000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2960758800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                2974064400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                2992208400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3005514000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3023658000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3036963600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3055712400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3068413200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3087162000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3099862800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3118611600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3131917200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3150061200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3163366800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3181510800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3194816400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3212960400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3226266000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3245014800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3257715600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3276464400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3289165200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3307914000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3321219600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3339363600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3352669200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3370813200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3384118800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3402867600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3415568400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3434317200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3447018000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3465766800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3479072400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3497216400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3510522000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3528666000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3541971600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3560115600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3573421200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3592170000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3604870800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3623619600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3636320400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3655069200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3668374800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3686518800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3699824400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3717968400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3731274000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3750022800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3762723600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3781472400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3794173200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3812922000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3825622800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3844371600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3857677200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3875821200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3889126800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3907270800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3920576400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3939325200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3952026000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                3970774800,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                3983475600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                4002224400,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                4015530000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                4033674000,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                4046979600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                4065123600,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
            (
                4078429200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 3600,
                    name: "CEST",
                },
            ),
            (
                4096573200,
                FixedTimespan {
                    utc_offset: 3600,
                    dst_offset: 0,
                    name: "CET",
                },
            ),
        ];
        FixedTimespanSet {
            first: FixedTimespan {
                utc_offset: 3208,
                dst_offset: 0,
                name: "LMT",
            },
            rest: REST,
        }
    }
}
