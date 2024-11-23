//! A synchronization primitive for passing the latest value to a task.
use core::cell::Cell;
use core::future::{poll_fn, Future};
use core::task::{Context, Poll, Waker};

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::blocking_mutex::Mutex;

/// Single-slot flagging primitive.
///
/// This is similar to a [`Channel`](crate::channel::Channel) with a buffer size of 1, except
/// "sending" to it (calling [`Flag::flag`]) simply raises a flag instead
/// of waiting for the receiver to pop the previous value.
///
/// It is useful for sending signals between tasks. This is often the case for "state"
/// updates.
///
/// For more advanced use cases, you might want to use [`Channel`](crate::channel::Channel) instead.
///
/// Flags are generally declared as `static`s and then borrowed as required.
///
/// ```
/// use crate::flag::Flag;
/// use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
///
/// enum SomeCommand {
///   On,
///   Off,
/// }
///
/// static SOME_FLAG: Flag<CriticalSectionRawMutex, SomeCommand> = Flag::new();
/// ```
pub struct Flag<M>
where
    M: RawMutex,
{
    state: Mutex<M, Cell<State>>,
}

enum State {
    None,
    Waiting(Waker),
    Flagged,
}

impl<M> Flag<M>
where
    M: RawMutex,
{
    /// Create a new `Flag`.
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(Cell::new(State::None)),
        }
    }
}

impl<M> Default for Flag<M>
where
    M: RawMutex,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<M> Flag<M>
where
    M: RawMutex,
{
    /// Mark this Flag as flagged.
    pub fn flag(&self) {
        self.state.lock(|cell| {
            let state = cell.replace(State::Flagged);
            if let State::Waiting(waker) = state {
                waker.wake();
            }
        })
    }

    /// Reset this `Flag`, if it is flagged.
    pub fn reset(&self) {
        self.state.lock(|cell| cell.set(State::None));
    }

    fn poll_wait_take(&self, cx: &mut Context<'_>) -> Poll<()> {
        self.state.lock(|cell| {
            let state = cell.replace(State::None);
            match state {
                State::None => {
                    cell.set(State::Waiting(cx.waker().clone()));
                    Poll::Pending
                }
                State::Waiting(w) if w.will_wake(cx.waker()) => {
                    cell.set(State::Waiting(w));
                    Poll::Pending
                }
                State::Waiting(w) => {
                    cell.set(State::Waiting(cx.waker().clone()));
                    w.wake();
                    Poll::Pending
                }
                State::Flagged => Poll::Ready(()),
            }
        })
    }

    /// Future that completes when this Flag has been flagged, resetting the flag.
    pub fn wait_take(&self) -> impl Future<Output = ()> + '_ {
        poll_fn(move |cx| self.poll_wait_take(cx))
    }

    /// non-blocking method to try and take the flag.
    // pub fn try_take(&self) -> Option<()> {
    //     self.state.lock(|cell| {
    //         let state = cell.replace(State::None);
    //         match state {
    //             State::Flagged => Some(()),
    //             state => {
    //                 cell.set(state);
    //                 None
    //             }
    //         }
    //     })
    // }

    fn poll_wait_peek(&self, cx: &mut Context<'_>) -> Poll<()> {
        self.state.lock(|cell| {
            let state = cell.replace(State::None);
            match state {
                State::None => {
                    cell.set(State::Waiting(cx.waker().clone()));
                    Poll::Pending
                }
                State::Waiting(w) if w.will_wake(cx.waker()) => {
                    cell.set(State::Waiting(w));
                    Poll::Pending
                }
                State::Waiting(w) => {
                    cell.set(State::Waiting(cx.waker().clone()));
                    w.wake();
                    Poll::Pending
                }
                State::Flagged => {
                    cell.set(State::Flagged);
                    Poll::Ready(())
                }
            }
        })
    }

    pub fn wait_peek(&self) -> impl Future<Output = ()> + '_ {
        poll_fn(move |cx| self.poll_wait_peek(cx))
    }

    /// non-blocking method to check whether this flag has been flagged. This does not clear the flag.  
    pub fn flagged(&self) -> bool {
        self.state.lock(|cell| {
            let state = cell.replace(State::None);

            let res = matches!(state, State::Flagged);

            cell.set(state);

            res
        })
    }
}
