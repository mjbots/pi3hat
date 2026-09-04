// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! An implementation of the `moteus` crate transport layer for the
//! pi3hat, structured like the Python `moteus_pi3hat` package.
//!
//! Like the C++ and Python transports, all pi3hat operations run in a
//! dedicated worker thread that can optionally be pinned to a CPU and
//! given realtime priority via [`Pi3HatMoteusOptions::cpu`].
//!
//! The easiest way to use this module is to call [`register()`] once
//! at startup, which is the Rust equivalent of linking
//! `lib/cpp/examples/pi3hat_moteus_transport_register.cc` into a C++
//! application.  After that, the `moteus` crate auto-detection will
//! find an attached pi3hat, and `--force-transport pi3hat` style
//! options will work.  With the `tokio` feature enabled, this
//! registers both the blocking and the async transport factories.
//!
//! Mirroring the Python `Pi3HatDevice`/`Pi3HatChildDevice` design,
//! the pi3hat is exposed as a parent `TransportDevice` representing
//! the shared SPI link (with no bus of its own), plus one child
//! device per CAN bus.  The moteus `Router` routes all traffic
//! through the parent with `Request::child_device` naming the bus, so
//! a single router cycle spanning several buses is performed as one
//! pi3hat cycle with all buses operating concurrently.  Received
//! frames are attributed to the child device (bus) they arrived on
//! via `CanFdFrame::channel`.

use std::collections::{HashMap, VecDeque};
use std::sync::mpsc;
use std::sync::{Arc, Mutex, Weak};
use std::thread::JoinHandle;
use std::time::Duration;

use moteus::transport::args::{ArgSpec, ArgType};
use moteus::transport::device::{TransportDevice, TransportDeviceInfo};
use moteus::transport::factory::{TransportFactory, TransportOptions};
use moteus::transport::transaction::Request;
use moteus::CanFdFrame;

#[cfg(feature = "tokio")]
use moteus::transport::async_factory::{AsyncTransportFactory, AsyncTransportOptions};
#[cfg(feature = "tokio")]
use moteus::transport::async_transport::BoxFuture;
#[cfg(feature = "tokio")]
use moteus::transport::device::AsyncTransportDevice;

use crate::pi3hat::{
    pi3hat_present, Attitude, CanFrame, Configuration, DeviceInfo, Input, Output, Pi3Hat,
};
use crate::realtime::configure_realtime;

/// Where the SPI worker thread runs; see [`Pi3HatMoteusOptions::cpu`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CpuAffinity {
    /// Pin to the highest isolated CPU if the system has any
    /// (`/sys/devices/system/cpu/isolated`, i.e. `isolcpus=`), and
    /// switch to realtime priority; otherwise run unpinned at normal
    /// priority.
    #[default]
    Auto,
    /// Pin to this CPU and switch to realtime priority; requires
    /// root.
    Pin(usize),
    /// Never pin and never change the scheduling class, like the C++
    /// transport's `cpu = -1`.
    Unpinned,
}

/// Options for the pi3hat moteus transport, mirroring the Python
/// `Pi3HatDevice` constructor arguments.
#[derive(Clone, Debug)]
pub struct Pi3HatMoteusOptions {
    /// The pi3hat hardware configuration.
    pub config: Configuration,

    /// Where the SPI worker thread runs.
    ///
    /// [`CpuAffinity::Auto`] (the default) pins the worker to the
    /// highest isolated CPU when the system has any, else leaves it
    /// unpinned.  Pinning the busy SPI worker onto an isolated core
    /// keeps it off the kernel's general-purpose CPUs -- which is
    /// what `isolcpus` is for, and what stops a realtime (e.g.
    /// `chrt`) run from starving the kernel on a single-general-CPU
    /// system.  [`CpuAffinity::Pin`] selects a specific CPU;
    /// [`CpuAffinity::Unpinned`] opts out of both pinning and the
    /// realtime scheduling class entirely.
    pub cpu: CpuAffinity,

    /// A map from servo ID to bus.  Like the Python library's
    /// `servo_bus_map`, this pre-populates the moteus router's
    /// routing table (via `TransportDeviceInfo::known_can_ids`), so
    /// that no on-demand discovery is needed for these IDs.
    pub servo_map: HashMap<u8, u8>,

    /// These configure the timing of each CAN cycle; see
    /// [`crate::Input`] for their meaning.
    pub min_tx_wait_ns: u32,
    pub rx_baseline_wait_ns: u32,
    pub rx_extra_wait_ns: u32,
}

impl Default for Pi3HatMoteusOptions {
    fn default() -> Self {
        Self {
            config: Configuration::default(),
            cpu: CpuAffinity::Auto,
            servo_map: HashMap::new(),
            min_tx_wait_ns: 200000,
            rx_baseline_wait_ns: 1000000,
            rx_extra_wait_ns: 0,
        }
    }
}

/// The per-cycle parameters sent to the worker thread.
#[derive(Clone, Copy, Debug, Default)]
struct CycleParams {
    timeout_ns: u32,
    min_tx_wait_ns: u32,
    rx_baseline_wait_ns: u32,
    rx_extra_wait_ns: u32,
    force_can_check: u32,
    request_attitude: bool,
    request_attitude_detail: bool,
    wait_for_attitude: bool,
}

/// The channel over which the worker returns a cycle's results.
enum ReplySender {
    Sync(mpsc::SyncSender<CycleResponse>),
    #[cfg(feature = "tokio")]
    Async(tokio::sync::oneshot::Sender<CycleResponse>),
}

impl ReplySender {
    /// Send the response, returning it back if the receiver is gone
    /// (e.g. an async caller was cancelled).  The response is boxed in
    /// the error so the common Ok path stays a thin result.
    fn send(self, response: CycleResponse) -> Result<(), Box<CycleResponse>> {
        match self {
            ReplySender::Sync(sender) => sender.send(response).map_err(|e| Box::new(e.0)),
            #[cfg(feature = "tokio")]
            ReplySender::Async(sender) => sender.send(response).map_err(Box::new),
        }
    }
}

struct CycleRequest {
    tx_can: Vec<CanFrame>,
    rx_capacity: usize,
    params: CycleParams,
    reply: ReplySender,
}

/// Retains a cycle response that would otherwise be lost when an
/// async caller is cancelled between the worker's successful send and
/// the receiver being polled.  `close()` first guarantees no send can
/// slip in between the drain and the receiver being dropped.
#[cfg(feature = "tokio")]
struct AsyncReplyGuard {
    reply_rx: Option<tokio::sync::oneshot::Receiver<CycleResponse>>,
    pending_rx: PendingQueue,
}

#[cfg(feature = "tokio")]
impl Drop for AsyncReplyGuard {
    fn drop(&mut self) {
        if let Some(mut reply_rx) = self.reply_rx.take() {
            reply_rx.close();
            if let Ok(response) = reply_rx.try_recv() {
                extend_pending(&self.pending_rx, response.rx_can);
            }
        }
    }
}

/// The result of one raw pi3hat cycle.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct CycleResponse {
    pub rx_can: Vec<CanFrame>,
    pub attitude: Option<Attitude>,
    pub output: Output,
}

/// Frames received but not yet delivered to any caller, shared
/// between the worker thread and the transport devices.  This mirrors
/// the Python parent's receive queue, which the child devices search
/// by bus.
///
/// The queue is bounded: a frame that never matches any request (for
/// example traffic from a non-moteus node sharing a bus) must not
/// accumulate forever, so once [`MAX_PENDING_FRAMES`] is reached the
/// oldest frame is discarded.  Every retained frame is offered to
/// each subsequent transaction, oldest first, so after a timeout or
/// cancellation a stale reply can be delivered ahead of a fresh one;
/// call the device's `flush()` to discard retained frames when exact
/// freshness matters.
type PendingQueue = Arc<Mutex<VecDeque<CanFrame>>>;

/// The most recently created transport, exposed via
/// [`active_transport`].  A `Weak` so that this registry never keeps
/// the worker thread alive on its own.
static ACTIVE_TRANSPORT: Mutex<Weak<Pi3HatMoteusTransport>> = Mutex::new(Weak::new());

/// The transport behind the most recently created set of pi3hat
/// devices, if one is still alive.
///
/// The moteus `Router` does not expose its transport devices, so an
/// application that lets the factory machinery create the transport
/// (the [`register()`] path) has no handle with which to reach
/// pi3hat-specific functionality such as
/// [`Pi3HatMoteusTransport::read_attitude`].  This function provides
/// that handle.  At most one pi3hat transport can exist per process
/// (the driver holds an exclusive lock), so "most recently created"
/// is simply "the one in use".
///
/// ```no_run
/// # fn main() -> Result<(), moteus::Error> {
/// moteus_pi3hat::transport::register();
/// let mut c = moteus::BlockingController::new(1)?;
/// c.set_stop()?;
///
/// let transport = moteus_pi3hat::transport::active_transport().unwrap();
/// let attitude = transport.read_attitude(true, false);
/// # Ok(())
/// # }
/// ```
pub fn active_transport() -> Option<Arc<Pi3HatMoteusTransport>> {
    ACTIVE_TRANSPORT.lock().unwrap().upgrade()
}

/// The bound on [`PendingQueue`]; see there.  Generous relative to
/// the on-device receive queues (tens of frames), small enough that
/// the per-transaction rescan stays cheap.
const MAX_PENDING_FRAMES: usize = 256;

/// Append frames to the pending queue, discarding the oldest entries
/// beyond [`MAX_PENDING_FRAMES`].
fn extend_pending(pending: &PendingQueue, frames: impl IntoIterator<Item = CanFrame>) {
    let mut pending = pending.lock().unwrap();
    for frame in frames {
        if pending.len() >= MAX_PENDING_FRAMES {
            pending.pop_front();
        }
        pending.push_back(frame);
    }
}

/// A shared pool of reusable CAN frame buffers.
///
/// Like the C++ transport's persistent `tx_can_`/`rx_can_` members,
/// this exists so that no heap allocation or deallocation happens on
/// the realtime worker thread in steady state: transport devices take
/// a warmed buffer to build their tx frames, the worker takes one for
/// rx and returns the spent tx buffer, and the devices return the rx
/// buffer once its frames have been dispatched.  Responses handed to
/// external callers (e.g. the public [`Pi3HatMoteusTransport::cycle`])
/// keep their buffer; the pool then refills lazily.
type BufferPool = Arc<Mutex<Vec<Vec<CanFrame>>>>;

/// The most buffers the pool retains: more than the transaction
/// pipeline can have in flight, small enough to bound memory.
const BUFFER_POOL_MAX: usize = 8;

fn pool_take(pool: &BufferPool) -> Vec<CanFrame> {
    pool.lock().unwrap().pop().unwrap_or_default()
}

fn pool_return(pool: &BufferPool, mut buffer: Vec<CanFrame>) {
    buffer.clear();
    let mut pool = pool.lock().unwrap();
    if pool.len() < BUFFER_POOL_MAX && buffer.capacity() > 0 {
        pool.push(buffer);
    }
}

/// A reusable blocking reply channel: the sender end is cloned into
/// each [`CycleRequest`] (a refcount operation, no allocation) and
/// the pair is returned to the pool once the response arrives, so the
/// blocking cycle path allocates nothing per cycle.
type ReplyChannel = (
    mpsc::SyncSender<CycleResponse>,
    mpsc::Receiver<CycleResponse>,
);

/// A shared pool of [`ReplyChannel`]s, capped like [`BufferPool`].
type ReplyPool = Arc<Mutex<Vec<ReplyChannel>>>;

fn reply_pool_take(pool: &ReplyPool) -> ReplyChannel {
    pool.lock()
        .unwrap()
        .pop()
        .unwrap_or_else(|| mpsc::sync_channel(1))
}

fn reply_pool_return(pool: &ReplyPool, channel: ReplyChannel) {
    let mut pool = pool.lock().unwrap();
    if pool.len() < BUFFER_POOL_MAX {
        pool.push(channel);
    }
}

/// Parse a Linux cpulist (e.g. `"1-3"`, `"0,2-4,7"`) into CPU indices.
fn parse_cpu_list(text: &str) -> Vec<usize> {
    let mut cpus = Vec::new();
    for part in text.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        match part.split_once('-') {
            Some((a, b)) => {
                if let (Ok(a), Ok(b)) = (a.trim().parse::<usize>(), b.trim().parse::<usize>()) {
                    cpus.extend(a..=b);
                }
            }
            None => {
                if let Ok(n) = part.parse::<usize>() {
                    cpus.push(n);
                }
            }
        }
    }
    cpus
}

/// The default worker CPU when none is configured: the highest CPU the
/// kernel isolates (`/sys/devices/system/cpu/isolated`), or `None` if
/// the system isolates no CPUs.  See [`Pi3HatMoteusOptions::cpu`].
fn detect_isolated_cpu() -> Option<usize> {
    let text = std::fs::read_to_string("/sys/devices/system/cpu/isolated").ok()?;
    parse_cpu_list(&text).into_iter().max()
}

fn worker_main(
    options: Pi3HatMoteusOptions,
    init_tx: mpsc::SyncSender<crate::Result<DeviceInfo>>,
    requests: mpsc::Receiver<CycleRequest>,
    pending_rx: PendingQueue,
    buffer_pool: BufferPool,
) {
    let cpu = match options.cpu {
        CpuAffinity::Pin(cpu) => Some(cpu),
        CpuAffinity::Auto => detect_isolated_cpu(),
        CpuAffinity::Unpinned => None,
    };
    if let Some(cpu) = cpu {
        if let Err(e) = configure_realtime(cpu) {
            let _ = init_tx.send(Err(e));
            return;
        }
    }

    let mut pi3hat = match Pi3Hat::new(&options.config) {
        Ok(p) => p,
        Err(e) => {
            let _ = init_tx.send(Err(e));
            return;
        }
    };

    let device_info = pi3hat.device_info();
    if init_tx.send(Ok(device_info)).is_err() {
        return;
    }

    while let Ok(request) = requests.recv() {
        let CycleRequest {
            tx_can,
            rx_capacity,
            params,
            reply,
        } = request;

        // Reuse pooled buffers: in steady state this loop performs no
        // heap allocation or deallocation (the C++ transport makes
        // the same guarantee with its persistent members).
        let mut rx_can = pool_take(&buffer_pool);
        rx_can.resize(rx_capacity, CanFrame::default());
        let mut attitude = Attitude::default();

        let output = pi3hat.cycle(Input {
            tx_can: &tx_can,
            timeout_ns: params.timeout_ns,
            min_tx_wait_ns: params.min_tx_wait_ns,
            rx_baseline_wait_ns: params.rx_baseline_wait_ns,
            rx_extra_wait_ns: params.rx_extra_wait_ns,
            request_attitude: params.request_attitude,
            request_attitude_detail: params.request_attitude_detail,
            wait_for_attitude: params.wait_for_attitude,
            force_can_check: params.force_can_check,
            rx_can: &mut rx_can,
            attitude: if params.request_attitude {
                Some(&mut attitude)
            } else {
                None
            },
            ..Input::default()
        });

        pool_return(&buffer_pool, tx_can);
        rx_can.truncate(output.rx_can_size);

        let response = CycleResponse {
            rx_can,
            attitude: if output.attitude_present {
                Some(attitude)
            } else {
                None
            },
            output,
        };

        if let Err(response) = reply.send(response) {
            // The caller went away (e.g. an async caller was
            // cancelled).  Keep the received frames so they are not
            // lost, like the Python parent's receive queue.
            extend_pending(&pending_rx, response.rx_can);
        }
    }
}

/// Owns the pi3hat worker thread.  All pi3hat I/O happens on that
/// thread; this handle is shared between the parent and child
/// transport devices.
pub struct Pi3HatMoteusTransport {
    sender: Option<mpsc::SyncSender<CycleRequest>>,
    thread: Option<JoinHandle<()>>,
    device_info: DeviceInfo,
    options: Pi3HatMoteusOptions,
    /// Frames received in prior cycles which did not match any
    /// request.  The pi3hat bus they arrived on is recorded in each
    /// frame; the transport devices search this queue by bus.
    pending_rx: PendingQueue,
    /// See [`BufferPool`].
    buffer_pool: BufferPool,
    /// See [`ReplyPool`].
    reply_pool: ReplyPool,
}

impl Pi3HatMoteusTransport {
    /// Start the worker thread and initialize the pi3hat.
    pub fn new(options: Pi3HatMoteusOptions) -> crate::Result<Self> {
        let (init_tx, init_rx) = mpsc::sync_channel(1);
        // A bounded channel uses a preallocated ring rather than one
        // heap node per send, keeping the worker's receive side free
        // of deallocations.  The depth just needs to exceed the
        // number of concurrently-cycling callers.
        let (sender, receiver) = mpsc::sync_channel(16);
        let pending_rx: PendingQueue = Arc::new(Mutex::new(VecDeque::new()));
        let buffer_pool: BufferPool = Default::default();
        let reply_pool: ReplyPool = Default::default();

        let thread_options = options.clone();
        let thread_pending = Arc::clone(&pending_rx);
        let thread_pool = Arc::clone(&buffer_pool);
        let thread = std::thread::Builder::new()
            .name("pi3hat".to_string())
            .spawn(move || {
                worker_main(
                    thread_options,
                    init_tx,
                    receiver,
                    thread_pending,
                    thread_pool,
                )
            })
            .map_err(|e| crate::Error::os("could not spawn the pi3hat worker thread", e))?;

        let device_info = match init_rx.recv() {
            Ok(Ok(device_info)) => device_info,
            Ok(Err(e)) => {
                let _ = thread.join();
                return Err(e);
            }
            Err(_) => {
                let _ = thread.join();
                return Err(crate::Error::message("pi3hat worker thread died"));
            }
        };

        let result = Self {
            sender: Some(sender),
            thread: Some(thread),
            device_info,
            options,
            pending_rx,
            buffer_pool,
            reply_pool,
        };

        // Try to clear any stale replies, matching the C++
        // transport's constructor.
        if let Ok(response) = result.cycle_raw(
            Vec::new(),
            5,
            CycleParams {
                force_can_check: result.all_bus_mask(),
                ..result.default_params(Duration::from_millis(0))
            },
        ) {
            pool_return(&result.buffer_pool, response.rx_can);
        }

        Ok(result)
    }

    /// Information about the processors on the attached pi3hat.
    pub fn device_info(&self) -> &DeviceInfo {
        &self.device_info
    }

    pub fn options(&self) -> &Pi3HatMoteusOptions {
        &self.options
    }

    /// The number of CAN buses available.
    fn bus_count(&self) -> u8 {
        if self.options.config.enable_aux {
            5
        } else {
            4
        }
    }

    /// A `force_can_check` mask covering every available bus.
    fn all_bus_mask(&self) -> u32 {
        (1..=self.bus_count() as u32).fold(0, |mask, bus| mask | (1 << bus))
    }

    fn default_params(&self, timeout: Duration) -> CycleParams {
        CycleParams {
            // The device protocol carries timeouts as u32 nanoseconds,
            // so anything longer saturates at ~4.29s.
            timeout_ns: timeout.as_nanos().min(u32::MAX as u128) as u32,
            min_tx_wait_ns: self.options.min_tx_wait_ns,
            rx_baseline_wait_ns: self.options.rx_baseline_wait_ns,
            rx_extra_wait_ns: self.options.rx_extra_wait_ns,
            force_can_check: 0,
            request_attitude: false,
            request_attitude_detail: false,
            wait_for_attitude: false,
        }
    }

    fn send_request(&self, request: CycleRequest) -> crate::Result<()> {
        self.sender
            .as_ref()
            .expect("sender alive until drop")
            .send(request)
            .map_err(|_| crate::Error::message("pi3hat worker thread died"))
    }

    fn cycle_raw(
        &self,
        tx_can: Vec<CanFrame>,
        rx_capacity: usize,
        params: CycleParams,
    ) -> crate::Result<CycleResponse> {
        let (reply_tx, reply_rx) = reply_pool_take(&self.reply_pool);
        if let Err(e) = self.send_request(CycleRequest {
            tx_can,
            rx_capacity,
            params,
            reply: ReplySender::Sync(reply_tx.clone()),
        }) {
            reply_pool_return(&self.reply_pool, (reply_tx, reply_rx));
            return Err(e);
        }

        // Because we keep our own sender end alive (for reuse), the
        // channel can never disconnect; a worker that dies without
        // replying is detected via its join handle instead.
        let response = loop {
            match reply_rx.recv_timeout(Duration::from_millis(100)) {
                Ok(response) => break response,
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    if self.worker_finished() {
                        return Err(crate::Error::message("pi3hat worker thread died"));
                    }
                }
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    return Err(crate::Error::message("pi3hat worker thread died"));
                }
            }
        };
        reply_pool_return(&self.reply_pool, (reply_tx, reply_rx));
        Ok(response)
    }

    fn worker_finished(&self) -> bool {
        self.thread
            .as_ref()
            .map_or(true, |thread| thread.is_finished())
    }

    /// The async equivalent of [`Self::cycle_raw`].
    ///
    /// Unlike the blocking path (which reuses pooled reply channels
    /// and is allocation-free in steady state), each async cycle
    /// allocates one small oneshot channel on the caller side:
    /// tokio's oneshot is single-use by design, and the async
    /// runtime allocates internally regardless.  The worker side is
    /// allocation-free in both cases.
    ///
    /// # Cancel safety
    ///
    /// If the returned future is dropped, the cycle still completes
    /// on the worker thread and any received frames are retained in
    /// the shared pending queue rather than lost.
    #[cfg(feature = "tokio")]
    async fn cycle_raw_async(
        &self,
        tx_can: Vec<CanFrame>,
        rx_capacity: usize,
        params: CycleParams,
    ) -> crate::Result<CycleResponse> {
        let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
        self.send_request(CycleRequest {
            tx_can,
            rx_capacity,
            params,
            reply: ReplySender::Async(reply_tx),
        })?;
        // The guard closes the cancel-safety race: if this future is
        // dropped after the worker's send succeeded but before the
        // response was received here, the response would otherwise be
        // destroyed inside the abandoned channel (the worker-side
        // retention only covers a send that *fails*).
        let mut guard = AsyncReplyGuard {
            reply_rx: Some(reply_rx),
            pending_rx: Arc::clone(&self.pending_rx),
        };
        let result = guard
            .reply_rx
            .as_mut()
            .expect("armed above")
            .await
            .map_err(|_| crate::Error::message("pi3hat worker thread died"));
        // Disarm: the response (or error) is now owned by the caller.
        guard.reply_rx = None;
        result
    }

    /// Send raw CAN frames (with explicit pi3hat bus numbers) and
    /// collect any replies, optionally also sampling the IMU.  This
    /// is the equivalent of the extended `Cycle` overload in the C++
    /// `Pi3HatMoteusTransport`.
    pub fn cycle(
        &self,
        tx_can: Vec<CanFrame>,
        timeout: Duration,
        request_attitude: bool,
    ) -> crate::Result<CycleResponse> {
        let rx_capacity = (tx_can.len() * 2).max(5);
        self.cycle_raw(
            tx_can,
            rx_capacity,
            CycleParams {
                request_attitude,
                ..self.default_params(timeout)
            },
        )
    }

    /// The async equivalent of [`Self::cycle`].
    #[cfg(feature = "tokio")]
    pub async fn cycle_async(
        &self,
        tx_can: Vec<CanFrame>,
        timeout: Duration,
        request_attitude: bool,
    ) -> crate::Result<CycleResponse> {
        let rx_capacity = (tx_can.len() * 2).max(5);
        self.cycle_raw_async(
            tx_can,
            rx_capacity,
            CycleParams {
                request_attitude,
                ..self.default_params(timeout)
            },
        )
        .await
    }

    fn attitude_params(&self, wait: bool, detail: bool) -> CycleParams {
        CycleParams {
            request_attitude: true,
            request_attitude_detail: detail,
            wait_for_attitude: wait,
            min_tx_wait_ns: 0,
            rx_baseline_wait_ns: 0,
            ..self.default_params(Duration::from_millis(0))
        }
    }

    /// Read the current attitude from the IMU.
    pub fn read_attitude(&self, wait: bool, detail: bool) -> crate::Result<Option<Attitude>> {
        let response = self.cycle_raw(Vec::new(), 0, self.attitude_params(wait, detail))?;
        Ok(response.attitude)
    }

    /// The async equivalent of [`Self::read_attitude`].
    #[cfg(feature = "tokio")]
    pub async fn read_attitude_async(
        &self,
        wait: bool,
        detail: bool,
    ) -> crate::Result<Option<Attitude>> {
        let response = self
            .cycle_raw_async(Vec::new(), 0, self.attitude_params(wait, detail))
            .await?;
        Ok(response.attitude)
    }

    fn push_pending(&self, frame: CanFrame) {
        extend_pending(&self.pending_rx, [frame]);
    }

    /// Remove and return the first pending frame, optionally only one
    /// that arrived on the given bus.
    fn pop_pending(&self, bus: Option<u8>) -> Option<CanFrame> {
        let mut pending = self.pending_rx.lock().unwrap();
        match bus {
            None => pending.pop_front(),
            Some(bus) => {
                let position = pending.iter().position(|frame| frame.bus == bus as i32)?;
                pending.remove(position)
            }
        }
    }

    /// Discard pending frames, optionally only those that arrived on
    /// the given bus.
    fn clear_pending(&self, bus: Option<u8>) {
        let mut pending = self.pending_rx.lock().unwrap();
        match bus {
            None => pending.clear(),
            Some(bus) => pending.retain(|frame| frame.bus != bus as i32),
        }
    }
}

impl Drop for Pi3HatMoteusTransport {
    fn drop(&mut self) {
        // Closing the channel causes the worker to exit.
        drop(self.sender.take());
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn to_pi3hat_frame(frame: &CanFdFrame, bus: u8, request: &Request) -> CanFrame {
    CanFrame {
        id: frame.arbitration_id,
        data: frame.data,
        size: frame.size,
        bus: bus as i32,
        expect_reply: request.expected_reply_count > 0,
        expected_reply_size: request.expected_reply_size,
    }
}

/// Convert a received pi3hat frame, attributing it to the child
/// device (router channel) for the bus it arrived on, like the Python
/// parent's `_make_frame_from_pi3hat`.
fn to_moteus_frame(frame: &CanFrame, parent_router_index: usize, bus_count: u8) -> CanFdFrame {
    let mut result = CanFdFrame::new();
    result.arbitration_id = frame.id;
    result.data = frame.data;
    result.size = frame.size;
    if (1..=bus_count as i32).contains(&frame.bus) {
        result.channel = Some(parent_router_index + frame.bus as usize);
    }
    result
}

/// Decide which pi3hat bus to use for a single request.
///
/// Requests routed through the parent carry a `child_device` naming
/// the bus; requests sent directly to a child use that child's own
/// bus.  Like the Python parent's assertion, a request reaching the
/// parent without a child marker is an error.
fn resolve_bus(
    request: &Request,
    own_bus: Option<u8>,
    parent_router_index: usize,
    bus_count: u8,
) -> Result<u8, moteus::Error> {
    if let Some(child) = request.child_device {
        return child
            .checked_sub(parent_router_index)
            .filter(|&offset| (1..=bus_count as usize).contains(&offset))
            .map(|offset| offset as u8)
            .ok_or_else(|| {
                moteus::Error::Protocol(format!(
                    "pi3hat: child device index {} out of range",
                    child
                ))
            });
    }

    own_bus.ok_or_else(|| {
        moteus::Error::Protocol(
            "pi3hat: requests to the parent device must specify a child device".to_string(),
        )
    })
}

/// Dispatch a received frame to the first request whose child device
/// and filter both match, mirroring the Python parent's
/// `filter_wrapper` which requires `request.child_device ==
/// frame.channel`.
///
/// `default_child` is the channel a request without an explicit child
/// marker is bound to: a child device binds its requests to its own
/// bus (so frames from sibling buses picked up in the same SPI cycle
/// are not misattributed), while the parent accepts frames from any
/// bus.
fn dispatch_bus_frame(
    frame: &CanFdFrame,
    requests: &[Request],
    default_child: Option<usize>,
) -> bool {
    for request in requests {
        // A request expecting no replies (a send-only command, which
        // carries FrameFilter::Any) must never swallow the reply of a
        // query batched after it -- the same rule as the moteus
        // crate's own dispatch_frame.
        if request.expected_reply_count == 0 {
            continue;
        }
        let child_matches = match request.child_device.or(default_child) {
            None => true,
            Some(child) => frame.channel == Some(child),
        };
        if child_matches && request.filter.matches(frame) {
            request.responses.push(frame.clone());
            return true;
        }
    }
    false
}

/// One pi3hat moteus `TransportDevice`.
///
/// Following the Python `Pi3HatDevice`/`Pi3HatChildDevice` design,
/// the parent device represents the shared SPI link and has no bus of
/// its own, while each child device represents one CAN bus.  The
/// moteus `Router` sends all traffic through the parent with
/// `Request::child_device` naming the bus, and probes the children
/// individually during discovery.
pub struct Pi3HatDevice {
    transport: Arc<Pi3HatMoteusTransport>,
    /// The pi3hat bus, 1-5, or None for the parent device.
    bus: Option<u8>,
    /// The router index of the parent device.  `child_device` values
    /// and received-frame channels are translated to buses relative
    /// to this.
    parent_router_index: usize,
    bus_count: u8,
    timeout: Duration,
    info: TransportDeviceInfo,
}

impl Pi3HatDevice {
    /// The shared transport this device operates on, e.g. to reach
    /// pi3hat-specific functionality such as
    /// [`Pi3HatMoteusTransport::read_attitude`].
    pub fn transport(&self) -> Arc<Pi3HatMoteusTransport> {
        Arc::clone(&self.transport)
    }

    fn new(
        transport: Arc<Pi3HatMoteusTransport>,
        bus: Option<u8>,
        parent_router_index: usize,
        timeout: Duration,
        known_can_ids: Vec<u8>,
    ) -> Self {
        let bus_count = transport.bus_count();
        let unknown_address_safe = transport.device_info().can_unknown_address_safe;

        let info = match bus {
            None => {
                // The parent: displays as "pi3hat()", like the Python
                // Pi3HatDevice repr.
                let mut info =
                    TransportDeviceInfo::new(parent_router_index, "pi3hat").with_parent_only();
                info.empty_bus_tx_safe = unknown_address_safe;
                info
            }
            Some(bus) => {
                // A child: displays as "pi3hat('JC1')", like the
                // Python Pi3HatChildDevice repr.
                let mut info =
                    TransportDeviceInfo::new(parent_router_index + bus as usize, "pi3hat")
                        .with_detail(format!("'JC{}'", bus))
                        .with_known_can_ids(known_can_ids);
                info.parent_index = Some(parent_router_index);
                // Legacy pi3hats treated JC1 as special and always
                // assumed there was something there.
                info.empty_bus_tx_safe = bus == 1 || unknown_address_safe;
                info
            }
        };

        Self {
            transport,
            bus,
            parent_router_index,
            bus_count,
            timeout,
            info,
        }
    }

    /// The router channel this device's frames are attributed to, or
    /// None for the parent (which accepts frames from any bus).
    fn own_channel(&self) -> Option<usize> {
        self.bus.map(|bus| self.parent_router_index + bus as usize)
    }

    /// The `force_can_check` mask used when looking for unsolicited
    /// frames on this device.  The parent checks all buses since the
    /// router only directs reads to parent devices.
    fn check_mask(&self) -> u32 {
        match self.bus {
            None => self.transport.all_bus_mask(),
            Some(bus) => 1 << bus,
        }
    }

    fn cycle_params(&self, force_can_check: u32) -> CycleParams {
        CycleParams {
            force_can_check,
            ..self.transport.default_params(self.timeout)
        }
    }

    /// Adjust the cycle for a transaction where some single request
    /// expects multiple replies (e.g. a discovery broadcast).  The
    /// pi3hat driver's expected-reply accounting is per tx frame and
    /// can only express one reply each, so it would otherwise stop
    /// reading after the first response; keeping the receive window
    /// open for the full timeout (and sizing the buffer for every
    /// possible reply) lets all responders be heard.  Transactions
    /// where every request expects at most one reply keep the fast
    /// early-exit path.
    fn adjust_for_multi_reply(
        multi_reply: Option<usize>,
        rx_capacity: usize,
        mut params: CycleParams,
    ) -> (usize, CycleParams) {
        if let Some(total) = multi_reply {
            params.rx_extra_wait_ns = params.rx_extra_wait_ns.max(params.timeout_ns);
            (rx_capacity.max(total.saturating_add(1)), params)
        } else {
            (rx_capacity, params)
        }
    }

    fn run_cycle(
        &self,
        tx_can: Vec<CanFrame>,
        force_can_check: u32,
        multi_reply: Option<usize>,
    ) -> Result<CycleResponse, moteus::Error> {
        let (rx_capacity, params) = Self::adjust_for_multi_reply(
            multi_reply,
            (tx_can.len() * 2).max(5),
            self.cycle_params(force_can_check),
        );
        self.transport
            .cycle_raw(tx_can, rx_capacity, params)
            .map_err(|e| moteus::Error::Protocol(format!("pi3hat: {}", e)))
    }

    #[cfg(feature = "tokio")]
    async fn run_cycle_async(
        &self,
        tx_can: Vec<CanFrame>,
        force_can_check: u32,
        multi_reply: Option<usize>,
    ) -> Result<CycleResponse, moteus::Error> {
        let (rx_capacity, params) = Self::adjust_for_multi_reply(
            multi_reply,
            (tx_can.len() * 2).max(5),
            self.cycle_params(force_can_check),
        );
        self.transport
            .cycle_raw_async(tx_can, rx_capacity, params)
            .await
            .map_err(|e| moteus::Error::Protocol(format!("pi3hat: {}", e)))
    }

    fn to_moteus(&self, frame: &CanFrame) -> CanFdFrame {
        to_moteus_frame(frame, self.parent_router_index, self.bus_count)
    }

    /// Build the frames to send, the `force_can_check` mask, and the
    /// multi-reply total for a transaction.  Receive-only requests
    /// cause us to poll the relevant buses even with nothing to send,
    /// mirroring the Python parent's force_can_check handling.  The
    /// multi-reply total is `Some(sum of expected replies)` when any
    /// single request expects more than one reply (e.g. a discovery
    /// broadcast) — see [`Self::adjust_for_multi_reply`].
    fn prepare_transaction(
        &self,
        requests: &[Request],
    ) -> Result<(Vec<CanFrame>, u32, Option<usize>), moteus::Error> {
        let mut tx_can = pool_take(&self.transport.buffer_pool);
        tx_can.reserve(requests.len());
        for request in requests.iter() {
            if let Some(frame) = &request.frame {
                let bus = resolve_bus(request, self.bus, self.parent_router_index, self.bus_count)?;
                tx_can.push(to_pi3hat_frame(frame, bus, request));
            }
        }

        let mut force_can_check = 0;
        for request in requests.iter() {
            if request.frame.is_none() && request.expected_reply_count > 0 {
                force_can_check |= match resolve_bus(
                    request,
                    self.bus,
                    self.parent_router_index,
                    self.bus_count,
                ) {
                    Ok(bus) => 1 << bus,
                    Err(_) => self.check_mask(),
                };
            }
        }

        let multi_reply = if requests.iter().any(|r| r.expected_reply_count > 1) {
            Some(
                requests
                    .iter()
                    .map(|r| r.expected_reply_count as usize)
                    .sum(),
            )
        } else {
            None
        };

        Ok((tx_can, force_can_check, multi_reply))
    }

    /// Dispatch the received frames of a completed cycle to the
    /// requests; unmatched frames are retained in the shared pending
    /// queue.
    fn complete_transaction(&self, response: CycleResponse, requests: &[Request]) {
        self.dispatch_pending(requests);
        for frame in &response.rx_can {
            let moteus_frame = self.to_moteus(frame);
            if !dispatch_bus_frame(&moteus_frame, requests, self.own_channel()) {
                self.transport.push_pending(*frame);
            }
        }
        pool_return(&self.transport.buffer_pool, response.rx_can);
    }

    /// Offer previously-retained frames to the requests, oldest
    /// first.  A frame for this device's bus can be picked up during
    /// a sibling bus's cycle -- each aux CAN controller serves two
    /// buses and a cycle drains whole controller FIFOs -- and parked
    /// in the shared pending queue, so a transaction that only looked
    /// at its own cycle's frames would miss it.  This mirrors the
    /// Python design, where the child devices search the parent's
    /// receive queue.  Unmatched frames stay queued in arrival order.
    fn dispatch_pending(&self, requests: &[Request]) {
        // Take the queued frames out before dispatching:
        // `dispatch_bus_frame` evaluates user-provided `FrameFilter`s,
        // which must not run (or panic) while the shared lock is held
        // -- a poisoned pending mutex would cascade panics into every
        // caller and the worker itself.
        let frames: Vec<CanFrame> = {
            let mut pending = self.transport.pending_rx.lock().unwrap();
            if pending.is_empty() {
                return;
            }
            pending.drain(..).collect()
        };

        let mut unmatched = Vec::new();
        for frame in frames {
            let moteus_frame = self.to_moteus(&frame);
            if !dispatch_bus_frame(&moteus_frame, requests, self.own_channel()) {
                unmatched.push(frame);
            }
        }

        if !unmatched.is_empty() {
            // Restore arrival order ahead of anything a concurrent
            // cycle queued while the lock was released.
            let mut pending = self.transport.pending_rx.lock().unwrap();
            for frame in unmatched.into_iter().rev() {
                pending.push_front(frame);
            }
        }
    }

    /// Retain all received frames of a cycle in the shared pending
    /// queue, recycling the response's buffer.
    fn retain_response(&self, response: CycleResponse) {
        for frame in &response.rx_can {
            self.transport.push_pending(*frame);
        }
        pool_return(&self.transport.buffer_pool, response.rx_can);
    }
}

impl TransportDevice for Pi3HatDevice {
    fn transaction(&mut self, requests: &mut [Request]) -> Result<(), moteus::Error> {
        let (tx_can, force_can_check, multi_reply) = self.prepare_transaction(requests)?;
        let response = self.run_cycle(tx_can, force_can_check, multi_reply)?;
        self.complete_transaction(response, requests);
        Ok(())
    }

    fn write(&mut self, frame: &CanFdFrame) -> Result<(), moteus::Error> {
        let request = Request::new(frame.clone()).with_expected_replies(0);
        let bus = resolve_bus(&request, self.bus, self.parent_router_index, self.bus_count)?;
        let mut tx_can = pool_take(&self.transport.buffer_pool);
        tx_can.push(to_pi3hat_frame(frame, bus, &request));
        let response = self.run_cycle(tx_can, 0, None)?;
        self.retain_response(response);
        Ok(())
    }

    fn read(&mut self) -> Result<Option<CanFdFrame>, moteus::Error> {
        let deadline = std::time::Instant::now() + self.timeout;
        loop {
            if let Some(frame) = self.transport.pop_pending(self.bus) {
                return Ok(Some(self.to_moteus(&frame)));
            }

            let response = self.run_cycle(Vec::new(), self.check_mask(), None)?;
            self.retain_response(response);

            if let Some(frame) = self.transport.pop_pending(self.bus) {
                return Ok(Some(self.to_moteus(&frame)));
            }
            if std::time::Instant::now() >= deadline {
                return Ok(None);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    fn flush(&mut self) -> Result<(), moteus::Error> {
        self.transport.clear_pending(self.bus);
        let response = self.run_cycle(Vec::new(), self.check_mask(), None)?;
        // Frames from other buses picked up in the same cycle are
        // retained for their devices; only our own are discarded.
        self.retain_response(response);
        self.transport.clear_pending(self.bus);
        Ok(())
    }

    fn empty_bus_tx_safe(&self) -> bool {
        self.info.empty_bus_tx_safe
    }

    fn info(&self) -> &TransportDeviceInfo {
        &self.info
    }

    fn set_timeout(&mut self, timeout: Duration) {
        self.timeout = timeout;
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }
}

/// One pi3hat moteus `AsyncTransportDevice`, sharing the same worker
/// thread model as the blocking [`Pi3HatDevice`].
///
/// # Cancel safety
///
/// All futures are cancel safe.  A pi3hat cycle always runs to
/// completion on the worker thread once submitted; if the caller's
/// future is dropped, the worker retains any received frames in the
/// shared pending queue.  No recovery step is required, so
/// `recover()` is the default no-op.
#[cfg(feature = "tokio")]
pub struct AsyncPi3HatDevice {
    inner: Pi3HatDevice,
}

#[cfg(feature = "tokio")]
impl AsyncPi3HatDevice {
    /// Wrap a blocking device for async use.
    pub fn new(inner: Pi3HatDevice) -> Self {
        Self { inner }
    }
}

#[cfg(feature = "tokio")]
impl AsyncTransportDevice for AsyncPi3HatDevice {
    fn transaction<'a>(
        &'a mut self,
        requests: &'a mut [Request],
    ) -> BoxFuture<'a, Result<(), moteus::Error>> {
        Box::pin(async move {
            let (tx_can, force_can_check, multi_reply) =
                self.inner.prepare_transaction(requests)?;
            let response = self
                .inner
                .run_cycle_async(tx_can, force_can_check, multi_reply)
                .await?;
            self.inner.complete_transaction(response, requests);
            Ok(())
        })
    }

    fn write<'a>(&'a mut self, frame: &'a CanFdFrame) -> BoxFuture<'a, Result<(), moteus::Error>> {
        Box::pin(async move {
            let request = Request::new(frame.clone()).with_expected_replies(0);
            let bus = resolve_bus(
                &request,
                self.inner.bus,
                self.inner.parent_router_index,
                self.inner.bus_count,
            )?;
            let mut tx_can = pool_take(&self.inner.transport.buffer_pool);
            tx_can.push(to_pi3hat_frame(frame, bus, &request));
            let response = self.inner.run_cycle_async(tx_can, 0, None).await?;
            self.inner.retain_response(response);
            Ok(())
        })
    }

    fn read(&mut self) -> BoxFuture<'_, Result<Option<CanFdFrame>, moteus::Error>> {
        Box::pin(async move {
            let deadline = std::time::Instant::now() + self.inner.timeout;
            loop {
                if let Some(frame) = self.inner.transport.pop_pending(self.inner.bus) {
                    return Ok(Some(self.inner.to_moteus(&frame)));
                }

                let response = self
                    .inner
                    .run_cycle_async(Vec::new(), self.inner.check_mask(), None)
                    .await?;
                self.inner.retain_response(response);

                if let Some(frame) = self.inner.transport.pop_pending(self.inner.bus) {
                    return Ok(Some(self.inner.to_moteus(&frame)));
                }
                if std::time::Instant::now() >= deadline {
                    return Ok(None);
                }
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
    }

    fn flush(&mut self) -> BoxFuture<'_, Result<(), moteus::Error>> {
        Box::pin(async move {
            self.inner.transport.clear_pending(self.inner.bus);
            let response = self
                .inner
                .run_cycle_async(Vec::new(), self.inner.check_mask(), None)
                .await?;
            self.inner.retain_response(response);
            self.inner.transport.clear_pending(self.inner.bus);
            Ok(())
        })
    }

    fn empty_bus_tx_safe(&self) -> bool {
        self.inner.info.empty_bus_tx_safe
    }

    fn info(&self) -> &TransportDeviceInfo {
        &self.inner.info
    }
}

/// Parse a servo map of the form `1=11,12;2=13,14`, mapping bus 1 to
/// servos 11 and 12, and bus 2 to servos 13 and 14, as accepted by
/// the C++ `--pi3hat-cfg` argument.
///
/// Unlike the C++ parser, out-of-range buses and servo IDs mapped to
/// more than one bus are reported as errors rather than silently
/// misrouting (C++ keeps the first mapping and accepts any bus
/// number).
pub fn parse_servo_map(text: &str) -> Result<HashMap<u8, u8>, String> {
    let mut result = HashMap::new();

    for bus_spec in text.split(';').filter(|s| !s.is_empty()) {
        let (bus_text, servo_text) = bus_spec
            .split_once('=')
            .ok_or_else(|| format!("Invalid bus specifier: {}", bus_spec))?;
        let bus: u8 = bus_text
            .trim()
            .parse()
            .map_err(|_| format!("Invalid bus: {}", bus_text))?;
        if !(1..=5).contains(&bus) {
            return Err(format!("Bus out of range 1-5: {}", bus));
        }
        for servo in servo_text.split(',') {
            let id: u8 = servo
                .trim()
                .parse()
                .map_err(|_| format!("Invalid servo id: {}", servo))?;
            if let Some(previous) = result.insert(id, bus) {
                if previous != bus {
                    return Err(format!(
                        "Servo {} mapped to both bus {} and bus {}",
                        id, previous, bus
                    ));
                }
            }
        }
    }

    Ok(result)
}

/// The servo IDs from a servo map that live on the given bus, in
/// ascending order.
fn bus_known_ids(servo_map: &HashMap<u8, u8>, bus: u8) -> Vec<u8> {
    let mut ids: Vec<u8> = servo_map
        .iter()
        .filter(|(_, &b)| b == bus)
        .map(|(&id, _)| id)
        .collect();
    ids.sort_unstable();
    ids
}

/// Create the moteus `TransportDevice`s for a pi3hat: the parent
/// device first, followed by one child device per CAN bus.
///
/// This is mostly useful for direct construction; most users should
/// instead call [`register()`] and let the moteus factory machinery
/// create the devices.
///
/// The returned devices interpret `Request::child_device` and frame
/// channels *positionally*: they must be installed in the router at
/// exactly positions `parent_router_index` (the parent) through
/// `parent_router_index + bus count` (the children, in bus order),
/// with nothing in between.  The factory path guarantees this by
/// construction.
pub fn create_devices(
    options: Pi3HatMoteusOptions,
    timeout: Duration,
    parent_router_index: usize,
) -> crate::Result<Vec<Pi3HatDevice>> {
    let servo_map = options.servo_map.clone();
    let transport = Arc::new(Pi3HatMoteusTransport::new(options)?);
    *ACTIVE_TRANSPORT.lock().unwrap() = Arc::downgrade(&transport);
    let bus_count = transport.bus_count();

    let mut devices = Vec::with_capacity(1 + bus_count as usize);
    devices.push(Pi3HatDevice::new(
        Arc::clone(&transport),
        None,
        parent_router_index,
        timeout,
        Vec::new(),
    ));
    for bus in 1..=bus_count {
        devices.push(Pi3HatDevice::new(
            Arc::clone(&transport),
            Some(bus),
            parent_router_index,
            timeout,
            bus_known_ids(&servo_map, bus),
        ));
    }
    Ok(devices)
}

/// The transport-specific CLI arguments understood by the pi3hat
/// factories, mirroring the C++ `--pi3hat-*` flags.  Declaring these
/// lets the `moteus` argument machinery (e.g. `add_transport_args`)
/// surface them; they are then extracted into `TransportOptions::extra`
/// and consumed by [`Pi3HatFactory::options_from`].
fn pi3hat_arg_specs() -> Vec<ArgSpec> {
    vec![
        ArgSpec::new(
            "pi3hat-cpu",
            "CPU to pin the realtime pi3hat worker thread to; -1 or 'none' \
             to never pin (default: an isolated CPU if the system has any, \
             else unpinned)",
            ArgType::Integer,
        ),
        ArgSpec::new("pi3hat-spi-hz", "pi3hat SPI speed in Hz", ArgType::Integer),
        ArgSpec::new(
            "pi3hat-cfg",
            "Servo to bus map, e.g. 1=11,12;2=13,14 (may be repeated)",
            ArgType::MultiString,
        ),
        ArgSpec::new(
            "pi3hat-disable-aux",
            "Prevent use of the IMU/JC5 (AUX processor)",
            ArgType::Bool,
        ),
    ]
}

/// The moteus `TransportFactory` for the pi3hat.
///
/// Most users should call [`register()`] instead of using this
/// directly.
#[derive(Debug, Default)]
pub struct Pi3HatFactory;

impl Pi3HatFactory {
    fn options_from(
        &self,
        options: &TransportOptions,
    ) -> Result<Pi3HatMoteusOptions, moteus::Error> {
        let mut result = Pi3HatMoteusOptions::default();

        let extra = &options.extra;
        let first = |key: &str| extra.get(key).and_then(|v| v.first());

        if let Some(value) = first("pi3hat-cpu") {
            result.cpu = match value.trim() {
                "-1" | "none" => CpuAffinity::Unpinned,
                other => CpuAffinity::Pin(other.parse::<usize>().map_err(|_| {
                    moteus::Error::Protocol(format!("invalid pi3hat-cpu: {}", value))
                })?),
            };
        }

        if let Some(value) = first("pi3hat-spi-hz") {
            result.config.spi_speed_hz = value.parse::<i32>().map_err(|_| {
                moteus::Error::Protocol(format!("invalid pi3hat-spi-hz: {}", value))
            })?;
        }

        if let Some(values) = extra.get("pi3hat-cfg") {
            for value in values {
                let map = parse_servo_map(value).map_err(moteus::Error::Protocol)?;
                result.servo_map.extend(map);
            }
        }

        if let Some(value) = first("pi3hat-disable-aux") {
            if value != "false" {
                result.config.enable_aux = false;
            }
        }

        if options.disable_brs {
            for can in result.config.can.iter_mut() {
                can.bitrate_switch = false;
            }
        }

        Ok(result)
    }
}

impl TransportFactory for Pi3HatFactory {
    fn priority(&self) -> u32 {
        // Lower than fdcanusb and socketcan, so that when a pi3hat is
        // present its devices come first in the router.  The
        // parent/child bus routing relies on this ordering.
        5
    }

    fn name(&self) -> &'static str {
        "pi3hat"
    }

    fn arg_specs(&self) -> Vec<ArgSpec> {
        pi3hat_arg_specs()
    }

    fn create(
        &self,
        options: &TransportOptions,
    ) -> Result<Vec<Box<dyn TransportDevice>>, moteus::Error> {
        if !pi3hat_present() {
            return Ok(Vec::new());
        }

        let pi3hat_options = self.options_from(options)?;

        let devices = create_devices(pi3hat_options, options.timeout, 0)
            .map_err(|e| moteus::Error::Protocol(format!("pi3hat: {}", e)))?;

        Ok(devices
            .into_iter()
            .map(|d| Box::new(d) as Box<dyn TransportDevice>)
            .collect())
    }
}

/// The moteus `AsyncTransportFactory` for the pi3hat.
///
/// Most users should call [`register()`] instead of using this
/// directly.
#[cfg(feature = "tokio")]
#[derive(Debug, Default)]
pub struct AsyncPi3HatFactory;

#[cfg(feature = "tokio")]
impl AsyncTransportFactory for AsyncPi3HatFactory {
    fn priority(&self) -> u32 {
        5
    }

    fn name(&self) -> &'static str {
        "pi3hat"
    }

    fn arg_specs(&self) -> Vec<ArgSpec> {
        pi3hat_arg_specs()
    }

    fn create<'a>(
        &'a self,
        options: &'a AsyncTransportOptions,
    ) -> BoxFuture<'a, Result<Vec<Box<dyn AsyncTransportDevice>>, moteus::Error>> {
        Box::pin(async move {
            if !pi3hat_present() {
                return Ok(Vec::new());
            }

            let pi3hat_options = Pi3HatFactory.options_from(options)?;
            let timeout = options.timeout;

            // Device construction starts the worker thread and
            // performs blocking hardware initialization.
            let devices =
                tokio::task::spawn_blocking(move || create_devices(pi3hat_options, timeout, 0))
                    .await
                    .map_err(|e| moteus::Error::Protocol(format!("pi3hat: {}", e)))?
                    .map_err(|e| moteus::Error::Protocol(format!("pi3hat: {}", e)))?;

            Ok(devices
                .into_iter()
                .map(|d| Box::new(AsyncPi3HatDevice::new(d)) as Box<dyn AsyncTransportDevice>)
                .collect())
        })
    }
}

/// Register the pi3hat transport with the moteus crate, so that it
/// participates in transport auto-detection.  With the `tokio`
/// feature enabled, this registers both the blocking and the async
/// factories.
///
/// This is the Rust equivalent of linking
/// `lib/cpp/examples/pi3hat_moteus_transport_register.cc` into a C++
/// application.  It is safe to call more than once.
pub fn register() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        moteus::transport::factory::register(Box::new(Pi3HatFactory));
        #[cfg(feature = "tokio")]
        moteus::transport::async_factory::register_async(Arc::new(AsyncPi3HatFactory));
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use moteus::transport::transaction::FrameFilter;

    #[test]
    fn test_parse_cpu_list() {
        assert_eq!(parse_cpu_list("1-3"), vec![1, 2, 3]);
        assert_eq!(parse_cpu_list("0,2-4,7"), vec![0, 2, 3, 4, 7]);
        assert_eq!(parse_cpu_list("2"), vec![2]);
        assert_eq!(parse_cpu_list("1-3\n"), vec![1, 2, 3]);
        assert!(parse_cpu_list("").is_empty());
        assert!(parse_cpu_list("\n").is_empty());
        // The default worker CPU is the highest isolated one.
        assert_eq!(parse_cpu_list("1-3").into_iter().max(), Some(3));
    }

    #[test]
    fn test_parse_servo_map() {
        let map = parse_servo_map("1=11,12;2=13,14").unwrap();
        assert_eq!(map.len(), 4);
        assert_eq!(map[&11], 1);
        assert_eq!(map[&12], 1);
        assert_eq!(map[&13], 2);
        assert_eq!(map[&14], 2);

        assert!(parse_servo_map("nonsense").is_err());
        assert!(parse_servo_map("1=x").is_err());
        assert!(parse_servo_map("").unwrap().is_empty());
        // Out-of-range buses and conflicting duplicate ids are
        // rejected; a repeated identical mapping is fine.
        assert!(parse_servo_map("9=5").is_err());
        assert!(parse_servo_map("0=5").is_err());
        assert!(parse_servo_map("1=7;2=7").is_err());
        assert!(parse_servo_map("1=7;1=7").is_ok());
    }

    #[test]
    fn test_bus_known_ids() {
        let map = parse_servo_map("1=12,11;2=13").unwrap();
        assert_eq!(bus_known_ids(&map, 1), vec![11, 12]);
        assert_eq!(bus_known_ids(&map, 2), vec![13]);
        assert!(bus_known_ids(&map, 3).is_empty());
    }

    #[test]
    fn test_frame_conversion() {
        let mut frame = CanFdFrame::new();
        frame.arbitration_id = 0x8001;
        frame.data[0] = 0xaa;
        frame.size = 1;

        let request = Request::new(frame.clone());
        let pi3hat_frame = to_pi3hat_frame(&frame, 3, &request);
        assert_eq!(pi3hat_frame.id, 0x8001);
        assert_eq!(pi3hat_frame.bus, 3);
        assert_eq!(pi3hat_frame.size, 1);
        assert!(pi3hat_frame.expect_reply);

        // With the parent at router index 0, a frame received on bus
        // 3 is attributed to channel 3 (the JC3 child device).
        let roundtrip = to_moteus_frame(&pi3hat_frame, 0, 5);
        assert_eq!(roundtrip.arbitration_id, 0x8001);
        assert_eq!(roundtrip.size, 1);
        assert_eq!(roundtrip.data[0], 0xaa);
        assert_eq!(roundtrip.channel, Some(3));
    }

    #[test]
    fn test_resolve_bus() {
        let frame = CanFdFrame::new();

        // A request routed through the parent uses the child marker.
        let mut request = Request::new(frame.clone());
        request.child_device = Some(2);
        assert_eq!(resolve_bus(&request, None, 0, 5).unwrap(), 2);
        // With the parent at a non-zero router index.
        request.child_device = Some(12);
        assert_eq!(resolve_bus(&request, None, 10, 5).unwrap(), 2);

        // Out of range child markers are an error.
        request.child_device = Some(6);
        assert!(resolve_bus(&request, None, 0, 5).is_err());
        request.child_device = Some(0);
        assert!(resolve_bus(&request, None, 0, 5).is_err());

        // A request sent directly to a child uses its own bus.
        let request = Request::new(frame.clone());
        assert_eq!(resolve_bus(&request, Some(4), 0, 5).unwrap(), 4);

        // The parent requires a child marker, like the Python
        // parent's assertion.
        assert!(resolve_bus(&request, None, 0, 5).is_err());
    }

    #[test]
    fn test_buffer_pool_round_trip() {
        let pool: BufferPool = Default::default();

        // An empty pool hands out a fresh (empty) buffer.
        let mut buffer = pool_take(&pool);
        assert!(buffer.is_empty());
        buffer.resize(32, CanFrame::default());
        let capacity = buffer.capacity();

        // A returned buffer comes back cleared with its capacity
        // intact, so the next take allocates nothing.
        pool_return(&pool, buffer);
        let buffer = pool_take(&pool);
        assert!(buffer.is_empty());
        assert_eq!(buffer.capacity(), capacity);

        // The pool never retains more than BUFFER_POOL_MAX buffers,
        // nor zero-capacity ones.
        for _ in 0..(BUFFER_POOL_MAX + 4) {
            pool_return(&pool, vec![CanFrame::default()]);
        }
        pool_return(&pool, Vec::new());
        assert_eq!(pool.lock().unwrap().len(), BUFFER_POOL_MAX);
        assert!(pool.lock().unwrap().iter().all(|b| b.capacity() > 0));
    }

    #[test]
    fn test_reply_pool_reuse() {
        let response = || CycleResponse {
            rx_can: Vec::new(),
            attitude: None,
            output: Default::default(),
        };

        let pool: ReplyPool = Default::default();
        let (reply_tx, reply_rx) = reply_pool_take(&pool);
        reply_tx.clone().send(response()).unwrap();
        assert!(reply_rx.try_recv().is_ok());
        reply_pool_return(&pool, (reply_tx, reply_rx));
        assert_eq!(pool.lock().unwrap().len(), 1);

        // The recycled channel comes back empty and usable.
        let (reply_tx, reply_rx) = reply_pool_take(&pool);
        assert!(pool.lock().unwrap().is_empty());
        assert!(reply_rx.try_recv().is_err());
        reply_tx.clone().send(response()).unwrap();
        assert!(reply_rx.try_recv().is_ok());
    }

    #[test]
    fn test_extend_pending_cap() {
        let pending: PendingQueue = Default::default();
        extend_pending(
            &pending,
            (0..MAX_PENDING_FRAMES + 10).map(|i| CanFrame {
                id: i as u32,
                ..CanFrame::default()
            }),
        );
        let queue = pending.lock().unwrap();
        assert_eq!(queue.len(), MAX_PENDING_FRAMES);
        // The oldest frames were discarded; the newest survive.
        assert_eq!(queue.front().unwrap().id, 10);
        assert_eq!(queue.back().unwrap().id, (MAX_PENDING_FRAMES + 9) as u32);
    }

    #[cfg(feature = "tokio")]
    #[test]
    fn test_async_reply_guard_retains_unclaimed_response() {
        // A response sent into the channel but never received (the
        // caller was cancelled after the worker's send succeeded) is
        // retained in the pending queue when the guard drops.
        let pending: PendingQueue = Default::default();
        let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
        let guard = AsyncReplyGuard {
            reply_rx: Some(reply_rx),
            pending_rx: Arc::clone(&pending),
        };
        reply_tx
            .send(CycleResponse {
                rx_can: vec![CanFrame {
                    id: 42,
                    ..CanFrame::default()
                }],
                attitude: None,
                output: Default::default(),
            })
            .map_err(|_| ())
            .expect("receiver alive");
        drop(guard);
        assert_eq!(pending.lock().unwrap().len(), 1);
        assert_eq!(pending.lock().unwrap()[0].id, 42);
    }

    #[test]
    fn test_dispatch_bus_frame_respects_child() {
        // Two receive requests, one per child device.
        let mut request1 = Request::receive_only(FrameFilter::Any);
        request1.child_device = Some(1);
        let mut request2 = Request::receive_only(FrameFilter::Any);
        request2.child_device = Some(2);
        let requests = vec![request1, request2];

        // A frame attributed to channel 2 must go to the second
        // request even though the first request's filter matches.
        let mut frame = CanFdFrame::new();
        frame.channel = Some(2);
        assert!(dispatch_bus_frame(&frame, &requests, None));
        assert_eq!(requests[0].responses.len(), 0);
        assert_eq!(requests[1].responses.len(), 1);

        // A frame from a bus with no interested request is not
        // dispatched.
        let mut frame = CanFdFrame::new();
        frame.channel = Some(3);
        assert!(!dispatch_bus_frame(&frame, &requests, None));
    }

    #[test]
    fn test_adjust_for_multi_reply() {
        let params = CycleParams {
            timeout_ns: 100_000_000,
            rx_extra_wait_ns: 40_000,
            ..Default::default()
        };

        // Ordinary transactions are untouched, keeping the driver's
        // fast early-exit once every expected reply has arrived.
        let (capacity, adjusted) = Pi3HatDevice::adjust_for_multi_reply(None, 5, params);
        assert_eq!(capacity, 5);
        assert_eq!(adjusted.rx_extra_wait_ns, 40_000);

        // A multi-reply transaction (e.g. a discovery broadcast)
        // holds the receive window open for the full timeout and
        // sizes the buffer for every possible responder.
        let (capacity, adjusted) = Pi3HatDevice::adjust_for_multi_reply(Some(127), 5, params);
        assert_eq!(capacity, 128);
        assert_eq!(adjusted.rx_extra_wait_ns, 100_000_000);

        // An already-larger configured extra wait is preserved.
        let big = CycleParams {
            timeout_ns: 1_000,
            rx_extra_wait_ns: 2_000,
            ..Default::default()
        };
        let (_, adjusted) = Pi3HatDevice::adjust_for_multi_reply(Some(2), 5, big);
        assert_eq!(adjusted.rx_extra_wait_ns, 2_000);
    }

    #[test]
    fn test_dispatch_bus_frame_skips_send_only_requests() {
        // A send-only write (zero expected replies) batched ahead of
        // a query on the same bus must not receive the query's reply.
        let send_only = Request::new(CanFdFrame::new()).with_expected_replies(0);
        let query = Request::receive_only(FrameFilter::Any);
        let requests = vec![send_only, query];

        let frame = CanFdFrame::new();
        assert!(dispatch_bus_frame(&frame, &requests, None));
        assert_eq!(requests[0].responses.len(), 0);
        assert_eq!(requests[1].responses.len(), 1);
    }

    #[test]
    fn test_dispatch_bus_frame_default_child() {
        // A request with no explicit child marker, as a child device
        // sees during a direct discovery probe.
        let requests = vec![Request::receive_only(FrameFilter::Any)];

        // When bound to channel 1 (a child device's own bus), frames
        // from sibling buses are not dispatched.
        let mut frame = CanFdFrame::new();
        frame.channel = Some(2);
        assert!(!dispatch_bus_frame(&frame, &requests, Some(1)));

        let mut frame = CanFdFrame::new();
        frame.channel = Some(1);
        assert!(dispatch_bus_frame(&frame, &requests, Some(1)));

        // The parent (no default) accepts frames from any bus.
        let mut frame = CanFdFrame::new();
        frame.channel = Some(2);
        assert!(dispatch_bus_frame(&frame, &requests, None));
    }

    #[test]
    fn test_factory_metadata() {
        let factory = Pi3HatFactory;
        assert_eq!(factory.name(), "pi3hat");
        assert_eq!(factory.priority(), 5);
    }

    #[test]
    fn test_factory_arg_specs() {
        // The declared specs must match the keys (and types) that
        // `options_from` reads back out of `TransportOptions::extra`.
        let by_name: HashMap<&str, ArgType> = Pi3HatFactory
            .arg_specs()
            .iter()
            .map(|s| (s.name, s.arg_type))
            .collect();
        assert_eq!(by_name.len(), 4);
        assert_eq!(by_name["pi3hat-cpu"], ArgType::Integer);
        assert_eq!(by_name["pi3hat-spi-hz"], ArgType::Integer);
        assert_eq!(by_name["pi3hat-cfg"], ArgType::MultiString);
        assert_eq!(by_name["pi3hat-disable-aux"], ArgType::Bool);
    }

    #[cfg(feature = "tokio")]
    #[test]
    fn test_async_factory_metadata() {
        let factory = AsyncPi3HatFactory;
        assert_eq!(factory.name(), "pi3hat");
        assert_eq!(factory.priority(), 5);
    }

    #[test]
    fn test_factory_options() {
        let factory = Pi3HatFactory;
        let options = TransportOptions::new()
            .extra("pi3hat-cpu", "2")
            .extra("pi3hat-spi-hz", "5000000")
            .extra("pi3hat-cfg", "1=11;2=12")
            .extra("pi3hat-disable-aux", "true")
            .disable_brs(true);

        let parsed = factory.options_from(&options).unwrap();
        assert_eq!(parsed.cpu, CpuAffinity::Pin(2));
        assert_eq!(parsed.config.spi_speed_hz, 5000000);
        assert_eq!(parsed.servo_map[&11], 1);
        assert_eq!(parsed.servo_map[&12], 2);
        assert!(!parsed.config.enable_aux);
        assert!(parsed.config.can.iter().all(|c| !c.bitrate_switch));

        for never in ["-1", "none"] {
            let options = TransportOptions::new().extra("pi3hat-cpu", never);
            let parsed = factory.options_from(&options).unwrap();
            assert_eq!(parsed.cpu, CpuAffinity::Unpinned);
        }
        let options = TransportOptions::new().extra("pi3hat-cpu", "junk");
        assert!(factory.options_from(&options).is_err());
    }
}
