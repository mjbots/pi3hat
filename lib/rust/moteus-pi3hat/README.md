# moteus-pi3hat #

A Rust client library for the [mjbots pi3hat](https://mjbots.com/products/mjbots-pi3hat-r4-5),
a Raspberry Pi hat providing 5x CAN-FD buses and an attitude reference
system.

This is a direct translation of the C++ library in
`lib/cpp/mjbots/pi3hat`.  Like the C++ library, it drives the
Raspberry Pi SoC SPI peripherals directly through `/dev/mem`, so
applications **must run as root**, and at most one process may use the
pi3hat at a time.

## Using with moteus controllers ##

The crate implements the transport layer of the
[`moteus`](https://crates.io/crates/moteus) crate, which an
application depends on directly alongside this one:

```toml
[dependencies]
moteus = "0.5"
moteus-pi3hat = "0.1"
```

Call `moteus_pi3hat::transport::register()` once at startup -- the
Rust equivalent of linking `pi3hat_moteus_transport_register.cc` into
a C++ application -- and then use the `moteus` crate normally:

```rust
use moteus::command::PositionCommand;
use moteus::{BlockingController, TransportOptions};

fn main() -> Result<(), moteus::Error> {
    moteus_pi3hat::transport::register();

    let opts = TransportOptions::new().force_transport("pi3hat");
    let mut c = BlockingController::with_options(1, &opts)?;

    c.set_stop()?;
    loop {
        let state = c.set_position(PositionCommand::new().position(f32::NAN))?;
        println!("{:?}", state);
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
}
```

Once registered, the pi3hat also participates in moteus transport
auto-detection, so `force_transport` is optional when a pi3hat is the
only attached transport.

## Async ##

Enabling the `tokio` feature provides the async transport as well.
The same `register()` call then registers both the blocking and async
factories, and `moteus::AsyncController` / `AsyncRouter` work
normally:

```toml
[dependencies]
moteus-pi3hat = { version = "0.1", features = ["tokio"] }
```

```rust
let mut c = moteus::AsyncController::with_options(1, &opts).await?;
c.set_stop().await?;
```

The SPI I/O still runs on the dedicated worker thread in both
cases; the async transport just awaits its completion instead of
blocking.  All futures are cancel safe: a cycle always runs to
completion on the worker thread, and frames received during a
cancelled call are retained for later delivery.

Mirroring the Python `moteus_pi3hat` package, the pi3hat is exposed
as a parent transport device representing the shared SPI link (with
no bus of its own) plus one child device per CAN bus.  The moteus
`Router` discovers which bus each servo is attached to automatically,
routes all traffic through the parent, and performs a single router
cycle spanning servos on multiple buses as one pi3hat cycle with all
buses operating concurrently.  Received frames are attributed to the
bus they arrived on via `CanFdFrame::channel`.

Transport-specific options may be passed through
`TransportOptions::extra`:

| Key                  | Equivalent C++ argument | Meaning                                  |
|----------------------|-------------------------|------------------------------------------|
| `pi3hat-cpu`         | `--pi3hat-cpu`          | CPU to pin the realtime worker thread to; `-1`/`none` to never pin (default: auto) |
| `pi3hat-spi-hz`      | `--pi3hat-spi-hz`       | SPI speed                                |
| `pi3hat-cfg`         | `--pi3hat-cfg`          | Servo to bus map, e.g. `1=11,12;2=13,14` |
| `pi3hat-disable-aux` | `--pi3hat-disable-aux`  | Prevent use of the IMU/JC5               |

Like the Python `servo_bus_map`, `pi3hat-cfg` pre-populates the
routing table so that no on-demand discovery is needed for the listed
servos.

For example:

```rust
let opts = TransportOptions::new()
    .force_transport("pi3hat")
    .extra("pi3hat-cpu", "3")
    .extra("pi3hat-cfg", "1=11,12;2=13,14");
```

For best timing performance, isolate one or more CPUs with `isolcpus`.
By default (when `pi3hat-cpu` is unset) the worker is pinned to the
highest isolated CPU automatically and switched to realtime priority;
pass `pi3hat-cpu` to choose a specific one, or `-1`/`none` to run
unpinned at normal priority.  Keeping the busy SPI
worker on an isolated core is what lets the loop run at realtime
priority (e.g. under `chrt`) without starving the kernel on the
general-purpose CPUs.

## Reading the IMU alongside moteus ##

Only one process (and one transport) may own the pi3hat, so an
application using the moteus transport reaches the IMU through that
same transport rather than by opening a second handle.  The moteus
`Router` does not expose its transport devices, so the crate keeps a
handle to the active transport:

```rust,no_run
# fn main() -> Result<(), moteus::Error> {
moteus_pi3hat::transport::register();
let mut c = moteus::BlockingController::new(1)?;
c.set_stop()?;

let transport = moteus_pi3hat::transport::active_transport().unwrap();
let attitude = transport.read_attitude(true, false);
# Ok(())
# }
```

## Raw device access ##

The `Pi3Hat` type provides the same integrated CAN + IMU + RF cycle
interface as the C++ `mjbots::pi3hat::Pi3Hat`:

```rust
use moteus_pi3hat::{Attitude, Configuration, Input, Pi3Hat};

fn main() -> Result<(), moteus_pi3hat::Error> {
    let mut pi3hat = Pi3Hat::new(&Configuration::default())?;

    let mut attitude = Attitude::default();
    let output = pi3hat.cycle(Input {
        request_attitude: true,
        wait_for_attitude: true,
        attitude: Some(&mut attitude),
        ..Input::default()
    });
    assert!(output.attitude_present);
    println!("{:?}", attitude.attitude);
    Ok(())
}
```

## Examples ##

The crate ships three self-contained examples, buildable with plain
`cargo` (no Bazel required):

| Example        | Feature | Description                                 |
|----------------|---------|---------------------------------------------|
| `imu`          | —       | stream IMU data using the raw driver        |
| `simple`       | —       | hold position on servo ID 1 (blocking)      |
| `simple_async` | `tokio` | hold position on servo ID 1 (async)         |

Because the pi3hat accesses `/dev/mem`, the examples must run as root:

```
# Blocking examples (no extra features required)
cargo build --examples
sudo ./target/debug/examples/imu      # stream IMU data
sudo ./target/debug/examples/simple   # hold position on servo ID 1

# Async example
cargo build --features tokio --example simple_async
sudo ./target/debug/examples/simple_async
```

## Supported hardware ##

Like the C++ library, the Raspberry Pi 3 and 4 families (including
the corresponding Compute Modules) are supported.  The Raspberry Pi 5
uses a different I/O architecture and is not supported.
