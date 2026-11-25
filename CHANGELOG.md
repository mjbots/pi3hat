# Changelog

pi3hat ships firmware, the `moteus_pi3hat` Python client, and the C++ client together under a single version; see [RELEASING.md](RELEASING.md)
for the process.  The Rust crate is versioned separately while pre-stable and noted in each release heading.

The format is loosely based on [Keep a Changelog](https://keepachangelog.com/).

Entries are added under `## Unreleased` by the change that introduces
them; cutting a release only renames that heading to the version and
date.

## Unreleased

- Python: the minimum supported version is now 3.9, and wheels are no
  longer published for 3.7.  The `cp37-abi3` wheel that pip previously
  selected for 3.8, and for versions newer than the newest published
  wheel, could be installed but not imported there.
- Python: wheels are now tagged for exactly the python version they
  were built for, both in the filename and in the wheel metadata.

- The Raspberry Pi binaries are now built with clang and libc++
  20.1.8 (was 10.0.0).  Building no longer requires `libtinfo5` on the
  host.

## 1.1.0 - 2026-09-09 (rust 0.1.0)

- New Rust client library and moteus transport, `moteus-pi3hat`
  (`lib/rust`), a direct translation of the C++ library. It is
  published to crates.io from the release workflow on its own `0.x`
  version line (`utils/release.py --rust`).
- The tools bundle also ships the Rust examples under `examples/rust/`
- `--pi3hat-cpu` (C++ and Rust) now defaults to the highest isolated
  CPU when the system has any (`isolcpus`), pinning the SPI worker
  there at realtime priority; previously the worker stayed unpinned
  unless a CPU was given. Pass `-1` to never pin.
- C++: `--pi3hat-cfg` rejects out-of-range buses and a servo id mapped
  to more than one bus instead of silently misrouting.
- C++: guard against malformed short CAN frames in `ReadCanFrames`
- C++: fix a race on `Pi3HatMoteusTransport` construction where the
  worker thread could start before its members were constructed
  (intermittent segfault).
- C++: document that `CanConfiguration::restricted_mode` is ignored.
- Documentation: the JC1-4 high speed vs JC5 low speed distinction
  only applied to boards older than r4.5; modern boards make none.
- CI also compiles the Rust crate in its published configuration
  (crates.io `moteus`, locked) and on its declared MSRV (1.75).

## 1.0.0

- First unified semver release. Firmware, the `moteus_pi3hat` Python
  package, and the C++ client now share one version and one `vX.Y.Z` tag.
- Release engineering: tag-driven GitHub Actions build all three
  artifacts into a single Release; promoting the Release publishes the
  Python wheels to PyPI via OIDC trusted publishing.
- The Python package now depends on `moteus>=1.0.0rc1`.
