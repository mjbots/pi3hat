# Changelog

pi3hat ships firmware, the `moteus_pi3hat` Python client, and the C++
client together under a single version; see [RELEASING.md](RELEASING.md)
for the process.

The format is loosely based on [Keep a Changelog](https://keepachangelog.com/).

## Unreleased

- New Rust client library and moteus transport, `moteus-pi3hat`
  (`lib/rust`), a direct translation of the C++ library.
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
  (crates.io `moteus`, locked).

## 1.0.0

- First unified semver release. Firmware, the `moteus_pi3hat` Python
  package, and the C++ client now share one version and one `vX.Y.Z` tag.
- Release engineering: tag-driven GitHub Actions build all three
  artifacts into a single Release; promoting the Release publishes the
  Python wheels to PyPI via OIDC trusted publishing.
- The Python package now depends on `moteus>=1.0.0rc1`.
