# Changelog

pi3hat ships firmware, the `moteus_pi3hat` Python client, and the C++
client together under a single version; see [RELEASING.md](RELEASING.md)
for the process.

The format is loosely based on [Keep a Changelog](https://keepachangelog.com/).

## Unreleased

- C++: guard against malformed short CAN frames in `ReadCanFrames`
- C++: fix a race on `Pi3HatMoteusTransport` construction where the
  worker thread could start before its members were constructed
  (intermittent segfault).
- C++: document that `CanConfiguration::restricted_mode` is ignored.
- Documentation: the JC1-4 high speed vs JC5 low speed distinction
  only applied to boards older than r4.5; modern boards make none.

## 1.0.0

- First unified semver release. Firmware, the `moteus_pi3hat` Python
  package, and the C++ client now share one version and one `vX.Y.Z` tag.
- Release engineering: tag-driven GitHub Actions build all three
  artifacts into a single Release; promoting the Release publishes the
  Python wheels to PyPI via OIDC trusted publishing.
- The Python package now depends on `moteus>=1.0.0rc1`.
