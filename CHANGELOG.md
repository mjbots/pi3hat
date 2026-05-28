# Changelog

pi3hat ships firmware, the `moteus_pi3hat` Python client, and the C++
client together under a single version; see [RELEASING.md](RELEASING.md)
for the process.

The format is loosely based on [Keep a Changelog](https://keepachangelog.com/).

## Unreleased

(no entries yet)

## 1.0.0

- First unified semver release. Firmware, the `moteus_pi3hat` Python
  package, and the C++ client now share one version and one `vX.Y.Z` tag.
- Release engineering: tag-driven GitHub Actions build all three
  artifacts into a single Release; promoting the Release publishes the
  Python wheels to PyPI via OIDC trusted publishing.
- The Python package now depends on `moteus>=1.0.0rc1`.
