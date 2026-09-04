# Releasing pi3hat

pi3hat ships three artifacts that move together under a **single
version** and a **single `vX.Y.Z` tag**:

| Artifact | What it is                                              |
|----------|---------------------------------------------------------|
| firmware | the flashable STM32G4 image (`pi3_hat.elf` / `.bin`)    |
| python   | the `moteus_pi3hat` wheels (one per Python version × Pi architecture) |
| cpp      | the header/`FetchContent` C++ client source archive     |

A single GitHub Release for the tag carries all of them. The version
lives in two files, kept in sync by `utils/release.py`:

| File             | Field                          | Form                         |
|------------------|--------------------------------|------------------------------|
| `lib/python/BUILD` | `VERSION="X.Y.Z"`            | full PEP 440 (`1.0.0rc1`)    |
| `CMakeLists.txt`   | `project(... VERSION X.Y.Z)` | `M.m.p` only (no suffix)     |

The Python package is published to PyPI, so the version string must be a
normalized [PEP 440](https://peps.python.org/pep-0440/) version
(`1.0.0`, `1.0.0rc1`, `1.0.0b1`, `1.0.0.dev1`) — *not* the semver
spelling `1.0.0-rc.1`. CMake's `project(VERSION)` only accepts `M.m.p`,
so any pre-release suffix is dropped from `CMakeLists.txt` while the full
string is kept for the wheel and the tag.

## Cutting a release

Run the helper from the repo root, then push:

```bash
# patch bump (1.0.0 -> 1.0.1)
utils/release.py patch

# minor / major
utils/release.py minor
utils/release.py major

# start a pre-release cycle (explicit)
utils/release.py 1.1.0rc1

# iterate a pre-release (1.1.0rc1 -> 1.1.0rc2)
utils/release.py rc

# graduate a pre-release to its final release (1.1.0rc2 -> 1.1.0)
utils/release.py patch

# also bump the Rust crate (on its own pre-stable 0.x line)
utils/release.py patch --rust patch
```

The `patch` bump is overloaded: when current is already a pre-release it
strips the suffix instead of incrementing the patch number (matches
npm/cargo conventions). `minor` and `major` always advance `M.m.p`.

The Rust crate (`lib/rust/moteus-pi3hat`) is versioned separately while
it is pre-stable: it stays on its own `0.x` line rather than the unified
`1.x` number, and is bumped — and later published to crates.io — only
when you pass `--rust`. Omit `--rust` and the crate version is left
untouched, so a release that does not change the crate does not
republish it. `--rust` takes the same `major`/`minor`/`patch` bumps or
an explicit semver version (e.g. `--rust 0.2.0`). When the crate is
declared stable, bump it onto the unified number once with `--rust <that
version>` and pass `--rust` on every release thereafter.

The script:

1. Reads the current version from `lib/python/BUILD`.
2. Computes the new version (or accepts an explicit one) and validates it
   is normalized PEP 440.
3. Writes the new version to `lib/python/BUILD` and `CMakeLists.txt`
   (and, with `--rust`, the crate version to `lib/rust/Cargo.toml`).
4. Commits (if a file changed) and creates the annotated tag `vX.Y.Z`.
5. Prints the `git push` commands.

Before any of that it checks that `CHANGELOG.md` has a `## X.Y.Z`
section with entries for the new version and refuses to tag otherwise
-- rename `## Unreleased` first.  (Entries themselves are added with
the change that introduces them, not at release time.)

Inspect the diff and tag before pushing.

```bash
git push origin HEAD
git push origin v1.0.1
```

You can also kick off a release from the GitHub UI via the
[`Release`](.github/workflows/release.yml) workflow — it runs the same
script in CI, useful when releasing from somewhere without your local
toolchain.

## What the build produces

Pushing the tag fires
[`build-release.yml`](.github/workflows/build-release.yml), which builds
on `ubuntu-24.04` and attaches all of the following to one **draft**
GitHub Release:

- **Firmware**: `pi3hat-fw-<version>+g<sha>.elf` and `.bin`, built with
  `tools/bazel build --cpu stm32g4 -c opt //:target`. The `+g<sha>`
  portion is semver build metadata (the 10-char git short SHA of the
  tagged commit), ignored for version precedence.
- **Python**: for each Python version (3.7, 3.9–3.13) × Pi architecture
  (`armv7l`, `aarch64`), it cross-compiles `//:pi3hat_tools` and attaches
  both:
  - `moteus_pi3hat-<version>-cp<py>-<abi>-<plat>.whl` — the wheel that
    goes to PyPI.
  - `pi3hat_tools-<version>-cp<py>-<arch>.tar.bz2` — the full tool
    bundle (the `pi3hat_tool` binary, the wheel, and the C++ examples +
    sources).
- **C++**: `pi3hat-cpp-<version>.tar.gz`, a source archive of the
  `FetchContent` C++ client (`CMakeLists.txt`, `LICENSE`,
  `lib/cpp/mjbots/pi3hat`, `lib/cpp/examples`). The C++ client is
  normally consumed via `FetchContent` pointing at the `vX.Y.Z` tag;
  this archive is just a convenience download.

The build refuses to run if the pushed tag's version disagrees with
`VERSION` in `lib/python/BUILD` (the wheel filenames are derived from
that file). `utils/release.py` keeps them in sync, so this only trips on
a hand-made tag.

Drafts are not visible to users browsing the Releases page or to `pip`
consumers — they exist only on the maintainer's view of the page.

## Promoting a draft release to public

After the build workflow finishes, validate the draft locally:

1. Go to the repo's Releases page → the draft shows under "Draft" at the
   top. Download the attached artifacts.
2. Flash the firmware on representative hardware; `pip install` a wheel
   into a fresh venv on a Pi and exercise `moteus_tool`/`pi3hat_tool`.
3. When satisfied, publish — click "Publish release" in the UI, or:

   ```bash
   gh release edit v1.0.0 --draft=false
   ```

If something is wrong, abort instead of publishing:

```bash
gh release delete v1.0.0 --yes
git push --delete origin v1.0.0
git tag -d v1.0.0
# ...fix the issue, then re-tag
```

**Promoting the draft is what triggers the PyPI publish.**
[`publish-python-pypi.yml`](.github/workflows/publish-python-pypi.yml)
listens for the `release: published` event, downloads the `*.whl` assets,
and uploads them to PyPI via OIDC. Do not publish a draft until you
intend the wheels to land on PyPI — PyPI is append-only, you cannot
un-publish (only yank).

A pre-release version (anything PEP 440 considers pre-release, e.g.
`1.0.0rc1`) goes to PyPI but `pip install moteus_pi3hat` won't pick it up
unless the user passes `--pre`. So pre-releases are safe to publish.

## Publishing to PyPI — one-time setup

PyPI publishing uses
[trusted publishing](https://docs.pypi.org/trusted-publishers/) via OIDC,
so no API token is stored in this repo.

1. Sign in to PyPI as a maintainer of the `moteus_pi3hat` project.
2. Project settings → Publishing → Add a new pending publisher (or
   trusted publisher if the project already exists). Configure:
   - Owner: `mjbots`
   - Repository name: `pi3hat`
   - Workflow name: `publish-python-pypi.yml`
   - Environment name: `pypi`
3. In this GitHub repo: Settings → Environments → New environment named
   `pypi`. The environment exists so PyPI's trusted-publishing config has
   something to bind to; no required reviewers are needed because the
   "Publish release" action on the draft is the gate.

## Publishing to crates.io — one-time setup

The Rust crate publishes the same way, via crates.io
[trusted publishing](https://crates.io/docs/trusted-publishing) over
OIDC, so no token is stored in the repo.

crates.io trusted publishing is configured under an existing crate's
settings, so the crate must exist first. For the initial `0.x` publish,
run `cargo publish` once manually with a personal API token to create
the crate; then:

1. Sign in to crates.io as an owner of the `moteus-pi3hat` crate.
2. Crate settings → Trusted Publishing → Add a GitHub publisher:
   - Owner: `mjbots`
   - Repository name: `pi3hat`
   - Workflow filename: `publish-rust-crates.yml`
   - Environment name: `crates-io`
3. In this GitHub repo: Settings → Environments → New environment named
   `crates-io` (same role as `pypi`).

Prefer a stored token instead of OIDC? Drop the auth step in
`publish-rust-crates.yml` and add a `CARGO_REGISTRY_TOKEN` secret to the
`crates-io` environment.

## What each workflow does

- [`release.yml`](.github/workflows/release.yml) — manual
  `workflow_dispatch` to bump the version and tag from CI.
- [`build-release.yml`](.github/workflows/build-release.yml) — fires on a
  `v*` tag. Builds the firmware, the cross-compiled Python wheels + tool
  bundles, and the C++ source archive, and attaches them all to one draft
  Release.
- [`publish-python-pypi.yml`](.github/workflows/publish-python-pypi.yml)
  — fires when a `v*` Release is promoted out of draft. Downloads the
  attached wheels and uploads them to PyPI via OIDC trusted publishing.
- [`publish-rust-crates.yml`](.github/workflows/publish-rust-crates.yml)
  — fires when a `v*` Release is promoted out of draft. Builds the
  tagged `moteus-pi3hat` crate and publishes it to crates.io via OIDC
  trusted publishing, skipping if that version is already published.
- [`ci.yml`](.github/workflows/ci.yml) — runs `travis-ci.sh` on every
  push and pull request.

## Local builds

`make_release.py` still builds the same artifacts locally into an output
directory (naming them by date + git hash rather than by version). It is
handy for ad-hoc builds, but the tag-driven `build-release.yml` is the
canonical release path.

## Rust dependency lockfiles

`lib/rust` carries two committed lockfiles: `Cargo.lock` (cargo's own)
and `Cargo.bazel.lock` (the rules_rust crate_universe resolution built
from it). After changing any dependency specification (a crate's
`Cargo.toml`, or the crate list in `WORKSPACE`), regenerate both:

```bash
cd lib/rust && cargo update --workspace   # refreshes Cargo.lock minimally
CARGO_BAZEL_REPIN=1 ./tools/bazel sync --only=crate_index
```

A stale `Cargo.bazel.lock` fails the bazel build with a message that
names `CARGO_BAZEL_REPIN`, so drift is caught rather than silent.
