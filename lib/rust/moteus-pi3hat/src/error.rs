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

//! Error types for the pi3hat library.

use std::fmt;

/// Error type for pi3hat operations.
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// An I/O error from the operating system.
    #[non_exhaustive]
    Io {
        /// What the library was doing when the error occurred.
        context: String,
        /// The underlying OS error, with `raw_os_error()` intact so
        /// callers can match specific errno values (e.g. `EAGAIN`
        /// from the single-instance lock file).
        source: std::io::Error,
    },
    /// Any other error, equivalent to the C++ `pi3hat::Error`.
    Message(String),
}

impl Error {
    pub(crate) fn message(msg: impl Into<String>) -> Self {
        Error::Message(msg.into())
    }

    /// Construct an error from the current value of `errno`, with a
    /// descriptive prefix, like the C++ `ThrowIfErrno`.
    pub(crate) fn errno(msg: &str) -> Self {
        Self::os(msg, std::io::Error::last_os_error())
    }

    /// Construct an error wrapping an already-captured OS error.
    pub(crate) fn os(msg: &str, source: std::io::Error) -> Self {
        Error::Io {
            context: msg.to_string(),
            source,
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io { context, source } if context.is_empty() => write!(f, "{}", source),
            Error::Io { context, source } => write!(f, "{} : {}", context, source),
            Error::Message(msg) => write!(f, "{}", msg),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io { source, .. } => Some(source),
            _ => None,
        }
    }
}

impl From<std::io::Error> for Error {
    fn from(source: std::io::Error) -> Self {
        Error::Io {
            context: String::new(),
            source,
        }
    }
}

/// Result type for pi3hat operations.
pub type Result<T> = std::result::Result<T, Error>;
