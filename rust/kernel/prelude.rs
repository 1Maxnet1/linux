// SPDX-License-Identifier: GPL-2.0

//! The `kernel` prelude.
//!
//! These are the most common items used by Rust code in the kernel,
//! intended to be imported by all Rust code, for convenience.
//!
//! # Examples
//!
//! ```
//! use kernel::prelude::*;
//! ```

pub use core::pin::Pin;

pub use alloc::{boxed::Box, string::String, vec::Vec};

pub use macros::module;

pub use super::build_assert;

pub use super::{dbg, pr_alert, pr_crit, pr_debug, pr_emerg, pr_err, pr_info, pr_notice, pr_warn};

pub use super::module_misc_device;

#[cfg(CONFIG_ARM_AMBA)]
pub use super::module_amba_driver;

pub use super::static_assert;

pub use super::{Error, KernelModule, Result};

pub use super::{str::CStr, ThisModule};
