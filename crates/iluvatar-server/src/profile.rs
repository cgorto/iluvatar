//! Tracy profiling integration.
//!
//! When the `profile` feature is enabled, this module provides Tracy profiling
//! zones. When disabled, all macros compile to no-ops.

// Re-export tracy_client when feature is enabled so macros can use it.
#[cfg(feature = "profile")]
pub use tracy_client;

/// Mark a profiling scope (zone) with the given name.
/// Compiles to nothing when `profile` feature is disabled.
#[macro_export]
macro_rules! profile_scope {
    ($name:expr) => {
        #[cfg(feature = "profile")]
        let _tracy_span = $crate::profile::tracy_client::span!($name, 0);
    };
}

/// Mark a frame boundary for Tracy's frame view.
/// Compiles to nothing when `profile` feature is disabled.
#[macro_export]
macro_rules! profile_frame {
    () => {
        #[cfg(feature = "profile")]
        $crate::profile::tracy_client::Client::running().map(|c| c.frame_mark());
    };
    ($name:expr) => {
        #[cfg(feature = "profile")]
        $crate::profile::tracy_client::Client::running()
            .map(|c| c.secondary_frame_mark($crate::profile::tracy_client::frame_name!($name)));
    };
}

/// Plot a value on Tracy's plot view.
#[macro_export]
macro_rules! profile_plot {
    ($name:expr, $value:expr) => {
        #[cfg(feature = "profile")]
        $crate::profile::tracy_client::Client::running().map(|c| {
            c.plot(
                $crate::profile::tracy_client::plot_name!($name),
                $value as f64,
            )
        });
    };
}
