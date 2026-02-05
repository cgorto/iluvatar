//! K230 VICAP camera capture backend.
//!
//! Uses the K230 SDK's VICAP API for hardware-accelerated camera capture,
//! bypassing V4L2's userspace copies for better performance.

use crate::arena::FrameArena;
use crate::capture::{CaptureError, CameraCapture, GrayscaleFrame};
use iluvatar_core::CameraPose;
use std::ffi::CStr;

/// FFI bindings to the C++ shim
mod ffi {
    use std::os::raw::c_char;

    #[repr(C)]
    pub struct K230CaptureContext {
        _private: [u8; 0],
    }

    #[repr(i32)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum K230Error {
        Ok = 0,
        VbInit = -1,
        VicapDev = -2,
        VicapChn = -3,
        VicapInit = -4,
        VicapStart = -5,
        FrameDump = -6,
        Mmap = -7,
        InvalidArg = -8,
        Alloc = -9,
        Timeout = -10,
    }

    #[repr(u32)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum K230SensorType {
        Ov5647 = 0,
        Imx335 = 1,
        Gc2093 = 2,
    }

    #[repr(C)]
    #[derive(Debug, Clone)]
    pub struct K230CaptureConfig {
        pub width: u32,
        pub height: u32,
        pub fps: u32,
        pub sensor_type: K230SensorType,
        pub dev_num: u32,
        pub chn_num: u32,
    }

    unsafe extern "C" {
        pub fn k230_capture_init(
            config: *const K230CaptureConfig,
            err: *mut K230Error,
        ) -> *mut K230CaptureContext;

        pub fn k230_capture_grayscale(
            ctx: *mut K230CaptureContext,
            buffer: *mut u8,
            len: usize,
        ) -> K230Error;

        pub fn k230_capture_deinit(ctx: *mut K230CaptureContext);

        pub fn k230_error_string(err: K230Error) -> *const c_char;
    }
}

pub use ffi::K230SensorType;

impl ffi::K230Error {
    fn is_ok(self) -> bool {
        self == ffi::K230Error::Ok
    }

    fn to_string(self) -> String {
        unsafe {
            let ptr = ffi::k230_error_string(self);
            if ptr.is_null() {
                return format!("Unknown error: {:?}", self);
            }
            CStr::from_ptr(ptr)
                .to_string_lossy()
                .into_owned()
        }
    }
}

impl From<ffi::K230Error> for CaptureError {
    fn from(err: ffi::K230Error) -> Self {
        match err {
            ffi::K230Error::Ok => unreachable!("Ok is not an error"),
            ffi::K230Error::VbInit => CaptureError::DeviceOpen(err.to_string()),
            ffi::K230Error::VicapDev => CaptureError::Format(err.to_string()),
            ffi::K230Error::VicapChn => CaptureError::Format(err.to_string()),
            ffi::K230Error::VicapInit => CaptureError::DeviceOpen(err.to_string()),
            ffi::K230Error::VicapStart => CaptureError::StreamCreation(err.to_string()),
            ffi::K230Error::FrameDump => CaptureError::Capture(err.to_string()),
            ffi::K230Error::Mmap => CaptureError::Capture(err.to_string()),
            ffi::K230Error::InvalidArg => CaptureError::Format(err.to_string()),
            ffi::K230Error::Alloc => CaptureError::DeviceOpen(err.to_string()),
            ffi::K230Error::Timeout => CaptureError::Capture(err.to_string()),
        }
    }
}

/// K230 VICAP camera capture backend.
///
/// Uses the K230 SDK's native VICAP API instead of V4L2 for better
/// performance (reduced userspace copies, direct ISP access).
pub struct K230Camera {
    ctx: *mut ffi::K230CaptureContext,
    width: u32,
    height: u32,
}

// Safety: The K230CaptureContext is only accessed through &mut self,
// so it's effectively single-threaded. The C++ implementation is
// thread-safe for single-context use.
unsafe impl Send for K230Camera {}

impl K230Camera {
    /// Create a new K230 camera capture instance.
    ///
    /// # Arguments
    /// * `width` - Capture width in pixels
    /// * `height` - Capture height in pixels
    /// * `fps` - Target frame rate
    /// * `sensor_type` - Type of sensor (OV5647, IMX335, GC2093)
    ///
    /// # Errors
    /// Returns an error if VICAP initialization fails.
    pub fn new(
        width: u32,
        height: u32,
        fps: u32,
        sensor_type: K230SensorType,
    ) -> Result<Self, CaptureError> {
        Self::with_device(width, height, fps, sensor_type, 0, 0)
    }

    /// Create a new K230 camera with specific device/channel numbers.
    ///
    /// # Arguments
    /// * `width` - Capture width in pixels
    /// * `height` - Capture height in pixels
    /// * `fps` - Target frame rate
    /// * `sensor_type` - Type of sensor
    /// * `dev_num` - VICAP device number (usually 0)
    /// * `chn_num` - VICAP channel number (usually 0)
    pub fn with_device(
        width: u32,
        height: u32,
        fps: u32,
        sensor_type: K230SensorType,
        dev_num: u32,
        chn_num: u32,
    ) -> Result<Self, CaptureError> {
        let config = ffi::K230CaptureConfig {
            width,
            height,
            fps,
            sensor_type,
            dev_num,
            chn_num,
        };

        let mut err = ffi::K230Error::Ok;
        let ctx = unsafe { ffi::k230_capture_init(&config, &mut err) };

        if ctx.is_null() || !err.is_ok() {
            return Err(err.into());
        }

        tracing::info!(
            width,
            height,
            fps,
            ?sensor_type,
            "K230 VICAP camera initialized"
        );

        Ok(Self { ctx, width, height })
    }
}

impl Drop for K230Camera {
    fn drop(&mut self) {
        if !self.ctx.is_null() {
            unsafe {
                ffi::k230_capture_deinit(self.ctx);
            }
            self.ctx = std::ptr::null_mut();
        }
    }
}

impl CameraCapture for K230Camera {
    fn resolution(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn capture_grayscale<'a>(
        &mut self,
        arena: &'a FrameArena,
        _pose: &CameraPose,
    ) -> Result<GrayscaleFrame<&'a mut [u8]>, CaptureError> {
        let size = (self.width * self.height) as usize;
        let buffer = arena.alloc_slice(size);

        let result = unsafe {
            ffi::k230_capture_grayscale(self.ctx, buffer.as_mut_ptr(), buffer.len())
        };

        if !result.is_ok() {
            return Err(result.into());
        }

        Ok(GrayscaleFrame {
            width: self.width,
            height: self.height,
            data: buffer,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_conversion() {
        // Just verify error conversion compiles and runs
        let err: CaptureError = ffi::K230Error::VbInit.into();
        assert!(matches!(err, CaptureError::DeviceOpen(_)));

        let err: CaptureError = ffi::K230Error::FrameDump.into();
        assert!(matches!(err, CaptureError::Capture(_)));
    }
}
