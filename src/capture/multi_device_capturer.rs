use anyhow::anyhow;
use crate::AnyhowResult;
use crate::capture::capture_buffer::CaptureBuffer;
use kinect::{Device, Capture};
use std::sync::Arc;

const WAIT_INFINITE : i32 = -1;

pub struct MultiDeviceCapturer {
  devices: Vec<Device>,
  capture_buffer: Arc<CaptureBuffer>,
}

impl MultiDeviceCapturer {

  pub fn new(devices: Vec<Device>) -> Self {
    Self {
      capture_buffer: Arc::new(CaptureBuffer::new(devices.len())),
      devices,
    }
  }

  pub fn get_capture_buffer(&self) -> Arc<CaptureBuffer> {
    self.capture_buffer.clone()
  }

  pub fn run_capture_loop(&self) -> AnyhowResult<()> {
    loop {
      match self.get_synchronized_captures() {
        Ok(captures) => {
          let _r = self.capture_buffer.set_captures(captures)?;
        },
        Err(_e) => {
          continue;
        }
      }
    }
    Ok(())
  }

  fn get_synchronized_captures(&self) -> AnyhowResult<Vec<Capture>> {
    let mut captures = Vec::with_capacity(self.devices.len());

    for device in self.devices.iter() {
      let capture = device.get_capture(WAIT_INFINITE)?;

      if capture.get_color_image().is_none() {
        return Err(anyhow!("Capture contains no color image"));
      }
      if capture.get_depth_image().is_none() {
        return Err(anyhow!("Capture contains no depth image"));
      }

      captures.push(capture);
    }

    // TODO: Apparently there's some sync logic that goes here.
    Ok(captures)
  }
}