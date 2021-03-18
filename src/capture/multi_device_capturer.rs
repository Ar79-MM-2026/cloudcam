use kinect::{Device, Capture};
use crate::capture::capture_buffer::CaptureBuffer;
use crate::AnyhowResult;
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

  pub fn run_capture_loop(&self) {
    loop {
      match self.get_synchronized_captures() {
        Ok(captures) => {
          self.capture_buffer.set_captures();
        },
        Err(e) => {
          println!("Error with captures: {:?}", e);
          continue;
        }
      }
    }
  }

  fn get_synchronized_captures(&self) -> AnyhowResult<Vec<Capture>> {
    let mut captures = Vec::with_capacity(self.devices.len());
    for device in self.devices.iter() {
      let capture = device.get_capture(WAIT_INFINITE)?;
      captures.push(capture);
    }
    // TODO: Apparently there's some sync logic that goes here.
    Ok(captures)
  }
}