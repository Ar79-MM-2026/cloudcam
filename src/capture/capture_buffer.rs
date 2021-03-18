use kinect::Capture;
use std::sync::{RwLock, Arc, PoisonError, RwLockWriteGuard, Mutex};
use crate::AnyhowResult;
use anyhow::anyhow;

pub struct CaptureBuffer {
  num_devices: usize,
  captures: Arc<Mutex<Vec<Capture>>>,
}

impl CaptureBuffer {

  pub fn new(num_devices: usize) -> Self {
    CaptureBuffer {
      num_devices,
      captures: Arc::new(Mutex::new(Vec::with_capacity(num_devices)))
    }
  }

  pub fn set_captures(&self) -> AnyhowResult<()> {
    /*if captures.len() != self.num_devices {
      return Err(anyhow!("Number of captures ({}) != number of devices ({})",
        captures.len(), self.num_devices));
    }*/

    /*match self.captures.write() {
      Err(err) => {
        return Err(anyhow!("Lock error: {:?}", err))
      },
      Ok(mut caps) => {
        caps.clear();
        caps.extend(captures);
      },
    }*/

    match self.captures.lock() {
      Ok(mut lock) => {
        //*lock = captures;
        lock.clear();
      },
      Err(_) => {},
    }

    Ok(())
  }
}