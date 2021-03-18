use kinect::{Device, Calibration, Image, DeviceConfiguration};
use crate::pointcloud::cpu_transformation::DepthTransformer;

/// Extra metadata about a camera.
pub struct CameraInfo {
  pub config: DeviceConfiguration,
  pub calibration: Calibration,
  pub xy_table: Image,
  pub transformer: DepthTransformer,
}
