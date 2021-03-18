use kinect::{Device, Calibration, Image, DeviceConfiguration};
use crate::pointcloud::cpu_transformation::DepthTransformer;

pub struct CameraInfo {
  pub device: Device,
  pub config: DeviceConfiguration,
  pub calibration: Calibration,
  pub xy_table: Image,
  pub transformer: DepthTransformer,
}
