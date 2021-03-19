use kinect::{Device, Calibration, Image, DeviceConfiguration};
use crate::pointcloud::cpu_transformation::DepthTransformer;
use crate::calibration::world_calibration::WorldCalibration;
use crate::calibration::color_downscale_info::ColorDownscaleInfo;

/// Extra metadata about a camera.
pub struct CameraInfo {
  pub config: DeviceConfiguration,
  pub calibration: Calibration,
  pub xy_table: Image,
  pub transformer: DepthTransformer,
  pub world_calibration: WorldCalibration,
  pub color_downscale_info: ColorDownscaleInfo,
}
