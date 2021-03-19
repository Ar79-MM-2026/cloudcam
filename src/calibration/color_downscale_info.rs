use kinect::{Calibration, Transformation};
use crate::pointcloud::cpu_transformation::DepthTransformer;

pub struct ColorDownscaleInfo {
  pub color_image_downscaled_height: i32,
  pub color_image_downscaled_width: i32,
  pub calibration_color_downscaled: Calibration, // NB: Only exposed because other parts need it
  pub transformation_color_downscaled: Transformation,
  pub depth_transformer: DepthTransformer,
}

impl ColorDownscaleInfo {
  /// From LiveScan3D:
  ///   It's crucial for this program to output accurately mapped Pointclouds. The highest accuracy mapping is achieved
  ///   by using the k4a_transformation_depth_image_to_color_camera function. However this converts a small depth image
  ///   to a larger size, equivalent to the the color image size. This means more points to process and higher processing costs
  ///   We can however scale the color image to the depth images size beforehand, to reduce proccesing power.
  pub fn get_calibration_color_downscaled(calibration: &Calibration) -> Self {
    //No way to get the depth pixel values from the SDK at the moment, so this is hardcoded
    let mut depth_camera_width = 0;
    let mut depth_camera_height = 0;

    /*switch (config.depth_mode)
  {
    case K4A_DEPTH_MODE_NFOV_UNBINNED:
      depth_camera_width = 640;
    depth_camera_height = 576;
    break;
    case K4A_DEPTH_MODE_NFOV_2X2BINNED:
      depth_camera_width = 320;
    depth_camera_height = 288;
    break;
    case K4A_DEPTH_MODE_WFOV_UNBINNED:
      depth_camera_width = 1024;
    depth_camera_height = 1024;
    case K4A_DEPTH_MODE_WFOV_2X2BINNED:
      depth_camera_width = 512;
    depth_camera_height = 512;
    break;
    default:
    break;
  }*/

    // TODO(bt): Hardcoding for my use case.
    // NFOV_UNBINNED
    depth_camera_width = 640;
    depth_camera_height = 576;


    // We calculate the minimum size that the color Image can be, while preserving its aspect ration
    let rescale_ratio: f32 = calibration.0.color_camera_calibration.resolution_height as f32 / depth_camera_height as f32;

    let color_image_downscaled_height = depth_camera_height;
    let color_image_downscaled_width = (calibration.0.color_camera_calibration.resolution_width as f32 / rescale_ratio) as i32;

    // We don't only need the size in pixels of the downscaled color image, but also a new k4a_calibration_t which fits the new
    // sizes
    let mut calibration_color_downscaled = calibration.clone();

    unsafe {
      calibration_color_downscaled.0.color_camera_calibration.resolution_width =
          (calibration_color_downscaled.0.color_camera_calibration.resolution_width as f32 / rescale_ratio) as i32;

      calibration_color_downscaled.0.color_camera_calibration.resolution_height =
          (calibration_color_downscaled.0.color_camera_calibration.resolution_height as f32 / rescale_ratio) as i32;

      calibration_color_downscaled.0.color_camera_calibration.intrinsics.parameters.param.cx /= rescale_ratio;
      calibration_color_downscaled.0.color_camera_calibration.intrinsics.parameters.param.cy /= rescale_ratio;
      calibration_color_downscaled.0.color_camera_calibration.intrinsics.parameters.param.fx /= rescale_ratio;
      calibration_color_downscaled.0.color_camera_calibration.intrinsics.parameters.param.fy /= rescale_ratio;
    }

    let transformation_color_downscaled = Transformation::from_calibration(&calibration_color_downscaled);

    let depth_transformer = DepthTransformer::new(&calibration_color_downscaled);

    Self {
      color_image_downscaled_height,
      color_image_downscaled_width,
      calibration_color_downscaled,
      transformation_color_downscaled,
      depth_transformer,
    }
  }
}
