use kinect::{Device, DeviceConfiguration, Calibration, Transformation};
use crate::CommandArgs;
use crate::setup::camera_info::CameraInfo;
use crate::pointcloud::xy_table::create_xy_table_from_color_calibration;
use crate::pointcloud::cpu_transformation::DepthTransformer;
use crate::AnyhowResult;
use k4a_sys_temp as k4a_sys;
use crate::calibration::world_calibration::WorldCalibration;

pub fn open_and_configure_camera(args: &CommandArgs, index: u32) -> AnyhowResult<(Device, CameraInfo)>
{
  let device = Device::open(index)?;

  let mut config = DeviceConfiguration::init_disable_all();

  let mut camera_fps = k4a_sys::k4a_fps_t_K4A_FRAMES_PER_SECOND_30;

  let depth_mode_fov = if args.wide {
    // NB: Wide FOV lowers the possible frame rate.
    camera_fps = k4a_sys::k4a_fps_t_K4A_FRAMES_PER_SECOND_15;
    k4a_sys::k4a_depth_mode_t_K4A_DEPTH_MODE_WFOV_UNBINNED
  } else {
    k4a_sys::k4a_depth_mode_t_K4A_DEPTH_MODE_NFOV_UNBINNED
  };

  config.0.camera_fps = camera_fps;
  config.0.depth_mode = depth_mode_fov;
  config.0.color_format = k4a_sys::k4a_image_format_t_K4A_IMAGE_FORMAT_COLOR_BGRA32;

  config.0.color_resolution = k4a_sys::k4a_color_resolution_t_K4A_COLOR_RESOLUTION_720P; // 1280x721
  //config.0.color_resolution = k4a_sys::k4a_color_resolution_t_K4A_COLOR_RESOLUTION_1080P; // 1920x1080 (this gets truncated.)
  //config.0.color_resolution = k4a_sys::k4a_color_resolution_t_K4A_COLOR_RESOLUTION_2160P; // 4K, what the original program did

  let calibration = device.get_calibration(config.0.depth_mode, config.0.color_resolution)?;

  //let xy_table = create_xy_table_from_depth_calibration(&calibration)?;
  let xy_table = create_xy_table_from_color_calibration(&calibration)?;

  let transformer = DepthTransformer::new(&calibration);

  println!("Loading world calibration...");
  let world_calibration = WorldCalibration::load_from_file_and_device(&device)?;

  /*

   */


  let camera_info = CameraInfo {
    config,
    calibration,
    xy_table,
    transformer,
    world_calibration,
  };

  Ok((device, camera_info))
}

/// From LiveScan3D:
///   It's crucial for this program to output accurately mapped Pointclouds. The highest accuracy mapping is achieved
///   by using the k4a_transformation_depth_image_to_color_camera function. However this converts a small depth image
///   to a larger size, equivalent to the the color image size. This means more points to process and higher processing costs
///   We can however scale the color image to the depth images size beforehand, to reduce proccesing power.
fn get_calibration_color_downscaled(calibration: &Calibration) {
  //No way to get the depth pixel values from the SDK at the moment, so this is hardcoded
  let mut depth_camera_width = 0;
  let mut depth_camera_height= 0;

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

  // NFOV_UNBINNED
  depth_camera_width = 640;
  depth_camera_height = 576;


  // We calculate the minimum size that the color Image can be, while preserving its aspect ration
  let rescale_ratio : f32 = calibration.0.color_camera_calibration.resolution_height as f32 / depth_camera_height as f32;

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
}
