use kinect::{Device, DeviceConfiguration, Calibration, Transformation};
use crate::CommandArgs;
use crate::setup::camera_info::CameraInfo;
use crate::pointcloud::xy_table::create_xy_table_from_color_calibration;
use crate::pointcloud::cpu_transformation::DepthTransformer;
use crate::AnyhowResult;
use k4a_sys_temp as k4a_sys;
use crate::calibration::world_calibration::WorldCalibration;
use crate::calibration::color_downscale_info::ColorDownscaleInfo;

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

  let color_downscale_info = ColorDownscaleInfo::get_calibration_color_downscaled(&calibration);

  let xy_table_livescan  = create_xy_table_from_color_calibration(&color_downscale_info.calibration_color_downscaled)?;
  /*

void AzureKinectCapture::UpdateDepthPointCloudForColorFrame()
	k4a_transformation_depth_image_to_color_camera(transformationColorDownscaled, depthImage, transformedDepthImage);
	k4a_transformation_depth_image_to_point_cloud(transformationColorDownscaled, transformedDepthImage, K4A_CALIBRATION_TYPE_COLOR, pointCloudImage);


void AzureKinectCapture::UpdateDepthPointCloud()
	k4a_transformation_depth_image_to_point_cloud(transformation, depthImage, K4A_CALIBRATION_TYPE_DEPTH, pointCloudImage);

   */


  let camera_info = CameraInfo {
    config,
    calibration,
    xy_table,
    transformer,
    world_calibration,
    color_downscale_info,
    xy_table_livescan,
  };

  Ok((device, camera_info))
}
