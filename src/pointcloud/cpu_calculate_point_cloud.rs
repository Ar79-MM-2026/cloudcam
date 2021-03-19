use anyhow::{anyhow, Error};
use crate::AnyhowResult;
use crate::CommandArgs;
use crate::pointcloud::color::Color;
use crate::pointcloud::point::Point;
use k4a_sys_temp as k4a_sys;
use kinect::Image;
use kinect::ImageFormat;
use num_traits::Float;
use std::mem::size_of;
use crate::calibration::world_calibration::WorldCalibration;
use crate::calibration::resize::resize_image;
use crate::calibration::color_downscale_info::ColorDownscaleInfo;

pub struct PointCloudResult {
    pub point_cloud_image: Image,
    pub point_count: u32,
}

/// This has been adapted from libk4a sources (MIT-licensed).
pub fn calculate_point_cloud(depth_image: &Image, xy_table_image: &Image) -> AnyhowResult<PointCloudResult> {
    let width = depth_image.get_width_pixels();
    let height = depth_image.get_height_pixels();

    let point_cloud_image= Image::create(
        ImageFormat::Custom,
        width as u32,
        height as u32,
        width as u32 * size_of::<k4a_sys::k4a_float3_t>() as u32,
    )?;

    let point_cloud_data = point_cloud_image.get_buffer() as *mut k4a_sys::k4a_float3_t;
    let xy_table_data = xy_table_image.get_buffer() as *mut k4a_sys::k4a_float2_t;
    let depth_data = depth_image.get_buffer() as *mut u16; // uint16_t

    let depth_data_length = (width * height) as isize;

    let mut point_count = 0;

    for i in 0 .. depth_data_length {
        unsafe {
            // TODO: This is missing `isnan` checks.
            //  if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
            if (*depth_data.offset(i)) != 0 {
                (*point_cloud_data.offset(i)).xyz.x = (*xy_table_data.offset(i)).xy.x * ((*depth_data.offset(i)) as f32);
                (*point_cloud_data.offset(i)).xyz.y = (*xy_table_data.offset(i)).xy.y * ((*depth_data.offset(i)) as f32);
                (*point_cloud_data.offset(i)).xyz.z = (*depth_data.offset(i)) as f32;
                point_count += 1;
            } else {
                (*point_cloud_data.offset(i)).xyz.x = f32::nan();
                (*point_cloud_data.offset(i)).xyz.y = f32::nan();
                (*point_cloud_data.offset(i)).xyz.z = f32::nan();
            }
        }
    }

    Ok(PointCloudResult {
        point_cloud_image,
        point_count,
    })
}

// Directly return as a vector.
pub fn calculate_point_cloud2(depth_image: &Image, xy_table_image: &Image, color: Color) -> AnyhowResult<Vec<Point>> {
    let width = depth_image.get_width_pixels();
    let height = depth_image.get_height_pixels();

    {
        let xy_width = xy_table_image.get_width_pixels();
        let xy_height = xy_table_image.get_height_pixels();
        if width != xy_width || height != xy_height {
            return Err(anyhow!("Depth image ({}x{}) and XY table ({}x{}) dimensions are not equal!",
                width, height, xy_width, xy_height));
        }
    }

    let xy_table_data = xy_table_image.get_buffer() as *mut k4a_sys::k4a_float2_t;
    let depth_data = depth_image.get_buffer() as *mut u16; // uint16_t

    let depth_data_length = (width * height) as isize;

    let mut points = Vec::new();

    for i in 0 .. depth_data_length {
        unsafe {
            // TODO: This is missing `isnan` checks.
            //  if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
            if (*depth_data.offset(i)) != 0 {
                let x = (*xy_table_data.offset(i)).xy.x * ((*depth_data.offset(i)) as f32);
                let y = (*xy_table_data.offset(i)).xy.y * ((*depth_data.offset(i)) as f32);
                let z = (*depth_data.offset(i)) as f32;
                points.push(Point::at(x, y, z, color));
            }
        }
    }

    Ok(points)
}

#[repr(C, packed)]
#[derive(Copy, Clone)]
struct ColorCameraPoint {
    pub b: u8,
    pub g: u8,
    pub r: u8,
    pub a: u8,
}

// Directly return as a vector.
pub fn calculate_point_cloud3(
    depth_image: &Image,
    xy_table_image: &Image,
    maybe_world_calibration: Option<&WorldCalibration>,
    color_image: &Image,
    command_args: &CommandArgs,
    color_downscale_info: &ColorDownscaleInfo
) -> AnyhowResult<Vec<Point>>
{
    let width = depth_image.get_width_pixels();
    let height = depth_image.get_height_pixels();

    /*match res {
        Ok(_) => {}
        Err(e) => {
            println!("Error resizing image: {}", e);
        }
    }*/


    {
        let xy_width = xy_table_image.get_width_pixels();
        let xy_height = xy_table_image.get_height_pixels();
        if width != xy_width || height != xy_height {
            return Err(anyhow!("Depth image ({}x{}) and XY table ({}x{}) dimensions are not equal!",
                width, height, xy_width, xy_height));
        }
    }

    let xy_table_data = xy_table_image.get_buffer() as *mut k4a_sys::k4a_float2_t;
    let depth_data = depth_image.get_buffer() as *mut u16; // uint16_t
    let color_data = color_image.get_buffer() as *mut ColorCameraPoint;

    let depth_data_length = (width * height) as isize;

    let mut points = Vec::new();

    for i in 0 .. depth_data_length {
        unsafe {
            // TODO: This is missing `isnan` checks.
            //  if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
            if (*depth_data.offset(i)) != 0 {
                let x_lut = (*xy_table_data.offset(i)).xy.x;
                let y_lut = (*xy_table_data.offset(i)).xy.y;

                if x_lut.is_nan() || y_lut.is_nan() {
                    continue;
                }

                let mut x = x_lut * ((*depth_data.offset(i)) as f32);
                let mut y = y_lut * ((*depth_data.offset(i)) as f32);
                let mut z = (*depth_data.offset(i)) as f32;

                if let Some(world_calibration) = maybe_world_calibration {
                    x += world_calibration.translation_x;
                    y += world_calibration.translation_y;
                    z += world_calibration.translation_z;

                    let (x2, y2, z2) = world_calibration.rotate_point(x, y, z);

                    x = x2;
                    y = y2;
                    z = z2;
                }

                let color = (*color_data.offset(i)) as ColorCameraPoint;

                // The z-direction moves directly towards and away from the camera
                if command_args.depth_cull != 0
                    && z > command_args.depth_cull as f32 {
                    continue;
                }

                if command_args.left_cull != 0
                    && x > command_args.left_cull as f32 {
                    continue;
                }

                if command_args.right_cull != 0
                    && x < command_args.right_cull as f32 {
                    continue;
                }

                if command_args.xoff != 0 {
                    x += command_args.xoff as f32;
                }
                if command_args.yoff != 0 {
                    y += command_args.yoff as f32;
                }
                if command_args.zoff != 0 {
                    z += command_args.zoff as f32;
                }

                let point_color = Color::Custom {
                    r: color.r,
                    b: color.b,
                    g: color.g,
                    a: 255
                };

                points.push(Point::at(x, y, z, point_color));
            }
        }
    }

    if command_args.debug {
        // TODO/FIXME: Make this a constant allocation + append.
        for i in -1000 .. 1000 {
            points.push(Point::at(i as f32, 0.0, 0.0, Color::Red));
            points.push(Point::at(0.0, i as f32, 0.0, Color::Green));
            points.push(Point::at(0.0, 0.0, i as f32, Color::Blue));
        }
    }

    Ok(points)
}
