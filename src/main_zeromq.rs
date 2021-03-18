//! Generate random points to send over ZeroMQ to a consumer

#![cfg_attr(debug_assertions, allow(dead_code, unused_imports))]
#[allow(unused_imports)]

#[macro_use] extern crate clap;

mod capture;
mod pointcloud;
mod setup;
mod zeromq;

use anyhow::Result as AnyhowResult;
use anyhow::anyhow;
use byteorder::{WriteBytesExt, LittleEndian, BigEndian};
use clap::Clap;
use k4a_sys_temp as k4a_sys;
use kinect::{Device, DeviceConfiguration, Image};
use std::io::Write;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use pointcloud::color::Color;
use pointcloud::cpu_calculate_point_cloud::PointCloudResult;
use pointcloud::cpu_calculate_point_cloud::calculate_point_cloud2;
use pointcloud::cpu_calculate_point_cloud::calculate_point_cloud3;
use pointcloud::cpu_transformation::DepthTransformer;
use pointcloud::point::Point;
use pointcloud::xy_table::create_xy_table;
use pointcloud::xy_table::create_xy_table_from_color_calibration;
use pointcloud::xy_table::create_xy_table_from_depth_calibration;
use zmq::{Error, Socket, Context, DONTWAIT};
use crate::zeromq::protocol::{encode_point_data, send_message};
use crate::setup::open_devices::open_and_configure_camera;
use crate::setup::camera_info::CameraInfo;
use crate::capture::multi_device_capturer::MultiDeviceCapturer;
use std::sync::{Arc, RwLock};


const SOCKET_ADDRESS : &'static str = "tcp://127.0.0.1:8888";
//const SOCKET_ADDRESS : &'static str = "tcp://192.168.50.3:8888";

/// The command line args for the program.
#[derive(Clap, Debug, Clone)]
pub struct CommandArgs {
  /// Number cameras
  #[clap(long, default_value = "1")]
  pub num_cameras: i32,

  /// Render axes
  #[clap(long, parse(try_from_str = true_or_false), default_value = "false")]
  pub debug: bool,

  /// Set a wide FOV in the depth camera
  #[clap(long, parse(try_from_str = true_or_false), default_value = "false")]
  pub wide: bool,

  /// Set the depth culling point
  /// A good value for my office with a NFOV camera approximately located
  /// parallel to where my desktop computer is located would be "3000"
  #[clap(long, default_value = "0")]
  pub depth_cull: i32,

  /// Cull everything to the right of this point
  /// "Right" is when the camera is inverted
  /// A value of "500" is reasonable
  #[clap(long, default_value = "0")]
  pub right_cull: i32,

  #[clap(long, default_value = "0")]
  pub left_cull: i32,

  /// X offset.
  #[clap(long, default_value = "0", allow_hyphen_values = true)]
  pub xoff: i32,

  /// Y offset.
  #[clap(long, default_value = "0", allow_hyphen_values = true)]
  pub yoff: i32,

  /// Z offset.
  #[clap(long, default_value = "0", allow_hyphen_values = true)]
  pub zoff: i32,
}

fn true_or_false(s: &str) -> Result<bool, &'static str> {
  match s {
    "true" => Ok(true),
    "false" => Ok(false),
    _ => Err("expected `true` or `false`"),
  }
}

enum MessagingState {
  Sending_PointDataBegin,
  Sending_PointDataContinue,
  GrabPointCloud,
}

fn main() -> AnyhowResult<()> {
  println!("Starting...");
  let args = CommandArgs::parse();

  println!("Command args: {:?}", args);

  println!("Opening device...");
  thread::sleep(Duration::from_millis(100));

  let mut devices = Vec::with_capacity(args.num_cameras as usize);
  let mut camera_infos: Vec<CameraInfo> = Vec::with_capacity(args.num_cameras as usize);

  println!("Starting cameras with supplied config...");
  for index in 0 .. args.num_cameras {
    thread::sleep(Duration::from_millis(100));

    let (device, camera_info) = open_and_configure_camera(&args, index as u32)?;

    device.start_cameras(&camera_info.config)?;

    devices.push(device);
    camera_infos.push(camera_info);
  }

  let multi_device_capturer = MultiDeviceCapturer::new(devices, true, true);

  thread::spawn(move || {
    multi_device_capturer.run_capture_loop()
        .expect("Capture thread errored mysteriously");
  });

  // ===============================================================

  loop {
    thread::sleep(Duration::from_millis(5000));
  }

  /*
  let context = zmq::Context::new();

  println!("Connecting ZeroMQ socket...");
  let mut socket = context.socket(zmq::PUSH).unwrap();
  //let mut socket = context.socket(zmq::REQ).unwrap();

  //socket.bind(SOCKET_ADDRESS).unwrap();
  socket.connect(SOCKET_ADDRESS).unwrap();

  let mut messaging_state = MessagingState::GrabPointCloud;

  let mut color = Color::get_time_based_color();

  let mut points : Vec<Point> = Vec::new();

  println!("Grabbing first frame...");
  loop {
    let maybe_points = get_point_cloud(&cameras, color, &args);
    points = match maybe_points {
      Err(e) => {
        println!("Error: {:?}", e);
        thread::sleep(Duration::from_millis(100));
        continue;
      },
      Ok(points) => points,
    };
    break;
  };

  println!("Main loop begin...");

  let mut packet_number = 0;
  let mut frame_number : u64 = 0;

  loop {
    match messaging_state {
      MessagingState::GrabPointCloud => {
        //println!("Grabbing another frame...");
        color = Color::get_time_based_color();
        points = match get_point_cloud(&cameras, color, &args) {
          Ok(p) => p,
          Err(e) => {
            thread::sleep(Duration::from_millis(50));
            continue
          },
        };

        messaging_state = MessagingState::Sending_PointDataBegin;
        frame_number += 1;
        continue;
      },
      MessagingState::Sending_PointDataBegin => {
        if frame_number == 1 || frame_number % 10 == 0 {
          println!("Sending frame number: {} (points: {})", frame_number, points.len());
        }

        let message = encode_point_data(&mut points, true);
        send_message(&socket, &message)?;

        if points.is_empty() {
          messaging_state = MessagingState::GrabPointCloud;
        } else {
          messaging_state = MessagingState::Sending_PointDataContinue;
        }
      },
      MessagingState::Sending_PointDataContinue => {
        // NB: Unfortunately the C++ program has difficulty with all points.
        if packet_number > 2000000 {
          println!("Packet elapsed");
          packet_number = 0;
          messaging_state = MessagingState::GrabPointCloud;
          continue;
        }

        //println!("Continue Sending PCD (points: {})", points.len());
        let message = encode_point_data(&mut points, false);
        send_message(&socket, &message)?;

        if points.is_empty() {
          messaging_state = MessagingState::GrabPointCloud;
        } else {
          messaging_state = MessagingState::Sending_PointDataContinue;
        }

        packet_number += 1;
      },
    }
  }
  */

  Ok(())
}

fn get_point_cloud(
  cameras: &Vec<CameraInfo>,
  color: Color,
  command_args: &CommandArgs
) -> AnyhowResult<Vec<Point>>
{
  let mut points = Vec::with_capacity(10000);

  /*let mut index = 0;
  for camera in cameras.iter() {
    let capture = camera.device.get_capture(5000)?;


    let depth_image = match capture.get_depth_image() {
      Some(img) => img,
      None => continue,
    };

    let color_image = match capture.get_color_image() {
      Some(img) => img,
      None => continue,
    };

    let depth_image2 = camera.transformer.transform(&depth_image)?;

    let mut camera_points =
        calculate_point_cloud3(
          &depth_image2,
          &camera.xy_table,
          //color,
          &color_image,
          command_args,
        )?;

    //camera_points.truncate(6000);

    println!("Points (camera {}): {}", index, camera_points.len());
    points.extend(camera_points);
    index += 1;
  }

  if points.is_empty() {
    return Err(anyhow!("No points captured!"));
  }*/

  Ok(points)
}

