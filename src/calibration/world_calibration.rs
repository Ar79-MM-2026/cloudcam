use byteorder::{ReadBytesExt, LittleEndian, BigEndian};
use crate::AnyhowResult;
use kinect::Device;
use regex::Regex;
use std::fs::File;
use std::io::{BufReader, BufRead};

/// This borrows from LiveScan3D's (MIT-licensed) calibration system.
/// LiveScan3D can be used to derive calibration files.
/// https://github.com/storytold/LiveScan3D
pub struct WorldCalibration {
}

impl WorldCalibration {

  pub fn load_from_file_and_serial(camera_serial_number: &str) -> AnyhowResult<Self> {
    let filename = format!("calibration_{}", camera_serial_number);

    let file = File::open(filename)?;
    let mut reader = BufReader::new(file);

    let translation_regex : Regex = Regex::new(r"^([\d\-\.]{2,10})\s([\d\-\.]{2,10})\s([\d\-\.]{2,10})\s*$")?;

    let mut string_buffer = String::new();
    reader.read_line(&mut string_buffer)?;

    let mut x = 0.0f32;
    let mut y = 0.0f32;
    let mut z = 0.0f32;

    if let Some(captures) = translation_regex.captures(&string_buffer) {
      if captures.len() == 4 {
        x = captures[1].parse()?;
        y = captures[2].parse()?;
        z = captures[3].parse()?;
      }
    }

    println!("x: {}", x);
    println!("y: {}", y);
    println!("z: {}", z);

    let rotation_regex : Regex = Regex::new(r"^(\-?\d\.\d+)(\-?\d\.\d+)(\-?\d\.\d+)\s*$")?;

    for _i in 0..3 {
      let mut string_buffer = String::new();
      reader.read_line(&mut string_buffer)?;

      if let Some(captures) = rotation_regex.captures(&string_buffer) {
        if captures.len() == 4 {
          x = captures[1].parse()?;
          y = captures[2].parse()?;
          z = captures[3].parse()?;
        }
      }

      println!("x: {}", x);
      println!("y: {}", y);
      println!("z: {}", z);
      println!("");
    }



    Ok(Self {
    })
  }

  pub fn load_from_file_and_device(device: &Device) -> AnyhowResult<Self> {
    let serial_number = device.get_serial_number()?;
    Self::load_from_file_and_serial(&serial_number)
  }
}

/*

Point3f RotatePoint(Point3f &point, std::vector<std::vector<float>> &R)
{
	Point3f res;

	res.X = point.X * R[0][0] + point.Y * R[0][1] + point.Z * R[0][2];
	res.Y = point.X * R[1][0] + point.Y * R[1][1] + point.Z * R[1][2];
	res.Z = point.X * R[2][0] + point.Y * R[2][1] + point.Z * R[2][2];

	return res;
}

 */

/*
		//As the resizing function doesn't return a valid RGB-Reserved value which indicates that this pixel is invalid,
		//we cut all vertices under a distance of 0.0001mm, as the invalid vertices always have a Z-Value of 0
		if (vertices[vertexIndex].Z >= 0.0001 && colorInDepth[vertexIndex].rgbReserved == 255)
		{
			Point3f temp = vertices[vertexIndex];
			RGB tempColor = colorInDepth[vertexIndex];
			if (calibration.bCalibrated)
			{
				temp.X += calibration.worldT[0];
				temp.Y += calibration.worldT[1];
				temp.Z += calibration.worldT[2];
				temp = RotatePoint(temp, calibration.worldR);

				if (temp.X < m_vBounds[0] || temp.X > m_vBounds[3]
					|| temp.Y < m_vBounds[1] || temp.Y > m_vBounds[4]
					|| temp.Z < m_vBounds[2] || temp.Z > m_vBounds[5])
				{
					AllVertices[vertexIndex] = invalidPoint;
					continue;
				}

			}

			AllVertices[vertexIndex] = temp;
			goodVerticesCount++;
		}
 */