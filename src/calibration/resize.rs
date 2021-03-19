use kinect::Image;
use crate::AnyhowResult;
use resize::Type::{Lanczos3, Triangle};
//use resize::Pixel::RGB8;
use resize::Pixel;
use std::ptr;

pub fn resize_image(image: &Image, dest_width: usize, dest_height: usize) -> AnyhowResult<()> {

  /*
  //We need to resize the color image, so that it's height fits the depth camera height, while preserving the aspect ratio of the color camera:

  //Convert the k4a_image to an OpenCV Mat
  cv::Mat cImg = cv::Mat(k4a_image_get_height_pixels(colorImage), k4a_image_get_width_pixels(colorImage), CV_8UC4, k4a_image_get_buffer(colorImage));

  //Resize the k4a_image to the precalculated size. Takes quite along time, maybe there is a faster algorithm?
  cv::resize(cImg, cImg, cv::Size(colorImageDownscaledWidth, colorImageDownscaledHeight), cv::INTER_LINEAR);

  //Create a k4a_image from the resized OpenCV Mat. Code taken from here: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/978#issuecomment-566002061
  k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, cImg.cols, cImg.rows, cImg.cols * 4 * (int)sizeof(uint8_t), &colorImageDownscaled);
  memcpy(k4a_image_get_buffer(colorImageDownscaled), &cImg.ptr<cv::Vec4b>(0)[0], cImg.rows * cImg.cols * sizeof(cv::Vec4b));
  */

  let src_width = image.get_width_pixels();
  let src_height = image.get_height_pixels();

  // Don't forget to fill `src` with image data (RGB8).
  // This requires Vec<RGB<u8>>. If you have Vec<u8>, then use .as_rgb()/.as_rgb_mut() to reinterpret it as a slice of pixels.
  //let src = vec![RGB::new(0,0,0); w1*h1];

  //let mut src = vec![RGB::new(0,0,0); src_width * src_height];
  let mut src = vec![0u8; src_width * src_height * 3];

  //let size = image.get_size();
  let mut buffer = image.get_buffer();

  let size_bytes = src_width * src_height * 3;
  unsafe {
    let vec_ptr = src.as_mut_ptr();
    //ptr::write_bytes(vec_ptr, color_src, size_bytes);
    ptr::copy_nonoverlapping(buffer, vec_ptr, size_bytes);
  }


  // Destination buffer. Must be mutable.
  //let mut dst = vec![RGB::new(0,0,0); dst_width * dst_height];
  let mut dst = vec![0u8; dest_width * dest_height * 3];
  //let mut dst = vec![0u8; w2 * h2 * info.color_type.samples()];

  //let mut resizer = resize::new(src_width, src_height, dest_width, dest_height, RGB8, Lanczos3)?;
  //resizer.resize(&src.as_rgb(), &dst.as_rgb_mut())?;

  resize::new(src_width, src_height, dest_width, dest_height, Pixel::RGB24, Triangle).unwrap().resize(&src, &mut dst).unwrap();

  /*resize::new(src_width, src_height, dest_width, dest_height, Pixel::RGB8, Lanczos3)
      .unwrap()
      .resize(&src[..].as_rgb(), &mut dst[..].as_rgb_mut())
      .unwrap();*/

  Ok(())

}

