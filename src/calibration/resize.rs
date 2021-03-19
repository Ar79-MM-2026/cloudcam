use kinect::Image;
use crate::AnyhowResult;
use resize::Type::{Lanczos3, Triangle};
//use resize::Pixel::RGB8;
use resize::Pixel;
use std::ptr;
use rgb::FromSlice;

pub fn resize_image(image: &Image, dest_width: usize, dest_height: usize) -> AnyhowResult<Image>
{

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

  //let mut src = vec![RGB::new(0,0,0); src_width * src_height];
  let mut src = vec![0u8; src_width * src_height * 4];

  //let size = image.get_size();
  let mut buffer = image.get_buffer();

  let size_bytes = src_width * src_height * 4;
  unsafe {
    let vec_ptr = src.as_mut_ptr();
    //ptr::write_bytes(vec_ptr, color_src, size_bytes);
    ptr::copy_nonoverlapping(buffer, vec_ptr, size_bytes);
  }

  // cImg.cols * 4 * (int)sizeof(uint8_t)
  let stride_bytes = dest_width * 4;

  let dest_image = Image::create(
    image.get_format(),
    dest_width as u32,
    dest_height as u32,
    stride_bytes as u32).unwrap();

  let mut dst = dest_image.get_buffer();
  let mut dst_slice = unsafe { std::slice::from_raw_parts_mut(dst, dest_width * dest_height * 4) };


  //let mut resizer = resize::new(src_width, src_height, dest_width, dest_height, RGB8, Lanczos3)?;
  //resizer.resize(&src.as_rgb(), &dst.as_rgb_mut())?;

  /*// OLD VERSION -- WORKS
  resize::new(src_width, src_height, dest_width, dest_height, Pixel::RGB24, Triangle)
      .unwrap()
      .resize(&src, &mut dst_slice)
      .unwrap();*/

  let mut resizer = resize::new(
    src_width,
    src_height,
    dest_width,
    dest_height,
    Pixel::RGBA8,
    Triangle
  ).unwrap();

  resizer.resize(src.as_rgba(), dst_slice.as_rgba_mut())
      .unwrap();


  /*resize::new(src_width, src_height, dest_width, dest_height, Pixel::RGB8, Lanczos3)
      .unwrap()
      .resize(&src[..].as_rgb(), &mut dst[..].as_rgb_mut())
      .unwrap();*/

  Ok(dest_image)
}


