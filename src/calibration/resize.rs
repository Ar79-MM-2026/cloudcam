use kinect::Image;
use crate::AnyhowResult;
use resize::Type::{Lanczos3, Triangle};
use resize::Pixel;
use std::ptr;
use rgb::FromSlice;

pub fn resize_image(image: &Image, dest_width: usize, dest_height: usize) -> AnyhowResult<Image>
{
  let src_width = image.get_width_pixels();
  let src_height = image.get_height_pixels();

  let mut src = image.get_buffer();
  let mut src_slice = unsafe { std::slice::from_raw_parts_mut(src, src_width * src_height * 4) };

  // stride = cImg.cols * 4 * (int)sizeof(uint8_t)
  let stride_bytes = dest_width * 4;

  let dest_image = Image::create(
    image.get_format(),
    dest_width as u32,
    dest_height as u32,
    stride_bytes as u32).unwrap();

  let mut dst = dest_image.get_buffer();
  let mut dst_slice = unsafe { std::slice::from_raw_parts_mut(dst, dest_width * dest_height * 4) };

  let mut resizer = resize::new(
    src_width,
    src_height,
    dest_width,
    dest_height,
    Pixel::RGBA8,
    Triangle
  ).unwrap();

  resizer.resize(src_slice.as_rgba(), dst_slice.as_rgba_mut())
      .unwrap();

  Ok(dest_image)
}
