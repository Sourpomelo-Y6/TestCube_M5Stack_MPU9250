struct gimp_image
{
  uint16_t    width;
  uint16_t    height;
  uint16_t    bytes_per_pixel; /* 2:RGB16, 3:RGB, 4:RGBA */ 
  uint8_t   pixel_data[135 * 135 * 2 + 1];
};
