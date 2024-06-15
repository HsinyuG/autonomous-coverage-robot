#!/usr/bin/env python
#!/usr/bin/env python3

import cv2
import numpy as np

def downsample_pgm_center_cv2(input_file, output_file):
  """
  Downsamples a PGM image from the center to a new resolution of 1 pixel
  and resizes it to 20x20 pixels using OpenCV (cv2).

  Args:
    input_file: Path to the input PGM file.
    output_file: Path to save the downsampled and resized image.
  """
  # Read the PGM image (assuming it's grayscale)
  image = cv2.imread(input_file, cv2.IMREAD_GRAYSCALE)

  if image is None:
    print(f"Error: Could not read image file: {input_file}")
    return

  # Get image height and width
  height, width = image.shape

#   # Calculate center coordinates
#   center_x = width // 2
#   center_y = height // 2

#   # Extract single pixel value from the center
#   downsampled_value = image[center_y, center_x]

#   # Create a new image with desired size and fill with the downsampled value
  new_image = 255 * np.ones((20, 20), dtype=np.uint8)

#   new_image = cv2.resize(image, (20, 20), interpolation = cv2.INTER_LINEAR)

  # Save the downsampled and resized image
  cv2.imwrite(output_file, new_image)

# Example usage
input_file = "1984.pgm"
output_file = "20.pgm"
downsample_pgm_center_cv2(input_file, output_file)

print(f"Downsampled PGM saved to: {output_file}")
