from PIL import Image
import numpy as np

# Create white rectangle (free space)
width, height = 155, 175
img = np.ones((height, width), dtype=np.uint8) * 255

# Add black border (walls) - 1 pixel thick (1 cm walls)
img[0, :] = 0   # top
img[-1, :] = 0  # bottom
img[:, 0] = 0   # left
img[:, -1] = 0  # right

# Save as PGM
Image.fromarray(img).save('small_rectangle.pgm')
print(f"Created {width}x{height} pixel map for 155x175 cm area")