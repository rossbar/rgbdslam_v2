import numpy as np

# Timestamped transform that stores the rotation and translation info in a
# 3x4 RT matrix
stamped_transform_type = np.dtype([('timestamp', 'u8'),('RT', '(3,4)f4')])
# Translation type
translation_type = np.dtype([('x', 'f8'), ('y', 'f8'), ('z', 'f8')])
quaternion_type = np.dtype([('x', 'f8'), ('y', 'f8'), ('z', 'f8'), ('w', 'f8')])

#------------------------------ CLOUD TYPES -----------------------------------

# 3D info only, no colors
xyz_cloud_type = np.dtype([('x', 'f4'),('y', 'f4'),('z', 'f4')])

# Store 3D info and rgba info for display by opengl. Opengl expects color info
# in rgba with each color value a float between 0 and 1
glCldType = np.dtype({ 'names':['x', 'y', 'z', 'r', 'g', 'b', 'a'],\
                     'formats':[np.float32, np.float32, np.float32, np.float32,\
                               np.float32, np.float32, np.float32]})

# Store 3D info and color info as rgb. This is the format that the ros
# PointCloud2 message is in. the rgb values are expected to be unsigned ints 
# in the range [0, 255]
rosCldType = np.dtype({ 'names':['x', 'y', 'z', 'r', 'g', 'b'],\
                     'formats':[np.float32, np.float32, np.float32, np.uint8,\
                               np.uint8, np.uint8]})
