import numpy as np
from tf import transformations as tf_trans

def transformation_to_euler_and_translation(transformation_matrix):
    """
    Convert a transformation matrix to Euler angles and translation vector.

    Args:
        transformation_matrix (numpy.ndarray): 4x4 transformation matrix.

    Returns:
        tuple: Tuple containing Euler angles (in radians) and translation vector.
    """
    # Extract rotation matrix from the transformation matrix
    rotation_matrix = transformation_matrix[:3, :3]

    # Compute Euler angles from the rotation matrix
    euler_angles = np.array([np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2]),  # Roll
                             np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2)),  # Pitch
                             np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])])  # Yaw

    # Extract translation vector from the transformation matrix
    translation_vector = transformation_matrix[:3, 3]

    return euler_angles, translation_vector

def tf_to_matrix(translation, rotation):
    """
    Convert ROS TF translation and rotation to a transformation matrix.

    Args:
        translation (list): List containing translation values [x, y, z].
        rotation (list): List containing quaternion [x, y, z, w].

    Returns:
        numpy.ndarray: Transformation matrix.
    """
    # Convert translation and rotation to homogeneous transformation matrix
    translation_matrix = tf_trans.translation_matrix(translation)
    rotation_matrix = tf_trans.quaternion_matrix(rotation)

    # Combine translation and rotation to get transformation matrix
    transformation_matrix = np.dot(translation_matrix, rotation_matrix)

    return transformation_matrix

# Example usage:
translation = [1.0, 2.0, 3.0]
rotation = [0.0, 0.0, 0.0, 1.0]  # Identity quaternion (no rotation)
transformation_matrix = tf_to_matrix(translation, rotation)
print("Transformation matrix:")
print(transformation_matrix)

# base_link -> camera_color_optical_frame
trans = [0, 0, 0]
rot = [-0.706825181105366, 0, 0, 0.7073882691671998]
base_link2camera_color_opotical_frame = tf_to_matrix(trans, rot)
print("base_link2camera_color_opotical_frame = ", base_link2camera_color_opotical_frame)

# camera_color_frame -> camera_color_opotical_frame
trans = [0, 0, 0]
rot = [-0.5, 0.5, -0.5, 0.5]
camera_color_frame2camera_color_opotical_frame = tf_to_matrix(trans, rot)
print("camera_color_frame2camera_color_opotical_frame = ", camera_color_frame2camera_color_opotical_frame)
print("camera_color_opotical_frame2camera_color_frame = ", np.linalg.inv(camera_color_frame2camera_color_opotical_frame))

# camera_link -> camera_color_frame
trans = [0.0018520707894105577, -0.025125320659075866, 0]
rot = [-0.001637385361476989, -0.001213267691936693, -0.0018902954390777236, 0.9999961368594574]
camera_link2camera_color_frame = tf_to_matrix(trans, rot)
print("camera_link2camera_color_frame = ", camera_link2camera_color_frame)
print("camera_color_frame2camera_link = ", np.linalg.inv(camera_link2camera_color_frame))


base_link2camera_link = base_link2camera_color_opotical_frame @ \
    np.linalg.inv(camera_color_frame2camera_color_opotical_frame) @ \
    np.linalg.inv(camera_link2camera_color_frame)
base_T_cam = base_link2camera_link
print("base_T_cam = ", base_T_cam)

euler_angles, translation_vector = transformation_to_euler_and_translation(base_T_cam)
print("Euler angles (radians):", euler_angles)
print("Translation vector:", translation_vector)
