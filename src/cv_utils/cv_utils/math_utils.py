from typing import Tuple
import numpy as np

''' 
i'm a not physicsist, i'm a mathematician!!!
theta is the angle from the x-axis (0<=theta<=2*pi)
phi is the angle from the z-axis (0<=phi<=pi)
'''


def theta_phi_to_sphere_point(theta: float, phi: float, theta_shift: float = 0, phi_shift: float = 0) -> Tuple[float, float, float]:
    '''
    lat, lon in radians! 
    returns the corresponding point on the 1-unit sphere 
    '''
    # x = -np.cos(theta) * np.sin(phi)
    # y = np.cos(theta) * np.cos(phi)
    # z = np.sin(theta)
    # theta_shift = -np.pi/2
    x = np.cos(theta+theta_shift) * np.sin(phi+phi_shift)
    y = np.sin(phi+phi_shift) * np.sin(theta+theta_shift)
    z = np.cos(phi+phi_shift)
    return (x, y, z)


def angle_from_center(value: int, max_value: int, fov: float) -> float:
    '''
    from pixel value in one axis and the num of pixels in that axis (max_value) and the fov angle of that axis
    return the angle of the value from the center of the image (in fov units)'''
    percentage_value = (value/max_value - 0.5)*2

    return np.arctan(percentage_value * np.tan(fov/2))


def image_point_to_theta_phi(x, y, image_size_x, image_size_y, fov_x, fov_y) -> Tuple[float, float]:
    '''
    get image an image point, it's shape, and it's FOVS 
    return theta, phi angles in radians 
    '''
    x_angle_from_center = angle_from_center(x, image_size_x, fov_x)
    y_angle_from_center = angle_from_center(y, image_size_y, fov_y)

    theta = -x_angle_from_center + np.pi/2
    phi = y_angle_from_center + np.pi/2
    return (theta, phi)


def image_point_to_sphere_point(x, y, image_size_x, image_size_y, fov_x, fov_y, theta_shift: float = 0, phi_shift: float = 0) -> Tuple[float, float, float]:
    theta, phi = image_point_to_theta_phi(
        x, y, image_size_x, image_size_y, fov_x, fov_y)
    return theta_phi_to_sphere_point(theta, phi, theta_shift, phi_shift)
