import numpy as np
import unittest
import os
from cv_utils import math_utils
from typing import List, Tuple


class MathUtilsTest(unittest.TestCase): 

    def setUp(self) -> None:
        return super().setUp() 

    def test_theta_phi_to_sphere(self):
        theta_phi_angles: List[Tuple[float, float]] = [
            (0,np.pi/2),
            (0, 0),
            (np.pi/2, 0),
            (np.pi/2, np.pi/2),
            ]
        sphere_expected_results: List[Tuple[float, float, float]] =np.array([
            (1,0,0), 
            (0,0,1), 
            (0,0,1),
            (0,1,0),
        ])
        for i, angles in enumerate(theta_phi_angles): 
            result = np.array(math_utils.theta_phi_to_sphere_point(angles[0], angles[1]))
            self.assertTrue(np.allclose(result, sphere_expected_results[i]), f"expected: {sphere_expected_results[i]} got: {result}")

    def test_angle_from_center(self,): 
        test_fov = np.pi/2
        test_angles_max_values_and_fovs = [
            (0,480,test_fov), 
            (240,480,test_fov), 
            (480,480,test_fov), 
            (360,480,test_fov), 
            (120,480,test_fov), 
        ]
        expected_angles = np.array([
            -test_fov/2, 
            0, 
            test_fov/2, 
            np.arctan((1/2)*np.tan(test_fov/2)),
            np.arctan((-1/2)*np.tan(test_fov/2)),
        ])
        for i, test_values in enumerate(test_angles_max_values_and_fovs): 
            result = np.array(math_utils.angle_from_center(test_values[0], test_values[1],test_values[2],))
            self.assertTrue(np.allclose(result, expected_angles[i]), f"expected: {expected_angles[i]} got: {result}")

    def test_theta_phi_from_image_point(self,): 
        
        test_image_points_and_fovs = [
            (0, 0, 640, 480, np.pi/2, np.pi/2), 
            (320, 0, 640, 480, np.pi/2, np.pi/2), 
            (0, 240, 640, 480, np.pi/2, np.pi/2), 
            (320, 240, 640, 480, np.pi/2, np.pi/2), 
            (640, 480, 640, 480, np.pi/2, np.pi/2), 
        ]

        expected_angles = np.array([
            (np.pi*(3/4), np.pi/4),
            (np.pi/2, np.pi/4),
            (np.pi*(3/4), np.pi/2),
            (np.pi/2, np.pi/2),
            (np.pi/4, np.pi*(3/4)),
        ])
        for i, test_values in enumerate(test_image_points_and_fovs): 
            x, y, num_pixels_in_x_axis, num_pixels_in_y_axis, fov_x, fov_y = test_values 
            result = np.array(math_utils.image_point_to_theta_phi(x, y, num_pixels_in_x_axis, num_pixels_in_y_axis, fov_x, fov_y))
            self.assertTrue(np.allclose(result, expected_angles[i]), f"expected: {expected_angles[i]} got: {result}")

    def test_edges_image_point_to_sphere_point(self,): 
        test_image_points_and_fovs = [ 
            (0, 0, 640, 480, np.pi, np.pi), 
            (640, 480, 640, 480, np.pi, np.pi), 
            (320, 240, 640, 480, np.pi, np.pi), 
            # (480, 120, 640, 480, np.pi/4, np.pi/2), 
        ]
        expected_sphere_points = [ 
            (0,0,1), 
            (0,0,-1),
            (0,1,0),
            # (0.18023995,0.9061274,0.3826834),
        ]
        for i, test_values in enumerate(test_image_points_and_fovs): 
            x, y, num_pixels_in_x_axis, num_pixels_in_y_axis, fov_x, fov_y = test_values 
            result = np.array(math_utils.image_point_to_sphere_point(x, y, num_pixels_in_x_axis, num_pixels_in_y_axis, fov_x, fov_y))
            self.assertTrue(np.allclose(result, expected_sphere_points[i]), f"expected: {expected_sphere_points[i]} got: {result}") 

    def test_image_point_to_sphere_point(self,): 
        test_h_fov = np.pi/4
        test_v_fov = np.pi/2

        test_image_points_and_fovs = [ 
            (480, 120, 640, 480, test_h_fov, test_v_fov), 
        ]
        # print(math_utils.image_point_to_theta_phi(480, 120, 640, 480, np.pi/4, np.pi/2))
        # print(math_utils.theta_phi_to_sphere_point(1.3744467859455345, 1.1780972450961724)) 
        # note: I didn't found a better way to test an interesting point without doing the exact calculation
        theta = np.pi/2 - np.arctan(0.5 * np.tan(test_h_fov/2))
        phi = np.pi/2 + np.arctan(-0.5 * np.tan(test_v_fov/2))
        expected_sphere_points = [ 
            math_utils.theta_phi_to_sphere_point(theta, phi),
        ]

        for i, test_values in enumerate(test_image_points_and_fovs): 
            x, y, num_pixels_in_x_axis, num_pixels_in_y_axis, fov_x, fov_y = test_values 
            result = np.array(math_utils.image_point_to_sphere_point(x, y, num_pixels_in_x_axis, num_pixels_in_y_axis, fov_x, fov_y))
            self.assertTrue(np.allclose(result, expected_sphere_points[i]), f"expected: {expected_sphere_points[i]} got: {result}") 

if __name__ == '__main__':
    unittest.main()

