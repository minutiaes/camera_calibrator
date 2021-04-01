#!/usr/bin/env python3
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
import yaml


class CameraCalibrator(Node):
    def __init__(self):
        super().__init__("camera_calibrator")
        self.declare_parameters(
            namespace='',
            parameters=[
                ("camera_source", None),
                ("frame_width", None),
                ("frame_height", None),
                ("camera_fps", None),
                ("horizontal_corner", None),
                ("vertical_corner", None),
                ("square_size", None),
                ("sample_no", None),
                ("delay", None)
            ])
        self.camera_source = self.get_parameter(name="camera_source").get_parameter_value().integer_value
        self.frame_width = self.get_parameter(name="frame_width").get_parameter_value().integer_value
        self.frame_height = self.get_parameter(name="frame_height").get_parameter_value().integer_value
        self.camera_fps = self.get_parameter(name="camera_fps").get_parameter_value().integer_value
        self.corner_h = self.get_parameter(name="horizontal_corner").get_parameter_value().integer_value
        self.corner_v = self.get_parameter(name="vertical_corner").get_parameter_value().integer_value
        self.square_size = self.get_parameter(name="square_size").get_parameter_value().integer_value
        self.sample_no = self.get_parameter(name="sample_no").get_parameter_value().integer_value
        self.delay = self.get_parameter(name="delay").get_parameter_value().integer_value

        # termination criteria
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((self.corner_h * self.corner_v, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.corner_h, 0:self.corner_v].T.reshape(-1, 2) * self.square_size
        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints = []  # 2d points in image plane.
        self.time_done = 0

        self.camera_calibrator()

    def dump_yaml(self, frame, mtx, dist):
        output = {
            "image_width": frame.shape[1],
            "image_height": frame.shape[0],
            "camera_source": self.camera_source,
            "camera_fps": self.camera_fps,
            "camera_name": "camera",
            "camera_matrix": {"rows": 3, "cols": 3, "data": np.array(np.concatenate(mtx)).tolist()},
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {"rows": 1, "cols": 5, "data": np.array(np.concatenate(dist)).tolist()},
        }
        home_dir = str(Path.home())
        try:
            with open(home_dir+r'/camera.yaml', 'w+') as f:
                yaml.dump(output, f, sort_keys=False, default_flow_style=None, default_style=None, Dumper=yaml.Dumper)
        except IOError as err:
            self.get_logger().info(f"Error: {err}")
        self.get_logger().info(f"camera.yaml file is created in {home_dir}\nPress CTRL+C to exit")
    def camera_calibrator(self):
        cap = cv.VideoCapture(self.camera_source)
        cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        cap.set(cv.CAP_PROP_FPS, self.camera_fps)
        counter = 0

        while True:
            ret, frame = cap.read()

            if ret:
                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                # Find the chess board corners
                ret, corners = cv.findChessboardCorners(gray, (self.corner_h, self.corner_v), None)
                
                if ret:
                    corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                    # Draw and display the corners
                    cv.drawChessboardCorners(frame, (self.corner_h, self.corner_v), corners2, ret)

                    time_current = time.time()
                    time_left = self.delay - (time_current - self.time_done)
                    time_left = "{:.2f}".format(time_left)
                    label_size, base_line = cv.getTextSize(time_left, cv.FONT_HERSHEY_SIMPLEX, 2, 5)
                    cv.putText(frame, time_left, (base_line, label_size[1]+base_line), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 255), 5)

                    if time_current - self.time_done > self.delay:
                        self.objpoints.append(self.objp)
                        self.imgpoints.append(corners)
                        counter += 1
                        self.get_logger().info(f"Sample no: {counter}")
                        self.time_done = time.time()
            cv.imshow('img', frame)

            if cv.waitKey(1) & 0xFF == ord('q') or counter == self.sample_no:
                break

        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None, None,
                                                        criteria=self.criteria)
        self.get_logger().info(f"\nmtx: \n{mtx}")
        self.get_logger().info(f"\ndist: \n{dist}")
        self.dump_yaml(frame, mtx, dist)
        cv.destroyAllWindows()


def main():
    rclpy.init()
    camera_calibrator = CameraCalibrator()

    try:
        rclpy.spin(camera_calibrator)
    except KeyboardInterrupt:
        pass
    finally:
        camera_calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
