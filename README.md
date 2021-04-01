# Simple Camera Calibrator
A standalone camera calibration node for ROS2. Unlike official package, it does not involve any highgui function of OpenCV which are not fully supported in [OpenCV on Wheels](https://pypi.org/project/opencv-python/).

## Features
- Interactive calibration
- Exporting necessary configuration file for [camera_rgb](https://git.fh-aachen.de/kurhan/camera_rgb) node
- All necessary parameters are stored in [Launch File](https://git.fh-aachen.de/kurhan/camera_calibrator#launch-file)

## Getting Started
### Dependencies
* OpenCV  
To install:
  ```sh
  $ python3 -m pip install opencv-python
  ```
* PyYAML  
To install:
  ```sh
  $ python3 -m pip install pyyaml
  ```
  When you run the node, if you still get an error about PyYAML, check its version
  ```sh
  $ python3
  > import yaml
  > yaml.__version__
  ```
  If it is smaller than `5.1`, you should update it  
  ```sh
  $ python3 -m pip install -U pyyaml
  ```
### Installation
1. Clone repository to `/your_workspace/src`
    ```sh
    $ git clone https://git.fh-aachen.de/kurhan/camera_calibrator.git
    ```
2. Build `/your_workspace` with `colcon`
   ```sh
   $ colcon build
   ```

## Usage
### Launch File 
It contains parameters under 3 topic.
1. Camera  
   * `camera_source` holds index number of camera. If there is only one camera on the system, its index is probably `0`.   
   If it throws an error, you can check it
     ```sh
     $ ls -l /dev/video*
     ```
     
   * `frame_width` and `frame_height` set camera resolution, e.g. 1920x1080 or 1280x720  
   To check possible resolution, `v4l2-utils` is required.
     ```sh
     $ sudo apt install v4l2-utils
     ```
     ```sh
     $ v4l2-ctl --list-formats-ext
     ```
   
   * `camera_fps` sets camera frame rate. Possible values can be seen with same command above. However this setting may not work with every camera.
2. Checkerboard
   * `horizontal_corner` and `vertical_corner` are set according to inner corners of chekerboard like below.
   
   <img src="https://markhedleyjones.com/storage/logos/calibration-checkerboard-collection.svg" alt="Checkerboard" width="400"/>
   
   In this example `horizontal_corner: 8` and `vertical_corner: 6`
   * `square_size` is set according to edge length[mm] of squares on checkerboard.
3. Calibration
   * `sample_no` designates desired number of snapshots for calibration process. 
   * `delay` set time gap[sec] between two snapshots. In the meantime desired pose of checkerboard can be set.

### How to Run Node
After setting all parameters in Launch File properly,
```sh
$ ros2 launch camera_rgb camera_rgb.launch.py
```
starts the node and camera window pop ups 

## Exported Configuration File
The node exports a YAML file for [camera_rgb](https://git.fh-aachen.de/kurhan/camera_rgb). The YAML file contains matrices for calibration and some more additional parameters which are already explained above as [Launch File](https://git.fh-aachen.de/kurhan/camera_calibrator#launch-file) parameters.
* After calculation of calibration parameters, `camera.yaml` file is exported to `home` directory of user
* `image_width` and `image_height` are equivalent of `frame_width` and `frame_height`
* `camera_source` and `camera_fps` share same name and value in both files.
* `camera_name` is useful when there are multiple cameras on the system.
* `camera_matrix` is a unique result derived from calculations done on checkerboard. $`f_x`$ and $`f_y`$ are focal lengths and $`c_x`$ and $`c_y`$ are optical centers
```math
camera~intrinsic~matrix = \begin{bmatrix}
f_x & 0 & c_x\\
0 & f_y & c_y\\
0 & 0 & 1
\end{bmatrix}
```
* `distortion_model` is set to `plumb bob` by default. This model requires `1x5` distrotion matrix.
* `distortion_coefficients` contains 5 coefficients.
```math
distortion~coefficients = \begin{bmatrix}
k_1 & 0 & k_2 & p_1 & p_2 & k_3
\end{bmatrix}
```
  For more info you can check OpenCV [tutorial](https://docs.opencv.org/master/d4/d94/tutorial_camera_calibration.html) and [documentation](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d)
