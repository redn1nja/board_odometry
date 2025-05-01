# Onboard Odometry -- code for thesis 

Contains code for "Autonomous Navigation of Aerial Vehicles in GNSS-Denied Environments using Sensor Fusion" bachelor thesis.

Contains `libnavigation` library (`odometry` dir ), which contains the following classes:
- `Odometry` -- class for odometry estimation, with two inherited:
  - `ORBOdometry` -- class for odometry estimation using ORB features
  - `FlowOdometry` -- class for odometry estimation using optical flow
- `ImageCorrection` -- class for image correction that uses attitude from IMU to perspective transform the image into the image parallel to the plane
- `AccelOdometryFilter` -- class for filtering the accelerometer data using a Kalman filter
- `ImageProc` -- class for image processing, which combines the above classes and provides a simple interface for odometry estimation

There is also a submodule `external/madgwick` which contains the firmware for STM32 MCU of Madgwick filter for attitude estimation.

`local` directory contains code to interface with Gazebo/Ardupilot stack to create a sample data for the experiments.
`experiments` directory contains code to run the experiments as well as MUN-FRL dataset preprocessing ROS2 node.

Main experiment executable is `combined_tests` which runs the odometry estimation using ORB features as well as runs a Madgwick filter. 

```
./combined_tests <video_path> <raw_imu_path> <output_dir> 
```
The `video_path` is the path to the video file, `raw_imu_path` is the path to the raw IMU data file, and `output_dir` is the directory where the output files will be saved. The output files will contain the estimated odometry and attitude data.

For isolated Madgwick filter test, use `imu_tests` executable, source code for which is in `experiments directory`. 

`samples` dir contains `.csv` files that were used by the `analysis/experiments.ipynb` notebook to generate plots for the thesis.

videos and `.csv` files are stored on the external Google Drive. The link is available [here](https://drive.google.com/drive/folders/1QFJhDlLSO5Tnqq1jmmqQuTdOfHDxSDUK?usp=sharing).



