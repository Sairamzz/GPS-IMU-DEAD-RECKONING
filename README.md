# GPS-IMU-DEAD-RECKONING

This project was done as a part of lab assignment of the ME 5554 (Robotics Sensing and Navigation) course at Northeastern University.

* [Objective](#Objective)
* [Data Collection](#Data Collection)
* [Features](#Features)
* [Implementation](#Implementation)
* [Results](#Results)
  * [Actual path taken](#Actualpathtaken)
  * [GPS Path Trajectory](#GPSPathTrajectory)
  * [IMU Path Trajectory](#IMUPathTrajectory)
  * [Comparison of GPS+IMU](#ComparisonofGPS+IMU)
* [How to run](#Howtorun)


## Objective:

The objective of this project was to build a navigation stack using GPS and IMU data to estimate the trajectory of a moving vehicle. By combining measurements from a GPS puck and a VN-100 Inertial Measurement Unit (IMU), we aimed to achieve dead reckoning and compare its accuracy against GPS-only trajectories. The project focused on addressing the limitations of each sensor—GPS providing global accuracy but lower short-term resolution, and IMU offering high sensitivity but prone to drift—by applying filtering and calibration techniques to achieve robust trajectory estimation.

## Data Collection:

Data collection was performed using rosbags during two experiments: 
- Driving in circles
- Driving through Boston streets for approximately 1–2 km.

The IMU was mounted flat on the vehicle dashboard with proper alignment, while the GPS was positioned on the car roof. After collecting the rosbags, the data was converted to CSV format for data analysis using python.

## Features:

- The system integrated data from multiple sensing modalities, including GPS, accelerometer, gyroscope, and magnetometer. 
- Magnetometer readings were calibrated for hard-iron and soft-iron distortions using ellipse fitting and transformation, while yaw orientation was estimated through both magnetometer data and gyroscope integration.
- A complementary filter was implemented to combine these estimates, balancing the smoothness of gyroscope integration with the stability of magnetometer readings. Vehicle velocity was estimated from both GPS position changes and IMU acceleration data, with bias correction applied to reduce drift. These processed signals enabled the reconstruction of the vehicle’s trajectory using dead reckoning.

## Implementation:

Magnetometer calibration used ellipse fitting to correct distortions, while yaw estimates were derived from both magnetometer and gyroscope data. A complementary filter fused these signals to improve heading accuracy. Velocity estimates were derived from GPS positional changes and integrated IMU accelerations, with bias correction applied using stationary detection. Finally, the trajectory was reconstructed by combining velocity and orientation estimates, and compared against the GPS-derived path.

## Results:

### Actual path taken

<img width="710" height="765" alt="image" src="https://github.com/user-attachments/assets/7f9a3d8d-783b-44b6-bc18-54b109a2cfa7" />

### GPS Path Trajectory

<img width="859" height="547" alt="image" src="https://github.com/user-attachments/assets/c4c6425d-0fe5-405b-9f59-72c6bdb408f9" />

### IMU Path Trajectory

<img width="862" height="547" alt="image" src="https://github.com/user-attachments/assets/d7c7212f-724c-48ca-80ac-6dbcd4ab75f8" />

### Comparison of GPS+IMU

<img width="1014" height="547" alt="image" src="https://github.com/user-attachments/assets/5741ff05-60cd-410d-918d-4e1fbd95a033" />

The results demonstrated that GPS trajectories provided accurate long-term positioning, while IMU data excelled in capturing short-term orientation and small movements. The complementary filtered yaw closely matched the IMU’s orientation output, validating the effectiveness of sensor fusion. However, IMU-derived velocities exhibited drift and bias, which caused deviations in the reconstructed dead-reckoning trajectory compared to GPS. Despite these limitations, the IMU-based trajectory generally followed the shape of the GPS path, with errors increasing during longer drives and sharp turns. Overall, the project confirmed the strengths and weaknesses of GPS and IMU as standalone sensors, while highlighting their complementary nature. Future improvements could include implementing more advanced filtering techniques such as Kalman filters or extended sensor fusion frameworks to further improve trajectory estimation accuracy.

## How To Run:

### 1. Running the GPS Driver
``` bash
ros2 launch gps_driver gps_launch.py port:=/dev/ttyUSB*
```

### 1. Running the IMU Driver
``` bash
ros2 launch imu_driver imu_launch.py port:=/dev/ttyUSB*
```

