# SSY345_NL_Filtering
Quaternion-based EKF for real-time smartphone orientation estimation using gyroscope, accelerometer, and magnetometer data in MATLAB.

# Smartphone Orientation Estimation using Sensor Fusion

This project implements a quaternion-based Extended Kalman Filter (EKF) for real-time orientation estimation using smartphone sensor measurements. Data is streamed from an Android device and processed in MATLAB using gyroscope, accelerometer, and magnetometer measurements.

The gyroscope is used for the time update, while the accelerometer and magnetometer are used as measurement updates to correct roll, pitch, and yaw drift. Static sensor data is used for calibration, including estimation of sensor means and covariance matrices. The filter also includes outlier rejection to reduce the effect of strong translational accelerations and magnetic disturbances.

## Main features

- Real-time smartphone sensor streaming to MATLAB
- Quaternion-based orientation representation
- Gyroscope bias compensation from static calibration
- EKF time update using angular velocity measurements
- Accelerometer update using gravity direction
- Magnetometer update using the local magnetic field
- Outlier rejection for accelerometer and magnetometer disturbances
- Comparison with the phone's built-in orientation estimate
