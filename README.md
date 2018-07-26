# IMU_MPU6050_euler
Calibrating MPU6050 sensor to transform inertial coordinate to world coordinate using rotation matrix method.

Transformed acceleration and angular rate (rad/sec) are displayed on an OLED display.

I used VRomanov89's program to obtain the raw sensor data. https://github.com/VRomanov89/EEEnthusiast/blob/master/MPU-6050%20Implementation/MPU6050_Implementation/MPU6050_Implementation.ino

You may refer to the links below to understand the maths behind the transformation:
http://www.chrobotics.com/library/understanding-euler-angles
http://www.freepatentsonline.com/20130081442.pdf
