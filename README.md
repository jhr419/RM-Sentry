# RM-Sentry
This repo is for robomaster sentry, includes chassis, gimbal embedded development

Sentry_Chassis is code for the chassis and main gimbal, change it mode by modify the definition in main.h

Sentry_GimbalV2 is code for the vice gimbal



1. Sentry_Chassis 是给底盘和大云台控制的，一份代码给底盘A板与C板，要在main.h中修改宏定义注释：CHASSIS, MAIN_GIMBAL
2. Sentry_GimbalV2 是给两个小云台的，要在main.h中修改宏定义注释：VICE_GIMBAL_LEFT, VICE_GIMBAL_RIGHT
3. IMU角度：yaw正常，但是pitch电机用的是IMU roll数据，这是因为IMU安装问题，
