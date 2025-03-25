# RM-Sentry
This repo is for robomaster sentry, includes chassis, gimbal embedded development

Sentry_Chassis is code for the chassis and main gimbal, change it mode by modify the definition in main.h

Sentry_GimbalV2 is code for the vice gimbal



1. Sentry_Chassis 是给底盘和大云台控制的，一份代码给底盘A板与C板，要在main.h中修改宏定义注释：CHASSIS, MAIN_GIMBAL
2. Sentry_GimbalV2 是给两个小云台的，要在main.h中修改宏定义注释：VICE_GIMBAL_LEFT, VICE_GIMBAL_RIGHT
3. IMU角度：yaw正常，但是pitch电机用的是IMU roll数据，这是因为IMU安装问题，



3.26 

4:34 AM

修改了角度控制部分逻辑，角度控制现在包括基于机体坐标系的控制（添加完了逻辑，但PID没调完，正在调的rpmPID又出现抖动，如果有问题请回退到此版本之前最新一版）；基于IMU的角度控制（添加完新逻辑后，没有再做测试，主要因为角度控制是用于自瞄的，遥控器不再好做控制映射）
