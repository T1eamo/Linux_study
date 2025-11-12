#ifndef MPU6050_DRV_H
#define MPU6050_DRV_H

#define CDEV_NAME "mpu6050"

#define MPU6050_RA_ACCEL_XOUT_H     0x3B    //加速度
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F

#define MPU6050_RA_GYRO_XOUT_H      0x43    //转速陀螺仪
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

#define	MPU6050_PWR_MGMT_1		0x6B     //电源管理1
#define	MPU6050_PWR_MGMT_2		0x6C     //电源管理2

#define	MPU6050_SMPLRT_DIV		0x19     //采样频率分频器
#define	MPU6050_CONFIG			0x1A     //配置

#define	MPU6050_GYRO_CONFIG		0x1B     //陀螺仪配置
#define	MPU6050_ACCEL_CONFIG	0x1C     //加速度计配置


#endif