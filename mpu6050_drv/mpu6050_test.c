#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>

#define PI 3.1415926
#define COUNT_ERROR 1000
#define THRESHOLD   0.12000
#define BINARY_COUNT  3000
#define DUTY_THRESHOLD 0.3

int16_t mpu6050data[12];
float mpu6050_trans_data[6];



typedef struct {
    float angle;   // 估计的角度
    float bias;    // 角速度偏置
    float P[2][2]; // 误差协方差矩阵
    float Q_angle; // 过程噪声（角度）
    float Q_bias;  // 过程噪声（偏置）
    float R_measure; // 测量噪声
} KalmanFilter;
typedef struct {
    float alpha;       // 滤波系数
    float prev_input;  // 上一次的输入值
    float prev_output; // 上一次的输出值
} HighPassFilter;

KalmanFilter kalman_yaw;
KalmanFilter kalman_pitch;
HighPassFilter hpf;
   

// 初始化卡尔曼滤波器
void Kalman_Init(KalmanFilter *kf) {
    kf->angle = 0.0;
    kf->bias = 0.0;
    kf->P[0][0] = 1.0; kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0; kf->P[1][1] = 1.0;
    kf->Q_angle = 0.001;
    kf->Q_bias = 0.003;
    kf->R_measure = 0.03;
}
/*
@param 	kf         是pitch或者roll的KalmanFilter结构体输入
		newAngle   是通过加速度测量出的角度
		newRate    角速度
		dt         离散时间间隔
*/

float kalmanFilterUpdate(KalmanFilter *kf, float newAngle, float newRate, float dt){

	// 预测步骤
    kf->angle += dt * (newRate - kf->bias);
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // 计算卡尔曼增益
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // 更新角度
    float y = newAngle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    // 更新误差协方差
    float P00_temp = kf->P[0][0], P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;

}


void angle_transform(float ax,float ay,float az,float *output_roll_angle,float *output_pitch_angle)
{
	// *output_roll_angle = atan(ay/az);//roll(x)
	// *output_pitch_angle = -atan(ax/sqrt(ay*ay+az*az));//pitch(y) 
    *output_roll_angle = atan2(ay, az);  // roll(x)
    *output_pitch_angle = -atan2(ax, sqrt(ay * ay + az * az)); // pitch(y)
}

float Scale_Transform(float Sample_Value, float URV, float LRV)
{
    float Data;             //定义用来保存变换后的数据变量
    float Value_L = -32767.0; //定义采样值下限变量   MPU6050寄存器是16位的，最高位是符号位，
    float Value_U = 32767.0;  //定义采样值上限变量   所以寄存器输出范围是-7FFF~7FFF,对应十进制-32767~32767
    
    /* 公式：当前数据 =（采样值 - 采样值下限）/（采样值上限 - 采样值下限）*（量程上限 - 量程下限）+ 量程下限     */
    Data = (Sample_Value - Value_L) / (Value_U - Value_L) * (URV - LRV) + LRV;
           
    return Data;
}

void trans_data(){

	mpu6050_trans_data[0] = Scale_Transform( (float)mpu6050data[0], 16.0, -16.0);  //转换X轴     acc加速度计数据
    mpu6050_trans_data[1] = Scale_Transform( (float)mpu6050data[1], 16.0, -16.0);  //转换Y轴
    mpu6050_trans_data[2] = Scale_Transform( (float)mpu6050data[2], 16.0, -16.0);  //转换Z轴

	mpu6050_trans_data[3] = Scale_Transform( (float)mpu6050data[3], 2000.0, -2000.0)*PI/180;  //转换X轴   gyro陀螺仪数据  并转换为rad/s
    mpu6050_trans_data[4] = Scale_Transform( (float)mpu6050data[4], 2000.0, -2000.0)*PI/180;  //转换Y轴
    mpu6050_trans_data[5] = Scale_Transform( (float)mpu6050data[5], 2000.0, -2000.0)*PI/180;  //转换Z轴

}
double getDeltaTime() {
    static struct timespec lastTime = {0, 0}; // 记录上一次时间
    struct timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);

    // 计算时间差 (dt)，单位为秒
    double dt = (currentTime.tv_sec - lastTime.tv_sec) + 
                (currentTime.tv_nsec - lastTime.tv_nsec) / 1e9;

    lastTime = currentTime; // 更新上次时间
    return dt;
}
int binaryQuantization(float src_data){//数据二值化

    if (fabsf(src_data) > THRESHOLD){
        return 1;
    }else{
        return 0;
    }
}
float caculateDutyCycle(int *data,int size){//计算占空比

    float duty;
    duty = 0;
    for(int i= 0;i < size;i++){
        duty = duty + data[i];
    }
    duty = duty/size;
    return duty;
}
bool fatigueJudgment(float data){
    int binary;
    bool judge;
    float duty;
    static int binary_arr[BINARY_COUNT];
    static int count = 0;
    binary_arr[count] = binaryQuantization(data);
    count = count + 1;
    if (count >= BINARY_COUNT){
        count = 0;
    }

    duty = caculateDutyCycle(binary_arr,BINARY_COUNT);
    if(duty > DUTY_THRESHOLD){
        printf("please stop !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
        judge = true;  //表示检测到车辆状态异常
    }

    judge = false;

    printf("binary is %d\r\n",binary_arr[count]);

    return judge;
}

// 初始化高通滤波器
void HighPassFilter_Init(HighPassFilter *filter, float alph_a) {

    filter->alpha = alph_a;
    printf("alfa is %f,",filter->alpha);
}
// 高通滤波计算
float HighPassFilter_Update(HighPassFilter *filter, float input) {
    float output = filter->alpha * (filter->prev_output + input - filter->prev_input);
    filter->prev_input = input;
    filter->prev_output = output;
    printf("input is %f,output is %f,",input,output);
    return output;
}

//********************************************** */
typedef struct {
    double roll;   // Roll角度（度）
    double pitch;  // Pitch角度（度）
} AngleData;


// 保存数据到文件（CSV格式）
void save_data(const char *filename, AngleData data) {
    FILE *file = fopen(filename, "a");  // 追加模式
    if (file == NULL) {
        printf("无法打开文件！\n");
        return;
    }
    fprintf(file, "%.6f, %.6f\n", data.roll, data.pitch);  // 格式：roll,pitch
    fclose(file);
}
//*****************************************888********************** */

int main(int argc,char **argv)
{
	int fd,err;
	fd = open("/dev/mpu6050",O_RDWR);
	if(fd == -1){
		printf("can not find the /dev/mpu6050\n");
		return -1;
	}

	Kalman_Init(&kalman_yaw);//应该是roll
	Kalman_Init(&kalman_pitch);
    HighPassFilter_Init(&hpf, 0.94f);
    hpf.prev_input = 0.0f;
    hpf.prev_output = 0.0f;
    
	float roll_angle_from_acc;
	float pitch_angle_from_acc;
	float roll_after_kalman;
	float pitch_after_kalman;
	float yaw_after_kalman;
	float distence_x;
	double dt;
	yaw_after_kalman = 0;
	distence_x = 0;

    int count_to_offset;
    count_to_offset = 0;
    float offset_yaw_rate;          //角速度零点偏移
    float offset_ax_rate;
    float last_yaw;                 //上一次的角度
    float last_ax;
    float yaw_error_accumulation;   //角速度累计零漂
    float ax_error_accumulation;
    float diff_de;                  //差分值
    yaw_error_accumulation = 0;
    ax_error_accumulation = 0;
    last_yaw = 0;
    last_ax = 0;
    diff_de = 0;
    float test_yaw;

    float yaw_test;


    //*********************************** */
    const char *filename = "angle_data.csv";

    // 创建文件并写入表头
    FILE *file = fopen(filename, "w");
    if (file != NULL) {
        fprintf(file, "Roll(deg), Pitch(deg)\n");  // CSV表头
        fclose(file);
    } else {
        printf("无法创建文件！\n");
        return -1;
    }

    printf("开始采集数据（保存到 %s）...\n", filename);
    AngleData data;
//********************************
    int i = 0;
	while(1){
		err = read(fd,mpu6050data,12);
		trans_data();
       
		angle_transform(mpu6050_trans_data[0],mpu6050_trans_data[1],mpu6050_trans_data[2],&roll_angle_from_acc,&pitch_angle_from_acc);
		dt = getDeltaTime();
        
       

		roll_after_kalman  = kalmanFilterUpdate(&kalman_yaw, roll_angle_from_acc, mpu6050_trans_data[3], dt);
		pitch_after_kalman = kalmanFilterUpdate(&kalman_pitch, pitch_angle_from_acc, mpu6050_trans_data[4], dt);
       

        if(count_to_offset <COUNT_ERROR){
            yaw_error_accumulation = yaw_error_accumulation + mpu6050_trans_data[5];
            ax_error_accumulation = ax_error_accumulation + mpu6050_trans_data[0];
            count_to_offset = count_to_offset + 1;
        }else if(count_to_offset == COUNT_ERROR){

            offset_yaw_rate = yaw_error_accumulation/COUNT_ERROR;
            offset_ax_rate = ax_error_accumulation/COUNT_ERROR;
            //yaw_after_kalman = -offset_yaw;
            count_to_offset = count_to_offset + 1;
        }else{
            //************************* */
            distence_x = distence_x + (mpu6050_trans_data[0] - offset_ax_rate)*dt;
	        yaw_after_kalman   = yaw_after_kalman + (mpu6050_trans_data[5] - offset_yaw_rate)*dt;
            yaw_test           = yaw_test         + (mpu6050_trans_data[5] - offset_yaw_rate)*dt;
            if(fabsf(mpu6050_trans_data[5] - offset_yaw_rate) < 0.0015){
                // printf("last %f ,yaw %f",last_yaw,yaw_after_kalman);
                yaw_after_kalman = last_yaw;
            }
            if(fabsf(mpu6050_trans_data[0] - offset_ax_rate) < 0.006){
                // printf("last %f ,yaw %f",last_yaw,yaw_after_kalman);
                distence_x = last_ax;
            }        
            diff_de = (yaw_after_kalman - last_yaw)/dt;  //差分表示他的变化率
            last_yaw = yaw_after_kalman;
            last_ax = distence_x;
            fatigueJudgment(diff_de);
            data.roll = yaw_test;
            data.pitch = yaw_after_kalman;
        }


		if(err ==0){
			// printf("the acc is %f ,%f, %f\r\n",mpu6050_trans_data[0],&mpu6050_trans_data[1],mpu6050_trans_data[2]);//加速度计数据
			// printf("the gyro is %f ,%f, %f\r\n",mpu6050_trans_data[3],&mpu6050_trans_data[4],mpu6050_trans_data[5]);//陀螺仪数据
			//printf("roll is %f ,pitch is %f,yaw is %f,distence is %f \r\n",mpu6050_trans_data[0] - offset_ax_rate,distence_x,yaw_after_kalman,diff_de);
			//printf("mpu6050_trans_data[5] is %f ,pu6050_trans_data[0] is %f \r\n",mpu6050_trans_data[5],mpu6050_trans_data[0]);
            //printf("bias roll is %f ,bias pitch is %f \r\n",kalman_yaw.bias,kalman_pitch.bias);
            
            //read_angles(&data.roll, &data.pitch);  // 模拟读取数据
            
            save_data(filename, data);              // 保存到文件
        
            // 打印到控制台（可选）
            printf("Sample %d: Roll=%.6f°, Pitch=%.6f°\n", 
               i+1, data.roll, data.pitch);
		}
	}
	

	close(fd);
	return 0;
	

}




