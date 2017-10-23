#include  <math.h>
#include  <sys/time.h>

class Sensor{
public:
	float gyroY,gyroZ,accelX,accelY,accelZ;
	int HgyroY,LgyroY,HgyroZ,LgyroZ,HaccelX,LaccelX,HaccelZ,LaccelZ;
	float balanceAngle, balanceGyro;
	Sensor(const unsigned int&);
	void updateAttitude();
private:
	int fd;
	void KalmanFilter(const float&,const float&,const float&);
};


const float Q_ANGLE=0.001;// 过程噪声的协方差
const float Q_GYRO=0.003;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
const float R_ANGLE=0.5;// 测量噪声的协方差 既测量偏差
const float C_0=1;

Sensor::Sensor(const unsigned int& address){
	gpioInitialise();
	fd = i2cOpen (1,address,0);
	//i2cWriteByteData(fd,0x6B,0x82);//重置 关闭休眠 陀螺仪Y轴作为时钟源

	i2cWriteByteData(fd,0x1B,0x18);//Setting gyro sensitivity to +/- 2000 deg/sec
	i2cWriteByteData(fd,0x1C,0x00);//加速度计+/-2g 

	i2cWriteByteData(fd,0x19,0x04);//SMPLRT_DIV 采样率200Hz

	i2cWriteByteData(fd,0x1A,0x02);//REGISTER 26 – CONFIGURATION 配置 - DLPF Acc:94Hz Gyro:98Hz
	//i2cWriteByteData(fd,0x37,0x50);//REGISTER 55 - INT 引脚/旁路有效
	i2cWriteByteData(fd,0x38,0x00);//INTERRUPT ENABLE - 关闭中断

	i2cWriteByteData(fd,0x6B,0x02);//关闭休眠 陀螺仪Y轴作为时钟源
}
void Sensor::updateAttitude(){
	float t,Kalman_dt;

	static float last=gpioTick()/1000000;
	t=gpioTick();
	t=t/1000000;
	Kalman_dt=t-last;
	last = t;

	gyroY=(i2cReadByteData(fd,0x45)<<8)+i2cReadByteData(fd,0x46);    //读取Y轴陀螺仪
	gyroZ=(i2cReadByteData(fd,0x47)<<8)+i2cReadByteData(fd,0x48);    //读取Z轴陀螺仪
	accelX = (i2cReadByteData(fd,0x3B)<<8)+i2cReadByteData(fd,0x3C);			//读取X轴加速度计
	accelZ = (i2cReadByteData(fd,0x3F)<<8)+i2cReadByteData(fd,0x40);			//读取Z轴加速度计
	if(gyroY>32768)  gyroY-=65536;                       //数据类型转换  也可通过short强制类型转换
	if(gyroZ>32768)  gyroZ-=65536;                       //数据类型转换
	if(accelX>32768) accelX-=65536;                      //数据类型转换
	if(accelZ>32768) accelZ-=65536;											 //数据类型转换

//balanceGyro=-gyroY;                                  //更新平衡角速度

	accelY=atan2(accelX,accelZ)*180/3.1415926-1.5;                 //计算倾角
	gyroY/=-16.4;                                    //陀螺仪量程转换
	gyroZ/=16.4;
	gyroY+=1.2;
	gyroZ-=1;
	balanceGyro=gyroY;
	KalmanFilter(accelY,gyroY,Kalman_dt);  //更新平衡倾角   //卡尔曼滤波
	//Gyro_Turn=gyroZ;                                      //更新转向角速度
	//Acceleration_Z=accelZ;                                //===更新Z轴加速度计
}


const float Q_angle = 0.001f;
const float Q_bias = 0.003f;
const float R_measure = 0.03f;

void Sensor::KalmanFilter(const float& newAngle, const float& newRate,const float& dt)
{
	static float angle = 0.0f,bias = 0.0f,P[2][2]={0,0,0,0};

    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    float rate;
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    balanceAngle=angle;
}