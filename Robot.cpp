#include <iostream>
#include <pigpio.h>
#include "PID.hpp"
#include "Motor.hpp"
#include "Sensor.hpp"
#include "Remote.hpp"

#define MOTOR_P2 22
#define MOTOR_P1 27
#define MOTOR_P3 23
#define MOTOR_P4 24
#define ENCODER_P2 12
#define ENCODER_P1 16
#define ENCODER_P3 20
#define ENCODER_P4 21

using namespace std;

volatile int encoderLeft,encoderRight;

void encoderLeftInterrupt(int,int,unsigned int){
	static short lastEncoded=0;
	short MSB = gpioRead(ENCODER_P1);
	short LSB = gpioRead(ENCODER_P2);

	short encoded = (MSB << 1) | LSB;
	short sum = (lastEncoded << 2) | encoded;

	if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderLeft++;
	if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderLeft--;

	lastEncoded = encoded;
}
void encoderRightInterrupt(int,int,unsigned int){
	static short lastEncoded=0;
	int MSB = gpioRead(ENCODER_P3);
	int LSB = gpioRead(ENCODER_P4);

	int encoded = (MSB << 1) | LSB;
	int sum = (lastEncoded << 2) | encoded;

	if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderRight++;
	if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderRight--;

	lastEncoded = encoded;
}

int main(){
	float pwmBalance=0;
	int pwmVelocity=0,pwmTurn=0;
	int countVelocity=0,countTurn=0;
	stringstream stream;
	gpioInitialise();
	Sensor MPU6050(0x68);
	gpioDelay(1000*5);
	Motor MotorL(MOTOR_P1,MOTOR_P2,ENCODER_P1,ENCODER_P2);
	Motor MotorR(MOTOR_P3,MOTOR_P4,ENCODER_P3,ENCODER_P4);
	// 启动远程控制线程
	gpioStartThread(Remote::Server,NULL);
	// 编码器中断注册
	gpioSetISRFunc(ENCODER_P1,EITHER_EDGE,1,encoderLeftInterrupt);
	gpioSetISRFunc(ENCODER_P2,EITHER_EDGE,1,encoderLeftInterrupt);
	gpioSetISRFunc(ENCODER_P3,EITHER_EDGE,1,encoderRightInterrupt);
	gpioSetISRFunc(ENCODER_P4,EITHER_EDGE,1,encoderRightInterrupt);
	while(1){
		unsigned int lastTime = gpioTick();
		unsigned int delayTime = 0;
		MPU6050.updateAttitude();

		pwmBalance = PID::getBalancePWM(MPU6050.balanceAngle,MPU6050.balanceGyro);

		if(++countVelocity >= 8){
			pwmVelocity = PID::getVelocityPWM(&encoderLeft,&encoderRight,Remote::data[0]);
			countVelocity = 0;
		}
		if (++countTurn >= 4){
			pwmTurn = PID::getTurnPWM(MPU6050.gyroZ,Remote::data[1]);
			countTurn = 0;
		}
		MotorL.PWM=pwmBalance + pwmVelocity + pwmTurn;
		MotorR.PWM=pwmBalance + pwmVelocity - pwmTurn;
		MotorL.setPWM();
		MotorR.setPWM();
		//cout << MotorL.PWM  << "," << MotorR.PWM   << "\t";
		//cout << encoderLeft << "," << encoderRight << "\t";
		cout << MPU6050.balanceAngle << ","<<MPU6050.balanceGyro << ","<< MPU6050.gyroZ<<"\t";
		cout << Remote::data[0] << "," << Remote::data[1] << "|" << endl;
		//cout << endl;

		if(MPU6050.balanceAngle < -50 || MPU6050.balanceAngle > 50){
			// 翻车检测
			MotorL.stopMotor();
			MotorR.stopMotor();
			break;
		}

		// 控制循环时间 5ms
		delayTime = gpioTick()-lastTime;
		delayTime = delayTime < 5000 ? 5000 - delayTime : 0;
		gpioDelay(delayTime);
	}
  	gpioTerminate();
	return 0;
}
