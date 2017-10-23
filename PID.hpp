#define BALANCE_K 0.6
#define BALANCE_KP BALANCE_K*30
#define BALANCE_KD BALANCE_K*0.2 

#define VELOCITY_KP 0.8//0.65
#define VELOCITY_KI VELOCITY_KP/200
#define VELOCITY_LIMIT 25000 //限幅

#define TURN_KP 2
#define TURN_KD 0.1
#define TURN_LIMIT 20

class PID{
public:
	static float getBalancePWM(const float&, const float&);
	static int getVelocityPWM(volatile int*,volatile int*,const int);
	static int getTurnPWM(const float& gyro,int);
};

float PID::getBalancePWM(const float& Angle, const float& Gyro){
	// 平衡环
	return BALANCE_KP * Angle +  Gyro * BALANCE_KD; //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
}
int PID::getVelocityPWM(volatile int* Encoder_Left,volatile int* Encoder_Right,const int Movement){
	// 速度环
	static float Velocity,Encoder_Least,Encoder;
	static float Encoder_Integral;

	Encoder_Least=(*Encoder_Left+*Encoder_Right)-0;//获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）

	*Encoder_Left=0;
	*Encoder_Right=0;

	Encoder*=0.7;//===一阶低通滤波器
	Encoder+=Encoder_Least*0.3;//===一阶低通滤波器
	Encoder_Integral+=Encoder;//===积分出位移积分时间：40ms
	Encoder_Integral=Encoder_Integral-(Movement);//Movement;//接收遥控器数据，控制前进后退

	if(Movement==0)
	{
		if (Encoder_Integral > 1000)   Encoder_Integral -= 800;
    	else if (Encoder_Integral < -1000)  Encoder_Integral += 800;
	}


	if(Encoder_Integral>VELOCITY_LIMIT)Encoder_Integral=VELOCITY_LIMIT;//积分限幅
	if(Encoder_Integral<-VELOCITY_LIMIT)Encoder_Integral=-VELOCITY_LIMIT;//限制遥控最大速度
	Velocity=Encoder*VELOCITY_KP+Encoder_Integral*VELOCITY_KI;//===速度控制
	//if(Turn_Off(Angle_Balance,Voltage)==1)Encoder_Integral=0;//电机关闭后清除积分
	return Velocity;
}
int PID::getTurnPWM(const float& gyro,int Turn_Target){
	// 转向环
	static float Turn, Turn_Convert = 3;
	if (Turn_Target > TURN_LIMIT)  Turn_Target = TURN_LIMIT; //===转向速度限幅
	if (Turn_Target < -TURN_LIMIT) Turn_Target = -TURN_LIMIT;
	Turn = Turn_Target * TURN_KP + gyro * TURN_KD;         //===结合Z轴陀螺仪进行PD控制
	return Turn;
}