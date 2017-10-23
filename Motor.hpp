class Motor{
public:
	Motor(const char&,const char&,const char&,const char&);
	void stopMotor();
	void setPWM();
	int PWM;
private:
	const char& motorP1,motorP2,encoderP1,encoderP2;
};

Motor::Motor(const char &motorP1,const char &motorP2,const char &encoderP1,const char &encoderP2):motorP1(motorP1),motorP2(motorP2),encoderP1(encoderP1),encoderP2(encoderP2){
  gpioSetMode(motorP1,PI_OUTPUT);
  gpioSetMode(motorP2,PI_OUTPUT);
  gpioSetPWMfrequency(motorP1,50);
  gpioPWM(motorP1,0);
  gpioSetPWMfrequency(motorP2,50);
  gpioPWM(motorP2,0);

  gpioSetMode(encoderP1,PI_INPUT);
  gpioSetMode(encoderP2,PI_INPUT);
}

void Motor::stopMotor()
{
  gpioPWM(motorP1,0);
  gpioPWM(motorP2,0);
}

void Motor::setPWM()
{
  if(PWM < -255)
  {
    PWM = -255;
  }else if(PWM > 255)
  {
    PWM = 255;
  }
  if (PWM < 0)  {
    gpioPWM(motorP1,-PWM);
    gpioPWM(motorP2,0);
  }
  else {
    gpioPWM(motorP2,PWM);
    gpioPWM(motorP1,0);
  }
}