#include "motor_ctrl.h"
#include "tim.h"
#include "speed.h"
#include "transmit.h"

#define MAX_OUTPUT 1000

uint8_t positive[4];


int16_t X_expect;
int16_t Y_expect;
int16_t Z_expect;
int16_t height_expect;
float float_height;

remote_t remote;

float kp=0.5f;
float ki=1.0f;
float kd=0.0f;
	

pid_t motor[4];


void motor_ctrl_at_TB6612()
{
	static uint8_t i;
	//下方为期望值到速度的转换
	
	X_expect=-(remote.rocker[1]-2050)/100;
	Y_expect=(remote.rocker[0]-2050)/100;
	Z_expect=(remote.rocker[2]-2120)/100;
	height_expect=-(remote.rocker[3]-2050)/100;
	float_height+=0.005*height_expect;
	
	if(float_height<10.0) float_height=10.1;
	else if(float_height>40) float_height=39.9;
	
	main_board.servo_mode_and_height=(uint8_t)float_height;


	if(OK)
	{
		motor[0].expect=(int16_t)(-1.5*X_expect+1*Y_expect-1*Z_expect);
		motor[1].expect=(int16_t)(1.5*X_expect+1*Y_expect-1*Z_expect);
		motor[2].expect=(int16_t)(1.5*X_expect-1*Y_expect-1*Z_expect);
		motor[3].expect=(int16_t)(-1.5*X_expect-1*Y_expect-1*Z_expect);
	}
	
	for(i=0;i<4;i++)
	{
		motor[i].current=motor_speed[i];
		PID_Cal(&motor[i]);
	}
	
	//下方为基础的速度控制函数
	for(i=0;i<4;++i)
	{
		if(motor[i].output>0)
			positive[i]=1;
		else
		{
			positive[i]=0;
		}
	}

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,positive[0]);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,!positive[0]);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,positive[1]);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,!positive[1]);
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,positive[2]);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,!positive[2]);
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,positive[3]);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,!positive[3]);	
	
	if(positive[0])
		TIM3->CCR1=motor[0].output;
	else
		TIM3->CCR1=-motor[0].output;
	
	if(positive[1])
		TIM3->CCR2=motor[1].output;
	else
		TIM3->CCR2=-motor[1].output;
	
	if(positive[2])
		TIM3->CCR3=motor[2].output;
	else
		TIM3->CCR3=-motor[2].output;
	
	if(positive[3])
		TIM3->CCR4=motor[3].output;
	else
		TIM3->CCR4=-motor[3].output;
	

}

void PID_Init()
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
		motor[i].Error_last=0.0f;
		motor[i].Error_prev=0.0f;
	}
}

void PID_Cal(pid_t* T) //增量式
{
	short Error = T->expect - T->current;
	short pwm_add=0;
	pwm_add = kp*(Error - T->Error_last) + ki*Error + kd*(Error-2.0f*T->Error_last+T->Error_prev);
	T->I+=ki*Error;
	
	T->Error_prev = T->Error_last;	  	    // 保存上上次误差
  T->Error_last = Error;	              // 保存上次偏差
	
	T->output+=(int16_t)pwm_add;
	
	if(T->output>MAX_OUTPUT) T->output=MAX_OUTPUT;
	else if(T->output<-MAX_OUTPUT) T->output=-MAX_OUTPUT;
}