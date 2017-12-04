#include "pidControl.h"

/**** Get remote controller data***/
u8 Rx[32];
u16 rcYaw,rcMotor,rcRoll,rcPitch; //remote controller data
u16 getRcData(int le,int ri)
{
	u16 temp=0;
	temp|=Rx[ri];
	temp<<=8;
	temp|=Rx[le];
	return temp;
}

void readData()
{
	NRF24L01_RxPacket(Rx);
	rcYaw=getRcData(0,1);
	rcMotor=getRcData(2,3);
	rcRoll=getRcData(4,5);
	rcPitch=getRcData(6,7);
}

/****declare globol variable***/

typedef struct
{
	float shell_i;
	float old_e;
	float rc;
	float speed;
	float speed_metric;
	float old_speed;
	float shell_kp;
	float shell_ki;
	float shell_kd;
	float core_kp;
	float core_kd;
} pid;

typedef struct
{
	float motor1;
	float motor2;
	float motor3;
	float motor4;
} motor;

float Pitch_core_out;
float Yaw_core_out;
float Roll_core_out;

pid Pitch_PID;
pid Yaw_PID;
pid Roll_PID;

motor mo;

float PID(pid* p)
{
	float err = (p->rc) - (p->old_e);
	(p->shell_i) += err;
	float shell_out = p->shell_kp * err + p->shell_ki * p->shell_i + p->shell_kd * (err - p->old_e);
	p->old_e = err;
	float core_out = p->core_kp * (shell_out + p->speed * p->speed_metric) + p->core_kd * (p->speed - p->old_speed);
	p->old_speed = p->speed;
	return core_out;
}

void getMotor(u16 thr, motor* mo,float Pitch_core_out,float Yaw_core_out,float Roll_core_out)
{
	mo->motor1=(thr - Roll_core_out - Pitch_core_out- Yaw_core_out);
  mo->motor2=(thr + Roll_core_out - Pitch_core_out+ Yaw_core_out);
  mo->motor3=(thr + Roll_core_out + Pitch_core_out- Yaw_core_out);
  mo->motor4=(thr - Roll_core_out + Pitch_core_out+ Yaw_core_out);
}

void init()
{
	Pitch_PID.old_e=0;
	Pitch_PID.old_speed=0;
	Pitch_PID.shell_i=0;
	
	Yaw_PID.old_e=0;
	Yaw_PID.old_speed=0;
	Yaw_PID.shell_i=0;
	
	Roll_PID.old_e=0;
	Roll_PID.old_speed=0;
	Roll_PID.shell_i=0;
}

void updataData()
{
	Pitch_PID.rc = rcPitch;
	Yaw_PID.rc = rcYaw;
	Roll_PID.rc = rcRoll;
	
	Pitch_PID.speed = GETGYRO(IX);
	Yaw_PID.speed = GETGYRO(IZ);
	Roll_PID.speed = GETGYRO(IY);
}

void Control()
{
	readData();
	updataData();
	Pitch_core_out = PID(&Pitch_PID);
	Yaw_core_out = PID(&Yaw_PID);
	Roll_core_out = PID(&Roll_PID);
	getMotor(rcMotor,&mo,Pitch_core_out,Yaw_core_out,Roll_core_out);
}
