#include "pidControl.h"

/**** Get remote controller data***/
u16 rcYaw,rcMotor,rcRoll,rcPitch; //remote controller data
u8 ifrun;
u16 getRcData(int le,int ri)
{
	u16 temp=0;
	temp|=Rx[ri];
	temp<<=8;
	temp|=Rx[le];
	return temp;
}

u8 getifrun(void)
{
	return ifrun;
}

u16 failcnt=0;

void readData(void)
{
	//NRF24L01_RxPacket(Rx);
	if(NRF24L01_RxPacket(Rx))
	{
		
		if(failcnt>=200) 
		{
			Rx[20]=0;
			return ;
		}
		++failcnt;
		
	}
	failcnt=0;
	rcYaw=getRcData(0,1);
	rcMotor=getRcData(2,3);
	rcRoll=getRcData(4,5);
	rcPitch=getRcData(6,7);

	rcMotor=4096-rcMotor;
//-----------
	ifrun=Rx[20];
}

/****declare globol variable***/

typedef struct
{
	float shell_i;
	float old_e;
	float rc;
	float state;
	float speed;
	float speed_metric;
	float old_speed_e;
	float shell_kp;
	float shell_ki;
	float shell_kd;
	float core_kp;
	float core_kd;
} pid;


float Pitch_core_out;
float Yaw_core_out;
float Roll_core_out;

pid Pitch_PID;
pid Yaw_PID;
pid Roll_PID;

extern motor mo;

//rc: 0 - 4096
//state: -32768 - 32767

float PID(pid* p)
{
	//p -> rc = p -> rc - 2048;
	//p -> state = p->state / 8;
	//p -> speed = p->speed / 8;
	
	float err = (p->rc) - (p->state);
	(p->shell_i) += err;
	p->shell_i*=0.9;
	if(p->shell_i > 100)
		p->shell_i=100;
	else if(p->shell_i <-100)
		p->shell_i=-100;
	if(p->shell_i*err<0)
		p->shell_i=0;
	float shell_out = p->shell_kp * err + p->shell_ki * p->shell_i - p->shell_kd * p->speed;//(err - p->old_e);
	p->old_e = err;
	float core_out = p->core_kp * (shell_out * p->speed_metric - p->speed) - p->core_kd * (shell_out * p->speed_metric - p->speed - p->old_speed_e);
	p->old_speed_e = shell_out * p->speed_metric - p->speed;
	//return core_out/5;
	shell_out/=5;
	if(shell_out>=300)
		shell_out=300;
	if(shell_out<=-300)
		shell_out=-300;
	return shell_out;
	
	core_out/=5;
	if(core_out>=300)
		core_out=300;
	if(core_out<=-300)
		core_out=-300;
	//return core_out;
}

//kp > kd      kp:kd = 10:1

float PID_YAW(pid* p)
{
	//p -> rc = p -> rc - 2048;
	//p -> speed = p->speed / 8;
	float err = p->rc -p->speed;
	(p->shell_i) += err;
	float core_out = p->shell_kp * err + p->shell_ki * p->shell_i - p->shell_kd * (err - p->old_e);
	p->old_e = err;
	return core_out/5;
}

void getMotor(u16 thr, motor* mo,float Pitch_core_out,float Yaw_core_out,float Roll_core_out)
{
	mo->motor1=(thr/5.12 + Roll_core_out - Pitch_core_out- Yaw_core_out);
  mo->motor2=(thr/5.12 - Roll_core_out - Pitch_core_out+ Yaw_core_out);
  mo->motor3=(thr/5.12 - Roll_core_out + Pitch_core_out- Yaw_core_out);
  mo->motor4=(thr/5.12 + Roll_core_out + Pitch_core_out+ Yaw_core_out);
	
	if(mo->motor1>999) mo->motor1=999;
	if(mo->motor2>999) mo->motor2=999;
	if(mo->motor3>999) mo->motor3=999;
	if(mo->motor4>999) mo->motor4=999;
	
	if(mo->motor1<0) mo->motor1=0;
	if(mo->motor2<0) mo->motor2=0;
	if(mo->motor3<0) mo->motor3=0;
	if(mo->motor4<0) mo->motor4=0;
	
	
}

void init()
{
	Pitch_PID.old_e=0;
	Pitch_PID.old_speed_e=0;
	Pitch_PID.shell_i=0;
	
	Yaw_PID.old_e=0;
	Yaw_PID.old_speed_e=0;
	Yaw_PID.shell_i=0;
	
	Roll_PID.old_e=0;
	Roll_PID.old_speed_e=0;
	Roll_PID.shell_i=0;
	
	//set metrics
	Pitch_PID.speed_metric=1;
	Pitch_PID.shell_kp=0.066;//0.064;
	Pitch_PID.shell_ki=0;//0.001;
	Pitch_PID.shell_kd=0.238;//0.238;//0.235;//0.23;;
	Pitch_PID.core_kp=0.9;
	Pitch_PID.core_kd=0.1;//0.03;
	
	Roll_PID.speed_metric=1;
	Roll_PID.shell_kp=0.066;//0.064;
	Roll_PID.shell_ki=0;//0.001;
	Roll_PID.shell_kd=-0.238;//-0.238;//-0.235;//-0.23;;
	Roll_PID.core_kp=-0.9;
	Roll_PID.core_kd=0.1;//0.03;
	
	Yaw_PID.shell_kp=-5;
	Yaw_PID.shell_ki=0;
	Yaw_PID.shell_kd=0;
	
	//104b
}

void updataData()
{
	Pitch_PID.rc = rcPitch-2077;
	Yaw_PID.rc = rcYaw-2105;//-2048;
	Roll_PID.rc = rcRoll-2080;
	
	Pitch_PID.state = GETANGLE(IX);//-275;//-460;
	Yaw_PID.state = GETANGLE(IZ) + baseAngle.Angle[IZ];
	Roll_PID.state = GETANGLE(IY);//-290;
	
	Pitch_PID.speed = GETGYRO(IX);
	Yaw_PID.speed = GETGYRO(IZ) + baseGyro.w[IZ];
	Roll_PID.speed = GETGYRO(IY);
}

void Control()
{
	readData();
	updataData();
	Pitch_core_out = PID(&Pitch_PID);
	Yaw_core_out = PID_YAW(&Yaw_PID);
	Roll_core_out = PID(&Roll_PID);
	
	//s16 pitch_int=Pitch_core_out;
	//u8 temp=0;
	//temp|=pitch_int;
	//nrfbuffer[24]=pitch_int>>8;
	//nrfbuffer[25]=temp;
	
	
	getMotor(rcMotor,&mo,Pitch_core_out,Yaw_core_out,Roll_core_out);
}
