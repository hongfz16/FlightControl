#include "string.h"
#include "math.h"
#include "motor.h"
#include "24l01.h" 
#include "JY901.h"
#include "delay.h"

#define IX (0)
#define IY (1)
#define IZ (2)

extern struct SAcc baseAcc;
extern struct SGyro baseGyro;
extern struct SAngle baseAngle;

extern struct SAcc stcAcc;
extern struct SGyro	stcGyro;
extern struct SAngle stcAngle;

extern u8 nrfbuffer[268];

#define GETACC(x) (stcAcc.a[(x)]-baseAcc.a[(x)]) //accelerator
#define GETGYRO(x) (stcGyro.w[(x)]-baseGyro.w[(x)]) //angel speed
#define GETANGLE(x) (stcAngle.Angle[(x)]-baseAngle.Angle[(x)]) //angle

typedef struct
{
	float motor1;
	float motor2;
	float motor3;
	float motor4;
} motor;

u8 getifrun(void);

void readData(void);

void init(void);

void Control(void);

extern u8 Rx[32];
