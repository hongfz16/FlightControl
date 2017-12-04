#include "string.h"
#include "math.h"
#include "motor.h"
#include "24l01.h" 
#include "JY901.h"
#include "delay.h"

#define IX (0)
#define IY (1)
#define IZ (2)

extern struct SAcc stcAcc;
extern struct SGyro	stcGyro;
extern struct SAngle stcAngle;

#define GETACC(x) (stcAcc.a[(x)]) //accelerator
#define GETGYRO(x) (stcGyro.w[(x)]) //angel speed
#define GETANGLE(x) (stcAngle.Angle[(x)]) //angle
