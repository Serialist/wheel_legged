/*
 * File: leg_pos_1.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Nov-2024 21:22:04
 */

/* Include Files */
#include "leg_pos_1.h"
#include "arm_math.h"
#include <math.h>


/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */

float l5=0.15f;
float l1=0.15f;
float l2=0.25f;
float l3=0.25f;
float l4=0.15f;
float predict_dt = 0.0001f;
float phi2_w=0;
/*
 * LEG_POS_1
 *     POS = LEG_POS_1(PHI1,PHI4)
 *
 * Arguments    : double phi1
 *                double phi4
 *                double pos[2]
 * Return Type  : void
 */
void leg_pos_1(float phi1, float phi4, float pos[2])
{
float YD;
float YB;
float XD;
float XB;
float lBD;
float A0;
float B0;
float C0;
float phi2;
float XC;
float YC;
float L0;
float phi0;

XB = l1*cos(phi1); 
YB = l1*sin(phi1);
XD = l5 + l4*cos(phi4);
YD = l4*sin(phi4);

lBD = sqrt((XD - XB)*(XD - XB) + (YD - YB)*(YD - YB));
A0 = 2*l2*(XD - XB);
B0 = 2*l2*(YD - YB);
C0 = l2*l2 + lBD*lBD - l3*l3;
phi2 = (2*atan2((B0 + sqrt(A0*A0 + B0*B0 - C0*C0)),A0 + C0));
XC = l1*cos(phi1) + l2*cos(phi2);
YC = l1*sin(phi1) + l2*sin(phi2);
L0 = sqrt((XC - l5/2)*(XC - l5/2) + YC*YC);
phi0 = atan2(YC,XC - l5/2);
 float phi2_pred=(2*atan2((B0 + sqrt(A0*A0 + B0*B0 - C0*C0)),A0 + C0));
 phi2_w=(phi2_pred - phi2) / predict_dt;
  pos[0] = L0;
  pos[1] = phi0;
	
}

/*
 * File trailer for leg_pos_1.c
 *
 * [EOF]
 */
