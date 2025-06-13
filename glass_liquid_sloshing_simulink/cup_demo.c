#include "udf.h"
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#define AMP_X   0.02    /* смещение по X, м */
#define AMP_Y   0.005   /* «прыг» по Y, м   */
#define ROT_AMP 0.10    /* наклон, рад      */
#define FREQ    0.5     /* Гц               */
#define TWO_PI (2.0*M_PI)

DEFINE_CG_MOTION(cup_demo, dt, vel, omega, time, dtime)
{
    //if (!I_AM_NODE_ZERO_P) return;
    const real phase     = TWO_PI*FREQ*time;
    const real dphase_dt = TWO_PI*FREQ;        /* 2πf */

    /* --- линейные смещения --- */
    real x_pos = AMP_X * sin(phase);
    real y_pos = AMP_Y * sin(phase);

    vel[0] = AMP_X * dphase_dt * cos(phase);
    vel[1] = AMP_Y * dphase_dt * cos(phase);
    vel[2] = 0.0;

    /* --- крен вокруг OZ --- */
    real ang_z = ROT_AMP * sin(phase);         /* θ(t) */
    omega[0] = 0.0;
    omega[1] = 0.0;
    omega[2] = ROT_AMP * dphase_dt * cos(phase);  /* dθ/dt */

    /* --- абсолютная ориентация (кватернион) --- */
    DT_Q(dt)[0] = cos(0.5*ang_z);   /* q0 */
    DT_Q(dt)[1] = 0.0;              /* q1 */
    DT_Q(dt)[2] = 0.0;              /* q2 */
    DT_Q(dt)[3] = sin(0.5*ang_z);   /* q3 */

    /* --- положение CG --- */
    DT_CG(dt)[0] = x_pos;
    DT_CG(dt)[1] = y_pos;
    DT_CG(dt)[2] = 0.0;
    Message("⚠️ From function DEFINE_CG_MOTION  (t = %.6e s)\n", time);
	    Message("pos = %.6lf %.6lf %.6lf | quat = %.6lf %.6lf %.6lf %.6lf | vel = %.6lf %.6lf %.6lf | omega = %.6lf %.6lf %.6lf\n", DT_CG(dt)[0], DT_CG(dt)[1], DT_CG(dt)[2], DT_Q(dt)[0], DT_Q(dt)[1], DT_Q(dt)[2], DT_Q(dt)[3], vel[0], vel[1], vel[2], omega[0], omega[1], omega[2]);
}

