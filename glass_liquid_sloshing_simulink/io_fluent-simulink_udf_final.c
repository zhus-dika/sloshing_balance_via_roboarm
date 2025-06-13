#include "udf.h"
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>
#include "sg.h"          /* даёт C_ADAPT_CELL и многое другое */
#include <math.h>        /* fmax() */
#include <stdio.h>
#include "sg.h"      /* C_CENTROID, C_VOLUME, C_R …  */
#include "mem.h"     /* PRF_GRSUM1 */


#define AMP_X   0.02    /* смещение по X, м */
#define AMP_Y   0.005   /* «прыг» по Y, м   */
#define ROT_AMP 0.10    /* наклон, рад      */
#define FREQ    0.5     /* Гц               */
#define TWO_PI (2.0*M_PI)


#define FLAG_IN  "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/pose.ok"
#define POSE_IN  "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/pose.dat"
#define FLAG_OUT "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/feed.ok"
#define FEED_OUT "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/feed.dat"

#define DYNA_ZONE_ID 1
#define OUTLET_ZONE_ID  6     /* ID переливной грани */
#define M_VESSEL 0.35                /* масса, кг          */

#define MAX_RETRY 100     /* макс. число попыток (100 × 1 сек = 100 сек) */
#define WAIT_US 1000000     /* 1 с ожидания между попытками */

/* глобальный указатель, чтобы потом использовать в других UDF */
static Dynamic_Thread *dt_vessel = NULL;

/* --- глобальные буферы: актуальная поза —> CG_MOTION --- */
real cx = 0, cy = 0, cz = 0, q[4] = {1, 0, 0, 0}, velocity[3] = {0, 0, 0}, omega[3] = {0, 0, 0};
/* --- статическая «память» прошлого кадра --- */
real t_prev = 0.0;

const real vessel_offset[3] = {0.0, 0.0, 0.0};

int pose_ready = 0;    /* 1 = в буфере лежит свежий кадр */

int flag_exists(const char *fname)
{
	struct stat st;
	return (stat(fname,&st)==0);
}

void remove_flag(const char *fname){ unlink(fname); }

static void quat_rotate(const real q[4], const real v[3], real out[3])
{
    real s=q[0], x=q[1], y=q[2], z=q[3];

    real t[3] = { 2*(y*v[2]-z*v[1]),
                  2*(z*v[0]-x*v[2]),
                  2*(x*v[1]-y*v[0]) };

    out[0] = v[0] + s*t[0] + (y*t[2]-z*t[1]);
    out[1] = v[1] + s*t[1] + (z*t[0]-x*t[2]);
    out[2] = v[2] + s*t[2] + (x*t[1]-y*t[0]);
}

DEFINE_CG_MOTION(cup_ext, dt, vel, om, time, dtime)
{
    DT_CG(dt)[0] = cx;  DT_CG(dt)[1] = cy;  DT_CG(dt)[2] = cz;
    DT_Q(dt)[0]  = q[0]; DT_Q(dt)[1] = q[1]; DT_Q(dt)[2] = q[2]; DT_Q(dt)[3] = q[3];
    NV_V(vel, =, velocity);
    NV_V(om,  =, omega);
}

DEFINE_ADJUST(read_pose_from_simulink, domain)
{
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
	//if (!I_AM_NODE_ZERO_P) return;
	
	real time = CURRENT_TIME;
	if (fabs(time - t_prev) < 1e-12)  return;
	
	t_prev = time;
	
	const real phase     = TWO_PI*FREQ*time;
	const real dphase_dt = TWO_PI*FREQ; 
	real q_loc[4], vel_loc[3], omega_loc[3]; 
	real cx_loc, cy_loc, cz_loc;
	real x_pos = AMP_X * sin(phase);
	real y_pos = AMP_Y * sin(phase);

	real ang_z = ROT_AMP * sin(phase);

	q_loc[0] = cos(0.5*ang_z);
	q_loc[1] = 0.0;
	q_loc[2] = 0.0;
	q_loc[3] = sin(0.5*ang_z);


	cx_loc = x_pos;
	cy_loc = y_pos;
	cz_loc = 0.0;
	
	vel_loc[0] = AMP_X * dphase_dt * cos(phase);
	vel_loc[1] = AMP_Y * dphase_dt * cos(phase);
	vel_loc[2] = 0.0;

	omega_loc[0] = 0.0;
	omega_loc[1] = 0.0;
	omega_loc[2] = ROT_AMP * dphase_dt * cos(phase);
	FILE *fp = fopen(POSE_IN,"w");
	if (!I_AM_NODE_ZERO_P) {
		if (fp) {
			fprintf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			    cx_loc, cy_loc, cz_loc, q_loc[0], q_loc[1], q_loc[2], q_loc[3], vel_loc[0], vel_loc[1], vel_loc[2], omega_loc[0], omega_loc[1], omega_loc[2]);
			Message("⚠️ POSE_IN is fully filled  (t = %.6e s)\n", time);
			fclose(fp);  
			FILE *ff = fopen(FLAG_IN, "w");
			if (ff) fclose(ff);
		}
	}
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
	
	int retry = 0;
	while (!flag_exists(FLAG_IN) && retry < MAX_RETRY) {
		usleep(WAIT_US);
		retry++;
	}
	
	if (flag_exists(FLAG_IN)) {
		if (!I_AM_NODE_ZERO_P) Message("⚠️ FLAG_IN found\n");
		FILE *fp = fopen(POSE_IN,"r");
		if (fp) {
			if (fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &cx,&cy,&cz,&q[0],&q[1],&q[2],&q[3],&velocity[0],&velocity[1],&velocity[2],&omega[0],&omega[1],&omega[2]) == 13) {
				if (!I_AM_NODE_ZERO_P) Message("⚠️ CG data are read\n");

			}
		}
	}
}



DEFINE_EXECUTE_AT_END(write_com_system)
{
    /* ---------- 1) масса и момент жидкости ---------- */
    Domain *d = Get_Domain(1);
    Thread *t; cell_t c;

    real m_fluid = 0.0;
    real M_r[3] = {0};

    thread_loop_c(t, d)
    {
        if (FLUID_THREAD_P(t))
        {
            begin_c_loop_all(c, t)
            {
                real vol  = C_VOLUME(c,t);
                real rho  = C_R(c,t);        /* плотность ячейки */
                real mass = rho * vol;

                real xc[ND_ND];
                C_CENTROID(xc,c,t);

                m_fluid  += mass;
                M_r[0]   += mass * xc[0];
                M_r[1]   += mass * xc[1];
                M_r[2]   += mass * xc[2];
            }
            end_c_loop_all(c, t)
        }
    }
    /* глобальные редукции – ВСЕ ранки должны дойти до этой точки */
    m_fluid  = PRF_GRSUM1(m_fluid);
    M_r[0]   = PRF_GRSUM1(M_r[0]);
    M_r[1]   = PRF_GRSUM1(M_r[1]);
    M_r[2]   = PRF_GRSUM1(M_r[2]);

    /* после редукций «лишние» ранки можем отпустить */
    if(!I_AM_NODE_ZERO_P) return;

    /* ---------- 2) COM жидкости ---------- */
    real com_fluid[3] = {0};
    if (m_fluid > 0) {
        com_fluid[0] = M_r[0]/m_fluid;
        com_fluid[1] = M_r[1]/m_fluid;
        com_fluid[2] = M_r[2]/m_fluid;
    }

    /* ---------- 3) COM пустого сосуда ---------- */
    real offset_world[3];
    quat_rotate(q, vessel_offset, offset_world);

    real com_vessel[3] = { cx + offset_world[0],
                           cy + offset_world[1],
                           cz + offset_world[2] };

    /* ---------- 4) общий COM системы ---------- */
    const real EPS = 1e-12;
    real m_total = M_VESSEL + m_fluid;
    if (m_total < EPS) return;      /* защита от деления на 0 */

    real com_total[3] = {
        (M_VESSEL*com_vessel[0] + m_fluid*com_fluid[0]) / m_total,
        (M_VESSEL*com_vessel[1] + m_fluid*com_fluid[1]) / m_total,
        (M_VESSEL*com_vessel[2] + m_fluid*com_fluid[2]) / m_total
    };

    /* ---------- 5) вывод для Simulink ---------- */
    FILE *fp = fopen("/path/to/feed.dat","w");
    if(fp){
        fprintf(fp, "%.6e %.6e %.6e %.6e\n",
                CURRENT_TIME, com_total[0], com_total[1], com_total[2]);
        fclose(fp);
        FILE *flag = fopen("/path/to/feed.ok","w"); if(flag) fclose(flag);
    }
}

