#include "udf.h"
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>
#include "sg.h"
#include <stdbool.h> 
#include <math.h>        /* fmax() */
#include <stdio.h>
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

#define ZONE_VESSEL_LIQ  15
#define WATER_PHASE_IDX   1      /* индекс water-фазы (см. Phases) */
/* порог VOF, чтобы ячейку считать «водой» */
#define VOF_THR  0.5
#define M_VESSEL 0.35                /* масса, кг          */

#define MAX_RETRY 100     /* макс. число попыток (100 × 1 сек = 100 сек) */
#define WAIT_US 1000000     /* 1 с ожидания между попытками */

/* --- глобальные буферы: актуальная поза —> CG_MOTION --- */
real cx = 0, cy = 0, cz = 0, q[4] = {1, 0, 0, 0}, velocity[3] = {0, 0, 0}, omega[3] = {0, 0, 0};
/* --- статическая «память» прошлого кадра --- */
real t_prev = 0.0;
static real pose[13];
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
    DT_CG(dt)[0] = cx;   DT_CG(dt)[1] = cy;   DT_CG(dt)[2] = cz;

    DT_Q(dt)[0]  = q[0]; DT_Q(dt)[1]  = q[1];
    DT_Q(dt)[2]  = q[2]; DT_Q(dt)[3]  = q[3];

    NV_V(vel, =, velocity);
    NV_V(om,  =, omega);

    if (I_AM_NODE_ZERO_P)
        Message("CG_MOTION t=%.4g  cx=%.4g\n", time, cx);
    /* никаких return;  хук должен отработать в КАЖДОМ процессе */
}

DEFINE_ADJUST(read_pose_from_simulink, domain)
{
	real time = CURRENT_TIME;
	if (fabs(time - t_prev) < 1e-12)  return;
	t_prev = time;

	/* -------- Node-0 читает файл pose.dat (или пишет демо-синус) -------- */
	if (I_AM_NODE_ZERO_P)
	{
		/* пример: «демка» синусом (как у тебя) */
		const real phase = TWO_PI*FREQ*time;
		pose[0] = AMP_X * sin(phase);                 /* cx */
		pose[1] = AMP_Y * sin(phase);                 /* cy */
		pose[2] = 0.0;                                /* cz */
		pose[3] = cos(0.5*ROT_AMP*sin(phase));        /* q0 */
		pose[4] = 0.0;                                /* q1 */
		pose[5] = 0.0;                                /* q2 */
		pose[6] = sin(0.5*ROT_AMP*sin(phase));        /* q3 */
		pose[7] = AMP_X*TWO_PI*FREQ*cos(phase);       /* vx */
		pose[8] = AMP_Y*TWO_PI*FREQ*cos(phase);       /* vy */
		pose[9] = 0.0;                                /* vz */
		pose[10]= 0.0; pose[11]=0.0;
		pose[12]= ROT_AMP*TWO_PI*FREQ*cos(phase);     /* ωz */

		FILE *fp = fopen(POSE_IN, "w");

		if (fp) {
			fprintf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			    pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], pose[7], pose[8], pose[9], pose[10], pose[11], pose[12]);
			Message("⚠️ POSE_IN is fully filled  (t = %.6e s)\n", time);
			fclose(fp);  
			FILE *ff = fopen(FLAG_IN, "w");
			if (ff) fclose(ff);
		}
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

		int retry = 0;
		while (!flag_exists(FLAG_IN) && retry < MAX_RETRY) {
			usleep(WAIT_US);
			retry++;
		}

		if (flag_exists(FLAG_IN)) {
			Message("⚠️ FLAG_IN found\n");
			FILE *fp = fopen(POSE_IN, "r");
			if (fp) {
				if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", 
				&pose[0], &pose[1], &pose[2], &pose[3], &pose[4], &pose[5], &pose[6], &pose[7], &pose[8], &pose[9], &pose[10], &pose[11], &pose[12]) == 13) {
					Message("⚠️ CG data are read\n");

				}
			}
		}
	}
	/* -------- 2.  Рассылаем всем ранкам -------- */
	PRF_BCAST_REAL(pose, 13, 0);

	/* -------- 3.  Копируем в глобальные переменные -------- */
	cx = pose[0];  cy = pose[1];  cz = pose[2];
	memcpy(q,        &pose[3], 4*sizeof(real));
	memcpy(velocity, &pose[7], 3*sizeof(real));
	memcpy(omega,    &pose[10],3*sizeof(real));

}

/* -------------------------------------------------------------- */
DEFINE_EXECUTE_AT_END(write_com_system_spill)
{
	Domain *d = Get_Domain(1);
	Thread *t;  cell_t c;

	real m_fluid = 0.0;          /* масса воды            */
	real M_r[3] = {0.0,0.0,0.0}; /* ∑ m · x               */
	real vol_spill = 0.0;        /* объём разлива         */

	/* --- проходим все fluid-треды -------------------------------- */
	thread_loop_c(t, d)
	{
		if (!FLUID_THREAD_P(t)) continue;

		Thread *t_water = THREAD_SUB_THREAD(t, WATER_PHASE_IDX);
		if (!t_water) continue;               /* в зоне нет water */

		const int inside = (THREAD_ID(t) == ZONE_VESSEL_LIQ);

		begin_c_loop_all(c, t)
		{
			real vf = C_VOF(c, t_water);      /* <-- только water-тред */
			//if (vf > 0.0 && THREAD_ID(t)!=ZONE_VESSEL_LIQ)
				//Message("rank %d, cell %ld, vf = %.3e, vol = %.3e\n", myid, c, vf, C_VOLUME(c,t));

			if (vf < VOF_THR) continue;

			real vol  = C_VOLUME(c, t);       /* объём ячейки */
			real rho  = C_R(c, t_water);      /* плотность water*vf */
			real mass = rho * vol;

			real xc[ND_ND]; C_CENTROID(xc,c,t);

			m_fluid  += mass;
			M_r[0]   += mass*xc[0];
			M_r[1]   += mass*xc[1];
			M_r[2]   += mass*xc[2];

			if (!inside)
				vol_spill += vf * vol;        /* пролив */
		}
		end_c_loop_all(c,t)
	}

	/* --- MPI-редукции ------------------------------------------- */
	m_fluid   = PRF_GRSUM1(m_fluid);
	M_r[0]    = PRF_GRSUM1(M_r[0]);
	M_r[1]    = PRF_GRSUM1(M_r[1]);
	M_r[2]    = PRF_GRSUM1(M_r[2]);
	vol_spill = PRF_GRSUM1(vol_spill);

	/* --- файл пишет только node-0 ------------------------------- */
	if (!I_AM_NODE_ZERO_P) return;

	/* COM жидкости */
	real com_f[3] = {0};
	if (m_fluid > 0.0) {
		com_f[0] = M_r[0]/m_fluid;
		com_f[1] = M_r[1]/m_fluid;
		com_f[2] = M_r[2]/m_fluid;
	}

	/* COM пустого сосуда */
	real off_w[3];  quat_rotate(q, vessel_offset, off_w);
	real com_v[3] = { cx+off_w[0], cy+off_w[1], cz+off_w[2] };

	/* COM системы «сосуд+жидкость» */
	const real m_tot = M_VESSEL + m_fluid;
	if (m_tot <= 1e-9) return;          /* защита */

	real com_tot[3] = {
		(M_VESSEL*com_v[0] + m_fluid*com_f[0]) / m_tot,
		(M_VESSEL*com_v[1] + m_fluid*com_f[1]) / m_tot,
		(M_VESSEL*com_v[2] + m_fluid*com_f[2]) / m_tot
	};

	/* --- вывод -------------------------------------------------- */
	FILE *fp = fopen(FEED_OUT,"w");
	if (fp) {
		fprintf(fp,"%.6e %.6e %.6e %.6e %.6e\n",
			CURRENT_TIME, com_tot[0], com_tot[1], com_tot[2],
			vol_spill);
	fclose(fp);
	FILE *flag = fopen(FLAG_OUT,"w"); if(flag) fclose(flag);
	Message("✅ feed.dat written (t = %.4e, vol_spill = %.4e))\n", CURRENT_TIME, vol_spill);
	}
}

