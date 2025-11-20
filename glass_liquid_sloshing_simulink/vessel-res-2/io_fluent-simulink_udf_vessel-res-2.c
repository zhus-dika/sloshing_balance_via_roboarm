/*--------------------------------------------------------------------
 * io_fluent-simulink_udf_vessel-res-2.c  (minimal: world outputs for Simulink)
 *------------------------------------------------------------------*/
#include "udf.h"
#include "sg.h"
#include "mem.h"

#include <sys/stat.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

/* ----------- настройки/пути ---------- */
#define SKIP              20
#define MAX_ATTEMPTS     1000
#define WAIT_US      2000000

#define DEFAULT_FLAG_IN    "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/pose.ok"
#define DEFAULT_POSE_IN    "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/pose.dat"
#define DEFAULT_FLAG_OUT   "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/feed.ok"
#define DEFAULT_FEED_OUT   "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/feed.dat"
#define DEFAULT_RESET_FLAG "/home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/vessel-res-2/reset/reset.ok"

/* ENV override: UDF_FLAG_IN, UDF_POSE_IN, UDF_FLAG_OUT, UDF_FEED_OUT, UDF_RESET_FLAG */
static const char* resolve_path_env(const char *env_key, const char *fallback)
{
    const char *s = getenv(env_key);
    return (s && s[0] != '\0') ? s : fallback;
}

/* ----------- геометрия/материал ---------- */
#define ZONE_VESSEL_LIQ   16
#define WATER_PHASE_IDX    1
#define VOF_THR        1.0e-3
#define RHO_W        1000.0
#define M_VESSEL       0.035
#define I_VES_XX    1.2e-3
#define I_VES_YY    1.2e-3
#define I_VES_ZZ    2.3e-3
/* численный порог, чтобы гасить шум */
#define FLOW_EPS   1e-12

/* если STL не в своём CoM — задать смещение локального COM сосуда */
static const real vessel_offset[3] = {0.0, 0.0, 0.0};

/* --- spill metrics --- */
static real vol_initial     = -1.0;   /* фиксация суммарного объёма в домене на первом шаге эпизода */
static real spill_out_total = 0.0;    /* кумулятивный реальный слив наружу (м^3) */
static real prev_vol_outside = 0.0;   /* для Δ внутри-домена перелива */

/* ID(ы) overflow-граней: укажи свои! */
static const int OVERFLOW_BC_IDS[] = { 6 /* top_outer_flow */ };
static const int N_OVERFLOW = sizeof(OVERFLOW_BC_IDS)/sizeof(OVERFLOW_BC_IDS[0]);

/* ----------- состояние ---------- */
static int  step_cnt_in  = 0;
static int  step_cnt_out = 0;
static real t_prev = 0.0;


/* поза/скорости GVM-в-мире (из pose.dat) */
static real CX=0, CY=0, CZ=0;
static real Q[4] = {1.0, 0.0, 0.0, 0.0};    /* [w,x,y,z] */
static real VELOCITY[3] = {0.0, 0.0, 0.0};
static real OMEGA[3]    = {0.0, 0.0, 0.0};
static real pose[13];

static real C0[3] = {0,0,0};
static real Q0[4] = {1,0,0,0};
static int  pose_inited = 0;

/* ----------- утилиты ---------- */

/* умножение кватернионов: out = a ⊗ b, формат [w x y z] */
static void quat_mul(const real a[4], const real b[4], real out[4])
{
    real aw=a[0], ax=a[1], ay=a[2], az=a[3];
    real bw=b[0], bx=b[1], by=b[2], bz=b[3];
    out[0] = aw*bw - ax*bx - ay*by - az*bz;  // w
    out[1] = aw*bx + ax*bw + ay*bz - az*by;  // x
    out[2] = aw*by - ax*bz + ay*bw + az*bx;  // y
    out[3] = aw*bz + ax*by - ay*bx + az*bw;  // z
}
static int flag_exists(const char *f) { struct stat st; return !stat(f, &st); }

static void quat_normalize(real q[4])
{
    real n = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (n > 1e-12) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
    else { q[0]=1.0; q[1]=q[2]=q[3]=0.0; }
}

static void quat_to_rot_matrix(const real Q[4], real R[9])
{
    real w=Q[0], x=Q[1], y=Q[2], z=Q[3];
    R[0]=1-2*y*y-2*z*z; R[1]=2*x*y-2*w*z;   R[2]=2*x*z+2*w*y;
    R[3]=2*x*y+2*w*z;   R[4]=1-2*x*x-2*z*z; R[5]=2*y*z-2*w*x;
    R[6]=2*x*z-2*w*y;   R[7]=2*y*z+2*w*x;   R[8]=1-2*x*x-2*y*y;
}

static void quat_rotate(const real Q[4], const real v[3], real out[3])
{
    /* вращает локальный вектор v в мир: out = R_WL * v */
    real R[9]; quat_to_rot_matrix(Q, R);
    out[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
    out[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
    out[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}

/* вращение симм. тензора из лок. в мир: I_rot = R I R^T */
static void rotate_tensor(const real R[9], const real I_local[6], real I_rot[6])
{
    real I[9] = { I_local[0], I_local[3], I_local[5],
                  I_local[3], I_local[1], I_local[4],
                  I_local[5], I_local[4], I_local[2] };
    real tmp[9]={0}, Ir[9]={0};
    for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        for(int k=0;k<3;k++) tmp[i*3+j] += R[i*3+k]*I[k*3+j];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        for(int k=0;k<3;k++) Ir[i*3+j] += tmp[i*3+k]*R[j*3+k];
    I_rot[0]=Ir[0]; I_rot[1]=Ir[4]; I_rot[2]=Ir[8];
    I_rot[3]=Ir[1]; I_rot[4]=Ir[5]; I_rot[5]=Ir[2];
}

/* перенос по параллельным осям: I += m (||d||^2 I - d d^T) */
static void parallel_axis_shift(real I[6], real m, const real d[3])
{
    real dx=d[0], dy=d[1], dz=d[2];
    real dx2=dx*dx, dy2=dy*dy, dz2=dz*dz;
    I[0] += m*(dy2+dz2);
    I[1] += m*(dx2+dz2);
    I[2] += m*(dx2+dy2);
    I[3] -= m*(dx*dy);
    I[4] -= m*(dy*dz);
    I[5] -= m*(dx*dz);
}

static void quat_conjugate(const real Q_in[4], real Q_out[4])
{
    Q_out[0] =  Q_in[0];
    Q_out[1] = -Q_in[1];
    Q_out[2] = -Q_in[2];
    Q_out[3] = -Q_in[3];
}

/* вращает мировой вектор v в локальный: out = R_WL^T * v = R_LW * v */
static void quat_rotate_inverse(const real Q[4], const real v[3], real out[3])
{
    real Q_conj[4];
    quat_conjugate(Q, Q_conj);
    quat_rotate(Q_conj, v, out); // Используем существующую функцию с сопряженным кватернионом
}

DEFINE_CG_MOTION(cup_ext, dt, vel, omega, time, dtime)
{
    /* 1) Интегрируем положение (ZOH: скорости постоянны между пакетами) */
    CX += VELOCITY[0]*dtime;
    CY += VELOCITY[1]*dtime;
    CZ += VELOCITY[2]*dtime;

    /* 2) Интегрируем ориентацию через экспоненту угла */
    real wnorm = sqrt(OMEGA[0]*OMEGA[0] + OMEGA[1]*OMEGA[1] + OMEGA[2]*OMEGA[2]);
    real half  = 0.5*wnorm*dtime;
    real dq[4];
    if (wnorm > 1e-12) {
        real s = sin(half)/wnorm;
        dq[0] = cos(half);
        dq[1] = OMEGA[0]*s;
        dq[2] = OMEGA[1]*s;
        dq[3] = OMEGA[2]*s;
    } else {
        dq[0]=1.0; dq[1]=dq[2]=dq[3]=0.0;
    }
    real qnew[4];  /* q_{k+1} = dq ⊗ q_k */
    qnew[0] = dq[0]*Q[0] - dq[1]*Q[1] - dq[2]*Q[2] - dq[3]*Q[3];
    qnew[1] = dq[0]*Q[1] + dq[1]*Q[0] + dq[2]*Q[3] - dq[3]*Q[2];
    qnew[2] = dq[0]*Q[2] - dq[1]*Q[3] + dq[2]*Q[0] + dq[3]*Q[1];
    qnew[3] = dq[0]*Q[3] + dq[1]*Q[2] - dq[2]*Q[1] + dq[3]*Q[0];
    /* нормализация */
    real n = sqrt(qnew[0]*qnew[0]+qnew[1]*qnew[1]+qnew[2]*qnew[2]+qnew[3]*qnew[3]);
    if (n>1e-12){ Q[0]=qnew[0]/n; Q[1]=qnew[1]/n; Q[2]=qnew[2]/n; Q[3]=qnew[3]/n; }
    else { Q[0]=1; Q[1]=Q[2]=Q[3]=0; }

    /* 3) Выдать абсолютную позу и (по желанию) скорости */
    DT_CG(dt)[0]=CX; DT_CG(dt)[1]=CY; DT_CG(dt)[2]=CZ;
    DT_Q(dt)[0]=Q[1]; DT_Q(dt)[1]=Q[2]; DT_Q(dt)[2]=Q[3]; DT_Q(dt)[3]=Q[0]; /* [x y z w] */

    NV_V(vel  , =, VELOCITY);   /* опционально, для согласованности */
    NV_V(omega, =, OMEGA);      /* опционально */
}




/* ------------- 2) читаем pose.dat ------------- */
DEFINE_ADJUST(read_pose_from_simulink, domain)
{
    real t = CURRENT_TIME;
    if (fabs(t - t_prev) < 1e-12) return;
    t_prev = t;

    if (++step_cnt_in % SKIP != 0) return;
    step_cnt_in = 0;

    const char *flag_in = resolve_path_env("UDF_FLAG_IN", DEFAULT_FLAG_IN);
    const char *pose_in = resolve_path_env("UDF_POSE_IN", DEFAULT_POSE_IN);
    

    if (I_AM_NODE_ZERO_P) {
        for (int k=0; k<MAX_ATTEMPTS; ++k) {
            if (flag_exists(flag_in)) {
                FILE *fp = fopen(pose_in, "r");
                if (fp) {
                    int ok = fscanf(fp,
                        "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                        &pose[0], &pose[1], &pose[2],
                        &pose[3], &pose[4], &pose[5], &pose[6],
                        &pose[7], &pose[8], &pose[9],
                        &pose[10], &pose[11], &pose[12]);
                    fclose(fp);
                    if (ok==13) {
                        if (remove(flag_in)!=0)
                            Message("Warning: can't remove %s (errno=%d)\n", flag_in, errno);
                        break;
                    } else {
                        Message("Warning: pose.dat corrupted (%d/13) — keep prev pose\n", ok);
                        if (remove(flag_in)!=0)
                            Message("Warning: can't remove %s (errno=%d)\n", flag_in, errno);
                        break;
                    }
                } else {
                    Message("Warning: cannot open %s — keep prev pose\n", pose_in);
                    if (remove(flag_in)!=0)
                        Message("Warning: can't remove %s (errno=%d)\n", flag_in, errno);
                    break;
                }
            }
            usleep(WAIT_US);
        }
    }
    PRF_BCAST_REAL(pose, 13, 0);

    CX = pose[0]; CY = pose[1]; CZ = pose[2];
    memcpy(Q,        &pose[3],  4*sizeof(real)); quat_normalize(Q);
    memcpy(VELOCITY, &pose[7],  3*sizeof(real));
    memcpy(OMEGA,    &pose[10], 3*sizeof(real));
    if (!pose_inited) {
        // Выполняется один раз на первом шаге И каждый раз после reset.ok
        C0[0] = CX; C0[1] = CY; C0[2] = CZ;
        memcpy(Q0, Q, 4*sizeof(real));
        pose_inited = 1;
    }
}


DEFINE_EXECUTE_AT_END(write_props_to_simulink)
{
    const char *flag_out = resolve_path_env("UDF_FLAG_OUT", DEFAULT_FLAG_OUT);
    const char *feed_out = resolve_path_env("UDF_FEED_OUT", DEFAULT_FEED_OUT);
    const char *reset_flag = resolve_path_env("UDF_RESET_FLAG", DEFAULT_RESET_FLAG);
    
    if (I_AM_NODE_ZERO_P) {
        if (flag_exists(reset_flag)) {
            if (remove(reset_flag) == 0) {
                Message("reset.ok processed: resetting UDF state and base pose.\n");
                pose_inited = 0; // Устанавливаем флаг, что нужна переинициализация
            } else {
                Message("Warning: can't remove %s (errno=%d)\n", reset_flag, errno);
            }
        }
    }
    PRF_BCAST_INT(&pose_inited, 1, 0); // Рассылаем флаг всем

    /*if (!pose_inited) {
        // Сбрасываем все счетчики
        vol_initial      = -1.0;
        spill_out_total  = 0.0;
        prev_vol_outside = 0.0;
        
        pose_inited = 1; // Устанавливаем флаг, что инициализация прошла
        
    }*/

    if (++step_cnt_out % SKIP != 0) return;
    step_cnt_out = 0;

    Domain *d = Get_Domain(1);
    Thread *t; cell_t c;

	/* ---------- 0) Реальный отток наружу через overflow-грань(и), VOF-safe ---------- */
	real outflow_step_vol = 0.0;                  /* м^3 за текущий шаг */
	const real dt = RP_Get_Real("physical-time-step");

	for (int i = 0; i < N_OVERFLOW; ++i) {
	    Thread *tf = Lookup_Thread(d, OVERFLOW_BC_IDS[i]);  /* ЭТО face-thread СМЕСИ */
	    if (!tf) continue;

	    face_t f;
	    begin_f_loop(f, tf) {
		/* Массовый поток СМЕСИ через грань (кг/с), знак Fluent: <0 — ИЗ домена наружу */
		real mdot_mix = F_FLUX(f, tf);
		if (mdot_mix < -FLOW_EPS) {
		    /* внутренняя ячейка и её тред смеси */
		    cell_t  c0    = F_C0(f, tf);
		    Thread *tmix0 = THREAD_T0(tf);
		    if (tmix0) {
		        /* объёмная доля воды в ячейке (под-тред воды у ЯЧЕЙКИ) */
		        Thread *twc = THREAD_SUB_THREAD(tmix0, WATER_PHASE_IDX);
		        real alpha = 0.0;
		        if (twc) alpha = C_VOF(c0, twc);

		        /* плотность смеси в ячейке */
		        real rho_mix = C_R(c0, tmix0);
		        if (rho_mix < 1e-12) rho_mix = 1e-12;

		        /* масса воды (кг/с) и её объём за шаг (м^3) */
		        real mdot_w = (-mdot_mix) * alpha * (RHO_W / rho_mix);
		        outflow_step_vol += mdot_w / RHO_W * dt;
		    }
		}
	    } end_f_loop(f, tf)
	}
	outflow_step_vol = PRF_GRSUM1(outflow_step_vol);
	if (I_AM_NODE_ZERO_P) spill_out_total += outflow_step_vol;
    /* ---------- 1) Масса/первый момент в ЛОКАЛЬНЫХ координатах ---------- */
    real m_liq = 0.0;
    real M1_local[3] = {0.0, 0.0, 0.0};
    real vol_outside = 0.0; /* объём воды вне cell-зоны сосуда, но внутри домена */

    /* относительная поза сосуда (нулевая в t0) */
    real C_rel[3] = { CX - C0[0], CY - C0[1], CZ - C0[2] };
    real Q0_conj[4] = { Q0[0], -Q0[1], -Q0[2], -Q0[3] };
    real Q_rel[4]; quat_mul(Q, Q0_conj, Q_rel); quat_normalize(Q_rel);
    real vessel_pos[3] = { C_rel[0], C_rel[1], C_rel[2] };

    thread_loop_c(t, d) {
        if (!FLUID_THREAD_P(t)) continue;
        Thread *tw = THREAD_SUB_THREAD(t, WATER_PHASE_IDX);
        if (!tw) continue;

        begin_c_loop_int(c, t) {
            real vf = C_VOF(c, tw);
            if (vf < VOF_THR) continue;

            real vol = C_VOLUME(c, t) * vf;
            const bool inside = (THREAD_ID(t) == ZONE_VESSEL_LIQ);
            if (!inside) { vol_outside += vol; continue; }

            real dm = RHO_W * vol;
            real xc_world[ND_ND]; C_CENTROID(xc_world, c, t);

            /* мир -> локал (с учётом текущей позы сосуда) */
            real xc_rel_world[3] = { xc_world[0]-vessel_pos[0], xc_world[1]-vessel_pos[1], xc_world[2]-vessel_pos[2] };
            real xc_local[3]; quat_rotate_inverse(Q_rel, xc_rel_world, xc_local);

            m_liq       += dm;
            M1_local[0] += dm * xc_local[0];
            M1_local[1] += dm * xc_local[1];
            M1_local[2] += dm * xc_local[2];
        } end_c_loop_int(c, t)
    }

    m_liq       = PRF_GRSUM1(m_liq);
    M1_local[0] = PRF_GRSUM1(M1_local[0]);
    M1_local[1] = PRF_GRSUM1(M1_local[1]);
    M1_local[2] = PRF_GRSUM1(M1_local[2]);
    vol_outside = PRF_GRSUM1(vol_outside);

    /* ---------- 2) Внутренний перелив (Δ объёма вне зоны сосуда) ---------- */
    static real spill_in_step = 0.0;
    if (I_AM_NODE_ZERO_P) {
        real delta = vol_outside - prev_vol_outside;
        spill_in_step = (delta > FLOW_EPS) ? delta : 0.0;
        prev_vol_outside = vol_outside;
    }
    PRF_BCAST_REAL(&spill_in_step, 1, 0);

    /* ---------- 3) Потерянный из домена объём (диагностика дрейфа) ---------- */
    real vol_liq = (m_liq > 1e-12) ? m_liq / RHO_W : 0.0;
    real current_total = vol_liq + vol_outside;  /* в домене */
    if (vol_initial < 0.0) vol_initial = current_total;
    real vol_spill = (vol_initial > current_total) ? (vol_initial - current_total) : 0.0;

    /* ---------- 4) COM жидкости (локально) ---------- */
    real com_liq_L[3] = {0.0, 0.0, 0.0};
    if (m_liq > 1e-12) {
        com_liq_L[0] = M1_local[0] / m_liq;
        com_liq_L[1] = M1_local[1] / m_liq;
        com_liq_L[2] = M1_local[2] / m_liq;
    }
    PRF_BCAST_REAL(com_liq_L, 3, 0);

    /* ---------- 5) Инерция жидкости (локально, относительно com_liq_L) ---------- */
    real I_liq_L[6] = {0,0,0,0,0,0};
    thread_loop_c(t, d) {
        if (!FLUID_THREAD_P(t) || THREAD_ID(t) != ZONE_VESSEL_LIQ) continue;
        Thread *tw = THREAD_SUB_THREAD(t, WATER_PHASE_IDX);
        if (!tw) continue;

        begin_c_loop_int(c, t) {
            real vf = C_VOF(c, tw);
            if (vf < VOF_THR) continue;

            real dm = RHO_W * C_VOLUME(c, t) * vf;

            real xc_world[ND_ND]; C_CENTROID(xc_world, c, t);
            real xc_rel_world[3] = { xc_world[0]-vessel_pos[0], xc_world[1]-vessel_pos[1], xc_world[2]-vessel_pos[2] };
            real xc_local[3]; quat_rotate_inverse(Q_rel, xc_rel_world, xc_local);

            real xr[3] = { xc_local[0]-com_liq_L[0], xc_local[1]-com_liq_L[1], xc_local[2]-com_liq_L[2] };
            I_liq_L[0] += dm*(xr[1]*xr[1] + xr[2]*xr[2]);
            I_liq_L[1] += dm*(xr[0]*xr[0] + xr[2]*xr[2]);
            I_liq_L[2] += dm*(xr[0]*xr[0] + xr[1]*xr[1]);
            I_liq_L[3] -= dm*(xr[0]*xr[1]);
            I_liq_L[4] -= dm*(xr[1]*xr[2]);
            I_liq_L[5] -= dm*(xr[0]*xr[2]);
        } end_c_loop_int(c, t)
    }
    for (int i=0;i<6;i++) I_liq_L[i] = PRF_GRSUM1(I_liq_L[i]);

    if (!I_AM_NODE_ZERO_P) return;

    /* ---------- 6) Система «сосуд+жидкость» (локально) ---------- */
    real m_tot = M_VESSEL + m_liq;
    real com_sys_L[3] = {0,0,0};
    if (m_tot > 1e-12) {
        com_sys_L[0] = (M_VESSEL*vessel_offset[0] + m_liq*com_liq_L[0]) / m_tot;
        com_sys_L[1] = (M_VESSEL*vessel_offset[1] + m_liq*com_liq_L[1]) / m_tot;
        com_sys_L[2] = (M_VESSEL*vessel_offset[2] + m_liq*com_liq_L[2]) / m_tot;
    }

    real I_v_local[6] = { I_VES_XX, I_VES_YY, I_VES_ZZ, 0.0, 0.0, 0.0 };
    real d_l[3] = { com_liq_L[0]-com_sys_L[0], com_liq_L[1]-com_sys_L[1], com_liq_L[2]-com_sys_L[2] };
    real d_v[3] = { vessel_offset[0]-com_sys_L[0], vessel_offset[1]-com_sys_L[1], vessel_offset[2]-com_sys_L[2] };

    parallel_axis_shift(I_liq_L, m_liq, d_l);
    parallel_axis_shift(I_v_local, M_VESSEL, d_v);

    real I_tot_L[6];
    for (int i=0;i<6;i++) I_tot_L[i] = I_liq_L[i] + I_v_local[i];

    /* мировой COM системы (диагностика) */
    real Rcm[9]; quat_to_rot_matrix(Q, Rcm);
    real com_sys_W[3] = {
        Rcm[0]*com_sys_L[0] + Rcm[1]*com_sys_L[1] + Rcm[2]*com_sys_L[2] + CX,
        Rcm[3]*com_sys_L[0] + Rcm[4]*com_sys_L[1] + Rcm[5]*com_sys_L[2] + CY,
        Rcm[6]*com_sys_L[0] + Rcm[7]*com_sys_L[1] + Rcm[8]*com_sys_L[2] + CZ
    };

    /* ---------- 7) Запись в feed.dat ---------- */
    FILE *fp = fopen(feed_out, "w");
    if (fp) {
        fprintf(fp,
            "%.9e "                               /* time */
            "%.9e %.9e "                          /* m_tot, m_liq */
            "%.9e %.9e %.9e "                     /* com_liq_L */
            "%.9e %.9e %.9e %.9e %.9e %.9e "      /* I_tot_L (xx yy zz xy yz xz) */
            "%.9e "                               /* outflow_step_vol */
            "%.9e %.9e %.9e\n",                   /* com_sys_W (diag) */
            CURRENT_TIME,
            m_tot, m_liq,
            com_liq_L[0], com_liq_L[1], com_liq_L[2],
            I_tot_L[0], I_tot_L[1], I_tot_L[2], I_tot_L[3], I_tot_L[4], I_tot_L[5],
            spill_in_step,
            com_sys_W[0], com_sys_W[1], com_sys_W[2]
        );
        fclose(fp);

        FILE *fl = fopen(flag_out, "w"); if (fl) fclose(fl);

        /* ВНИМАНИЕ: ep=%d у тебя раньше падало из-за отсутствия epoch_id. Либо убери, либо добавь глобаль и инкремент при reset. */
        Message("feed.dat t=%.3e  m_liq=%.3f kg  dV_out=%.3e  V_out=%.3e  dV_in=%.3e  vol_spill=%.3e\n",
                CURRENT_TIME, m_liq, outflow_step_vol, spill_out_total, spill_in_step, vol_spill);
    } else {
        Message("Warning: cannot open %s for writing (errno=%d)\n", feed_out, errno);
    }
}


