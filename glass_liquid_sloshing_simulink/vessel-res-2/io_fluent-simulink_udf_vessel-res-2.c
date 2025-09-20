/*--------------------------------------------------------------------
 * io_fluent-simulink_udf_vessel-res-2.c
 * Обмен 6-DOF позой ↔ Simulink и передача m_tot, COM_W, I_W@COM, spill.
 * Без RP-переменных: пути берутся из дефолтов или из ENV (см. ниже).
 * Физика:
 *   - масса/COM жидкости (1-й проход), затем I_liq @ COM_liq (2-й проход);
 *   - I_vessel (локальный диагональный) → поворот в мир;
 *   - перенос по параллельным осям КАЖДОЙ части к системному COM, затем сумма;
 *   - кватернион интегрируется через экспоненту угла + нормализация.
 * Выход feed.dat: t, m_tot, COM_W(3), I_W@COM(6), spill_vol.
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
#include <stdlib.h>   /* getenv */

/* ------------------- настройки/пути ------------------------------ */
/* Частота обмена (каждые SKIP шагов dtime) */
#define SKIP              20     /* 20 × 0.0005 = 0.01 s */
#define MAX_ATTEMPTS     1000     /* ожидание pose.ok */
#define WAIT_US      2000000     /* 1 с */

#define DEFAULT_FLAG_IN    "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/pose.ok"
#define DEFAULT_POSE_IN    "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/pose.dat"
#define DEFAULT_FLAG_OUT   "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/feed.ok"
#define DEFAULT_FEED_OUT   "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/feed.dat"
#define DEFAULT_RESET_FLAG "/home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/vessel-res-2/reset/reset.ok"

/* Если хочешь переопределять пути без перекомпиляции — задай ENV:
   UDF_FLAG_IN, UDF_POSE_IN, UDF_FLAG_OUT, UDF_FEED_OUT, UDF_RESET_FLAG */
static const char* resolve_path_env(const char *env_key, const char *fallback)
{
    const char *s = getenv(env_key);
    return (s && s[0] != '\0') ? s : fallback;
}

/* ------------------- геометрия/материал -------------------------- */
#define ZONE_VESSEL_LIQ   16     /* id зоны жидкости ВНУТРИ сосуда */
#define WATER_PHASE_IDX    1     /* индекс фазы «water» (иногда 0, проверь) */
#define VOF_THR        1.0e-3    /* порог VOF (точнее массы/COM) */
#define RHO_W        1000.0      /* плотность воды, kg/m^3 */
#define M_VESSEL       0.035     /* масса пустого сосуда, kg */
#define I_VES_XX    1.2e-3       /* локальные главные моменты сосуда */
#define I_VES_YY    1.2e-3
#define I_VES_ZZ    2.3e-3

/* Смещение КС сосуда относительно его геометрцентра (если STL не в CoM) */
static const real vessel_offset[3] = {0.0, 0.0, 0.0};

/* ------------------- состояние/глобальные ------------------------ */
static int  step_cnt_in  = 0;
static int  step_cnt_out = 0;
static real t_prev = 0.0;

/* Поза и скорости (обновляются из pose.dat) */
static real CX=0, CY=0, CZ=0;                 /* позиция сосуда, мир */
static real Q[4] = {1.0, 0.0, 0.0, 0.0};      /* кватернион [w,x,y,z] */
static real VELOCITY[3] = {0.0, 0.0, 0.0};    /* лин. скорость сосуда */
static real OMEGA[3]    = {0.0, 0.0, 0.0};    /* угл. скорость сосуда */

static real pose[13]; /* CX CY CZ q0 q1 q2 q3 vx vy vz wx wy wz (q=[w,x,y,z]) */

/* ------------------- утилиты ------------------------------------ */
static int flag_exists(const char *f) {
    struct stat st; return !stat(f, &st);
}

/* кватернионы */
static void quat_mul(const real a[4], const real b[4], real out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
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
    real s=Q[0], x=Q[1], y=Q[2], z=Q[3];
    real t[3] = { 2*(y*v[2]-z*v[1]), 2*(z*v[0]-x*v[2]), 2*(x*v[1]-y*v[0]) };
    out[0] = v[0] + s*t[0] + (y*t[2]-z*t[1]);
    out[1] = v[1] + s*t[1] + (z*t[0]-x*t[2]);
    out[2] = v[2] + s*t[2] + (x*t[1]-y*t[0]);
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
        for(int k=0;k<3;k++) Ir[i*3+j] += tmp[i*3+k]*R[j*3+k]; /* *R^T */
    I_rot[0]=Ir[0]; I_rot[1]=Ir[4]; I_rot[2]=Ir[8];
    I_rot[3]=Ir[1]; I_rot[4]=Ir[5]; I_rot[5]=Ir[2];
}

/* перенос по параллельным осям: I += m (||d||^2 I - d d^T) */
static void parallel_axis_shift(real I[6], real m, const real d[3])
{
    real dx=d[0], dy=d[1], dz=d[2];
    real dx2=dx*dx, dy2=dy*dy, dz2=dz*dz;
    I[0] += m*(dy2+dz2);  /* Ixx */
    I[1] += m*(dx2+dz2);  /* Iyy */
    I[2] += m*(dx2+dy2);  /* Izz */
    I[3] -= m*(dx*dy);    /* Ixy */
    I[4] -= m*(dy*dz);    /* Iyz */
    I[5] -= m*(dx*dz);    /* Ixz */
}

/* ------------------- 1) 6-DOF движение -------------------------- */
DEFINE_CG_MOTION(cup_ext, dt, vel, omega, time, dtime)
{
    /* интегрируем положение (если подаёшь абсолюты из Simulink — можно занулить) */
    CX += VELOCITY[0]*dtime;
    CY += VELOCITY[1]*dtime;
    CZ += VELOCITY[2]*dtime;

    /* кватернион: экспонента угла */
    real wnorm = sqrt(OMEGA[0]*OMEGA[0]+OMEGA[1]*OMEGA[1]+OMEGA[2]*OMEGA[2]);
    real half  = 0.5*wnorm*dtime;
    real s     = (wnorm>0.0) ? sin(half)/wnorm : 0.0;
    real dq[4] = { cos(half), OMEGA[0]*s, OMEGA[1]*s, OMEGA[2]*s };
    real qnew[4]; quat_mul(dq, Q, qnew); quat_normalize(qnew);
    memcpy(Q, qnew, 4*sizeof(real));

    /* отдать Fluent позу (излишне) */
    // DT_CG(dt)[0]=CX; DT_CG(dt)[1]=CY; DT_CG(dt)[2]=CZ;*/
    // DT_Q(dt)[0]=Q[1]; DT_Q(dt)[1]=Q[2]; DT_Q(dt)[2]=Q[3]; DT_Q(dt)[3]=Q[0]; /* [x y z w] */
    /* отдать Fluent скорости */
    NV_V(vel  , =, VELOCITY);
    NV_V(omega, =, OMEGA);
}

/* ------------------- 2) читаем pose.dat -------------------------- */
DEFINE_ADJUST(read_pose_from_simulink, domain)
{
    real t = CURRENT_TIME;
    if (fabs(t - t_prev) < 1e-12) return;  /* уже обновлялись на этом шаге */
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

    /* рассылаем позу всем ранкам */
    PRF_BCAST_REAL(pose, 13, 0);

    CX = pose[0]; CY = pose[1]; CZ = pose[2];
    memcpy(Q,        &pose[3],  4*sizeof(real)); quat_normalize(Q);
    memcpy(VELOCITY, &pose[7],  3*sizeof(real));
    memcpy(OMEGA,    &pose[10], 3*sizeof(real));
}

/* ------------------- 3) масса/COM/инерции + запись -------------- */
DEFINE_EXECUTE_AT_END(write_props_to_simulink)
{
    const char *flag_out   = resolve_path_env("UDF_FLAG_OUT",   DEFAULT_FLAG_OUT);
    const char *feed_out   = resolve_path_env("UDF_FEED_OUT",   DEFAULT_FEED_OUT);
    const char *reset_flag = resolve_path_env("UDF_RESET_FLAG", DEFAULT_RESET_FLAG);

    /* reset.ok (опционально) */
    if (I_AM_NODE_ZERO_P) {
        if (flag_exists(reset_flag)) {
            if (remove(reset_flag)==0)
                Message("reset.ok processed — model reload requested\n");
            else
                Message("Warning: can't remove %s (errno=%d)\n", reset_flag, errno);
        }
    }

    if (++step_cnt_out % SKIP != 0) return;
    step_cnt_out = 0;

    Domain *d = Get_Domain(1);
    Thread *t; cell_t c;

    real m_liq = 0.0;
    real M1[3] = {0.0, 0.0, 0.0};
    real vol_spill = 0.0;

    /* ---- Первый проход: масса, первый момент, spill -------------- */
    thread_loop_c(t, d) {
        if (!FLUID_THREAD_P(t)) continue;

        Thread *tw = THREAD_SUB_THREAD(t, WATER_PHASE_IDX);
        if (!tw) continue;

        begin_c_loop_int(c, t) {
            real vf = C_VOF(c, tw);
            if (vf < VOF_THR) continue;

            real vol = C_VOLUME(c, t) * vf;
            bool inside = (THREAD_ID(t) == ZONE_VESSEL_LIQ);

            if (!inside) { vol_spill += vol; continue; }

            real dm = RHO_W * vol;
            real xc[ND_ND]; C_CENTROID(xc, c, t);

            m_liq  += dm;
            M1[0]  += dm * xc[0];
            M1[1]  += dm * xc[1];
            M1[2]  += dm * xc[2];
        }
        end_c_loop_int(c, t)
    }

    /* MPI редукции */
    m_liq     = PRF_GRSUM1(m_liq);
    M1[0]     = PRF_GRSUM1(M1[0]);
    M1[1]     = PRF_GRSUM1(M1[1]);
    M1[2]     = PRF_GRSUM1(M1[2]);
    vol_spill = PRF_GRSUM1(vol_spill);

    /* COM жидкости (мир) */
    real com_l[3] = {0.0, 0.0, 0.0};
    if (m_liq > 1e-12) {
        com_l[0] = M1[0] / m_liq;
        com_l[1] = M1[1] / m_liq;
        com_l[2] = M1[2] / m_liq;
    } else {
        Message("Warning: m_liq≈0 — COM_l set to zero\n");
    }

    /* всем ранкам нужен com_l для второго прохода */
    PRF_BCAST_REAL(com_l, 3, 0);

    /* ---- Второй проход: инерция жидкости относительно com_l ------ */
    real I_liq[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
    thread_loop_c(t, d) {
        if (!FLUID_THREAD_P(t)) continue;

        Thread *tw = THREAD_SUB_THREAD(t, WATER_PHASE_IDX);
        if (!tw || THREAD_ID(t) != ZONE_VESSEL_LIQ) continue;

        begin_c_loop_int(c, t) {
            real vf = C_VOF(c, tw);
            if (vf < VOF_THR) continue;

            real vol = C_VOLUME(c, t) * vf;
            real dm  = RHO_W * vol;

            real xc[ND_ND]; C_CENTROID(xc, c, t);
            real xr[3] = { xc[0]-com_l[0], xc[1]-com_l[1], xc[2]-com_l[2] };

            I_liq[0] += dm*(xr[1]*xr[1] + xr[2]*xr[2]);  /* Ixx */
            I_liq[1] += dm*(xr[0]*xr[0] + xr[2]*xr[2]);  /* Iyy */
            I_liq[2] += dm*(xr[0]*xr[0] + xr[1]*xr[1]);  /* Izz */
            I_liq[3] -= dm*(xr[0]*xr[1]);                /* Ixy */
            I_liq[4] -= dm*(xr[1]*xr[2]);                /* Iyz */
            I_liq[5] -= dm*(xr[0]*xr[2]);                /* Ixz */
        }
        end_c_loop_int(c, t)
    }

    for (int i=0;i<6;i++) I_liq[i] = PRF_GRSUM1(I_liq[i]);

    if (!I_AM_NODE_ZERO_P) return;  /* запись делает только мастер */

    /* COM пустого сосуда в мире (учитывая оффсет) */
    real off_w[3]; quat_rotate(Q, vessel_offset, off_w);
    real com_v[3] = { CX + off_w[0], CY + off_w[1], CZ + off_w[2] };

    /* Итоговая масса и системный COM (мир) */
    real m_tot = M_VESSEL + m_liq;
    real com[3] = {0.0,0.0,0.0};
    if (m_tot > 1e-12) {
        com[0] = (M_VESSEL*com_v[0] + m_liq*com_l[0]) / m_tot;
        com[1] = (M_VESSEL*com_v[1] + m_liq*com_l[1]) / m_tot;
        com[2] = (M_VESSEL*com_v[2] + m_liq*com_l[2]) / m_tot;
    }

    /* I_vessel (локальный) → мир */
    real I_v_local[6] = { I_VES_XX, I_VES_YY, I_VES_ZZ, 0.0, 0.0, 0.0 };
    real R[9]; quat_to_rot_matrix(Q, R);
    real I_v_rot[6]; rotate_tensor(R, I_v_local, I_v_rot);

    /* Перенос КАЖДОЙ части к системному COM */
    real I_liq_sys[6]; memcpy(I_liq_sys, I_liq, sizeof(I_liq_sys));
    real I_v_sys[6];   memcpy(I_v_sys,   I_v_rot, sizeof(I_v_sys));

    real d_l[3] = { com_l[0]-com[0], com_l[1]-com[1], com_l[2]-com[2] };
    real d_v[3] = { com_v[0]-com[0], com_v[1]-com[1], com_v[2]-com[2] };

    parallel_axis_shift(I_liq_sys, m_liq,    d_l);
    parallel_axis_shift(I_v_sys,   M_VESSEL, d_v);

    /* Итоговый тензор системы относительно системного COM, в мировых осях */
    real I_tot[6];
    for (int i=0;i<6;i++) I_tot[i] = I_liq_sys[i] + I_v_sys[i];

    /* ---------- запись feed.dat + feed.ok ------------------------ */
    const char *feed_path = feed_out;
    FILE *fp = fopen(feed_path, "w");
    if (fp) {
        fprintf(fp,
            "%.9e %.9e "
            "%.9e %.9e %.9e "
            "%.9e %.9e %.9e %.9e %.9e %.9e "
            "%.6e\n",
            CURRENT_TIME, m_tot,
            com[0], com[1], com[2],
            I_tot[0], I_tot[1], I_tot[2],
            I_tot[3], I_tot[4], I_tot[5],
            vol_spill);
        fclose(fp);

        FILE *fl = fopen(flag_out, "w");  /* пустой файл-флаг */
        if (fl) fclose(fl);
        else Message("Warning: cannot create %s (errno=%d)\n", flag_out, errno);

        Message("feed.dat @ t=%.3e  m_liq=%.3f kg  spill=%.3e m^3\n",
                CURRENT_TIME, m_liq, vol_spill);
    } else {
        Message("Warning: cannot open %s for writing (errno=%d)\n", feed_out, errno);
    }
}

