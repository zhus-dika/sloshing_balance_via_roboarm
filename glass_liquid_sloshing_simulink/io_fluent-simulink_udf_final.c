/*--------------------------------------------------------------------
 *  cup_ext.c — обмен 6-DOF позой ↔ Simulink и передача
 *              m_total, COM, Inertia, spill назад в Simulink
 *------------------------------------------------------------------*/
#include "udf.h"
#include "sg.h"
#include "mem.h"

#include <sys/stat.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <string.h>   /* memcpy */
#include <errno.h>    /* errno  */

/* ---------- демо-параметры движения (если нет pose.dat) ---------- */
#define AMP_X   0.02
#define AMP_Z   0.005
#define ROT_AMP 0.10
#define FREQ    0.5
#define TWO_PI (2.0*M_PI)

/* ---------- ожидание входного флага pose.ok ---------------------- */
#define MAX_ATTEMPTS 100          /* сколько раз подряд ждать файл-флаг */
#define WAIT_US      1000000      /* задержка между попытками, мкс ≈1 с */

/* ---------- файлы обмена ----------------------------------------- */
#define FLAG_IN   "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/pose.ok"
#define POSE_IN   "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/pose.dat"
#define FLAG_OUT  "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/feed.ok"
#define FEED_OUT  "/home/dika/Documents/sloshing_balance_via_roboarm/rlKinova_marsRover_fluent_via_files/data/work_dir/feed.dat"
#define RESET_FLAG "/home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/reset/reset.ok"

/* ---------- геометрия и материал --------------------------------- */
#define ZONE_VESSEL_LIQ  9     /* id зоны внутри сосуда */
#define WATER_PHASE_IDX   1     /* «water» в списке фаз   */
#define VOF_THR        0.50     /* min VOF для учёта      */
#define RHO_W       1000.0      /* плотность воды, кг/м³  */
#define M_VESSEL       0.035     /* масса пустого сосуда   */
/* собственный тензор инерции пустого сосуда (КС сосуда) */
#define I_VES_X  1.2e-3
#define I_VES_Y  1.2e-3
#define I_VES_Z  2.3e-3
/* смещение КС сосуда (если STL-центр не совпал) */
static const real vessel_offset[3] = {0.0, 0.0, 0.0};

/*------------------------------------------------------------------*/
static real pose[13];          /* cx cy cz q0-q3 vx vy vz wx wy wz */
static real cx, cy, cz, q[4], velocity[3], omega[3];
static real t_prev = 0.0;

/*------------------------------------------------- утилиты --------*/
static int flag_exists(const char *f)
{
    struct stat st;
    return !stat(f, &st);
}

/* q∘v∘q*  — вращение вектора v кватернионом q */
static void quat_rotate(const real q[4], const real v[3], real out[3])
{
    real s = q[0], x = q[1], y = q[2], z = q[3];
    real t[3] = {
        2*(y*v[2] - z*v[1]),
        2*(z*v[0] - x*v[2]),
        2*(x*v[1] - y*v[0])
    };
    out[0] = v[0] + s*t[0] + (y*t[2] - z*t[1]);
    out[1] = v[1] + s*t[1] + (z*t[0] - x*t[2]);
    out[2] = v[2] + s*t[2] + (x*t[1] - y*t[0]);
}

/*------------------------------------------------------------------*/
/* 1. 6-DOF движение чашки */
DEFINE_CG_MOTION(cup_ext, dt, vel, omega, time, dtime)
{
    DT_CG(dt)[0] = cx; DT_CG(dt)[1] = cy; DT_CG(dt)[2] = cz;
    DT_Q(dt)[0]  = q[1]; DT_Q(dt)[1]  = q[2]; DT_Q(dt)[2] = q[3]; DT_Q(dt)[3] = q[0];

    /* скорости не нужны — зануляем */
    NV_S(vel  , =, 0.0);
    NV_S(omega, =, 0.0);
}

/* 2. Считываем pose.dat (или ждём) */
DEFINE_ADJUST(read_pose_from_simulink, domain)
{
    real t = CURRENT_TIME;
    if (fabs(t - t_prev) < 1e-12) return;  /* уже отработали на этом шаге */
    t_prev = t;

    if (I_AM_NODE_ZERO_P) {

        /* несколько попыток увидеть pose.ok */
        for (int k = 0; k < MAX_ATTEMPTS; ++k) {

            if (flag_exists(FLAG_IN)) {
                FILE *fp = fopen(POSE_IN, "r");
                if (fp) {
                    int ok = fscanf(fp,
                        "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                        &pose[0],  &pose[1],  &pose[2],  &pose[3],  &pose[4],
                        &pose[5],  &pose[6],  &pose[7],  &pose[8],  &pose[9],
                        &pose[10], &pose[11], &pose[12]);
                    fclose(fp);

                    if (ok == 13)
                        Message("Pose read (t = %.4g)\n", t);
                    else
                        Message("Warning: pose.dat corrupted (%d/13 numbers)\n", ok);
                } else {
                    Message("Warning: cannot open %s\n", POSE_IN);
                }

                /* флаг обработан — удаляем и выходим */
                remove(FLAG_IN);
                break;
            }

            /* флага нет — ждём и пробуем снова */
            usleep(WAIT_US);
        }
    }

    /* рассылаем позу всем ранкам */
    PRF_BCAST_REAL(pose, 13, 0);

    cx = pose[0]; cy = pose[1]; cz = pose[2];
    memcpy(q,       &pose[3],  4*sizeof(real));
    memcpy(velocity,&pose[7],  3*sizeof(real));
    memcpy(omega,   &pose[10], 3*sizeof(real));
}

/* 3. В конце шага вычисляем свойства жидкости и пишем feed.dat */
DEFINE_EXECUTE_AT_END(write_props_to_simulink)
{
    Domain *d = Get_Domain(1);
    Thread *t; cell_t c;

    real m_liq = 0.0, M1[3] = {0.0}, vol_spill = 0.0;
    real I_liq[6] = {0.0};  /* тензор жидкости */

    thread_loop_c(t, d) {                /* все cell-зоны */
        if (!FLUID_THREAD_P(t)) continue;

        Thread *tw = THREAD_SUB_THREAD(t, WATER_PHASE_IDX);
        if (!tw) continue;               /* нет фазы «water» */

        begin_c_loop_int(c, t) {         /* owner-ячейки, без ghost */
            real vf = C_VOF(c, tw);
            if (vf < VOF_THR) continue;

            bool inside = (THREAD_ID(t) == ZONE_VESSEL_LIQ);
            real vol = C_VOLUME(c, t)*vf;

            if (!inside) {               /* это разлив */
                vol_spill += vol;
                continue;
            }

            real dm = RHO_W*vol;
            real xc[ND_ND]; C_CENTROID(xc, c, t);

            m_liq  += dm;
            M1[0]  += dm*xc[0];
            M1[1]  += dm*xc[1];
            M1[2]  += dm*xc[2];

            I_liq[0] += dm*(xc[1]*xc[1] + xc[2]*xc[2]);   /* Ixx */
            I_liq[1] += dm*(xc[0]*xc[0] + xc[2]*xc[2]);   /* Iyy */
            I_liq[2] += dm*(xc[0]*xc[0] + xc[1]*xc[1]);   /* Izz */
            I_liq[3] -= dm*xc[0]*xc[1];                  /* Ixy */
            I_liq[4] -= dm*xc[1]*xc[2];                  /* Iyz */
            I_liq[5] -= dm*xc[0]*xc[2];                  /* Ixz */
        }
        end_c_loop_int(c, t)
    }

    /* MPI-суммы */
    m_liq       = PRF_GRSUM1(m_liq);
    for (int i = 0; i < 3; ++i) M1[i] = PRF_GRSUM1(M1[i]);
    for (int i = 0; i < 6; ++i) I_liq[i] = PRF_GRSUM1(I_liq[i]);
    vol_spill   = PRF_GRSUM1(vol_spill);

    if (!I_AM_NODE_ZERO_P) return;       /* только мастер ранк ниже */

    /* COM жидкости */
    real com_l[3] = {0.0, 0.0, 0.0};
    if (m_liq > 0.0) {
        com_l[0] = M1[0]/m_liq;
        com_l[1] = M1[1]/m_liq;
        com_l[2] = M1[2]/m_liq;
    }

    /* COM пустого сосуда в мировой КС */
    real off_w[3]; quat_rotate(q, vessel_offset, off_w);
    real com_v[3] = {cx + off_w[0], cy + off_w[1], cz + off_w[2]};

    /* итоговая масса и COM системы */
    real m_tot = M_VESSEL + m_liq;
    real com[3] = {
        (M_VESSEL*com_v[0] + m_liq*com_l[0]) / m_tot,
        (M_VESSEL*com_v[1] + m_liq*com_l[1]) / m_tot,
        (M_VESSEL*com_v[2] + m_liq*com_l[2]) / m_tot
    };

    /* итоговый тензор инерции (жидкость + сосуд) */
    real I_tot[6] = {
        I_VES_X + I_liq[0],
        I_VES_Y + I_liq[1],
        I_VES_Z + I_liq[2],
        I_liq[3], I_liq[4], I_liq[5]
    };

    /* ---------- запись feed.dat + feed.ok ------------------------ */
    FILE *fp = fopen(FEED_OUT, "w");
    if (fp) {
        fprintf(fp,
            "%.6e %.6e %.6e %.6e %.6e %.6e %.6e %.6e %.6e %.6e %.6e %.6e\n",
            CURRENT_TIME, m_tot,
            com[0], com[1], com[2],
            I_tot[0], I_tot[1], I_tot[2],
            I_tot[3], I_tot[4], I_tot[5],
            vol_spill);
        fclose(fp);

        FILE *fl = fopen(FLAG_OUT, "w");  /* пустой файл-флаг */
        if (fl) fclose(fl);

        Message("feed.dat written (t = %.3e, m_liq = %.3f kg, spill = %.3e m^3)\n",
                CURRENT_TIME, m_liq, vol_spill);
    } else {
        Message("Warning: cannot open %s for writing\n", FEED_OUT);
    }

    /* ---------- обработка reset.ok -------------------------------- */
    if (access(RESET_FLAG, F_OK) == 0) {
        if (remove(RESET_FLAG) == 0)
            Message("reset.ok processed → model reload requested.\n");
        else
            Message("Warning: can't remove reset.ok (errno = %d)\n", errno);
    }
}

