/* This file generated automatically. */
/*          Do not modify.            */
#include "udf.h"
#include "prop.h"
#include "dpm.h"
extern DEFINE_CG_MOTION(cup_ext, dt, vel, omega, time, dtime);
extern DEFINE_ADJUST(read_pose_from_simulink, domain);
extern DEFINE_EXECUTE_AT_END(write_props_to_simulink);
UDF_Data udf_data[] = {
{"cup_ext", (void (*)(void))cup_ext, UDF_TYPE_CG_MOTION},
{"read_pose_from_simulink", (void (*)(void))read_pose_from_simulink, UDF_TYPE_ADJUST},
{"write_props_to_simulink", (void (*)(void))write_props_to_simulink, UDF_TYPE_EXECUTE_AT_END},
};
int n_udf_data = sizeof(udf_data)/sizeof(UDF_Data);
#include "version.h"
void UDF_Inquire_Release(int *major, int *minor, int *revision)
{
  *major = RampantReleaseMajor;
  *minor = RampantReleaseMinor;
  *revision = RampantReleaseRevision;
}
