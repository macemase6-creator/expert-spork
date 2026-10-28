#ifndef SDRE_M4_AERO_H_
#define SDRE_M4_AERO_H_

#include "SDRE_M4_Tables.h"

namespace sdre_m4_data {

float Clamp(float value, float minimum, float maximum);
float Interp2Legacy(const Table2D &table, float x, float y);
float Interp3Legacy(const Table3D &table, float x, float y, float w);
float SlopeZeroToTwo(const Table2D &table, float mach);
float RollDerivativeZeroToTwo(const Table3D &table,
                              float angle_deg,
                              float mach);
double LookupImpulseFraction(const ImpulseFractionCurve &curve,
                             double time_s);

}  // namespace sdre_m4_data

#endif
