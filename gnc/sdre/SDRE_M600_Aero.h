#ifndef SDRE_M600_AERO_H_
#define SDRE_M600_AERO_H_

#include "SDRE_M600_Tables.h"

namespace sdre_m600_data {

/* Saturating scalar clamp.  NaN is returned unchanged. */
float Clamp(float value, float minimum, float maximum);

/*
 * Bilinear interpolation with the legacy M600 bracket and arithmetic order.
 * The arguments follow the descriptor axes: x (angle), then y (Mach).
 * Values outside an axis are linearly extrapolated.  Negative x uses the
 * legacy odd-symmetry convention: interpolate abs(x), then negate the result.
 */
float Interp2Legacy(const Table2D &table, float x, float y);

/*
 * Trilinear interpolation with the legacy M600 bracket and arithmetic order.
 * Arguments follow x (angle), y (Mach), w (control deflection).  Values are
 * extrapolated at the ends.  Negative w has the legacy odd symmetry.
 */
float Interp3Legacy(const Table3D &table, float x, float y, float w);

/* Exact operations used by SDRE to turn 0/2-degree samples into derivatives. */
float SlopeZeroToTwo(const Table2D &table, float mach);
float RollDerivativeZeroToTwo(const Table3D &table, float angle_deg, float mach);

/*
 * Analytic integral of the piecewise-linear thrust curve, normalized by the
 * legacy footer total impulse (kgf*s).  The stored final fraction is returned
 * at and after burn_time_s; it is intentionally not forced to exactly 1.0.
 */
double LookupImpulseFraction(const ImpulseFractionCurve &curve, double time_s);

}  // namespace sdre_m600_data

#endif  // SDRE_M600_AERO_H_
