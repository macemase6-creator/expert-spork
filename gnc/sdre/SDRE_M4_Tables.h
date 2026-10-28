#ifndef SDRE_M4_TABLES_H_
#define SDRE_M4_TABLES_H_

#include <stddef.h>

namespace sdre_m4_data {

struct Table2D { const float *x; size_t nx; const float *y; size_t ny; const float *z; };
struct Table3D { const float *x; size_t nx; const float *y; size_t ny; const float *w; size_t nw; const float *z; };
struct ImpulseFractionCurve { const float *time_s; const float *force_n; const float *prefix_fraction; size_t count; float burn_time_s; double total_impulse_kgf_s; };

extern const Table2D kBoostCmAlpha;
extern const Table2D kBoostCnAlpha;
extern const Table2D kBoostCmQ;
extern const Table2D kBoostCmDelta;
extern const Table2D kBoostClP;
extern const Table3D kBoostClDelta;
extern const Table2D kSeparatedCmAlpha;
extern const Table2D kSeparatedCnAlpha;
extern const Table2D kSeparatedCmQ;
extern const Table2D kSeparatedCmDelta;
extern const Table2D kSeparatedClP;
extern const Table3D kSeparatedClDelta;
extern const Table2D kTalBoostCmAlpha;
extern const Table2D kTalBoostCnAlpha;
extern const Table2D kTalBoostCmQ;
extern const Table2D kTalBoostCmDelta;
extern const Table2D kTalBoostClP;
extern const Table3D kTalBoostClDelta;
extern const Table2D kTalSeparatedCmAlpha;
extern const Table2D kTalSeparatedCnAlpha;
extern const Table2D kTalSeparatedCmQ;
extern const Table2D kTalSeparatedCmDelta;
extern const Table2D kTalSeparatedClP;
extern const Table3D kTalSeparatedClDelta;
extern const ImpulseFractionCurve kStage1ImpulseFraction;
extern const ImpulseFractionCurve kTalStage1ImpulseFraction;
extern const ImpulseFractionCurve kStage2ImpulseFraction;

}  // namespace sdre_m4_data

#endif
