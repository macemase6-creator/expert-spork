#ifndef SDRE_M600_TABLES_H_
#define SDRE_M600_TABLES_H_

#include <stddef.h>
#include <stdint.h>

/* Source SHA-256 (legacy M600 files):
 *   nv_mv600.dat: 51E391067AD99035C42D5223A3A684818A820447D8E77AB35E24134853854C1D
 *   nv_rmv600.dat: 295EBE5354E98B0A73912273336239E689C9AB34432ED05E641C960E059FA797
 *   nv_wv600.dat: EDA955C8284A23EDD4FCDF50EAABBD986CA9047233160D504383148D57A0EA40
 *   nv_rwv600.DAT: 4A1DE72A05B1B11ABC4C31923B72C7B103984391297F469D9F066933AA4024CD
 *   STHR_M591_TST2.dat: 7A6BB25A19E3376854562212A36CEC92BFED48F0410D9B6C2804DDC49BD87FA1
 *   STHR_M591_STG2.dat: 63343B10B505C73843EF69D7F514B53F7A3CF5D11EC1282BAFF32E716A21A94E
 */

namespace sdre_m600_data {

struct Table2D {
    const float *x;
    size_t nx;
    const float *y;
    size_t ny;
    const float *z;  /* packed as z[x_index * ny + y_index] */
};

struct Table3D {
    const float *x;
    size_t nx;
    const float *y;
    size_t ny;
    const float *w;
    size_t nw;
    const float *z;  /* packed as z[(x_index * ny + y_index) * nw + w_index] */
};

struct ImpulseFractionCurve {
    const float *time_s;
    const float *force_n;
    const float *prefix_fraction;
    size_t count;
    float burn_time_s;
    double total_impulse_kgf_s;
};

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

extern const ImpulseFractionCurve kStage1ImpulseFraction;
extern const ImpulseFractionCurve kStage2ImpulseFraction;

static const uint32_t kPackedAeroCrc32 = UINT32_C(0x2534E750);
static const uint32_t kPackedImpulseCrc32 = UINT32_C(0xC8AD4EA9);

}  // namespace sdre_m600_data

#endif  // SDRE_M600_TABLES_H_
