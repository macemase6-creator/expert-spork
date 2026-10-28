#include "SDRE_M600_Aero.h"

namespace sdre_m600_data {
namespace {

size_t LowerBracket(const float *axis, size_t count, float value)
{
    size_t index = 1u;
    while (index < count && value > axis[index]) {
        ++index;
    }
    --index;
    if (index == count - 1u) {
        index = count - 2u;
    }
    return index;
}

bool Valid(const Table2D &table)
{
    return table.x != 0 && table.y != 0 && table.z != 0 &&
           table.nx >= 2u && table.ny >= 2u;
}

bool Valid(const Table3D &table)
{
    return table.x != 0 && table.y != 0 && table.w != 0 && table.z != 0 &&
           table.nx >= 2u && table.ny >= 2u && table.nw >= 2u;
}

}  // namespace

float Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

float Interp2Legacy(const Table2D &table, float x, float y)
{
    if (!Valid(table)) {
        return 0.0f;
    }

    bool negative = false;
    if (x < 0.0f) {
        x = -x;
        negative = true;
    }

    const size_t ix = LowerBracket(table.x, table.nx, x);
    const size_t iy = LowerBracket(table.y, table.ny, y);
    const size_t row0 = ix * table.ny;
    const size_t row1 = (ix + 1u) * table.ny;

    const float c1 = table.z[row0 + iy] +
        (table.z[row0 + iy + 1u] - table.z[row0 + iy]) *
        (y - table.y[iy]) / (table.y[iy + 1u] - table.y[iy]);
    const float c2 = table.z[row1 + iy] +
        (table.z[row1 + iy + 1u] - table.z[row1 + iy]) *
        (y - table.y[iy]) / (table.y[iy + 1u] - table.y[iy]);
    const float c = c1 + (c2 - c1) *
        (x - table.x[ix]) / (table.x[ix + 1u] - table.x[ix]);
    return negative ? -c : c;
}

float Interp3Legacy(const Table3D &table, float x, float y, float w)
{
    if (!Valid(table)) {
        return 0.0f;
    }

    bool negative = false;
    if (w < 0.0f) {
        w = -w;
        negative = true;
    }

    const size_t ix = LowerBracket(table.x, table.nx, x);
    const size_t iy = LowerBracket(table.y, table.ny, y);
    const size_t iw = LowerBracket(table.w, table.nw, w);

    const size_t x0y0 = (ix * table.ny + iy) * table.nw;
    const size_t x0y1 = (ix * table.ny + iy + 1u) * table.nw;
    const size_t x1y0 = ((ix + 1u) * table.ny + iy) * table.nw;
    const size_t x1y1 = ((ix + 1u) * table.ny + iy + 1u) * table.nw;

    const float c1 = table.z[x0y0 + iw] +
        (table.z[x0y1 + iw] - table.z[x0y0 + iw]) *
        (y - table.y[iy]) / (table.y[iy + 1u] - table.y[iy]);
    const float c2 = table.z[x1y0 + iw] +
        (table.z[x1y1 + iw] - table.z[x1y0 + iw]) *
        (y - table.y[iy]) / (table.y[iy + 1u] - table.y[iy]);
    const float c3 = c1 + (c2 - c1) *
        (x - table.x[ix]) / (table.x[ix + 1u] - table.x[ix]);

    const float c4 = table.z[x0y0 + iw + 1u] +
        (table.z[x0y1 + iw + 1u] - table.z[x0y0 + iw + 1u]) *
        (y - table.y[iy]) / (table.y[iy + 1u] - table.y[iy]);
    const float c5 = table.z[x1y0 + iw + 1u] +
        (table.z[x1y1 + iw + 1u] - table.z[x1y0 + iw + 1u]) *
        (y - table.y[iy]) / (table.y[iy + 1u] - table.y[iy]);
    const float c6 = c4 + (c5 - c4) *
        (x - table.x[ix]) / (table.x[ix + 1u] - table.x[ix]);
    const float c = c3 + (c6 - c3) *
        (w - table.w[iw]) / (table.w[iw + 1u] - table.w[iw]);
    return negative ? -c : c;
}

float SlopeZeroToTwo(const Table2D &table, float mach)
{
    return (Interp2Legacy(table, 2.0f, mach) -
            Interp2Legacy(table, 0.0f, mach)) / 2.0f;
}

float RollDerivativeZeroToTwo(const Table3D &table, float angle_deg, float mach)
{
    return (Interp3Legacy(table, angle_deg, mach, 2.0f) -
            Interp3Legacy(table, angle_deg, mach, 0.0f)) / 2.0f;
}

double LookupImpulseFraction(const ImpulseFractionCurve &curve, double time_s)
{
    if (curve.time_s == 0 || curve.force_n == 0 ||
        curve.prefix_fraction == 0 || curve.count == 0u ||
        !(curve.total_impulse_kgf_s > 0.0)) {
        return 0.0;
    }
    if (!(time_s > static_cast<double>(curve.time_s[0]))) {
        return static_cast<double>(curve.prefix_fraction[0]);
    }
    if (time_s >= static_cast<double>(curve.time_s[curve.count - 1u])) {
        return static_cast<double>(curve.prefix_fraction[curve.count - 1u]);
    }

    size_t low = 0u;
    size_t high = curve.count - 1u;
    while (high - low > 1u) {
        const size_t middle = low + (high - low) / 2u;
        if (time_s <= static_cast<double>(curve.time_s[middle])) {
            high = middle;
        } else {
            low = middle;
        }
    }
    if (time_s == static_cast<double>(curve.time_s[high])) {
        return static_cast<double>(curve.prefix_fraction[high]);
    }

    const double t0 = static_cast<double>(curve.time_s[low]);
    const double t1 = static_cast<double>(curve.time_s[high]);
    const double f0 = static_cast<double>(curve.force_n[low]);
    const double f1 = static_cast<double>(curve.force_n[high]);
    const double dt = time_s - t0;
    const double interval = t1 - t0;
    const double impulse_ns = f0 * dt +
        0.5 * (f1 - f0) / interval * dt * dt;
    return static_cast<double>(curve.prefix_fraction[low]) +
        impulse_ns / (9.81 * curve.total_impulse_kgf_s);
}

}  // namespace sdre_m600_data
