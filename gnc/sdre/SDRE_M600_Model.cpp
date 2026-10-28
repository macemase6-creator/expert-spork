#include "SDRE_M600_Model.h"

#include "SDRE_M600_Aero.h"
#include "SDRE_M600_Tables.h"

#include <cmath>
#include <cstring>

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kRadToDeg = 57.295779513082320876798154814105;
constexpr double kBoostDiameterM = 0.602;
constexpr double kBoostAreaM2 =
    0.25 * kBoostDiameterM * kBoostDiameterM * kPi;
constexpr double kSeparatedDiameterM = 0.1;
/* This apparently unusual value is intentional and matches M600 AERO.CPP. */
constexpr double kSeparatedAreaM2 = 0.1;

constexpr double kStage1FullMassKg = 2635.0;
constexpr double kStage1EmptyMassKg = 640.0;
constexpr double kStage1FullIxxKgM2 = 119.97;
constexpr double kStage1EmptyIxxKgM2 = 37.25;
constexpr double kStage1FullIyyKgM2 = 8027.7;
constexpr double kStage1EmptyIyyKgM2 = 3875.6;

constexpr double kStage2FullMassKg = 360.0;
constexpr double kStage2EmptyMassKg = 320.0;
constexpr double kStage2FullIxxKgM2 = 8.5;
constexpr double kStage2EmptyIxxKgM2 = 9.1;
constexpr double kStage2FullIyyKgM2 = 124.0;
constexpr double kStage2EmptyIyyKgM2 = 97.2;

constexpr double kSeparatedMachMinimum = 0.3;
constexpr double kSeparatedRollMachMinimum = 0.8;
constexpr double kSeparatedMachMaximum = 15.0;
constexpr double kDampingAngleMaximumDeg = 20.0;

double clamp_double(double value, double minimum, double maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

bool finite_value(double value)
{
    return std::isfinite(value);
}

double interpolate_endpoint(double full, double empty, double fraction)
{
    return full - (full - empty) * fraction;
}

double combined_aero_angle_deg(double alpha_rad, double beta_rad)
{
    /* M600 AERO defines alfab as atan2(hypot(vy,vz), vx), not as the
     * small-angle hypot(alpha,beta).  Reconstruct it from the two atan2
     * angles.  A rear-hemisphere/inconsistent input is conservatively in the
     * clamped region, and non-finite trig output cannot reach a table. */
    const double tan_alpha = std::tan(alpha_rad);
    const double tan_beta = std::tan(beta_rad);
    const double radial_ratio = std::hypot(tan_alpha, tan_beta);
    if (!finite_value(radial_ratio)) {
        return kDampingAngleMaximumDeg;
    }

    double angle_rad = std::atan(radial_ratio);
    if (std::cos(alpha_rad) < 0.0 || std::cos(beta_rad) < 0.0) {
        angle_rad = kPi - angle_rad;
    }
    return clamp_double(
        std::fabs(angle_rad) * kRadToDeg,
        0.0,
        kDampingAngleMaximumDeg);
}

bool valid_curve(const sdre_m600_data::ImpulseFractionCurve &curve)
{
    return curve.time_s != nullptr &&
           curve.force_n != nullptr &&
           curve.prefix_fraction != nullptr &&
           curve.count >= 2u &&
           curve.burn_time_s > 0.0f &&
           curve.total_impulse_kgf_s > 0.0;
}

bool valid_table2(const sdre_m600_data::Table2D &table)
{
    return table.x != nullptr && table.y != nullptr && table.z != nullptr &&
           table.nx >= 2u && table.ny >= 2u;
}

bool valid_table3(const sdre_m600_data::Table3D &table)
{
    return table.x != nullptr && table.y != nullptr && table.w != nullptr &&
           table.z != nullptr && table.nx >= 2u && table.ny >= 2u &&
           table.nw >= 2u;
}

bool valid_common_input(const SdreM600Inputs &input)
{
    return finite_value(input.time_s) && input.time_s >= 0.0 &&
           finite_value(input.dynamic_pressure_pa) &&
           input.dynamic_pressure_pa >= 0.0 &&
           finite_value(input.velocity_mps) && input.velocity_mps > 0.0 &&
           finite_value(input.mach) &&
           finite_value(input.alpha_rad) &&
           finite_value(input.beta_rad) &&
           finite_value(input.second_stage_ignition_time_s) &&
           input.stage >= 1 && input.stage <= 6 &&
           input.second_separation == 0;
}

bool finite_roll(const SdreRollOperatingPoint &op)
{
    return finite_value(op.torque_scale_nm) &&
           finite_value(op.rate_scale_s) &&
           finite_value(op.inertia_roll_kg_m2) &&
           finite_value(op.cl_p) &&
           finite_value(op.cl_delta_a_per_deg) &&
           op.inertia_roll_kg_m2 > 0.0;
}

bool finite_pitch(const SdrePitchOperatingPoint &op)
{
    return finite_value(op.torque_scale_nm) &&
           finite_value(op.rate_scale_s) &&
           finite_value(op.inertia_pitch_kg_m2) &&
           finite_value(op.cm_q) &&
           finite_value(op.cm_alpha_per_deg) &&
           finite_value(op.cm_delta_e_per_deg) &&
           finite_value(op.z_alpha_per_s) &&
           finite_value(op.velocity_mps) &&
           op.inertia_pitch_kg_m2 > 0.0;
}

bool finite_yaw(const SdreYawOperatingPoint &op)
{
    return finite_value(op.torque_scale_nm) &&
           finite_value(op.rate_scale_s) &&
           finite_value(op.inertia_yaw_kg_m2) &&
           finite_value(op.cn_r) &&
           finite_value(op.cn_beta_per_deg) &&
           finite_value(op.cn_delta_r_per_deg) &&
           finite_value(op.z_beta_per_s) &&
           finite_value(op.velocity_mps) &&
           op.inertia_yaw_kg_m2 > 0.0;
}

}  // namespace

bool SdreM600_ComputeMassProperties(
    const SdreM600Inputs *input,
    SdreM600MassProperties *properties)
{
    if (properties == nullptr) {
        return false;
    }
    std::memset(properties, 0, sizeof(*properties));

    if (input == nullptr || !valid_common_input(*input)) {
        return false;
    }

    double fraction = 0.0;
    double full_mass = kStage1FullMassKg;
    double empty_mass = kStage1EmptyMassKg;
    double full_ixx = kStage1FullIxxKgM2;
    double empty_ixx = kStage1EmptyIxxKgM2;
    double full_iyy = kStage1FullIyyKgM2;
    double empty_iyy = kStage1EmptyIyyKgM2;

    if (input->separated == 0) {
        const auto &curve = sdre_m600_data::kStage1ImpulseFraction;
        if (!valid_curve(curve)) {
            return false;
        }
        if (input->first_stage_burned_out != 0 ||
            input->time_s > static_cast<double>(curve.burn_time_s)) {
            fraction = 1.0;
        } else {
            fraction = sdre_m600_data::LookupImpulseFraction(curve, input->time_s);
        }
    } else {
        const auto &curve = sdre_m600_data::kStage2ImpulseFraction;
        if (!valid_curve(curve)) {
            return false;
        }

        full_mass = kStage2FullMassKg;
        empty_mass = kStage2EmptyMassKg;
        full_ixx = kStage2FullIxxKgM2;
        empty_ixx = kStage2EmptyIxxKgM2;
        full_iyy = kStage2FullIyyKgM2;
        empty_iyy = kStage2EmptyIyyKgM2;

        const double elapsed_s =
            input->time_s - input->second_stage_ignition_time_s;
        if (input->second_stage_burned_out != 0 ||
            (input->second_stage_ignition_started != 0 &&
             elapsed_s > static_cast<double>(curve.burn_time_s))) {
            fraction = 1.0;
        } else if (input->second_stage_ignition_started != 0 && elapsed_s > 0.0) {
            fraction = sdre_m600_data::LookupImpulseFraction(curve, elapsed_s);
        }
    }

    if (!finite_value(fraction)) {
        return false;
    }
    /* Generated nominal curves are monotone.  This bound is defence against
     * corrupt generated data, not a runtime substitution of another model. */
    fraction = clamp_double(fraction, 0.0, 1.0);

    properties->mass_kg = interpolate_endpoint(full_mass, empty_mass, fraction);
    properties->inertia_roll_kg_m2 =
        interpolate_endpoint(full_ixx, empty_ixx, fraction);
    properties->inertia_transverse_kg_m2 =
        interpolate_endpoint(full_iyy, empty_iyy, fraction);
    properties->consumed_fraction = fraction;

    return finite_value(properties->mass_kg) && properties->mass_kg > 0.0 &&
           finite_value(properties->inertia_roll_kg_m2) &&
           properties->inertia_roll_kg_m2 > 0.0 &&
           finite_value(properties->inertia_transverse_kg_m2) &&
           properties->inertia_transverse_kg_m2 > 0.0;
}

uint32_t SdreM600_BuildOperatingPoint(
    const SdreM600Inputs *input,
    SdreOperatingPoint *output)
{
    if (output == nullptr) {
        return 0u;
    }
    std::memset(output, 0, sizeof(*output));

    if (input == nullptr || !valid_common_input(*input)) {
        return 0u;
    }

    SdreM600MassProperties mass = {};
    if (!SdreM600_ComputeMassProperties(input, &mass)) {
        return 0u;
    }

    const bool separated = input->separated != 0;
    const double diameter_m =
        separated ? kSeparatedDiameterM : kBoostDiameterM;
    const double area_m2 = separated ? kSeparatedAreaM2 : kBoostAreaM2;
    /* The caller deliberately supplies sqrt(Vm_t_2), before the legacy PID
     * coast path overwrites Vm_t.  Preserve that positive physical value. */
    const double velocity_mps = input->velocity_mps;
    /* The M600 pitch/yaw helper alone floors the velocity used in the normal
     * force derivative.  It does not apply this floor to ddv/rate_scale. */
    const double force_velocity_mps = velocity_mps > 1.0 ? velocity_mps : 1.0;
    const double torque_scale_nm =
        input->dynamic_pressure_pa * area_m2 * diameter_m;
    const double rate_scale_s = diameter_m / (2.0 * velocity_mps) * kRadToDeg;

    float mach = static_cast<float>(input->mach);
    if (separated) {
        mach = static_cast<float>(clamp_double(
            static_cast<double>(mach),
            kSeparatedMachMinimum,
            kSeparatedMachMaximum));
    }
    const float roll_mach = separated
        ? static_cast<float>(clamp_double(
              static_cast<double>(mach),
              kSeparatedRollMachMinimum,
              kSeparatedMachMaximum))
        : mach;

    const float alpha_deg = static_cast<float>(clamp_double(
        std::fabs(input->alpha_rad) * kRadToDeg,
        0.0,
        kDampingAngleMaximumDeg));
    const float beta_deg = static_cast<float>(clamp_double(
        std::fabs(input->beta_rad) * kRadToDeg,
        0.0,
        kDampingAngleMaximumDeg));
    const float combined_angle_deg = static_cast<float>(
        combined_aero_angle_deg(input->alpha_rad, input->beta_rad));

    const auto &cm_alpha_table = separated
        ? sdre_m600_data::kSeparatedCmAlpha
        : sdre_m600_data::kBoostCmAlpha;
    const auto &cn_alpha_table = separated
        ? sdre_m600_data::kSeparatedCnAlpha
        : sdre_m600_data::kBoostCnAlpha;
    const auto &cm_q_table = separated
        ? sdre_m600_data::kSeparatedCmQ
        : sdre_m600_data::kBoostCmQ;
    const auto &cm_delta_table = separated
        ? sdre_m600_data::kSeparatedCmDelta
        : sdre_m600_data::kBoostCmDelta;
    const auto &cl_p_table = separated
        ? sdre_m600_data::kSeparatedClP
        : sdre_m600_data::kBoostClP;
    const auto &cl_delta_table = separated
        ? sdre_m600_data::kSeparatedClDelta
        : sdre_m600_data::kBoostClDelta;

    /* Keep the interpolation inputs/results float: the source M600 aero tables
     * and all of its interpolation intermediates are float. */
    const float cm_alpha =
        sdre_m600_data::SlopeZeroToTwo(cm_alpha_table, mach);
    const float cn_force =
        sdre_m600_data::SlopeZeroToTwo(cn_alpha_table, mach);
    const float cm_delta =
        sdre_m600_data::SlopeZeroToTwo(cm_delta_table, mach);
    const float cm_q_alpha =
        sdre_m600_data::Interp2Legacy(cm_q_table, alpha_deg, mach);
    const float cm_q_beta =
        sdre_m600_data::Interp2Legacy(cm_q_table, beta_deg, mach);
    const float cl_p =
        sdre_m600_data::Interp2Legacy(cl_p_table, combined_angle_deg, roll_mach);
    const float cl_delta = sdre_m600_data::RollDerivativeZeroToTwo(
        cl_delta_table, combined_angle_deg, roll_mach);

    output->roll.torque_scale_nm = torque_scale_nm;
    output->roll.rate_scale_s = rate_scale_s;
    output->roll.inertia_roll_kg_m2 = mass.inertia_roll_kg_m2;
    output->roll.cl_p = static_cast<double>(cl_p);
    output->roll.cl_delta_a_per_deg = static_cast<double>(cl_delta);

    output->pitch.torque_scale_nm = torque_scale_nm;
    output->pitch.rate_scale_s = rate_scale_s;
    output->pitch.inertia_pitch_kg_m2 = mass.inertia_transverse_kg_m2;
    output->pitch.cm_q = static_cast<double>(cm_q_alpha);
    output->pitch.cm_alpha_per_deg = static_cast<double>(cm_alpha);
    output->pitch.cm_delta_e_per_deg = static_cast<double>(cm_delta);
    output->pitch.z_alpha_per_s =
        input->dynamic_pressure_pa * area_m2 * static_cast<double>(cn_force) *
        kRadToDeg / (mass.mass_kg * force_velocity_mps);
    output->pitch.velocity_mps = input->velocity_mps;

    output->yaw.torque_scale_nm = torque_scale_nm;
    output->yaw.rate_scale_s = rate_scale_s;
    output->yaw.inertia_yaw_kg_m2 = mass.inertia_transverse_kg_m2;
    output->yaw.cn_r = static_cast<double>(cm_q_beta);
    /* M600's validated yaw helper deliberately mirrors the pitch moment and
     * control slopes.  Keep the raw control derivative: SDRE.cpp applies the
     * single post-separation sign change. */
    output->yaw.cn_beta_per_deg = static_cast<double>(cm_alpha);
    output->yaw.cn_delta_r_per_deg = static_cast<double>(cm_delta);
    output->yaw.z_beta_per_s =
        input->dynamic_pressure_pa * area_m2 * static_cast<double>(cn_force) *
        kRadToDeg / (mass.mass_kg * force_velocity_mps);
    output->yaw.velocity_mps = input->velocity_mps;

    if (finite_roll(output->roll)) {
        output->valid_mask |= SDRE_MODEL_ROLL_VALID;
    }
    if (finite_pitch(output->pitch)) {
        output->valid_mask |= SDRE_MODEL_PITCH_VALID;
    }
    if (finite_yaw(output->yaw)) {
        output->valid_mask |= SDRE_MODEL_YAW_VALID;
    }
    return output->valid_mask;
}

bool SdreM600_ModelSelfTest(void)
{
    using namespace sdre_m600_data;

    if (!valid_curve(kStage1ImpulseFraction) ||
        !valid_curve(kStage2ImpulseFraction) ||
        !valid_table2(kBoostCmAlpha) ||
        !valid_table2(kBoostCnAlpha) ||
        !valid_table2(kBoostCmQ) ||
        !valid_table2(kBoostCmDelta) ||
        !valid_table2(kBoostClP) ||
        !valid_table3(kBoostClDelta) ||
        !valid_table2(kSeparatedCmAlpha) ||
        !valid_table2(kSeparatedCnAlpha) ||
        !valid_table2(kSeparatedCmQ) ||
        !valid_table2(kSeparatedCmDelta) ||
        !valid_table2(kSeparatedClP) ||
        !valid_table3(kSeparatedClDelta)) {
        return false;
    }

    const double stage1_end = LookupImpulseFraction(
        kStage1ImpulseFraction,
        static_cast<double>(kStage1ImpulseFraction.burn_time_s));
    const double stage2_end = LookupImpulseFraction(
        kStage2ImpulseFraction,
        static_cast<double>(kStage2ImpulseFraction.burn_time_s));
    const float probes[] = {
        SlopeZeroToTwo(kBoostCmAlpha, 1.0f),
        SlopeZeroToTwo(kBoostCnAlpha, 1.0f),
        SlopeZeroToTwo(kBoostCmDelta, 1.0f),
        Interp2Legacy(kBoostCmQ, 2.0f, 1.0f),
        Interp2Legacy(kBoostClP, 2.0f, 1.0f),
        RollDerivativeZeroToTwo(kBoostClDelta, 2.0f, 1.0f),
        SlopeZeroToTwo(kSeparatedCmAlpha, 3.0f),
        RollDerivativeZeroToTwo(kSeparatedClDelta, 2.0f, 3.0f)
    };
    for (float value : probes) {
        if (!std::isfinite(value)) {
            return false;
        }
    }

    /* The nominal footer and trapezoidal integral are close but not identical;
     * M600 changes to the exact empty endpoint when motor_state becomes 2. */
    return finite_value(stage1_end) && stage1_end > 0.95 && stage1_end < 1.05 &&
           finite_value(stage2_end) && stage2_end > 0.95 && stage2_end < 1.05;
}
