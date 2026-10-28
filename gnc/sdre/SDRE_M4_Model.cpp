#include "SDRE_M4_Model.h"

#include "SDRE_M4_Aero.h"
#include "SDRE_M4_Tables.h"

#include <cmath>
#include <cstring>

namespace {

const double kPi = 3.14159265358979323846;
const double kRadToDeg = 57.29577951308232;
/* Match M273N.CPP and both RAJEH2 workbooks (Lref=0.397 m,
   Aref=0.123786 m^2) in simulation and firmware. */
const double kBoostDiameterM = 0.397;
const double kBoostAreaM2 = 0.123786;
const double kSeparatedDiameterM = 0.397;
const double kSeparatedAreaM2 = 0.123786;

struct MassEndpoints {
    double full_mass, empty_mass;
    double full_ixx, empty_ixx;
    double full_iyy, empty_iyy;
};

const MassEndpoints kBaselineStage1 = {
    628.525450, 217.667450,
    13.434399, 5.450438,
    538.321029, 311.273479
};
const MassEndpoints kTalStage1 = {
    725.0, 208.0,
    15.582084, 5.726254,
    789.172151, 387.073118
};
const MassEndpoints kStage2 = {
    104.745184, 104.745184,
    1.833941, 1.833941,
    13.866809, 13.866809
};
const MassEndpoints kTalStage2 = {
    115.0, 107.745184,
    1.833941, 1.833941,
    13.866809, 13.866809
};

double clamp_double(double value, double minimum, double maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

bool finite_value(double value) { return std::isfinite(value); }

double interpolate_endpoint(double full, double empty, double fraction)
{
    return full - (full - empty) * fraction;
}

double combined_aero_angle_deg(double alpha_rad, double beta_rad)
{
    const double radial_ratio = std::hypot(std::tan(alpha_rad), std::tan(beta_rad));
    if (!finite_value(radial_ratio)) return 20.0;
    double angle_rad = std::atan(radial_ratio);
    if (std::cos(alpha_rad) < 0.0 || std::cos(beta_rad) < 0.0)
        angle_rad = kPi - angle_rad;
    return clamp_double(std::fabs(angle_rad) * kRadToDeg, 0.0, 20.0);
}

bool valid_input(const SdreM4Inputs &input)
{
    return finite_value(input.time_s) && input.time_s >= 0.0 &&
           finite_value(input.dynamic_pressure_pa) && input.dynamic_pressure_pa >= 0.0 &&
           finite_value(input.velocity_mps) && input.velocity_mps > 0.0 &&
           finite_value(input.mach) && finite_value(input.alpha_rad) &&
           finite_value(input.beta_rad) &&
           finite_value(input.second_stage_ignition_time_s) &&
           input.stage >= 1 && input.stage <= 6 && input.second_separation == 0;
}

bool valid_table(const sdre_m4_data::Table2D &table)
{
    return table.x && table.y && table.z && table.nx >= 2u && table.ny >= 2u;
}

bool valid_table(const sdre_m4_data::Table3D &table)
{
    return table.x && table.y && table.w && table.z &&
           table.nx >= 2u && table.ny >= 2u && table.nw >= 2u;
}

}  // namespace

bool SdreM4_ComputeMassProperties(const SdreM4Inputs *input,
                                  SdreM4MassProperties *properties)
{
    if (!properties) return false;
    std::memset(properties, 0, sizeof(*properties));
    if (!input || !valid_input(*input)) return false;

    const MassEndpoints *endpoints =
        input->tal_configuration ? &kTalStage1 : &kBaselineStage1;
    const sdre_m4_data::ImpulseFractionCurve *curve = input->tal_configuration ?
        &sdre_m4_data::kTalStage1ImpulseFraction :
        &sdre_m4_data::kStage1ImpulseFraction;
    double elapsed = input->time_s;
    int burned_out = input->first_stage_burned_out;

    if (input->separated) {
        endpoints = input->tal_configuration ? &kTalStage2 : &kStage2;
        curve = &sdre_m4_data::kStage2ImpulseFraction;
        elapsed = input->time_s - input->second_stage_ignition_time_s;
        burned_out = input->second_stage_burned_out;
        if (!input->second_stage_ignition_started || elapsed <= 0.0)
            elapsed = 0.0;
    }

    double fraction = burned_out ? 1.0 :
        sdre_m4_data::LookupImpulseFraction(*curve, elapsed);
    fraction = clamp_double(fraction, 0.0, 1.0);
    properties->mass_kg = interpolate_endpoint(
        endpoints->full_mass, endpoints->empty_mass, fraction);
    properties->inertia_roll_kg_m2 = interpolate_endpoint(
        endpoints->full_ixx, endpoints->empty_ixx, fraction);
    properties->inertia_transverse_kg_m2 = interpolate_endpoint(
        endpoints->full_iyy, endpoints->empty_iyy, fraction);
    properties->consumed_fraction = fraction;
    return finite_value(properties->mass_kg) && properties->mass_kg > 0.0 &&
           finite_value(properties->inertia_roll_kg_m2) &&
           properties->inertia_roll_kg_m2 > 0.0 &&
           finite_value(properties->inertia_transverse_kg_m2) &&
           properties->inertia_transverse_kg_m2 > 0.0;
}

uint32_t SdreM4_BuildOperatingPoint(const SdreM4Inputs *input,
                                    SdreOperatingPoint *output)
{
    if (!output) return 0u;
    std::memset(output, 0, sizeof(*output));
    if (!input || !valid_input(*input)) return 0u;

    SdreM4MassProperties mass = {};
    if (!SdreM4_ComputeMassProperties(input, &mass)) return 0u;

    const bool separated = input->separated != 0;
    const double diameter = separated ? kSeparatedDiameterM : kBoostDiameterM;
    const double area = separated ? kSeparatedAreaM2 : kBoostAreaM2;
    const double velocity = input->velocity_mps;
    const double force_velocity = velocity > 1.0 ? velocity : 1.0;
    const double torque_scale = input->dynamic_pressure_pa * area * diameter;
    const double rate_scale = diameter / (2.0 * velocity) * kRadToDeg;

    const bool tal = input->tal_configuration != 0;
    const sdre_m4_data::Table2D &cm_alpha_table = tal ?
        (separated ? sdre_m4_data::kTalSeparatedCmAlpha : sdre_m4_data::kTalBoostCmAlpha) :
        (separated ? sdre_m4_data::kSeparatedCmAlpha : sdre_m4_data::kBoostCmAlpha);
    const sdre_m4_data::Table2D &cn_alpha_table = tal ?
        (separated ? sdre_m4_data::kTalSeparatedCnAlpha : sdre_m4_data::kTalBoostCnAlpha) :
        (separated ? sdre_m4_data::kSeparatedCnAlpha : sdre_m4_data::kBoostCnAlpha);
    const sdre_m4_data::Table2D &cm_q_table = tal ?
        (separated ? sdre_m4_data::kTalSeparatedCmQ : sdre_m4_data::kTalBoostCmQ) :
        (separated ? sdre_m4_data::kSeparatedCmQ : sdre_m4_data::kBoostCmQ);
    const sdre_m4_data::Table2D &cm_delta_table = tal ?
        (separated ? sdre_m4_data::kTalSeparatedCmDelta : sdre_m4_data::kTalBoostCmDelta) :
        (separated ? sdre_m4_data::kSeparatedCmDelta : sdre_m4_data::kBoostCmDelta);
    const sdre_m4_data::Table2D &cl_p_table = tal ?
        (separated ? sdre_m4_data::kTalSeparatedClP : sdre_m4_data::kTalBoostClP) :
        (separated ? sdre_m4_data::kSeparatedClP : sdre_m4_data::kBoostClP);
    const sdre_m4_data::Table3D &cl_delta_table = tal ?
        (separated ? sdre_m4_data::kTalSeparatedClDelta : sdre_m4_data::kTalBoostClDelta) :
        (separated ? sdre_m4_data::kSeparatedClDelta : sdre_m4_data::kBoostClDelta);

    const float mach = static_cast<float>(clamp_double(input->mach, 0.3, 10.0));
    const float alpha_deg = static_cast<float>(clamp_double(
        std::fabs(input->alpha_rad) * kRadToDeg, 0.0, 20.0));
    const float beta_deg = static_cast<float>(clamp_double(
        std::fabs(input->beta_rad) * kRadToDeg, 0.0, 20.0));
    const float combined_deg = static_cast<float>(
        combined_aero_angle_deg(input->alpha_rad, input->beta_rad));

    const double cm_alpha = sdre_m4_data::SlopeZeroToTwo(cm_alpha_table, mach);
    const double cn_force = sdre_m4_data::SlopeZeroToTwo(cn_alpha_table, mach);
    const double cm_delta = sdre_m4_data::SlopeZeroToTwo(cm_delta_table, mach);
    const double cm_q_alpha = sdre_m4_data::Interp2Legacy(cm_q_table, alpha_deg, mach);
    const double cm_q_beta = sdre_m4_data::Interp2Legacy(cm_q_table, beta_deg, mach);
    const double cl_p = sdre_m4_data::Interp2Legacy(cl_p_table, combined_deg, mach);
    const double cl_delta = sdre_m4_data::RollDerivativeZeroToTwo(
        cl_delta_table, combined_deg, mach);

    output->roll.torque_scale_nm = torque_scale;
    output->roll.rate_scale_s = rate_scale;
    output->roll.inertia_roll_kg_m2 = mass.inertia_roll_kg_m2;
    output->roll.cl_p = cl_p;
    output->roll.cl_delta_a_per_deg = cl_delta;

    output->pitch.torque_scale_nm = torque_scale;
    output->pitch.rate_scale_s = rate_scale;
    output->pitch.inertia_pitch_kg_m2 = mass.inertia_transverse_kg_m2;
    output->pitch.cm_q = cm_q_alpha;
    output->pitch.cm_alpha_per_deg = cm_alpha;
    output->pitch.cm_delta_e_per_deg = cm_delta;
    output->pitch.z_alpha_per_s = input->dynamic_pressure_pa * area * cn_force *
        kRadToDeg / (mass.mass_kg * force_velocity);
    output->pitch.velocity_mps = velocity;

    output->yaw.torque_scale_nm = torque_scale;
    output->yaw.rate_scale_s = rate_scale;
    output->yaw.inertia_yaw_kg_m2 = mass.inertia_transverse_kg_m2;
    output->yaw.cn_r = cm_q_beta;
    output->yaw.cn_beta_per_deg = cm_alpha;
    output->yaw.cn_delta_r_per_deg = cm_delta;
    output->yaw.z_beta_per_s = input->dynamic_pressure_pa * area * cn_force *
        kRadToDeg / (mass.mass_kg * force_velocity);
    output->yaw.velocity_mps = velocity;

    output->valid_mask = SDRE_MODEL_ALL_VALID;
    return output->valid_mask;
}

bool SdreM4_ModelSelfTest(void)
{
    using namespace sdre_m4_data;
    if (!valid_table(kBoostCmAlpha) || !valid_table(kBoostCnAlpha) ||
        !valid_table(kBoostCmQ) || !valid_table(kBoostCmDelta) ||
        !valid_table(kBoostClP) || !valid_table(kBoostClDelta) ||
        !valid_table(kSeparatedCmAlpha) || !valid_table(kSeparatedCnAlpha) ||
        !valid_table(kSeparatedCmQ) || !valid_table(kSeparatedCmDelta) ||
        !valid_table(kSeparatedClP) || !valid_table(kSeparatedClDelta) ||
        !valid_table(kTalBoostCmAlpha) || !valid_table(kTalBoostCnAlpha) ||
        !valid_table(kTalBoostCmQ) || !valid_table(kTalBoostCmDelta) ||
        !valid_table(kTalBoostClP) || !valid_table(kTalBoostClDelta) ||
        !valid_table(kTalSeparatedCmAlpha) || !valid_table(kTalSeparatedCnAlpha) ||
        !valid_table(kTalSeparatedCmQ) || !valid_table(kTalSeparatedCmDelta) ||
        !valid_table(kTalSeparatedClP) || !valid_table(kTalSeparatedClDelta))
        return false;
    const double s1 = LookupImpulseFraction(
        kStage1ImpulseFraction, kStage1ImpulseFraction.burn_time_s);
    const double tal_s1 = LookupImpulseFraction(
        kTalStage1ImpulseFraction, kTalStage1ImpulseFraction.burn_time_s);
    const double s2 = LookupImpulseFraction(
        kStage2ImpulseFraction, kStage2ImpulseFraction.burn_time_s);
    return finite_value(s1) && s1 > 0.9 && s1 < 1.1 &&
           finite_value(tal_s1) && tal_s1 > 0.9 && tal_s1 < 1.1 &&
           finite_value(s2) && s2 > 0.9 && s2 < 1.1;
}
