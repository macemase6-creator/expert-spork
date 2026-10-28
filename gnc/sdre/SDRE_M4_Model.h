#ifndef SDRE_M4_MODEL_H_
#define SDRE_M4_MODEL_H_

#include <stdint.h>

#include "SDRE.h"

struct SdreM4Inputs {
    double time_s;
    double dynamic_pressure_pa;
    double velocity_mps;
    double mach;
    double alpha_rad;
    double beta_rad;
    double second_stage_ignition_time_s;
    int stage;
    int separated;
    int second_separation;
    int first_stage_burned_out;
    int second_stage_ignition_started;
    int second_stage_burned_out;
    int tal_configuration;
};

struct SdreM4MassProperties {
    double mass_kg;
    double inertia_roll_kg_m2;
    double inertia_transverse_kg_m2;
    double consumed_fraction;
};

bool SdreM4_ComputeMassProperties(const SdreM4Inputs *input,
                                  SdreM4MassProperties *properties);
uint32_t SdreM4_BuildOperatingPoint(const SdreM4Inputs *input,
                                    SdreOperatingPoint *output);
bool SdreM4_ModelSelfTest(void);

#endif /* SDRE_M4_MODEL_H_ */
