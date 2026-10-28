#ifndef SDRE_M600_MODEL_H_
#define SDRE_M600_MODEL_H_

#include <stdint.h>

#include "SDRE.h"

/* Target loading_data.flags[3] convention, used only for the one-time
 * compatibility gate in INIT.cpp.  The provider itself is fixed to the
 * validated M600 MASSCFG=1 (heavy) model. */
enum {
    SDRE_M600_TARGET_MASS_FLAG_HEAVY = 0,
    SDRE_M600_TARGET_MASS_FLAG_LIGHT = 1
};

struct SdreM600Inputs {
    double time_s;
    double dynamic_pressure_pa;
    double velocity_mps;
    double mach;
    double alpha_rad;
    double beta_rad;
    /* Actual thrust-curve origin.  Target caller supplies t_ign2+ign2_dely. */
    double second_stage_ignition_time_s;

    int stage;
    int separated;
    int second_separation;
    int first_stage_burned_out;
    int second_stage_ignition_started;
    int second_stage_burned_out;
};

struct SdreM600MassProperties {
    double mass_kg;
    double inertia_roll_kg_m2;
    double inertia_transverse_kg_m2;
    double consumed_fraction;
};

/* Compute the active M600 MASSCFG=1 (heavy) mass/inertia schedule.  Loading
 * compatibility is checked once in INIT.cpp, after packet checksum; it is not
 * a mutable flight input to this provider. */
bool SdreM600_ComputeMassProperties(
    const SdreM600Inputs *input,
    SdreM600MassProperties *properties);

/* Build one complete, immutable operating point for Sdre_SetOperatingPoint().
 * The return value equals output->valid_mask.  Invalid input produces a fully
 * zeroed output with valid_mask == 0; no stale axis data is retained. */
uint32_t SdreM600_BuildOperatingPoint(
    const SdreM600Inputs *input,
    SdreOperatingPoint *output);

/* Cheap boot/pre-arm structural check of the compiled M600 data. */
bool SdreM600_ModelSelfTest(void);

#endif  /* SDRE_M600_MODEL_H_ */
