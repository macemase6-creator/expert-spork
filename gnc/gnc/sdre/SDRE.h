#ifndef SDRE_H_
#define SDRE_H_

#include <stdint.h>
extern int pik_q;
/*
 * Embedded SDRE integration contract.
 *
 * The Riccati controller produces virtual roll/pitch/yaw fin commands.  Sensor
 * filtering, frame rotation, control allocation, actuator limits, and CAN
 * transport remain in PILOT.cpp.  All angles and angular rates passed through
 * this interface are SI (rad, rad/s).  Aerodynamic table slopes remain per
 * degree to preserve the validated M4 controller equations.
 */

enum SdreMode {
    SDRE_MODE_PID        = 0,
    SDRE_MODE_ALL        = 1,
    SDRE_MODE_ROLL       = 2,
    SDRE_MODE_PITCH      = 3,
    SDRE_MODE_YAW        = 4,
    SDRE_MODE_ROLL_PITCH = 5,
    SDRE_MODE_ROLL_YAW   = 6,
    SDRE_MODE_PITCH_YAW  = 7,
    SDRE_MODE_SDRE       = SDRE_MODE_ALL
};

enum CareStatus {
    CARE_OK           = 0,
    CARE_ERR_MAXITER  = 1,
    CARE_ERR_NAN      = 2,
    CARE_ERR_SINGULAR = 3,
    CARE_ERR_RESIDUAL = 4,
    CARE_ERR_DIM      = 5
};

enum SdreAxisStatus {
    SDRE_AXIS_OK            = 0,
    SDRE_AXIS_LOW_AUTHORITY = 1,
    SDRE_AXIS_BAD_GAIN      = 2,
    SDRE_AXIS_SOLVER_FAIL   = 3,
    SDRE_AXIS_MODEL_INVALID    = 4,
    SDRE_AXIS_DISABLED         = 5,
    SDRE_AXIS_UNSUPPORTED_MODE = 6,
    SDRE_AXIS_HOLD             = 7
};

enum SdreModelValidMask {
    SDRE_MODEL_ROLL_VALID  = 1u << 0,
    SDRE_MODEL_PITCH_VALID = 1u << 1,
    SDRE_MODEL_YAW_VALID   = 1u << 2,
    SDRE_MODEL_ALL_VALID   = SDRE_MODEL_ROLL_VALID |
                             SDRE_MODEL_PITCH_VALID |
                             SDRE_MODEL_YAW_VALID
};

struct CareDiag {
    int status;
    int iters;
    double sign_delta;
    double residual;
    double p_min_diag;
};

struct SdreRollOperatingPoint {
    double torque_scale_nm;       /* qbar * Sref * Dref [N m] */
    double rate_scale_s;          /* Dref/(2*V) * rad-to-deg [s] */
    double inertia_roll_kg_m2;
    double cl_p;
    double cl_delta_a_per_deg;
};

struct SdrePitchOperatingPoint {
    double torque_scale_nm;
    double rate_scale_s;
    double inertia_pitch_kg_m2;
    double cm_q;
    double cm_alpha_per_deg;
    double cm_delta_e_per_deg;  /* Raw table derivative; SDRE applies sep sign. */
    double z_alpha_per_s;
    double velocity_mps;
};

struct SdreYawOperatingPoint {
    double torque_scale_nm;
    double rate_scale_s;
    double inertia_yaw_kg_m2;   /* M4 reference uses transverse Iyy here. */
    double cn_r;
    double cn_beta_per_deg;
    double cn_delta_r_per_deg;  /* Raw table derivative; SDRE applies sep sign. */
    double z_beta_per_s;
    double velocity_mps;
};

struct SdreOperatingPoint {
    SdreRollOperatingPoint roll;
    SdrePitchOperatingPoint pitch;
    SdreYawOperatingPoint yaw;
    uint32_t valid_mask;
};

struct SdreAxisHealth {
    int status;
    double command_rad;
    double gain[3];
    CareDiag care;
};

struct SdreHealth {
    int mode;
    uint32_t model_valid_mask;
    SdreAxisHealth roll;
    SdreAxisHealth pitch;
    SdreAxisHealth yaw;
};

/* Runtime selection follows the project interface: 0=PID, 1=all SDRE axes
 * and 2..7 select individual axis combinations.  Inactive axes leave the PID
 * command untouched only because MATLAB explicitly excluded them.  A selected
 * SDRE axis never falls back to PID: invalid input/model, low authority, or a
 * CARE failure produces a zero command and is reported through SdreHealth. */
extern volatile int Ctrl_Mode;

/* Call from the same control-loop context after every operating-point update.
 * Set a valid-mask bit only when every field of that axis is current. */
void Sdre_SetOperatingPoint(const SdreOperatingPoint *operating_point);
void Sdre_InvalidateOperatingPoint(void);
/* Reset controller history/integrators only; the injected operating point is
 * intentionally preserved. */
void Sdre_Reset(void);
const SdreHealth *Sdre_GetHealth(void);

void Sdre_Controller(double *roll_command_rad,
                     double *pitch_command_rad,
                     double *yaw_command_rad,
                     double roll_angle_rad,
                     double roll_rate_rad_s,
                     double pitch_rate_rad_s,
                     double yaw_rate_rad_s,
                     double pitch_error,       /* rad in s1/2; ignored s3-5; g in s6 */
                     double alpha_rad,
                     double beta_rad,
                     double yaw_error,         /* g in s1/2 accel and s6; ignored s3-5 */
                     double lateral_command_g, /* retained for M4 interface parity */
                     int yaw_acceleration_mode, /* false in s1/2 => zero/UNSUPPORTED */
                     double time_s,
                     double sample_time_s,
                     double control_start_s,
                     int stage,
                     int separated,
                     int dynamic_pressure_peak_reached);

#endif /* SDRE_H_ */
