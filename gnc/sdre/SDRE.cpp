#include "SDRE.h"

#include <math.h>
#include <string.h>

/*
 * Shared embedded/SIM port of the M600_V8_1 SDRE controller law.
 *
 * Design constraints:
 *  - the CARE problem is fixed at n=3, m=1;
 *  - all large numerical work arrays are static (the controller is therefore
 *    deliberately non-reentrant and is intended for the single 100 Hz loop);
 *  - the aerodynamic operating point is injected explicitly;
 *  - Ctrl_Mode keeps the project-wide 0..7 selector; inactive axes retain
 *    the PID command supplied by PILOT;
 *  - an axis selected for SDRE never falls back to PID; any invalid model,
 *    low authority, unsupported condition, or CARE failure commands zero.
 */

volatile int Ctrl_Mode = SDRE_MODE_PID;

namespace {

static const int CARE_N = 3;
static const int CARE_H = 2 * CARE_N;
static const int CARE_MAX_ITER = 100;

static const double CARE_SIGN_TOL = 1.0e-11;
static const double CARE_RESIDUAL_TOL = 1.0e-6;
static const double SDRE_R2D = 57.29577951308232;
static const double SDRE_GRAVITY = 9.81;
static const double SDRE_ROLL_BMIN = 1.0;
static const double SDRE_PITCH_BMIN = 0.02;
static const double SDRE_YAW_BMIN = 0.02;
static const double SDRE_GAIN_MAX = 50.0;
static const double SDRE_PEAK_RAMP_TIME_S = 0.5;

/* Controller law and tuning copied from
 * D:/workcc/GH/SDRE/SDRE/M600_V8_1/SDRE.CPP. */
static const double SDRE_Q_ATT_S1[3] = 		{5.0, 100.0, 24.0};
static const double SDRE_Q_RATE_S1[3] = 	{2.0, 1.85, 50.0};
static const double SDRE_Q_RATE_S1_PEAK[3] = {2.0, 1.85, 30.0};
static const double SDRE_R_S1[3] = 			{0.5, 0.0901, 0.45};
static const double SDRE_R_S1_PEAK[3] =		{0.5, 0.130901, 0.75};
/*
static const double SDRE_Q_ATT_S12[3] = {5.0, 100.0, 20.0};
static const double SDRE_Q_RATE_S12[3] = {2, 1.2, 22.5};
static const double SDRE_Q_RATE_PEAK[3] = {2, 1.5, 30.5};
static const double SDRE_R_S12[3] = {0.04, 0.2, 0.2996};
static const double SDRE_R_PEAK[3] = {0.5, 0.15,  5.561996};
*/
static const double SDRE_Q_ATT_S2[3] = {5.0, 45.0, 12.0};
static const double SDRE_Q_RATE_S2[3] = {2.0, 1.40, 20.0};
static const double SDRE_R_S2[3]	 =	{0.5, 0.120901, 0.75};

static const double SDRE_Q_ATT_S345[3] = {1.0, 1.0, 1.0};
static const double SDRE_Q_RATE_S345[3] = {20.0, 20.0, 120.0};
static const double SDRE_R_S345[3] = {2.0, 4.5556, 3.5556};

static const double SDRE_Q_ATT_S6[3] = {1.0, 35.0, 45.0};
static const double SDRE_Q_RATE_S6[3]= {1.0, 80.0, 10.0};
static const double SDRE_R_S6[3] 	=  {0.04,0.454, 0.03051};

static const double SDRE_QINT_ATT = 2.9;
static const double SDRE_ZCL_ATT = 7.2;
//static const double SDRE_QINT_ACC_S12 = 2.9095;
//static const double SDRE_QINT_ACC_PEAK = 40;
//static const double SDRE_ZCL_ACC_S12 = 10.0;
//static const double SDRE_ZCL_ACC_PEAK = 0.9;

static const double SDRE_QINT_ACC_S1 = 2.5;
static const double SDRE_QINT_ACC_S1_PEAK = 4.0;

static const double SDRE_ZCL_ACC_S1 = 10.0;
static const double SDRE_ZCL_ACC_S1_PEAK = 1.;

static const double SDRE_QINT_ACC_S2 = 5.0;
static const double SDRE_ZCL_ACC_S2 = 6.0;

static const double SDRE_QINT_ACC_S6_PITCH = 3.5;
static const double SDRE_ZCL_ACC_S6_PITCH = 25.0;
static const double SDRE_QINT_ACC_S6_YAW = 2.5;
static const double SDRE_ZCL_ACC_S6_YAW = 20.0;
static const double SDRE_QINT_ROLL = 0.1;
static const double SDRE_ZCL_ROLL = 0.045;


struct CareWorkspace {
    /* Per-axis model buffers also stay out of the flight-task stack. */
    double axis_a[CARE_N * CARE_N];
    double axis_state[CARE_N];
    double axis_q[CARE_N * CARE_N];
    double axis_b[CARE_N];

    double br[CARE_N * CARE_N];
    double z[CARE_H * CARE_H];
    double z_inverse[CARE_H * CARE_H];
    double z_next[CARE_H * CARE_H];

    /* Shared Gauss-Jordan scratch, sized for the largest (6 x 6) inverse. */
    double inverse_left[CARE_H * CARE_H];
    double inverse_right[CARE_H * CARE_H];

    double subspace_m[CARE_H * CARE_N];
    double subspace_n[CARE_H * CARE_N];
    double subspace_mt[CARE_N * CARE_H];
    double mt_m[CARE_N * CARE_N];
    double mt_m_inverse[CARE_N * CARE_N];
    double mt_n[CARE_N * CARE_N];
    double p_temporary[CARE_N * CARE_N];

    double a_transpose[CARE_N * CARE_N];
    double at_p[CARE_N * CARE_N];
    double p_a[CARE_N * CARE_N];
    double p_br[CARE_N * CARE_N];
    double p_br_p[CARE_N * CARE_N];
    double residual_matrix[CARE_N * CARE_N];

    double p[CARE_N * CARE_N];
    double k[CARE_N];
};

struct AxisResult {
    int status;
    double command;
    double gain[CARE_N];
    CareDiag care;
};

struct ControllerRuntime {
    int initialized;
    int previous_mode;
    int previous_stage;
    int previous_separated;
    int have_time;
    double previous_time_s;

    int peak_active;
    double peak_start_time_s;

    double roll_integrator;
    int roll_previous_ok;

    double pitch_thrust_integrator;
    double pitch_coast_integrator;
    double pitch_terminal_integrator;
    int pitch_thrust_previous_ok;
    int pitch_coast_previous_ok;
    int pitch_terminal_previous_ok;

    double yaw_thrust_integrator;
    double yaw_coast_integrator;
    double yaw_terminal_integrator;
    int yaw_thrust_previous_ok;
    int yaw_coast_previous_ok;
    int yaw_terminal_previous_ok;
};

static CareWorkspace g_care_workspace;
static SdreOperatingPoint g_operating_point;
static SdreHealth g_health;
static ControllerRuntime g_runtime;

static int finite_scalar(double value)
{
    return (value == value) && (value <= 1.0e300) && (value >= -1.0e300);
}

static int finite_array(const double *values, int count)
{
    for (int i = 0; i < count; ++i) {
        if (!finite_scalar(values[i])) {
            return 0;
        }
    }
    return 1;
}

static double clamp_value(double value, double upper, double lower)
{
    if (value > upper) {
        return upper;
    }
    if (value < lower) {
        return lower;
    }
    return value;
}

static double absolute_value(double value)
{
    return (value < 0.0) ? -value : value;
}

static CareDiag make_care_diag(int status)
{
    CareDiag diag;
    diag.status = status;
    diag.iters = 0;
    diag.sign_delta = 0.0;
    diag.residual = 0.0;
    diag.p_min_diag = 0.0;
    return diag;
}

static AxisResult make_axis_result(int status)
{
    AxisResult result;
    result.status = status;
    result.command = 0.0;
    result.gain[0] = 0.0;
    result.gain[1] = 0.0;
    result.gain[2] = 0.0;
    result.care = make_care_diag((status == SDRE_AXIS_OK ||
                                  status == SDRE_AXIS_HOLD ||
                                  status == SDRE_AXIS_DISABLED)
                                     ? CARE_OK
                                     : CARE_ERR_DIM);
    return result;
}

static void set_axis_health(SdreAxisHealth *health, const AxisResult &result)
{
    health->status = result.status;
    health->command_rad = result.command;
    health->gain[0] = result.gain[0];
    health->gain[1] = result.gain[1];
    health->gain[2] = result.gain[2];
    health->care = result.care;
}

static void set_all_health(int status)
{
    const AxisResult result = make_axis_result(status);
    g_health.mode = static_cast<int>(Ctrl_Mode);
    g_health.model_valid_mask = g_operating_point.valid_mask;
    set_axis_health(&g_health.roll, result);
    set_axis_health(&g_health.pitch, result);
    set_axis_health(&g_health.yaw, result);
}

static void reset_runtime_storage(void)
{
    memset(&g_runtime, 0, sizeof(g_runtime));
    g_runtime.initialized = 1;
    g_runtime.previous_mode = -1;
    g_runtime.previous_stage = -1;
    g_runtime.peak_start_time_s = -1.0;
    g_runtime.roll_previous_ok = 1;
    g_runtime.pitch_thrust_previous_ok = 1;
    g_runtime.pitch_coast_previous_ok = 1;
    g_runtime.pitch_terminal_previous_ok = 1;
    g_runtime.yaw_thrust_previous_ok = 1;
    g_runtime.yaw_coast_previous_ok = 1;
    g_runtime.yaw_terminal_previous_ok = 1;
}

static void ensure_runtime_initialized(void)
{
    if (!g_runtime.initialized) {
        reset_runtime_storage();
        set_all_health(SDRE_AXIS_DISABLED);
    }
}

static void matrix_multiply(const double *a,
                            int a_rows,
                            int a_columns,
                            const double *b,
                            int b_columns,
                            double *product)
{
    for (int row = 0; row < a_rows; ++row) {
        for (int column = 0; column < b_columns; ++column) {
            double sum = 0.0;
            for (int inner = 0; inner < a_columns; ++inner) {
                sum += a[row * a_columns + inner] *
                       b[inner * b_columns + column];
            }
            product[row * b_columns + column] = sum;
        }
    }
}

static void matrix_transpose(const double *input,
                             int rows,
                             int columns,
                             double *output)
{
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            output[column * rows + row] = input[row * columns + column];
        }
    }
}

static double matrix_frobenius_norm(const double *matrix, int count)
{
    double sum = 0.0;
    for (int i = 0; i < count; ++i) {
        sum += matrix[i] * matrix[i];
    }
    return sqrt(sum);
}

/* Gauss-Jordan inverse with partial pivoting.  The large scratch is static. */
static int matrix_inverse(const double *input,
                          int dimension,
                          double *inverse,
                          double *determinant_out)
{
    double *left = g_care_workspace.inverse_left;
    double *right = g_care_workspace.inverse_right;

    for (int i = 0; i < dimension * dimension; ++i) {
        left[i] = input[i];
    }
    for (int row = 0; row < dimension; ++row) {
        for (int column = 0; column < dimension; ++column) {
            right[row * dimension + column] = (row == column) ? 1.0 : 0.0;
        }
    }

    double determinant = 1.0;
    for (int column = 0; column < dimension; ++column) {
        int pivot = column;
        double best = fabs(left[column * dimension + column]);
        for (int row = column + 1; row < dimension; ++row) {
            const double candidate = fabs(left[row * dimension + column]);
            if (candidate > best) {
                best = candidate;
                pivot = row;
            }
        }

        if (!finite_scalar(best) || best < 1.0e-300) {
            if (determinant_out != 0) {
                *determinant_out = 0.0;
            }
            return 1;
        }

        if (pivot != column) {
            for (int j = 0; j < dimension; ++j) {
                double temporary = left[column * dimension + j];
                left[column * dimension + j] = left[pivot * dimension + j];
                left[pivot * dimension + j] = temporary;

                temporary = right[column * dimension + j];
                right[column * dimension + j] = right[pivot * dimension + j];
                right[pivot * dimension + j] = temporary;
            }
            determinant = -determinant;
        }

        const double diagonal = left[column * dimension + column];
        determinant *= diagonal;
        for (int j = 0; j < dimension; ++j) {
            left[column * dimension + j] /= diagonal;
            right[column * dimension + j] /= diagonal;
        }

        for (int row = 0; row < dimension; ++row) {
            if (row == column) {
                continue;
            }
            const double factor = left[row * dimension + column];
            for (int j = 0; j < dimension; ++j) {
                left[row * dimension + j] -=
                    factor * left[column * dimension + j];
                right[row * dimension + j] -=
                    factor * right[column * dimension + j];
            }
        }
    }

    for (int i = 0; i < dimension * dimension; ++i) {
        inverse[i] = right[i];
    }
    if (determinant_out != 0) {
        *determinant_out = determinant;
    }
    return finite_array(inverse, dimension * dimension) ? 0 : 1;
}

/* Fixed n=3, m=1 CARE solver using the validated matrix-sign iteration. */
static int care_solve_3(const double a[CARE_N * CARE_N],
                        const double b[CARE_N],
                        const double q[CARE_N * CARE_N],
                        double r,
                        double p[CARE_N * CARE_N],
                        double k[CARE_N],
                        CareDiag *diagnostic)
{
    CareDiag diag = make_care_diag(CARE_OK);
    CareWorkspace &workspace = g_care_workspace;

    if (!finite_array(a, CARE_N * CARE_N) ||
        !finite_array(b, CARE_N) ||
        !finite_array(q, CARE_N * CARE_N) ||
        !finite_scalar(r)) {
        diag.status = CARE_ERR_NAN;
        if (diagnostic != 0) {
            *diagnostic = diag;
        }
        return diag.status;
    }
    if (absolute_value(r) < 1.0e-300) {
        diag.status = CARE_ERR_SINGULAR;
        if (diagnostic != 0) {
            *diagnostic = diag;
        }
        return diag.status;
    }

    const double r_inverse = 1.0 / r;
    for (int row = 0; row < CARE_N; ++row) {
        for (int column = 0; column < CARE_N; ++column) {
            workspace.br[row * CARE_N + column] =
                b[row] * r_inverse * b[column];
        }
    }

    for (int i = 0; i < CARE_H * CARE_H; ++i) {
        workspace.z[i] = 0.0;
    }
    for (int row = 0; row < CARE_N; ++row) {
        for (int column = 0; column < CARE_N; ++column) {
            workspace.z[row * CARE_H + column] =
                a[row * CARE_N + column];
            workspace.z[row * CARE_H + CARE_N + column] =
                -workspace.br[row * CARE_N + column];
            workspace.z[(CARE_N + row) * CARE_H + column] =
                -q[row * CARE_N + column];
            workspace.z[(CARE_N + row) * CARE_H + CARE_N + column] =
                -a[column * CARE_N + row];
        }
    }

    int converged = 0;
    int iteration = 0;
    for (iteration = 0; iteration < CARE_MAX_ITER; ++iteration) {
        if (!finite_array(workspace.z, CARE_H * CARE_H)) {
            diag.status = CARE_ERR_NAN;
            diag.iters = iteration;
            if (diagnostic != 0) {
                *diagnostic = diag;
            }
            return diag.status;
        }

        double determinant = 0.0;
        if (matrix_inverse(workspace.z,
                           CARE_H,
                           workspace.z_inverse,
                           &determinant) != 0) {
            diag.status = CARE_ERR_SINGULAR;
            diag.iters = iteration;
            if (diagnostic != 0) {
                *diagnostic = diag;
            }
            return diag.status;
        }

        double scale = 1.0;
        if (determinant != 0.0) {
            scale = pow(fabs(determinant), -1.0 / static_cast<double>(CARE_H));
        }
        if (!finite_scalar(scale) || scale <= 0.0) {
            diag.status = CARE_ERR_NAN;
            diag.iters = iteration;
            if (diagnostic != 0) {
                *diagnostic = diag;
            }
            return diag.status;
        }

        double difference_squared = 0.0;
        for (int i = 0; i < CARE_H * CARE_H; ++i) {
            const double next =
                0.5 * (scale * workspace.z[i] + workspace.z_inverse[i] / scale);
            const double difference = next - workspace.z[i];
            workspace.z_next[i] = next;
            difference_squared += difference * difference;
        }

        const double difference_norm = sqrt(difference_squared);
        for (int i = 0; i < CARE_H * CARE_H; ++i) {
            workspace.z[i] = workspace.z_next[i];
        }
        diag.sign_delta = difference_norm;
        if (difference_norm < CARE_SIGN_TOL) {
            converged = 1;
            ++iteration;
            break;
        }
    }
    diag.iters = iteration;

    if (!finite_array(workspace.z, CARE_H * CARE_H)) {
        diag.status = CARE_ERR_NAN;
        if (diagnostic != 0) {
            *diagnostic = diag;
        }
        return diag.status;
    }
    if (!converged) {
        diag.status = CARE_ERR_MAXITER;
        if (diagnostic != 0) {
            *diagnostic = diag;
        }
        return diag.status;
    }

    /* Recover P from the stable invariant subspace of sign(H). */
    for (int row = 0; row < CARE_N; ++row) {
        for (int column = 0; column < CARE_N; ++column) {
            const double s11 = workspace.z[row * CARE_H + column];
            const double s12 =
                workspace.z[row * CARE_H + CARE_N + column];
            const double s21 =
                workspace.z[(CARE_N + row) * CARE_H + column];
            const double s22 =
                workspace.z[(CARE_N + row) * CARE_H + CARE_N + column];

            workspace.subspace_m[row * CARE_N + column] = s12;
            workspace.subspace_m[(CARE_N + row) * CARE_N + column] =
                s22 + ((row == column) ? 1.0 : 0.0);
            workspace.subspace_n[row * CARE_N + column] =
                s11 + ((row == column) ? 1.0 : 0.0);
            workspace.subspace_n[(CARE_N + row) * CARE_N + column] = s21;
        }
    }

    matrix_transpose(workspace.subspace_m,
                     CARE_H,
                     CARE_N,
                     workspace.subspace_mt);
    matrix_multiply(workspace.subspace_mt,
                    CARE_N,
                    CARE_H,
                    workspace.subspace_m,
                    CARE_N,
                    workspace.mt_m);

    double mt_m_determinant = 0.0;
    if (matrix_inverse(workspace.mt_m,
                       CARE_N,
                       workspace.mt_m_inverse,
                       &mt_m_determinant) != 0) {
        diag.status = CARE_ERR_SINGULAR;
        if (diagnostic != 0) {
            *diagnostic = diag;
        }
        return diag.status;
    }

    matrix_multiply(workspace.subspace_mt,
                    CARE_N,
                    CARE_H,
                    workspace.subspace_n,
                    CARE_N,
                    workspace.mt_n);
    matrix_multiply(workspace.mt_m_inverse,
                    CARE_N,
                    CARE_N,
                    workspace.mt_n,
                    CARE_N,
                    workspace.p_temporary);

    for (int row = 0; row < CARE_N; ++row) {
        for (int column = 0; column < CARE_N; ++column) {
            p[row * CARE_N + column] =
                -0.5 * (workspace.p_temporary[row * CARE_N + column] +
                        workspace.p_temporary[column * CARE_N + row]);
        }
    }
    if (!finite_array(p, CARE_N * CARE_N)) {
        diag.status = CARE_ERR_NAN;
        if (diagnostic != 0) {
            *diagnostic = diag;
        }
        return diag.status;
    }

    for (int column = 0; column < CARE_N; ++column) {
        double sum = 0.0;
        for (int row = 0; row < CARE_N; ++row) {
            sum += b[row] * p[row * CARE_N + column];
        }
        k[column] = r_inverse * sum;
    }
    if (!finite_array(k, CARE_N)) {
        diag.status = CARE_ERR_NAN;
        if (diagnostic != 0) {
            *diagnostic = diag;
        }
        return diag.status;
    }

    matrix_transpose(a, CARE_N, CARE_N, workspace.a_transpose);
    matrix_multiply(workspace.a_transpose,
                    CARE_N,
                    CARE_N,
                    p,
                    CARE_N,
                    workspace.at_p);
    matrix_multiply(p, CARE_N, CARE_N, a, CARE_N, workspace.p_a);
    matrix_multiply(p,
                    CARE_N,
                    CARE_N,
                    workspace.br,
                    CARE_N,
                    workspace.p_br);
    matrix_multiply(workspace.p_br,
                    CARE_N,
                    CARE_N,
                    p,
                    CARE_N,
                    workspace.p_br_p);

    for (int i = 0; i < CARE_N * CARE_N; ++i) {
        workspace.residual_matrix[i] =
            workspace.at_p[i] + workspace.p_a[i] - workspace.p_br_p[i] + q[i];
    }
    diag.residual =
        matrix_frobenius_norm(workspace.residual_matrix, CARE_N * CARE_N);

    double minimum_diagonal = p[0];
    for (int i = 1; i < CARE_N; ++i) {
        if (p[i * CARE_N + i] < minimum_diagonal) {
            minimum_diagonal = p[i * CARE_N + i];
        }
    }
    diag.p_min_diag = minimum_diagonal;

    const double residual_scale =
        matrix_frobenius_norm(q, CARE_N * CARE_N) +
        matrix_frobenius_norm(p, CARE_N * CARE_N) + 1.0e-30;
    if (!finite_scalar(diag.residual) ||
        diag.residual / residual_scale > CARE_RESIDUAL_TOL) {
        diag.status = CARE_ERR_RESIDUAL;
        if (diagnostic != 0) {
            *diagnostic = diag;
        }
        return diag.status;
    }

    diag.status = CARE_OK;
    if (diagnostic != 0) {
        *diagnostic = diag;
    }
    return diag.status;
}

static AxisResult solve_axis(const double a[CARE_N * CARE_N],
                             double input_effectiveness,
                             double q_integrator,
                             double q_error,
                             double q_rate,
                             double r_coefficient,
                             const double state[CARE_N],
                             double authority_minimum,
                             double gain_hold[CARE_N])
{
    AxisResult result = make_axis_result(SDRE_AXIS_OK);

    if (!finite_array(a, CARE_N * CARE_N) ||
        !finite_array(state, CARE_N) ||
        !finite_scalar(input_effectiveness) ||
        !finite_scalar(q_integrator) ||
        !finite_scalar(q_error) ||
        !finite_scalar(q_rate) ||
        !finite_scalar(r_coefficient) ||
        r_coefficient < 0.0) {
        result.status = SDRE_AXIS_MODEL_INVALID;
        result.care.status = CARE_ERR_DIM;
        return result;
    }

    /* In the M600 tuning tables an exact R=0 is an explicit request to
     * suppress that SDRE axis.  Return the reference zero command instead
     * of treating it as an invalid model and falling back to PID. */
    if (r_coefficient == 0.0) {
        result.status = SDRE_AXIS_LOW_AUTHORITY;
        result.care.status = CARE_ERR_DIM;
        return result;
    }

    if (absolute_value(input_effectiveness) < authority_minimum) {
        result.status = SDRE_AXIS_LOW_AUTHORITY;
        result.care.status = CARE_ERR_DIM;
        return result;
    }

    const double r =
        input_effectiveness * input_effectiveness * r_coefficient;
    if (!finite_scalar(r) || r <= 1.0e-300) {
        result.status = SDRE_AXIS_LOW_AUTHORITY;
        result.care.status = CARE_ERR_DIM;
        return result;
    }

    double *b = g_care_workspace.axis_b;
    double *q = g_care_workspace.axis_q;
    b[0] = 0.0;
    b[1] = 0.0;
    b[2] = input_effectiveness;
    for (int i = 0; i < CARE_N * CARE_N; ++i) {
        q[i] = 0.0;
    }
    q[0] = q_integrator;
    q[4] = q_error;
    q[8] = q_rate;

    const int care_status = care_solve_3(a,
                                         b,
                                         q,
                                         r,
                                         g_care_workspace.p,
                                         g_care_workspace.k,
                                         &result.care);
    if (care_status != CARE_OK) {
        if (gain_hold != 0 &&
            (gain_hold[0] != 0.0 || gain_hold[1] != 0.0 || gain_hold[2] != 0.0)) {
            for (int i = 0; i < CARE_N; ++i) {
                result.gain[i] = gain_hold[i];
            }
            result.command = -(gain_hold[0] * state[0] +
                               gain_hold[1] * state[1] +
                               gain_hold[2] * state[2]);
            result.status = SDRE_AXIS_HOLD;
            return result;
        }
        result.status = SDRE_AXIS_SOLVER_FAIL;
        return result;
    }

    double maximum_gain = 0.0;
    for (int i = 0; i < CARE_N; ++i) {
        result.gain[i] = g_care_workspace.k[i];
        const double magnitude = absolute_value(result.gain[i]);
        if (magnitude > maximum_gain) {
            maximum_gain = magnitude;
        }
    }
    if (maximum_gain > SDRE_GAIN_MAX) {
        const double scale = SDRE_GAIN_MAX / maximum_gain;
        for (int i = 0; i < CARE_N; ++i) {
            result.gain[i] *= scale;
        }
        result.status = SDRE_AXIS_BAD_GAIN;
    }

    result.command = -(result.gain[0] * state[0] +
                       result.gain[1] * state[1] +
                       result.gain[2] * state[2]);
    if (!finite_scalar(result.command)) {
        result.command = 0.0;
        result.status = SDRE_AXIS_BAD_GAIN;
        return result;
    }

    if (gain_hold != 0) {
        for (int i = 0; i < CARE_N; ++i) {
            gain_hold[i] = result.gain[i];
        }
    }
    if (result.status != SDRE_AXIS_BAD_GAIN) {
        result.status = SDRE_AXIS_OK;
    }
    return result;
}

static int roll_model_valid(void)
{
    const SdreRollOperatingPoint &op = g_operating_point.roll;
    const double values[] = {
        op.torque_scale_nm,
        op.rate_scale_s,
        op.inertia_roll_kg_m2,
        op.cl_p,
        op.cl_delta_a_per_deg
    };
    return (g_operating_point.valid_mask & SDRE_MODEL_ROLL_VALID) != 0u &&
           finite_array(values, static_cast<int>(sizeof(values) / sizeof(values[0]))) &&
           op.inertia_roll_kg_m2 > 1.0e-9;
}

static int pitch_model_valid(void)
{
    const SdrePitchOperatingPoint &op = g_operating_point.pitch;
    const double values[] = {
        op.torque_scale_nm,
        op.rate_scale_s,
        op.inertia_pitch_kg_m2,
        op.cm_q,
        op.cm_alpha_per_deg,
        op.cm_delta_e_per_deg,
        op.z_alpha_per_s,
        op.velocity_mps
    };
    return (g_operating_point.valid_mask & SDRE_MODEL_PITCH_VALID) != 0u &&
           finite_array(values, static_cast<int>(sizeof(values) / sizeof(values[0]))) &&
           op.inertia_pitch_kg_m2 > 1.0e-9;
}

static int yaw_model_valid(void)
{
    const SdreYawOperatingPoint &op = g_operating_point.yaw;
    const double values[] = {
        op.torque_scale_nm,
        op.rate_scale_s,
        op.inertia_yaw_kg_m2,
        op.cn_r,
        op.cn_beta_per_deg,
        op.cn_delta_r_per_deg,
        op.z_beta_per_s,
        op.velocity_mps
    };
    return (g_operating_point.valid_mask & SDRE_MODEL_YAW_VALID) != 0u &&
           finite_array(values, static_cast<int>(sizeof(values) / sizeof(values[0]))) &&
           op.inertia_yaw_kg_m2 > 1.0e-9;
}

static AxisResult sdre_roll_apply(double roll_angle_rad,
                                  double roll_rate_rad_s,
                                  double sample_time_s,
                                  int stage,
                                  double peak_blend)
{
    if (!roll_model_valid() ||
        !finite_scalar(roll_angle_rad) ||
        !finite_scalar(roll_rate_rad_s)) {
        g_runtime.roll_previous_ok = 0;
        return make_axis_result(SDRE_AXIS_MODEL_INVALID);
    }

    if (g_runtime.roll_previous_ok) {
        g_runtime.roll_integrator += roll_angle_rad * sample_time_s;
    }
    g_runtime.roll_integrator = clamp_value(g_runtime.roll_integrator,
                                             SDRE_ZCL_ROLL,
                                             -SDRE_ZCL_ROLL);

   /* const double *q_attitude = SDRE_Q_ATT_S12;
    const double *q_rate = SDRE_Q_RATE_S12;
    const double *r_table = SDRE_R_S12;*/
	const double *q_attitude = (stage == 2) ? SDRE_Q_ATT_S2 : SDRE_Q_ATT_S1;
    const double *q_rate = (stage == 2) ? SDRE_Q_RATE_S2 : SDRE_Q_RATE_S1;
    const double *r_table = (stage == 2) ? SDRE_R_S2 : SDRE_R_S1;
    if (stage >= 3 && stage <= 5) {
        q_attitude = SDRE_Q_ATT_S345;
        q_rate = SDRE_Q_RATE_S345;
        r_table = SDRE_R_S345;
    } else if (stage == 6) {
        q_attitude = SDRE_Q_ATT_S6;
        q_rate = SDRE_Q_RATE_S6;
        r_table = SDRE_R_S6;
    }

    double q_rate_coefficient = q_rate[0];
    double r_coefficient = r_table[0];
    if (stage ==1 && peak_blend > 0.0) {
        q_rate_coefficient = SDRE_Q_RATE_S1[0] * (1.0 - peak_blend) +
                             SDRE_Q_RATE_S1_PEAK[0] * peak_blend;
        r_coefficient = SDRE_R_S1[0] * (1.0 - peak_blend) +
                        SDRE_R_S1_PEAK[0] * peak_blend;
    }

    const SdreRollOperatingPoint &op = g_operating_point.roll;
    const double damping =
        op.torque_scale_nm * op.cl_p * op.rate_scale_s /
        op.inertia_roll_kg_m2;
    const double effectiveness =
        op.torque_scale_nm * (-op.cl_delta_a_per_deg) * SDRE_R2D /
        op.inertia_roll_kg_m2;
    double *a = g_care_workspace.axis_a;
    double *state = g_care_workspace.axis_state;
    a[0] = 0.0; a[1] = 1.0; a[2] = 0.0;
    a[3] = 0.0; a[4] = 0.0; a[5] = 1.0;
    a[6] = 0.0; a[7] = 0.0; a[8] = damping;
    state[0] = g_runtime.roll_integrator;
    state[1] = roll_angle_rad;
    state[2] = roll_rate_rad_s;

    AxisResult result = solve_axis(a,
                                   effectiveness,
                                   SDRE_QINT_ROLL,
                                   q_attitude[0],
                                   q_rate_coefficient,
                                   r_coefficient,
                                   state,
                                   SDRE_ROLL_BMIN,
                                   0);
    g_runtime.roll_previous_ok = (result.status == SDRE_AXIS_OK);
    return result;
}

static AxisResult sdre_pitch_apply(double pitch_rate_rad_s,
                                   double pitch_error,
                                   double alpha_rad,
                                   double sample_time_s,
                                   int stage,
                                   int separated,
                                   double peak_blend)
{
    if (!pitch_model_valid() ||
        !finite_scalar(pitch_rate_rad_s) ||
        !finite_scalar(pitch_error) ||
        !finite_scalar(alpha_rad)) {
        g_runtime.pitch_thrust_previous_ok = 0;
        g_runtime.pitch_coast_previous_ok = 0;
        g_runtime.pitch_terminal_previous_ok = 0;
        return make_axis_result(SDRE_AXIS_MODEL_INVALID);
    }

    const SdrePitchOperatingPoint &op = g_operating_point.pitch;
    const double cm_delta_effective =
        separated ? -op.cm_delta_e_per_deg : op.cm_delta_e_per_deg;
    const double inertia = op.inertia_pitch_kg_m2;

    if (stage == 1 || stage == 2) {
        if (g_runtime.pitch_thrust_previous_ok) {
            const double integral_scale = (stage == 1) ? 5.0 : 1.0;
            g_runtime.pitch_thrust_integrator +=
                integral_scale * pitch_error * sample_time_s;
        }
        g_runtime.pitch_thrust_integrator =
            clamp_value(g_runtime.pitch_thrust_integrator,
                        SDRE_ZCL_ATT,
                        -SDRE_ZCL_ATT);

        const double a_qe =
            op.torque_scale_nm * (-op.cm_alpha_per_deg) / inertia;
        const double a_qq =
            op.torque_scale_nm * op.cm_q * op.rate_scale_s / inertia;
        const double effectiveness =
            op.torque_scale_nm * cm_delta_effective * SDRE_R2D / inertia;
        double *a = g_care_workspace.axis_a;
        double *state = g_care_workspace.axis_state;
        a[0] = 0.0; a[1] = 1.0; a[2] = 0.0;
        a[3] = 0.0; a[4] = 0.0; a[5] = -1.0;
        a[6] = 0.0; a[7] = a_qe; a[8] = a_qq;
        state[0] = g_runtime.pitch_thrust_integrator;
        state[1] = pitch_error;
        state[2] = pitch_rate_rad_s;
        const double *q_attitude =
            (stage == 1) ? SDRE_Q_ATT_S1 : SDRE_Q_ATT_S2;
        double r_coefficient =
            (stage == 1) ? SDRE_R_S1[1] : SDRE_R_S2[1];
        double q_rate_coefficient =
            (stage == 1) ? SDRE_Q_RATE_S1[1] : SDRE_Q_RATE_S2[1];
        if (stage == 1 && peak_blend > 0.0) {
            r_coefficient = SDRE_R_S1[1] * (1.0 - peak_blend) +
                            SDRE_R_S1_PEAK[1] * peak_blend;
            q_rate_coefficient = SDRE_Q_RATE_S1[1] * (1.0 - peak_blend) +
                                 SDRE_Q_RATE_S1_PEAK[1] * peak_blend;
        }
        AxisResult result = solve_axis(a,
                                       effectiveness,
                                       SDRE_QINT_ATT,
                                       q_attitude[1],
                                       q_rate_coefficient,
                                       r_coefficient,
                                       state,
                                       SDRE_PITCH_BMIN,
                                       0);
        g_runtime.pitch_thrust_previous_ok =
            (result.status == SDRE_AXIS_OK);
        return result;
    }

    const double z_alpha = op.z_alpha_per_s;
    const double m_alpha =
        op.torque_scale_nm * op.cm_alpha_per_deg * SDRE_R2D / inertia;
    const double m_q =
        op.torque_scale_nm * op.cm_q * op.rate_scale_s / inertia;
    const double effectiveness =
        op.torque_scale_nm * cm_delta_effective * SDRE_R2D / inertia;

    if (stage >= 3 && stage <= 5) {
        if (g_runtime.pitch_coast_previous_ok) {
            g_runtime.pitch_coast_integrator += alpha_rad * sample_time_s;
        }
        g_runtime.pitch_coast_integrator =
            clamp_value(g_runtime.pitch_coast_integrator,
                        SDRE_ZCL_ATT,
                        -SDRE_ZCL_ATT);

        double *a = g_care_workspace.axis_a;
        double *state = g_care_workspace.axis_state;
        a[0] = 0.0; a[1] = 1.0; a[2] = 0.0;
        a[3] = 0.0; a[4] = -z_alpha; a[5] = 1.0;
        a[6] = 0.0; a[7] = m_alpha; a[8] = m_q;
        state[0] = g_runtime.pitch_coast_integrator;
        state[1] = alpha_rad;
        state[2] = pitch_rate_rad_s;
        AxisResult result = solve_axis(a,
                                       effectiveness,
                                       SDRE_QINT_ATT,
                                       SDRE_Q_ATT_S345[1],
                                       SDRE_Q_RATE_S345[1],
                                       SDRE_R_S345[1],
                                       state,
                                       SDRE_PITCH_BMIN,
                                       0);
        g_runtime.pitch_coast_previous_ok =
            (result.status == SDRE_AXIS_OK);
        return result;
    }

    if (stage == 6) {
        if (g_runtime.pitch_terminal_previous_ok) {
            g_runtime.pitch_terminal_integrator +=
                pitch_error * sample_time_s;
        }
        g_runtime.pitch_terminal_integrator =
            clamp_value(g_runtime.pitch_terminal_integrator,
                        SDRE_ZCL_ACC_S6_PITCH,
                        -SDRE_ZCL_ACC_S6_PITCH);

        const double velocity = (op.velocity_mps > 1.0) ? op.velocity_mps : 1.0;
        double *a = g_care_workspace.axis_a;
        double *state = g_care_workspace.axis_state;
        a[0] = 0.0; a[1] = -z_alpha * velocity / SDRE_GRAVITY; a[2] = 0.0;
        a[3] = 0.0; a[4] = -z_alpha; a[5] = 1.0;
        a[6] = 0.0; a[7] = m_alpha; a[8] = m_q;
        state[0] = g_runtime.pitch_terminal_integrator;
        state[1] = alpha_rad;
        state[2] = pitch_rate_rad_s * 1.7;
        AxisResult result = solve_axis(a,
                                       effectiveness,
                                       SDRE_QINT_ACC_S6_PITCH,
                                       SDRE_Q_ATT_S6[1],
                                       SDRE_Q_RATE_S6[1],
                                       SDRE_R_S6[1],
                                       state,
                                       SDRE_PITCH_BMIN,
                                       0);
        g_runtime.pitch_terminal_previous_ok =
            (result.status == SDRE_AXIS_OK ||
             result.status == SDRE_AXIS_BAD_GAIN);
        return result;
    }

    return make_axis_result(SDRE_AXIS_UNSUPPORTED_MODE);
}

static AxisResult sdre_yaw_apply(double yaw_rate_rad_s,
                                 double beta_rad,
                                 double yaw_error,
                                 double sample_time_s,
                                 int stage,
                                 int separated,
                                 double peak_blend)
{
    if (!yaw_model_valid() ||
        !finite_scalar(yaw_rate_rad_s) ||
        !finite_scalar(beta_rad) ||
        !finite_scalar(yaw_error)) {
        g_runtime.yaw_thrust_previous_ok = 0;
        g_runtime.yaw_coast_previous_ok = 0;
        g_runtime.yaw_terminal_previous_ok = 0;
        return make_axis_result(SDRE_AXIS_MODEL_INVALID);
    }

    const SdreYawOperatingPoint &op = g_operating_point.yaw;
    const double cn_delta_effective =
        separated ? -op.cn_delta_r_per_deg : op.cn_delta_r_per_deg;
    const double inertia = op.inertia_yaw_kg_m2;
    const double z_beta = op.z_beta_per_s;
    const double n_beta =
        op.torque_scale_nm * op.cn_beta_per_deg * SDRE_R2D / inertia;
    const double n_r =
        op.torque_scale_nm * op.cn_r * op.rate_scale_s / inertia;
    const double effectiveness =
        op.torque_scale_nm * cn_delta_effective * SDRE_R2D / inertia;

    if (stage == 1 || stage == 2) {
        if (g_runtime.yaw_thrust_previous_ok) {
            double integral_input_scale = (stage == 1) ? 0.26 : 0.25;
            if (stage == 1 && peak_blend > 0.0)
                integral_input_scale =
                    0.14 * (1.0 - peak_blend) + 0.10 * peak_blend;
            g_runtime.yaw_thrust_integrator +=
                integral_input_scale * yaw_error * sample_time_s;
        }
        double zcl_acc_coefficient =
            (stage == 1) ? SDRE_ZCL_ACC_S1 : SDRE_ZCL_ACC_S2;
        if (stage == 1 && peak_blend > 0.0)
            zcl_acc_coefficient =
                SDRE_ZCL_ACC_S1 * (1.0 - peak_blend) +
                SDRE_ZCL_ACC_S1_PEAK * peak_blend;
        g_runtime.yaw_thrust_integrator =
            clamp_value(g_runtime.yaw_thrust_integrator,
                        zcl_acc_coefficient,
                        -zcl_acc_coefficient);

        const double velocity = (op.velocity_mps > 1.0) ? op.velocity_mps : 1.0;
        double *a = g_care_workspace.axis_a;
        double *state = g_care_workspace.axis_state;
        a[0] = 0.0; a[1] = -z_beta * velocity / SDRE_GRAVITY; a[2] = 0.0;
        a[3] = 0.0; a[4] = -z_beta; a[5] = 1.0;
        a[6] = 0.0; a[7] = n_beta; a[8] = n_r;
        state[0] = g_runtime.yaw_thrust_integrator;
        state[1] = beta_rad;
        double yaw_rate_scale = (stage == 1) ? 1.25 : 1.22;
        if (stage == 1 && peak_blend > 0.0)
            yaw_rate_scale = 1.25 * (1.0 - peak_blend) +
                             1.35 * peak_blend;
        state[2] = yaw_rate_rad_s * yaw_rate_scale;
        const double *q_attitude =
            (stage == 1) ? SDRE_Q_ATT_S1 : SDRE_Q_ATT_S2;
        double r_coefficient =
            (stage == 1) ? SDRE_R_S1[2] : SDRE_R_S2[2];
        double qint_acc_coefficient =
            (stage == 1) ? SDRE_QINT_ACC_S1 : SDRE_QINT_ACC_S2;
        double q_rate_coefficient =
            (stage == 1) ? SDRE_Q_RATE_S1[2] : SDRE_Q_RATE_S2[2];
        if (stage == 1 && peak_blend > 0.0) {
            r_coefficient = SDRE_R_S1[2] * (1.0 - peak_blend) +
                            SDRE_R_S1_PEAK[2] * peak_blend;
            qint_acc_coefficient =
                SDRE_QINT_ACC_S1 * (1.0 - peak_blend) +
                SDRE_QINT_ACC_S1_PEAK * peak_blend;
            q_rate_coefficient =
                SDRE_Q_RATE_S1[2] * (1.0 - peak_blend) +
                SDRE_Q_RATE_S1_PEAK[2] * peak_blend;
        }
        AxisResult result = solve_axis(a,
                                       effectiveness,
                                       qint_acc_coefficient,
                                       q_attitude[2],
                                       q_rate_coefficient,
                                       r_coefficient,
                                       state,
                                       SDRE_YAW_BMIN,
                                       0);
        g_runtime.yaw_thrust_previous_ok =
            (result.status == SDRE_AXIS_OK);
        return result;
    }

    if (stage >= 3 && stage <= 5) {
        if (g_runtime.yaw_coast_previous_ok) {
            g_runtime.yaw_coast_integrator += beta_rad * sample_time_s;
        }
        g_runtime.yaw_coast_integrator =
            clamp_value(g_runtime.yaw_coast_integrator,
                        SDRE_ZCL_ATT,
                        -SDRE_ZCL_ATT);

        double *a = g_care_workspace.axis_a;
        double *state = g_care_workspace.axis_state;
        a[0] = 0.0; a[1] = 1.0; a[2] = 0.0;
        a[3] = 0.0; a[4] = -z_beta; a[5] = 1.0;
        a[6] = 0.0; a[7] = n_beta; a[8] = n_r;
        state[0] = g_runtime.yaw_coast_integrator;
        state[1] = beta_rad;
        state[2] = yaw_rate_rad_s;
        AxisResult result = solve_axis(a,
                                       effectiveness,
                                       SDRE_QINT_ATT,
                                       SDRE_Q_ATT_S345[2],
                                       SDRE_Q_RATE_S345[2],
                                       SDRE_R_S345[2],
                                       state,
                                       SDRE_YAW_BMIN,
                                       0);
        g_runtime.yaw_coast_previous_ok =
            (result.status == SDRE_AXIS_OK);
        return result;
    }

    if (stage == 6) {
        if (g_runtime.yaw_terminal_previous_ok) {
            g_runtime.yaw_terminal_integrator += yaw_error * sample_time_s;
        }
        g_runtime.yaw_terminal_integrator =
            clamp_value(g_runtime.yaw_terminal_integrator,
                        SDRE_ZCL_ACC_S6_YAW,
                        -SDRE_ZCL_ACC_S6_YAW);

        const double velocity = (op.velocity_mps > 1.0) ? op.velocity_mps : 1.0;
        double *a = g_care_workspace.axis_a;
        double *state = g_care_workspace.axis_state;
        a[0] = 0.0; a[1] = -z_beta * velocity / SDRE_GRAVITY; a[2] = 0.0;
        a[3] = 0.0; a[4] = -z_beta; a[5] = 1.0;
        a[6] = 0.0; a[7] = n_beta; a[8] = n_r;
        state[0] = g_runtime.yaw_terminal_integrator;
        state[1] = beta_rad;
        state[2] = yaw_rate_rad_s;
        AxisResult result = solve_axis(a,
                                       effectiveness,
                                       SDRE_QINT_ACC_S6_YAW,
                                       SDRE_Q_ATT_S6[2],
                                       SDRE_Q_RATE_S6[2],
                                       SDRE_R_S6[2],
                                       state,
                                       SDRE_YAW_BMIN,
                                       0);
        g_runtime.yaw_terminal_previous_ok =
            (result.status == SDRE_AXIS_OK ||
             result.status == SDRE_AXIS_BAD_GAIN);
        return result;
    }

    return make_axis_result(SDRE_AXIS_UNSUPPORTED_MODE);
}

static void reset_for_stage_transition(int stage)
{
    if (stage == g_runtime.previous_stage) {
        return;
    }

    if (stage == 1 || stage == 2) {
        /* M4 resets the pitch attitude integrator between stages 1 and 2. */
        g_runtime.pitch_thrust_integrator = 0.0;
        g_runtime.pitch_thrust_previous_ok = 1;

        g_runtime.yaw_thrust_integrator = 0.0;
        g_runtime.yaw_thrust_previous_ok = 1;
    } else if (stage >= 3 && stage <= 5) {
        /* The validated implementation resets both coast integrators per stage. */
        g_runtime.pitch_coast_integrator = 0.0;
        g_runtime.pitch_coast_previous_ok = 1;
        g_runtime.yaw_coast_integrator = 0.0;
        g_runtime.yaw_coast_previous_ok = 1;
    } else if (stage == 6) {
        g_runtime.pitch_terminal_integrator = 0.0;
        g_runtime.pitch_terminal_previous_ok = 1;
        g_runtime.yaw_terminal_integrator = 0.0;
        g_runtime.yaw_terminal_previous_ok = 1;
    }
}

static void reset_for_separation_transition(void)
{
    g_runtime.roll_integrator = 0.0;
    g_runtime.roll_previous_ok = 1;
    g_runtime.pitch_thrust_integrator = 0.0;
    g_runtime.pitch_thrust_previous_ok = 1;
}

static double update_peak_blend(double time_s, int peak_reached)
{
    if (!peak_reached) {
        g_runtime.peak_active = 0;
        g_runtime.peak_start_time_s = -1.0;
        return 0.0;
    }
    if (!g_runtime.peak_active) {
        g_runtime.peak_active = 1;
        g_runtime.peak_start_time_s = time_s;
    }
    return clamp_value((time_s - g_runtime.peak_start_time_s) /
                           SDRE_PEAK_RAMP_TIME_S,
                       1.0,
                       0.0);
}

} /* namespace */

void Sdre_SetOperatingPoint(const SdreOperatingPoint *operating_point)
{
    ensure_runtime_initialized();
    if (operating_point == 0) {
        Sdre_InvalidateOperatingPoint();
        return;
    }

    g_operating_point = *operating_point;
    g_operating_point.valid_mask &= SDRE_MODEL_ALL_VALID;
    g_health.model_valid_mask = g_operating_point.valid_mask;
}

void Sdre_InvalidateOperatingPoint(void)
{
    ensure_runtime_initialized();
    memset(&g_operating_point, 0, sizeof(g_operating_point));
    reset_runtime_storage();
    set_all_health(SDRE_AXIS_MODEL_INVALID);
}

void Sdre_Reset(void)
{
    reset_runtime_storage();
    set_all_health(SDRE_AXIS_DISABLED);
}

const SdreHealth *Sdre_GetHealth(void)
{
    ensure_runtime_initialized();
    return &g_health;
}

void Sdre_Controller(double *roll_command_rad,
                     double *pitch_command_rad,
                     double *yaw_command_rad,
                     double roll_angle_rad,
                     double roll_rate_rad_s,
                     double pitch_rate_rad_s,
                     double yaw_rate_rad_s,
                     double pitch_error,
                     double alpha_rad,
                     double beta_rad,
                     double yaw_error,
                     double lateral_command_g,
                     int yaw_acceleration_mode,
                     double time_s,
                     double sample_time_s,
                     double control_start_s,
                     int stage,
                     int separated,
                     int dynamic_pressure_peak_reached)
{
    (void)lateral_command_g;
    (void)yaw_acceleration_mode;
    ensure_runtime_initialized();

    const int mode = static_cast<int>(Ctrl_Mode);
    g_health.mode = mode;
    g_health.model_valid_mask = g_operating_point.valid_mask;

    if (mode == SDRE_MODE_PID) {
        if (g_runtime.previous_mode != SDRE_MODE_PID) {
            reset_runtime_storage();
            g_runtime.previous_mode = SDRE_MODE_PID;
        }
        set_all_health(SDRE_AXIS_DISABLED);
        return; /* PID commands are deliberately untouched. */
    }

    if (mode < SDRE_MODE_ALL || mode > SDRE_MODE_PITCH_YAW) {
        if (roll_command_rad != 0) *roll_command_rad = 0.0;
        if (pitch_command_rad != 0) *pitch_command_rad = 0.0;
        if (yaw_command_rad != 0) *yaw_command_rad = 0.0;
        set_all_health(SDRE_AXIS_UNSUPPORTED_MODE);
        return;
    }

    const int roll_active =
        mode == SDRE_MODE_ALL || mode == SDRE_MODE_ROLL ||
        mode == SDRE_MODE_ROLL_PITCH || mode == SDRE_MODE_ROLL_YAW;
    const int pitch_active =
        mode == SDRE_MODE_ALL || mode == SDRE_MODE_PITCH ||
        mode == SDRE_MODE_ROLL_PITCH || mode == SDRE_MODE_PITCH_YAW;
    const int yaw_active =
        mode == SDRE_MODE_ALL || mode == SDRE_MODE_YAW ||
        mode == SDRE_MODE_ROLL_YAW || mode == SDRE_MODE_PITCH_YAW;

    /* Once an axis is selected for SDRE, PID is never used as an implicit
     * fallback.  Preload its output with zero so every validation/solver
     * failure remains a zero command.  Only Ctrl_Mode can leave an axis on
     * the PID command supplied by PILOT. */
    if (roll_active && roll_command_rad != 0) *roll_command_rad = 0.0;
    if (pitch_active && pitch_command_rad != 0) *pitch_command_rad = 0.0;
    if (yaw_active && yaw_command_rad != 0) *yaw_command_rad = 0.0;

    if (g_runtime.previous_mode != mode) {
        reset_runtime_storage();
        g_runtime.previous_mode = mode;
    }

    if (roll_command_rad == 0 || pitch_command_rad == 0 || yaw_command_rad == 0 ||
        !finite_scalar(time_s) ||
        !finite_scalar(sample_time_s) || sample_time_s <= 0.0 ||
        !finite_scalar(control_start_s)) {
        set_all_health(SDRE_AXIS_MODEL_INVALID);
        return;
    }

    if (stage < 1 || stage > 6) {
        set_all_health(SDRE_AXIS_UNSUPPORTED_MODE);
        return;
    }

    if (g_runtime.have_time && time_s < g_runtime.previous_time_s - 0.5) {
        reset_runtime_storage();
        g_runtime.previous_mode = mode;
    }

    if (time_s <= control_start_s) {
        const int saved_mode = g_runtime.previous_mode;
        reset_runtime_storage();
        g_runtime.previous_mode = saved_mode;
    }

    if (separated && !g_runtime.previous_separated) {
        reset_for_separation_transition();
    }
    reset_for_stage_transition(stage);

    const double peak_blend = update_peak_blend(
        time_s, dynamic_pressure_peak_reached);

    AxisResult result;
    if (roll_active) {
        result = sdre_roll_apply(roll_angle_rad,
                                 roll_rate_rad_s,
                                 sample_time_s,
                                 stage,
                                 peak_blend);
        *roll_command_rad = result.command;
        set_axis_health(&g_health.roll, result);
    } else {
        set_axis_health(&g_health.roll, make_axis_result(SDRE_AXIS_DISABLED));
    }

    if (pitch_active) {
        result = sdre_pitch_apply(pitch_rate_rad_s,
                                  pitch_error,
                                  alpha_rad,
                                  sample_time_s,
                                  stage,
                                  separated,
                                  peak_blend);
        *pitch_command_rad = result.command;
        set_axis_health(&g_health.pitch, result);
    } else {
        set_axis_health(&g_health.pitch, make_axis_result(SDRE_AXIS_DISABLED));
    }

    if (yaw_active) {
        result = sdre_yaw_apply(yaw_rate_rad_s,
                                beta_rad,
                                yaw_error,
                                sample_time_s,
                                stage,
                                separated,
                                peak_blend);
        *yaw_command_rad = result.command;
        set_axis_health(&g_health.yaw, result);
    } else {
        set_axis_health(&g_health.yaw, make_axis_result(SDRE_AXIS_DISABLED));
    }

    g_runtime.previous_stage = stage;
    g_runtime.previous_separated = separated ? 1 : 0;
    g_runtime.previous_time_s = time_s;
    g_runtime.have_time = 1;
}
