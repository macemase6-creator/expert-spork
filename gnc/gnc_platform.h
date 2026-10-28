#ifndef GNC_PLATFORM_H
#define GNC_PLATFORM_H

/* Mission payload stage override (stg[0]=0xAA, stg[1]=0x55).
 * Both simulation and firmware define these legacy payload fields. */
extern unsigned int t_state, t_stage;
#define GNC_FORCE_STAGE4            ((t_stage == 0xAAU) && (t_state == 0x55U))

/* Platform seams for the shared M4 / M397 guidance and control code.
 *
 * The shared implementation follows the flight-code representation.  These
 * macros cover the small set of real differences in state representation,
 * test-only gates, and hardware side effects.  Keep behavior changes out of
 * this file: each branch must expand to the platform's pre-merge behavior.
 */

#if defined(_WIN32) || defined(GNC_PLATFORM_SIM)

#define GNC_IS_SIM 1
#define GNC_CASE_ACTIVE             (nchoice == 5)
#define GNC_STAGE2_ACTIVE           (s_s == 2 && rocket == 0)
#define GNC_STAGE_VALUE             stage
#define GNC_AIMING_ANGLE            Aiming_Angle
#define GNC_RANGE                   Range
#define GNC_PI                      pi
#define GNC_ACC_X                   accb[0]
#define GNC_Q2M(q, m)               q2m((q), (m))
#define GNC_GI_VEC                  Arr2Mat(Gi)
#define GNC_SINGLE_TARGET_RETURN    (motor_flag || multi_trgts == 0)
#define GNC_MULTI_TARGET_STAGE6     (s_s == 6 && PointsCnt > 1 && nchoice > 4 && rocket == 0)
#define GNC_TARGET_SELECT_OVERRIDE  (ctrlNo == 4)
#define GNC_OUTPUT_ELEVATOR_FLOOR   9.0

#define GNC_MIRROR_MOVING_TARGET() \
    do { \
        Latt = rtn(0,0); \
        Longt = rtn(1,0); \
        rtf[0] = Xtrgt; \
        rtf[1] = Ytrgt; \
        rtf[2] = Ztrgt; \
    } while (0)

#define GNC_GUIDANCE_PLATFORM_TICK() \
    do { ++p_count; } while (0)

#define GNC_SIM_IGN2_DELAY() \
    do { \
        if ((vb == 0 && l_flag == 0) || (m_flag == 2)) \
            Del_t = t + ign2_dely; \
    } while (0)

#define GNC_BIND_MB2F(name) double (&name)[3][3] = mb2f

#else

#define GNC_IS_SIM 0
#define GNC_CASE_ACTIVE             1
#define GNC_STAGE2_ACTIVE           (s_s == 2)
#define GNC_STAGE_VALUE             s_s
#define GNC_AIMING_ANGLE            Aa
#define GNC_RANGE                   Rng
#define GNC_PI                      M_PI
#define GNC_ACC_X                   Ab(0,0)
#define GNC_Q2M(q, m)               q2m_((q), (m))
#define GNC_GI_VEC                  Gi
#define GNC_SINGLE_TARGET_RETURN    (multi_trgts == 0)
#define GNC_MULTI_TARGET_STAGE6     (s_s == 6 && PointsCnt > 1)
#define GNC_TARGET_SELECT_OVERRIDE  (multi_trgts == 0)
#define GNC_OUTPUT_ELEVATOR_FLOOR   9.0

#define GNC_MIRROR_MOVING_TARGET() ((void)0)

#define GNC_GUIDANCE_PLATFORM_TICK() \
    do { \
        if (t > 10. && Vm_t > 0.) \
            tgo = fabs(Slant_Range / Vm_t); \
        if (Slant_Range > 1. && s_s > 3 && fabs(Hme / Slant_Range) < 1.) \
            angle_tvc_sim = acos(Hme / Slant_Range) * r2d; \
    } while (0)

#define GNC_SIM_IGN2_DELAY() ((void)0)
#define GNC_BIND_MB2F(name) \
    double name[3][3]; \
    Mat2Arr3x3(&mb2f, name)

#endif

#endif /* GNC_PLATFORM_H */
