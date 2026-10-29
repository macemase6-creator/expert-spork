/* sim_fio.h — registered file handles for the sim (Step 1 / R5, 2026-08-18).
   The sim previously called fcloseall(), which closes EVERY CRT stream in the
   host process — including MATLAB's own (shared ucrtbase) — causing the
   intermittent access violations. sim_fcloseall() closes only handles the sim
   itself opened. All sim TUs use sim_fopen/sim_fclose via textual rename;
   this header must be included before any sim TU (see mex_sim.h). */
#ifndef SIM_FIO_H
#define SIM_FIO_H
#include <stdio.h>
#include <math.h>

#define SIM_FIO_MAX 256
static FILE *simFioReg[SIM_FIO_MAX];
static int simFioN = 0;

static FILE *sim_fopen(const char *path, const char *mode)
{
   FILE *f = fopen(path, mode);
   if (f && simFioN < SIM_FIO_MAX)
      simFioReg[simFioN++] = f;
   return f;
}

static int sim_fclose(FILE *f)
{
   int i;
   if (f == NULL)
      return EOF;
   for (i = 0; i < simFioN; i++)
      if (simFioReg[i] == f) {
         simFioReg[i] = simFioReg[--simFioN];
         break;
      }
   return fclose(f);
}

/* Close (and thereby flush) every handle the sim opened; leave the host
   process's other streams alone. */
static void sim_fcloseall(void)
{
   int i;
   for (i = 0; i < simFioN; i++)
      fclose(simFioReg[i]);
   simFioN = 0;
}

/* --- Step 2 instrumentation channels (telemetry-only; never read by control
   law). Written in GUIDANCE.cpp, output via spare O_vec columns in SUBSYS.CPP:
   O_vec[115]=sim_dbg_dvr, O_vec[116]=sim_dbg_guard, O_vec[119]=sim_dbg_theta_p
   (cols 116/117/120 in MATLAB). Columns 115/116 previously duplicated
   fab[1]/fab[2] (cols 48/49); 119 duplicated mab[1] — mab[2] remains
   un-telemetered by choice. */
static double sim_dbg_theta_p = 0., sim_dbg_dvr = 0.;
static int sim_dbg_guard = 0;

/* --- Target-motion capability (validates the Step-8 terminal-agility raise).
   Driven by OPTIONAL MEX fields TgtSpd [m/s], TgtHdg [deg absolute azimuth],
   TgtWvA [deg heading-weave amplitude], TgtWvT [s weave period]; absent -> 0
   -> target static -> behaviour byte-identical. Propagation lives in
   NED_guide() (GUIDANCE.cpp): geodetic flat-earth step of rtn, then the
   existing rtn->rte->rt_l chain refreshes Xtrgt/rtf. Generation (nchoice 3/4)
   never moves targets. */
static double sim_tgt_spd = 0., sim_tgt_hdg = 0., sim_tgt_wva = 0., sim_tgt_wvt = 20.;
static double sim_tgt_tprev = -1.;

/* --- Seeker LOS-noise model (validates the LATH navigation-ratio trade).
   Optional MEX fields SkrNoise [mrad 1-sigma lateral LOS angular noise] and
   SkrSeed [uint]. SkrNoise absent/0 -> OFF -> byte-identical (the PRNG is
   never advanced). Deterministic xorshift64* + Box-Muller so a given
   (seed,noise) reproduces exactly -> the noise runs are themselves gateable.
   Injected on the lateral LOS only (the channel LATH governs) to isolate the
   navigation-ratio vs noise-amplification trade. */
/* --- Corroded control-fin model. Optional MEX field FinCorr in (0,1] scales
   the aerodynamic effectiveness of the LOWER fins 2 and 3 (y[31], y[32]).
   1.0 = pristine -> byte-identical. Asymmetric loss couples channels: with the
   stock mixer, rudder leaks into roll and aileron leaks into yaw by
   0.5*(k-1), while all three channels lose (1+k)/2 of their authority. */
/* Pre-launch pitch/launch-angle bias [deg]. Applied to Teta0 from t=0, so it
   shifts the WHOLE boost program (unlike theta_p, which only latches at stage
   2) and also moves the Teta0-dependent gain-scheduling branches. */
static double sim_teta0_bias = 0.0;

static double sim_fin_corr = 1.0;
/* Mixer compensation for the KNOWN corrosion state (build-time airframe
   constant, not an unobservable dispersion). Inverting the degraded map
     dele_eff = 0.5(1+k)E ;  delr_eff = 0.5[(1+k)R-(1-k)A]
     dela_eff = 0.5[(1+k)A-(1-k)R]
   gives  E' = 2E/(1+k),  R' = [(1+k)R+(1-k)A]/(2k),
          A' = [(1-k)R+(1+k)A]/(2k)
   which restores pristine control response at the cost of larger deflections.
   sim_fin_comp is the k the mixer ASSUMES (0 = compensation off).
   DEFAULT 0.70: gated on the full 49 at PRISTINE hardware with zero
   regressions (44/49 held, CPA mean 5.24->4.88, worst 7.77->6.30) and it also
   recovers most of the corrosion loss (worst CPA 83.4->9.6 at k=0.70). Note
   the pristine gain is empirical -- the compensating mixer form outperforms
   the stock allocation even with healthy fins, which suggests the stock
   allocation is conservative; the mechanism is not fully explained. */
static double sim_fin_comp = 0.0;  /* fixed-assumption mode: OFF (tracking supersedes) */

/* --- Time-varying stage-6 fin wear. Corrosion is not static: the lower fins
   erode with accumulated aerodynamic loading, so the effectiveness starts at
   1.0 at stage-6 entry and falls to SIM_WEAR_END once the accumulated
   exposure W = integral(q dt) reaches sim_wear_max. Measured stage-6 exposure
   is 492-690 (median 546) so sim_wear_max sets where saturation lands.
   sim_wear_on = 0 -> disabled -> byte-identical.
   Because W is built from MEASURED q, the flight computer can integrate the
   same model onboard: sim_wear_k is therefore a legitimate observable and the
   compensation can TRACK the wear instead of assuming a fixed value. */
#define SIM_WEAR_END 0.70
/* DEFAULTS: wear ON (the stage-6 fins DO erode with accumulated q, so this is
   the modelled reality, not an option), tracking compensation ON. Gated on the
   full 49: 44/49 held, ZERO regressions, and it recovers ~45% of the wear-
   induced CPA loss (bare 5.45 -> 5.20 mean). A FIXED assumed 0.70 was tested
   and REJECTED: it over-corrects while the fins are still healthy and breaks
   T10n5 (-> 9404 m, 43/49). sim_wear_max 546 = median measured stage-6
   exposure, so a typical engagement reaches full wear at impact. */
static int    sim_wear_on = 0, sim_comp_track = 1;
static double sim_wear_max = 546.0;
static double sim_wear_acc = 0.0, sim_wear_tprev = -1.0, sim_wear_k = 1.0;

/* Advance the wear integral (guarded so repeated derivative evaluations in one
   integration step cannot double-count) and return the current effectiveness. */
static double sim_wear_update(int stage6, double qnow, double tnow)
{
   if (!sim_wear_on || !stage6) { if (!stage6) { sim_wear_acc = 0.0; sim_wear_tprev = -1.0; sim_wear_k = 1.0; } return sim_wear_k; }
   if (sim_wear_tprev < 0.0) sim_wear_tprev = tnow;
   if (tnow > sim_wear_tprev) { sim_wear_acc += qnow * (tnow - sim_wear_tprev); sim_wear_tprev = tnow; }
   double frac = (sim_wear_max > 1e-9) ? (sim_wear_acc / sim_wear_max) : 1.0;
   if (frac > 1.0) frac = 1.0; else if (frac < 0.0) frac = 0.0;
   sim_wear_k = 1.0 - (1.0 - SIM_WEAR_END) * frac;
   return sim_wear_k;
}

static double sim_skr_noise = 0.;
static unsigned long long sim_skr_state = 88172645463325252ULL;
static int sim_skr_have = 0;
static double sim_skr_cache = 0.;

static double sim_urand(void)
{
   sim_skr_state ^= sim_skr_state >> 12;
   sim_skr_state ^= sim_skr_state << 25;
   sim_skr_state ^= sim_skr_state >> 27;
   unsigned long long x = sim_skr_state * 2685821657736338717ULL;
   return ((double)(x >> 11) + 1.0) / 9007199254740993.0; /* in (0,1) */
}

static double sim_gauss(void)
{
   if (sim_skr_have) { sim_skr_have = 0; return sim_skr_cache; }
   double u1 = sim_urand(), u2 = sim_urand();
   double r = sqrt(-2.0*log(u1)), th = 6.283185307179586*u2;
   sim_skr_cache = r*sin(th); sim_skr_have = 1;
   return r*cos(th);
}

/* Correlated (Gauss-Markov) seeker error. White noise at the 100 Hz guidance
   rate is low-passed to nothing by the airframe (measured: |d(ayc)| x283 with
   rudder std unchanged), so it CANNOT discriminate navigation ratios. Real
   seeker error (glint, radome, boresight drift) is time-correlated and does
   propagate. SkrTau [s] > 0 selects this; <=0 keeps white. */
static double sim_skr_tau = 0.;
static double sim_skr_gm = 0.;
static double sim_skr_tprev = -1.;

static double sim_skr_sample(double tnow)
{
   if (sim_skr_tau <= 0.) return sim_gauss();          /* white */
   if (sim_skr_tprev < 0.) { sim_skr_tprev = tnow; sim_skr_gm = sim_gauss(); }
   double dt = tnow - sim_skr_tprev;
   if (dt > 0.) {
      double a = exp(-dt / sim_skr_tau);
      sim_skr_gm = a*sim_skr_gm + sqrt(1.0 - a*a)*sim_gauss();
      sim_skr_tprev = tnow;
   }
   return sim_skr_gm;
}

/* ------------------------------------------------------------------------
   tal_flag -- "Edit 4" airframe: the motor case is 45 cm longer, carrying
   +100 kg of propellant and +7 kg of casing; the nozzle and fins translate
   45 cm aft with it. Overall length 4.252088 -> 4.702088 m; body diameter
   unchanged at 0.397 m.

     sim_tal_flag = 0  -> the vehicle exactly as it is today. Every constant
                          below is restored explicitly, so this path is
                          BIT-IDENTICAL to the pre-flag build (gate G1).
     sim_tal_flag = 1  -> Edit 4.

   Driven by the OPTIONAL MEX field "TalFlag"; absent -> 0 -> byte-identical.

   The tal_flag=1 mass properties were derived from the IN-TREE baseline (not
   from any external block) so that flag=0 is provably the 45/49 reference.
   Method: roll inertia is additive for coaxial bodies, so the grain's own Ixx
   is ix1_1-ix2_1; the CG moment balance and pitch inertia then close the
   system. That reproduces all four published constants to 4.6e-13 and puts
   the grain at 1.525289 .. 3.751850 m from the nose -- its front face within
   0.8 mm of the CAD motor-case start (1.526088 m), an independent check.
   The 45 cm plug therefore sits at centroid 3.976850 m, and the 20.329 kg
   nozzle/fin lump at 4.007588 m translates +0.45 m with the stretch.        */
#define TAL_DFUEL      100.0    /* kg of propellant added                    */
#define TAL_DCASE        7.0    /* kg of casing added                        */
#define TAL_LEXT         0.45   /* m of motor-case stretch                   */
#define TAL_ISP_ADDED  250.0    /* s, specific impulse credited to the added
                                   propellant (user specification: total
                                   impulse rises by dFuel * 250)             */
#define TAL_XPLUG      3.977900 /* m from nose, centroid of the added grain  */
#define TAL_DFIN       0.45     /* m the fins/nozzle move aft                */
#define TAL_MFUEL0     409.0    /* baseline m_fuel, the flag=0 vehicle       */

/* DIAGNOSTIC: force the stage-2 boost trim theta_p to a commanded value, so
   range-vs-trim can be mapped directly instead of inferred. Driven by the
   OPTIONAL MEX field "TpSet"; absent -> off -> byte-identical. Applied after
   every native scaling and clamp, so it overrides them completely. */
static int    sim_tp_on  = 0;
static double sim_tp_val = 0.0;

/* Roll-damping restoration in the low-q branch of stage 2 -- see the
   TAL_ROLLQ note in PILOT.CPP. 1.0 = stock behaviour. */
#define TAL_ROLLQ_K  1.0
/* Roll-integrator clamp scale -- trim authority against the constant roll
   torque from the dispersion's nozzle misalignment. 1.0 = stock. */
#define TAL_ROLLI_K  1.0

/* ============ AILERON REVERSAL GUARD (root cause of the roll excursion) ====
   RAJEH_CO_ROLL_S1.dat holds 16 POSITIVE entries in an otherwise entirely
   negative table, forming an exact block: alpha rows 12/16/20 deg at Mach
   columns 0.3/0.5/0.8. Within those same alpha rows the sign flips back to
   negative at Mach 1.2+. Since cll = -cllda, positive means the AILERON ROLL
   EFFECTIVENESS REVERSES SIGN there.
   Early boost IS low Mach, so once the flg 1/2 misalignment drives incidence
   past 12 deg the roll loop commands a correction that DRIVES the excursion
   instead of arresting it. That explains why raising the aileron limit made
   peak roll WORSE (222 -> 300 deg/s), why the flag_alfa 4 deg clamp is
   protective rather than a defect, and why nominal arms -- which never reach
   12 deg incidence -- roll at only 0.2 deg/s.
   alfab_g and mach330 are both observable, so this branches on STATE, not on
   the dispersion. 0 = stock, 1 = ZERO the aileron in the region, 2 = INVERT
   it (deliberately use the reversed effectiveness).
   MEASURED, full 49 x both flags (7-case roll metric set):
     0 stock  -> boost peak roll 28.3 deg/s | peak |Azb| 5.168 g | max phi 56.2
     1 zero   -> 17.8 (-37%)                | 4.745            | 35.9 (-36%)
     2 invert -> **12.6 (-55%)**            | **4.651 (-10%)** | **33.7 (-40%)**
   All three hold tal_flag=1 at 49/49; invert also IMPROVES the mean miss
   (11.02 -> 10.94). Threshold check: the exact table edges (12 deg / M0.8)
   engage the guard less and give back half the gain (peak roll 19.0), while
   8 deg / M1.2 is equivalent (12.9). 10 deg / M1.0 chosen.
   tal_flag=0 untouched at 48/49 (gated on sim_tal_flag). */
#define TAL_AREV      2

/* SIDESLIP DAMPER (stage 1). Az is not a yaw-command problem and not a
   yaw-rate problem -- it is SIDESLIP. Measured over late boost,
   corr(fab[2], betta) = -0.988 (T13n2) and -0.995 (T11n0), against
   corr(fab[2], yawrate) = -0.03 and corr(fab[2], delr) = +0.06. cz itself is
   -cnb - cnqb*r*ddv - cndr, and the cnb term dominates: beta = -1.96 deg
   produces fab[2] = 10839 N. The yaw loop regulates ACCELERATION (du_yaw =
   -ayc + Azb_filter), not beta, so nothing actively drives sideslip to zero.
   This adds a beta-proportional rudder term in stage 1. Sign is determined by
   sweep. 0 = off. Edit-4 only. */
/* TAL_KBETA -- REJECTED 2026-08-23, CATASTROPHIC. Adding an explicit
   beta term to delrc0 destroys the loop at EVERY gain and BOTH signs:
   +-1.0 and +-3.0 all give peak |Azb| 34-55 g with beta saturated at 20 deg
   and 0-2 hits out of 49 (misses 450-850 km). Reason: Azb_filter is ITSELF
   proportional to beta (corr -0.99), so du_yaw = -ayc + Azb_filter ALREADY
   closes a sideslip loop; a second beta path makes it a double loop. The yaw
   channel does not need beta feedback added -- it has it. 0 = off. */
#define TAL_KBETA  0.0

/* Stage-1 yaw rate-damping trim. Since the loop already regulates beta and
   drives Az toward ayc, a 4.65 g peak against a 1.8 g cap is OVERSHOOT. The
   donor stage-1 schedule drops K_w_yaw 2.18x at q=20 (1.85*0.275*(1.5/q+
   0.125) below, 0.85*0.275*(...) above) and the Az peak sits at q~58, i.e. in
   the LOW-damping branch. 1.0 = stock. Edit-4 only. */
/* TAL_KWY -- REJECTED 2026-08-23. Raising stage-1 yaw rate damping to cure
   the overshoot makes Az WORSE and collapses the gate: peak |Azb| 4.651 ->
   5.188 -> 12.953 -> 41.737 g at 1.0/1.5/2.2/3.0, with 49 -> 47 -> 27 -> 9
   hits. The yaw channel is at a stability boundary in BOTH directions. */
#define TAL_KWY  1.0
#define TAL_AREV_ALF 10.0   /* deg alfab, region entry (table edge 12)  */
#define TAL_AREV_MACH 1.0   /* mach, region exit    (table edge 0.8)    */
/* Continuity scale for the q<4 roll proportional gain -- see TAL_ROLLP in
   PILOT.CPP. 12754 makes the two branches meet exactly at q=4; 1.0 = stock. */
#define TAL_ROLLP_K  1.0
/* Ramp exponent: gain scale is 1+(K-1)*(q/4)^N, so it meets the q>=4 branch
   at q=4 and stays near stock at q~0. Larger N = later, sharper closure. */
#define TAL_ROLLP_N  2.0

/* Early-boost roll-authority clamp -- see flag_alfa in PILOT.CPP.
   Stock: once body incidence exceeds 7 deg while t>1, q<10, s_s==1, the
   aileron (roll) budget is cut 8 deg -> 4 deg and LATCHES for the rest of
   the flight.  TAL_AILIM is the clamped value in deg (4.0 = stock);
   TAL_AILATCH 1 = stock latch, 0 = release the clamp when incidence
   drops back below 7 deg. */
#define TAL_AILIM    4.0
#define TAL_AILATCH  1

/* DIAGNOSTIC ONLY -- isolate which flg==2 misalignment term drives the
   boost incidence excursion.  All must be 0 for stock behaviour. */
/* NO TVC IN STAGE 1/2 (user, 2026-08-23). eth1/eth2 are the first-stage
   nozzle angles. NO control law anywhere writes them -- they are assigned
   only in the flg==1/flg==2 dispersion branches (+-8/60 deg) and zeroed in
   the reset paths, so there is no thrust-vector CONTROL in stage 1 or 2 at
   all; they are a fixed misalignment. They still feed the lateral thrust
   forces (ftb1/ftb2) and all three moments (mtb0/mtb1/mtb2), all scaled by
   thr, which is what drives the roll and the lateral acceleration.
   1 = zero them.  Note that the Yt/Zt
   thrust-offset terms already model thrust misalignment separately, so the
   cant may have been double-counting it.
   MUST STAY 0 (user, 2026-08-23): eth1/eth2 are part of the NOISE/dispersion
   model -- they are the nozzle-misalignment tolerance drawn per flg branch,
   not a control input. Zeroing them removes a modelled dispersion, which is a
   change to the noise assumptions, not a control-law fix. Kept only as a
   measured diagnostic. MEASURED at 1 (tal=1): boost roll 12.6 -> 5.8 deg/s,
   max phi 33.7 -> 19.9, peak beta 13.99 -> 7.95 deg -- but peak |Az| 4.651 ->
   6.466 g (WORSE) and the gate 49/49 -> 48/49. So it would not have helped Az
   even if it were permissible: with the cant gone the Yt/Zt thrust offset is
   left untrimmed and the surviving sideslip is more concentrated. */
#define TAL_NOTVC  0

/* DIAG_TELE -- diagnostic telemetry into the UNUSED O_vec slots 130..139
   (N_to_save is 164; stock fills only 0..129), so it cannot perturb any
   result.  MATLAB column = index+1, i.e. cols 131..140:
     131 out_integ_yaw   132 out_integ_yaw/0.0835 (rail fraction)
     133 delrc0 (pre-limit)  134 delrc (post-limit)  135 rudder_limit
     136 du_yaw          137 delec       138 delac
     139 mtb2 (thrust yaw moment)        140 mab[2] (aero yaw moment)   */
#define DIAG_TELE  1

/* TAL_KPAF -- FLOOR ON THE YAW ACCELERATION-LOOP PROPORTIONAL GAIN (boost).
   ROOT CAUSE, measured 2026-08-23.  K_p_integ_acc is scheduled in two
   different functional families.  The rational-fit family
     K_p_integ_acc = C*( -2.228e-05*q^3 + 0.003546*q^2 - 0.2357*q + 3.247 )
                       /( q^2 + 7.828*q + 21.85 )
   (PILOT.CPP:459 with C=0.05 for q>60, PILOT.CPP:472 with C=0.18 otherwise)
   has a NUMERATOR THAT CROSSES ZERO AT q ~ 18 and is NEGATIVE above it.  So
   through the entire high-q boost the yaw acceleration loop runs a NEGATIVE
   proportional gain -- verified against telemetry to 3 digits (q=62.0 gives
   -3.50e-5, q=50.65 gives -1.502e-4).  The other family gives +0.125/q,
   POSITIVE and ~57x larger.  The fit was evidently identified over a low-q
   range and is being extrapolated far outside it.
   EVIDENCE it is what sets the Az peak (T13n4, fine trace):
     t=18.15  Az -5.884 g  beta 3.21 deg  Kp +0.044e-3   <- peak, no authority
     t=18.25  Az -5.110    beta 2.64      Kp +2.494e-3   <- +0.125/q engages
     t=18.45  Az -0.904    beta 0.25      Kp +2.467e-3   <- killed in 0.3 s
   The excursion collapses the instant the positive gain takes over.  Also
   measured: at the peak the rudder is NOT saturated (0.08-1.72 deg commanded
   out of 19-20 deg available), the servo is not rate-limited (44-58 vs
   75 deg/s) and the integrator is railed only 0-13% of the time.  The channel
   is not short of authority -- it is short of correctly-signed gain.
   Applied ONLY at the yaw read site (PILOT.CPP:1211), never to the pitch read
   at :1300, and only for s_s <= 2 so terminal guidance is untouched.
   VALUE: <0 disables (bit-exact no-op).  0 clamps the gain at zero (removes
   the wrong sign only).  >0 floors it at TAL_KPAF*0.125/q, i.e. 1.0 restores
   the full intended schedule. */
#define TAL_KPAF  1.5

/* TAL_KPAE0 / TAL_KPAE1 -- make the TAL_KPAF floor TRANSIENT-ONLY.
   A flat floor also raises the gain in STEADY TRIM, which shifts the boost
   trim, hence burnout velocity, hence every miss -- that is why the flat
   floor runs out of accuracy margin at 0.40 (one case jumps to 96.8 m) and
   fails at 0.45.  The Az excursion is a TRANSIENT, so ramp the floor in on
   |du_yaw| (the yaw accel error, in g): no change below KPAE0, full floor
   above KPAE1.  KPAE1 <= KPAE0 disables the ramp (flat floor, as swept). */
#define TAL_KPAE0  1.0
#define TAL_KPAE1  3.0

/* TAL_AZLD -- REMOVE THE ONE-STEP LAG FROM THE Az FEEDBACK (boost yaw loop).
   SUBSYS.CPP calls Guidance() then PilotGain() then AutoPilot(), so the
   du_yaw built at GUIDANCE.cpp:1192 (du_yaw = -ayc + Azb_filter) carries
   Azb_filter[k-1], while PILOT.CPP:1024 refreshes Azb_filter to [k] BEFORE the
   yaw loop consumes du_yaw.  The proportional path is therefore one full step
   stale on top of the filter's own lag.  Since
       Azb_filter[k] = 0.25*az_b[k] + 0.75*Azb_filter[k-1]
   it follows exactly that
       az_b[k] - Azb_filter[k-1] == (4/3)*(az_b - Azb_filter)
   with BOTH terms current at the read site -- so no stored history and no
   per-run reset (the MEX cross-run persistence trap does not apply).  At
   TAL_AZLD = 1.0 the P path sees exactly -ayc + az_b[k], the true current
   accelerometer.  DC gain is 1 at every value, so trim and burnout velocity
   are untouched.  This is NOT TAL_KBETA: that added a SECOND beta path to
   delrc0; this reshapes a signal already in the loop.
   CAVEAT TO CHECK FIRST: the added term is a difference, so it carries a
   body-rate component -- if it has the same sign as wy_compensate it is
   acting as extra yaw RATE damping, and TAL_KWY measured that more rate
   damping makes Az WORSE (5.188 g, 47/49).  Applied to the PROPORTIONAL path
   only, never to out_integ_yaw, so 0.0 is bit-exact.
   0.0 = OFF.  Sweep 0.25 / 0.50 / 1.00. */
#define TAL_AZLD  1.0

/* TAL_AYLD6 / TAL_AZLD6 -- the SAME one-step feedback lag, in the TERMINAL
   loop.  GUIDANCE.cpp:1364-1365 build
       du_pitch = ac_pitch*k_5 - Ayb_filter      <- NOTE the MINUS
       du_yaw   = -ayc*k_5     + Azb_filter      <- PLUS, as in boost
   and PILOT.CPP:1025-1026 refresh ay_b/Ayb_filter only AFTER Guidance has
   run, so both carry the [k-1] sample.  Ayb_filter has the identical form
   (ay_b + 3*Ayb_filter)/4, so ay_b[k] - Ayb_filter[k-1] == (4/3)*(ay_b -
   Ayb_filter), again with both terms current at the read site: no state, no
   reset.  BECAUSE OF THE SIGN the pitch correction is SUBTRACTED and the yaw
   correction is ADDED -- getting that backwards doubles the lag instead of
   removing it.  Kept as two constants because the terminal axes are already
   known to behave OPPOSITELY (see TAL_KPA6Y vs TAL_KPA6P).  Proportional path
   only, so 0.0 is bit-exact.  1.0 = feed the true current accelerometer. */
#define TAL_AYLD6  0.0
#define TAL_AZLD6  0.0

/* TAL_KWP4 -- clamp the STAGE-4 K_w_pitch, the ONLY branch of this gain with
   no clamp.  PILOT.CPP:739 sets K_w_pitch = (1*0.5*0.2)/q = 0.1/q and
   PILOT.CPP:745 copies it straight into K_w_yaw, while the stage-5 branches
   clamp the SAME gain at 30 (PILOT.CPP:783) and 20 (PILOT.CPP:797).  Stage 4
   is the exo-atmospheric coast where q ~ 0 and it lasts ~315 s, so 0.1/q is
   unbounded there -- measured max K_w_pitch = 100.2 (T13n5), i.e. q reached
   ~0.001.  The yaw block gate includes s_s>2 so K_w_yaw = 100 is genuinely
   evaluated in stage 4.  0 = OFF (bit-exact), >0 = clamp in stage 4. */
#define TAL_KWP4  20.0

/* TAL_KWPF -- FLOOR ON STAGE-1/2 PITCH RATE DAMPING.  Fixes a V-shaped HOLE
   in K_w_pitch1 between q=20 and q=50, created by branch switching (the
   DONOR_S1 q=20 breakpoint and the if(q>50) branch).  Mean K_w_pitch1 by q
   band: 0.0822 (10-20) -> 0.0545 (20-30) -> 0.0599 (30-40) -> 0.0869 (40-55)
   -> 0.1027 (55-70).  The damping DIPS by 40% exactly in mid-boost.
   MEASURED stage-1 rms pitch rate (deg/s) over all 49, binned by q:
       q band   tal=1    tal=0   ratio
       10-20     4.37     1.20    3.6x
       20-30     7.06     0.77    9x
       30-40     9.48     0.46    20x
       40-55     3.89     0.56    7x
   The STOCK airframe rides through the hole (rms 0.46-0.77); the Edit-4
   airframe RINGS in it, because it is heavier and its short period is slower,
   so the same rate gain buys less damping.  Theta-error rms in those bands is
   30x worse too (1.34 / 1.74 vs 0.043 / 0.062 deg).  The elevator is NOT the
   limit: it uses ~6 deg of 15-20 deg available and 31 of 75 deg/s.
   Applied to a LOCAL at the wz_compensate1 site (PILOT.CPP:1044), which feeds
   only the s_s==1 and s_s==2 pitch branches (delec0 at :1329 and :1344).
   Stages 3/5/6 use wz_compensate and are untouched, and the global
   K_w_pitch1 is not modified, so the several K_w_yaw = k*K_w_pitch1
   derivations are unaffected.  0 = OFF (bit-exact). */
#define TAL_KWPF  0.085

/* TAL_ELOQ -- DIAGNOSTIC ONLY.  Multiplies the elevator command in stage 1
   below q = TAL_ELOQQ, to settle whether the large low-q attitude error is
   GAIN-limited or AUTHORITY-limited.  If multiplying the command does not
   reduce the error, the fin simply produces no moment at that q and no gain
   change can help.  MUST be 1.0 for any shipped build. */
#define TAL_ELOQ   1.0
#define TAL_ELOQQ  5.0

/* ---- STOCK-VEHICLE (tal_flag==0) YAW FIXES, added 2026-08-23 ----
   Every yaw fix so far was gated on sim_tal_flag to protect the stock
   baseline, so tal_flag==0 never received ANY of them.  That is exactly why
   the stock vehicle still shows a 13.55 g lateral spike at burnout while the
   Edit-4 vehicle is down to 3.47 g -- the user plot (ayc vs azbb, tal=0)
   shows it at t = 22-25 s, which is t_burn1.  TAL_KPAF0 / TAL_AZLD0 are the
   flag-0 twins of TAL_KPAF / TAL_AZLD and share KPAE0/KPAE1.
   <0 / 0.0 = OFF and bit-exact. */
#define TAL_KPAF0  1.0
#define TAL_AZLD0  0.0

/* TAL_KWPM / TAL_KWPMQ -- extra stage-1/2 pitch rate damping BELOW q=KWPMQ.
   TAL_KWPF filled the q=20-50 hole, but the ringing visible in the user plot
   (tetac vs tetam, tal=1) runs t = 6-15 s, which is q = 5-20, where the floor
   does NOT bind because the schedule there is already 0.163 (q 5-10) and
   0.082 (q 10-20).  Measured rms pitch rate after TAL_KWPF: q 5-10 = 3.60 and
   q 10-20 = 4.05 for tal=1, against 2.23 and 1.20 for tal=0 -- still 1.6x and
   3.4x worse.  Multiplier, not floor, because the schedule is not flat here.
   1.0 = OFF (bit-exact). */
#define TAL_KWPM   1.3
#define TAL_KWPMQ  20.0
#define TAL_KWPMQ0  5.0

/* TAL_KITM -- scale the stage-1/2 pitch INTEGRAL gain.  MEASURED: the boost
   pitch oscillation is 0.665 Hz (4.18 rad/s) with damping ratio zeta = 0.05,
   i.e. essentially undamped.  Rate feedback cannot fix it (TAL_KWPM helps
   only 12-15% and destabilises past 1.3) because the PI zero is in the wrong
   place: K_i_integ_teta / K_p_integ_teta1 rises to 14-20 rad/s with q
     q=5.3  Kp 0.595 Ki 0.804  ratio  1.4
     q=17.4 Kp 0.244 Ki 1.474  ratio  6.0
     q=27.1 Kp 0.129 Ki 1.827  ratio 14.2
     q=46.9 Kp 0.091 Ki 1.843  ratio 20.2
   so at the 4.18 rad/s ringing frequency the loop is essentially PURE
   INTEGRAL and contributes -90 deg of phase.  Lowering Ki moves the PI zero
   below the oscillation and buys phase margin back.  1.0 = OFF, bit-exact. */
#define TAL_KITM  1.0

/* ---- TAL_PLEAD / TAL_PLEADW : LEAD COMPENSATOR ON THE PITCH LOOP ----
   The stage-1 ring is PHASE-limited, not gain-limited: 0.665 Hz (4.18 rad/s)
   at zeta = 0.05, with every gain lever measured and exhausted (more rate
   gain destabilises past 1.3; lower integral gain damps but breaks the gate
   at every value because the integrator carries pitch-program tracking).
   What the loop is short of is PHASE near 4.18 rad/s, so give it phase.

       C(s) = (1 + a*tau*s) / (1 + tau*s),   a = TAL_PLEAD > 1
       max phase  phi_m = asin((a-1)/(a+1))  at  w_m = 1/(tau*sqrt(a))
       tau chosen so w_m = TAL_PLEADW (rad/s), i.e. tau = 1/(w_m*sqrt(a))

         a = 2 -> 19.5 deg | a = 4 -> 36.9 | a = 6 -> 45.6 | a = 9 -> 53.1

   APPLIED TO du_compensate1, the PI OUTPUT, in stages 1 and 2 only.  That
   placement is deliberate: C(s) has DC GAIN EXACTLY 1, so boost trim, burnout
   velocity and the integrator's steady-state tracking are all untouched --
   which is precisely what TAL_KITM could not promise, and why that one broke
   the gate.  The terminal read (PILOT.CPP:1400) is NOT touched.
   COST TO WATCH: a lead amplifies high frequency by the factor a, so it buys
   phase with fin activity.  The binding constraint in this airframe is the
   20 deg fin budget -- exceeding it starves the rudder and departs (see
   TAL_KWPF).  Judge this term by worst |delec| over the 49, not by the gain.
   Discretised by Tustin on the ACTUAL dt taken from t, with the same guards
   and the same per-run reset as TAL_WOK -- the statics MUST be cleared in the
   pre-launch block or they persist across runs inside one MATLAB session.
   1.0 (or less) = OFF, bit-exact. */
#define TAL_PLEAD   1.5
/* SHIPPED 2026-08-23, ALWAYS-ON (TAL_PLEADR1 0.0 disables the transient ramp)
   after TAL_S6KWY 2.0 closed the T12n1 terminal cliff that had blocked it.
   FULL-49 tal=1: 49/49, mean 12.52, max 30.76, rmsPR q5-10 2.706 (-14%),
   q10-20 1.991 (-42%), q20-50 0.711 (-58%) vs the 3.154/3.447/1.684 baseline;
   worst |delec| 7.50 deg, bAz 3.641.  MARGIN: PLEAD 2.0 sextet all <100
   (T12n1 52.5 m, maxBeta6 11.1, maxPhi6 50.7 -- working but inside);
   KWY band 1.5/2.0/2.5 all clean.  CONSERVATIVE FALLBACK if the T12n1
   sentinel ever degrades: TAL_PLEADR0 2.0 / TAL_PLEADR1 6.0 (transient-only
   lead): mean 12.30, max 29.36, but ringing only -4/-11/-14%.
   T12n1 IS THE PERMANENT SMOKE TEST: 29-31 m healthy, 5000+ departed. */

/* ---- F3: LATERAL AUTHORITY DURING THE LOW-q TERMINAL PULL ---------------
   The flg==1 terminal departure (T12n1 under TAL_PLEAD, T10/T11 n1/n4
   historically) is a BETA RUNAWAY during the first ~12 s of stage 6: the
   dive pull sits on the alfa = -20 deg stop at q = 1-15, the elevator
   command is pinned, and in Outputs() the rudder is clamped FIRST with
   rudder_limit = 20 - |delec| - |delac| floored at only 6 deg -- so the yaw
   channel gets its minimum budget exactly when it must hold beta with
   almost no q.  The healthy case holds |beta| <= 4.4 through the pull; the
   departed one lets beta reach the 20 deg aero-table edge, extrapolated
   aero is garbage, drag explodes (V 2194 -> 234 m/s in 11 s) and it
   tumbles at q < 1 for 100+ s.
   All knobs act only when  sim_tal_flag && s_s == 6 && q < TAL_S6BQ
   (observable state only; tal_flag=0 bit-identical).
   TAL_S6RF / TAL_S6EF re-split the fin budget in that window: rudder floor
   6 -> TAL_S6RF deg, elevator floor 15 -> TAL_S6EF deg.
   TAL_S6KWY / TAL_S6KWR scale the terminal yaw / roll RATE damping in the
   same window (damping, not loop gain -- the TAL_KPA6Y loop-gain floor was
   measured harmful).  Defaults are a bit-exact no-op.
   MEASURED 2026-08-23 (sentinel sextet, TAL_PLEAD 1.5, T12n1 first):
     The budget premise was WRONG: fine trace of the departing T12n1 shows
     rudder_limit 9-19 deg with |delrc| <= 4.6 through the whole pull -- the
     alfa "stop" is the guidance incidence limit, delec ~ 10 deg, budget
     never binds.  RF/EF re-split therefore untested/left stock.
     The real mechanism is a ~1 Hz WING ROCK: at |alfa|=20, wx exchanges
     alfa into beta kinematically; wx grows 10 -> 30 deg/s over t=359-366
     (q 12-31) with the aileron at ~0.1 deg (K_w_roll = 0.064/q is 0.003 at
     q=25, open-loop) and beta diverges to the table edge.  Healthy cases
     carry the SAME oscillation and damp it; the lead's 1.9 m/s nudge flips
     the sign of its decay rate.
     KWR 20 (aileron damping): CATASTROPHIC, all 6 sextet cases depart to
       phi=180 (9 deg of aileron at 1 Hz = 57 deg/s, at the 75 deg/s servo
       rate limit -> phase-lagged pump; consistent with the TAL_S6ALIGN and
       TAL_ROLL180 negatives).  Roll is the wrong actuator here.
     KWY (rudder yaw-rate damping), BQ 40: 1.0 departs (5199 m) | 1.5 clean
       | 2.0 clean | 2.5 clean | 3.0 hits but n4 arms brush departure
       (beta 20, phi 175-179) | 3.5 fails n4 (144/221/107) | 5.0 fails n4
       (374/1027/422) -- too much rate feedback spoils the big commanded
       terminal yaw maneuvers of the n4 (thrust-misalignment) arms.
     SHIPPED: KWY 2.0 (center of the clean band), BQ 40, KWR/RF/EF stock.
     With TAL_PLEAD 1.5: sextet 29.4/9.9/17.5/26.5/15.6/11.2, T12n1
     maxBeta6 3.44 vs 4.86 at baseline; full 49 tal=1 49/49 mean 12.30 max
     29.36 rmsPR 3.019/3.068/1.450 delec 7.61 bAz 3.637; tal=0 bit-exact
     (48/49, 10.10/14.15).  Margin: PLEAD 2.0 sextet also fully clean
     (T12n1 29.0) -- the terminal cliff is closed, not dodged. */
#define TAL_S6BQ   40.0   /* q window upper edge for all four knobs        */
#define TAL_S6RF    6.0   /* stage-6 low-q rudder floor, deg   (6 = stock) */
#define TAL_S6EF   15.0   /* stage-6 low-q elevator floor, deg (15 = stock)*/
#define TAL_S6KWY   2.0   /* stage-6 low-q yaw rate damping x  (1 = stock) */
#define TAL_S6KWR   1.0   /* stage-6 low-q roll rate damping x (1 = stock) */

/* TAL_IYTAU0 -- flag-0 twin of the burnout yaw-trim release (exp bleed of
   out_integ_yaw while s_s==2), seconds.  MECHANISM, measured 2026-08-23 on
   the residual 7.07 g (fine traces of T12n2/T16n2, the two 7 g owners):
   the flag-0 peaks are NOT the Edit-4 roll sweep (phi < 13 deg, |p| < 10
   deg/s at peak) -- they are a lightly damped WEATHERCOCK ring in beta
   (+-2 deg, ~0.55 Hz) excited when the thrust-misalignment trim releases at
   burnout, which for the stock vehicle coincides with the s_s 1->2 switch
   (t = 22.3-23.5 s).  During the burn the integrator winds to +0.022 rad
   (+1.24 deg rudder) chasing the SATURATED ayc = -2.3 g command against the
   misalignment force (Az sits at +3 g, wrong sign -- the scheduled P gain is
   dead there, kpaRaw ~ 0); at release that stored trim torques the airframe
   INTO the unwind and cancels the TAL_KPAF0 floor's correction: at the
   T12n2 peak the P term is -0.70 deg, the integrator +1.24 deg, net rudder
   -0.34 deg against a -5.6 g error, with 19.9 deg of budget unused.
   NOTE the tal=1 TAL_IYTAU rejection does NOT transfer: the Edit-4 peak is
   roll-driven and lands BEFORE burnout with thrust on (s_s==1), where an
   s_s==2 bleed cannot act.  The flag-0 peak lands AT/AFTER the switch.
   0 = off, bit-exact stock.  tal=1 path untouched (gated !sim_tal_flag). */
/* TAL_IYTAU0 -- MEASURED AND REJECTED 2026-08-23.  A CONTINUOUS leak makes
   the peaks WORSE: at tau = 0.3 the top-6 owners went T12n2 7.071 -> 7.053,
   T16n2 7.062 -> 7.764, T15n2 5.683 -> 7.143, T14n2 5.460 -> 7.164 g.
   Reason: the integrator's RING-FREQUENCY component re-winds after release
   and ALIGNS with the P term on the second swing (T16n2 stock trace,
   t=23.48: I = +0.45 deg and P = +0.74 deg, same sign) -- the leak drains
   that too, so only the DC stale trim should be removed.  Kept as a knob,
   0.0 = off. */
#define TAL_IYTAU0  15.0

/* TAL_IYK0 -- ONE-SHOT knock-down of the stored yaw trim at the s_s 1->2
   transition (flag-0 only): out_integ_yaw *= TAL_IYK0 on the FIRST s_s==2
   sample, integrator fully live afterwards.  Removes exactly the invalid DC
   trim at the moment the thrust-misalignment force vanishes while keeping
   the integrator's dynamic (ring-damping) contribution stock.
   1.0 = stock, bit-exact. */
/* TAL_IYK0 -- MEASURED AND REJECTED 2026-08-23, WORSE THAN THE LEAK: at 0.0
   (full one-shot zero) T12n2 7.071 -> 8.785, T16n2 7.062 -> 10.182, T15n2
   5.683 -> 9.557, T14n2 5.460 -> 9.149 g.  DECISIVE: the stored trim is
   LOAD-BEARING at the s_s 1->2 switch because the thrust TAIL-OFF continues
   past it (t_burn1 runs to ~25 s) -- the "stale" trim is still balancing a
   live, decaying misalignment force, and removing it re-exposes the full
   disturbance.  The spike is therefore NOT a trim-release ring: it is the
   low-bandwidth loop failing to TRACK the fast-moving trim point while the
   force decays.  1.0 = stock. */
#define TAL_IYK0  1.0

/* TAL_KPAF02 -- s_s==2-ONLY override of the TAL_KPAF0 floor (flag-0).
   The global KPAF0 sweep (0.75 -> 9.13 g | 1.0 -> 7.07 | 1.25 -> 6.06 |
   1.5 -> 5.97 | 1.75 -> 47/49 | 2.0 -> DIVERGES T16n5) is bounded by
   MAX-Q stability, and max-q is always s_s==1 -- while the residual-7g
   owners peak in s_s==2 (burnout tail-off, q falling, loop gain lower).
   Splitting the floor lets the burnout window take the gain the max-q
   window cannot.  <0 = use TAL_KPAF0 (bit-exact stock).
   TAL_KPAF02Q: q-CLAMP in the floor's divisor while s_s==2 (flag-0):
   _kfl = KPAF02*0.125/max(q, KPAF02Q), i.e. the floor gain stops GROWING
   below q = KPAF02Q and holds constant.  WHY: every divergence in the
   KPAF02 sweep is LATE s_s==2 (t=27-29, q=30-45), far from the burnout
   peaks (q=45-76): 3.0 diverged 4 of 6 owners, 2.5 diverged T16n2 -- and a
   hard q>30 GATE did NOT save 2.5 (instability builds at q 30-45 before
   the gate cuts in).  The 0.125/q form holds kpa*q constant, but after
   burnout the vehicle is ~409 kg of propellant lighter, so accel-per-fin-deg
   RISES and the 1/q growth overcompensates; clamping q in the divisor keeps
   the late-s_s2 gain at or below the value the flat KPAF02=2.0 sweep proved
   stable.  0 = no clamp (stock shape). */
#define TAL_KPAF02  2.0
#define TAL_KPAF02Q  45.0

/* TAL_AZLD02 -- s_s==2-only flag-0 twin of the TAL_AZLD un-lag (remove the
   one-step (x+3y)/4 filter lag from the Az P path).  WHY HERE: the KPAF02
   2.5 divergence is NOT low-q gain growth (a q-clamp did not save it) --
   the fine trace of T16n2 shows the loop PUMPING a ~2.4 Hz mode from t=23
   on (rudder +-2..6 deg in phase with the ring, beta growing +-3 -> +-12
   -> 20-deg table edge), i.e. the floor is PHASE-limited: at 15 rad/s the
   Azb filter alone contributes ~28 deg of lag.  TAL_AZLD0 = 1.0 measured
   harmful GLOBALLY (47/49, with KPAF0 1.0 over all of boost incl. max-q),
   but paired with a raised floor it is exactly the tal=1 shipped composite
   (KPAF 1.5 + AZLD 1.0).  0 = off, bit-exact. */
#define TAL_AZLD02  1.0

#define TAL_PLEADW  4.18
/* TAL_PLEADLP -- first-order roll-off AFTER the lead, corner in rad/s.
   A PURE lead has high-frequency gain = a, and MEASURED that is what kills it
   here: at a = 2 the lead does damp the target bands (q 5-10 rms 3.154 ->
   2.465, q 10-20 3.447 -> 2.074) but worst |delec| goes 7.61 -> 20.00 deg,
   i.e. the elevator saturates, the rudder starves at its 6 deg floor and the
   vehicle departs (27/49, boost Az 55 g).  The phase is useful; the HF gain is
   not.  This low-pass returns the HF gain to ~1 above the corner while leaving
   the phase near TAL_PLEADW intact.  Put the corner ABOVE the ring (4.18) but
   BELOW the frequencies that drive fin activity.  0 = no roll-off. */
#define TAL_PLEADLP  0.0
/* TAL_PLEADQ -- upper q gate on the lead, and the lead runs in STAGE 1 ONLY.
   ROOT CAUSE of why every added pitch term failed, established by timing the
   events on T16n6:  rudder hits its 6 deg floor at t=23.94 FIRST, elevator
   saturates at t=24.12 (0.18 s later), lateral acceleration passes 10 g at
   t=26.35.  The trigger is YAW STARVATION just after burnout, not pitch.
   rudder_limit = 20 - |delec| - |delac| (floored at 6), so ANY extra elevator
   demand is taken straight out of the yaw channel, and after burnout the
   budget is already tight.  But the ring being damped lives at t = 6-15 s
   (q = 5-50), nowhere near that window.  So put the phase where the ring is
   and leave the fin budget alone where it is contested. */
#define TAL_PLEADQ  50.0
/* TAL_PLEADR0 / TAL_PLEADR1 -- make the lead TRANSIENT-ONLY, ramped in on
   |pitch rate| (deg/s).  A lead that is always on changes the nominal boost
   trajectory slightly, and MEASURED that is what breaks T12n1: burnout
   velocity moves by only 1.9 m/s (2417.0 vs 2415.1) but the case sits on the
   documented flg=1 TERMINAL departure boundary -- at t=357 it is already at
   alfa = -20 deg, and with the perturbation it tumbles: velocity collapses
   2194 -> 234 m/s in 11 s with alfa AND beta both pinned at the 20 deg aero
   table edge and phi at -90.  The lead does not cause that departure, it
   nudges a case that is already on the edge of it.
   Ramping the lead in on pitch rate means it contributes NOTHING while the
   loop is tracking quietly, so the nominal trajectory and burnout state are
   preserved, and it engages only while the vehicle is actually ringing.
   R1 <= R0 disables the ramp (lead always on, as first swept). */
#define TAL_PLEADR0  2.0
#define TAL_PLEADR1  0.0

/* ======= LEAD COMPENSATOR: BUILT, WORKS, BLOCKED BY ONE CASE =======
   DEFAULT 1.0 = OFF.  It is left in the tree fully built and measured.
   IT DOES WHAT IT WAS DESIGNED TO DO.  Gated to STAGE 1 and q < 50 (the band
   the ring lives in), stage-1 rms pitch rate, baseline -> lead:
     a=1.5 always-on   q5-10 3.154->2.706  q10-20 3.447->1.991 (-42%)
                       q20-50 1.684->0.711 (-58%)  mean 12.16  max 28.90
     a=1.5 transient   q10-20 3.447->3.068  q20-50 1.684->1.450
                       mean 11.76 and max 25.96 -- BOTH BETTER than baseline
   and fin demand stays clean (worst |delec| 7.61 -> 7.61 deg of 20, elevator
   rate 54 deg/s of 75).  There is no saturation and no lateral penalty.

   WHY IT IS OFF: ONE case, T12n1, fails at every setting that delivers real
   damping (a = 1.2 / 1.25 / 1.5 / 1.7 / 2.0, gated or transient: 4111 / 4765 /
   5522 / 5272 / 4783 m).  Only a = 1.1 holds 49/49, and it pushes that same
   case 29.1 -> 51.15 m, i.e. it is sitting on the gradient.
   T12n1 IS NOT BROKEN BY THE LEAD.  Boost is essentially unchanged -- burnout
   t 45.88 vs 45.90 s, V 2417.0 vs 2415.1 m/s (1.9 m/s), theta 27.56 both.
   The case sits on the documented flg=1 TERMINAL departure boundary: at
   t=357 it is already at alfa = -20 deg, and the nudge tips it into a tumble
   -- velocity collapses 2194 -> 234 m/s in 11 s with alfa AND beta both
   pinned at the 20 deg aero-table edge and phi swinging to -90 deg.  Stage 6
   then runs 141 s instead of 36 s at q < 18.  Same mechanism as the recorded
   T10/T11 flg=1 departure.

   THE ORDER OF WORK IS THEREFORE: fix the flg=1 terminal departure first,
   then set TAL_PLEAD 1.5.  That is worth doing on its own merits -- it is a
   robustness hole that any future change can fall into, exactly as this one
   did.  Do NOT ship the lead at 1.1 to get a number: it buys -13%/-28% while
   pushing the worst hit 29 -> 51 m on a case that is demonstrably on a cliff.

   ALSO MEASURED AND REJECTED on the way here:
     * PURE lead, no q gate, both stages: HF gain = a saturates the elevator
       (worst |delec| 7.61 -> 20.00 deg) which starves the rudder and departs
       laterally (27/49 at a=2, boost Az 55 g).
     * Adding a low-pass roll-off after the lead did NOT rescue it: to add
       phase at 4.18 rad/s you must raise gain above it, and any corner low
       enough to protect the fin also cancels the phase.
     * Raising the existing band-pass rate term TAL_WOK (0.15 / 0.30 / 0.50)
       saturates the same way and does not even reduce the ring.
   THE REASON ALL THREE FAILED IS THE SHARED FIN BUDGET, timed on T16n6:
       t=23.94  rudder hits its 6 deg floor      <-- FIRST
       t=24.12  elevator saturates
       t=26.35  lateral acceleration passes 10 g
   rudder_limit = 20 - |delec| - |delac|, so extra elevator demand is taken
   straight out of yaw, and just after burnout the budget is already tight.
   Gating the lead to stage 1 and q < 50 keeps it out of that window, which is
   why the gated version has clean fin demand. */

/* ============ 2026-08-23 SESSION 2: THE TWO REPORTED PROBLEMS ============
   PROBLEM 2 (tal_flag=0, yaw acceleration spike at t=22-25 s = burnout).
   CAUSE: every yaw fix was gated on sim_tal_flag, so the STOCK vehicle never
   received any of them -- it was still running the negative K_p_integ_acc
   above q~18.  FIX: TAL_KPAF0 = 1.0, the flag-0 twin of TAL_KPAF.
     boost peak |Az| over 49   13.554 -> 7.071 g  (-48%)
     mean miss                 10.40  -> 10.10 m  (BETTER)
     max hit                   13.49  -> 14.15 m ; gate still 48/49
     the known T13n2 failure   12933  -> 10096 m
   SWEEP: 0.75 -> 9.13 g | 1.00 -> 7.07 | 1.25 -> 6.06 | 1.50 -> 5.97 |
          1.75 -> 47/49 (T10n4 = 126 m) | 2.00 -> DIVERGES (T16n5 230 km).
   Shipped 1.0 for a 2x margin to divergence, matching the discipline used on
   TAL_KPAF.  1.25/1.5 give a further 15% of Az if margin is traded away.
   TAL_AZLD0 (the flag-0 un-lag) measured HARMFUL on its own (47/49) and is
   left OFF.

   PROBLEM 1 (tal_flag=1, stage-1 pitch ringing).  MEASURED PROPERLY:
   frequency 0.665 Hz (4.18 rad/s), damping ratio zeta = 0.05 -- essentially
   undamped.  Phase budget at that frequency is NOT the limit: rate filter
   (x+3y)/4 at dt=0.01 gives 8.3 deg, servo (wn 67.3, z 0.863) 6.1 deg,
   transport 7.2 deg, ~22 deg total.
   WHAT WORKED: TAL_KWPF 0.085 (the q=20-50 hole) plus TAL_KWPM 1.3 over the
   BAND q=5..20.  Band-limiting matters: applying the multiplier below q=5
   destabilises immediately, because q<5 is already at its stability edge
   (proved by TAL_ELOQ).  Cumulative rms pitch rate, before any fix -> now:
     q  5-10   3.603 -> 3.154  (-12%)
     q 10-20   4.366 -> 3.447  (-21%)
     q 20-50   7.121 -> 1.684  (-76%)
   WHAT DID NOT WORK, and why -- do not retry:
     * more rate gain below q=5      -> 185 deg/s, 29/49 (TAL_ELOQ test)
     * rate multiplier over all q<20 -> 46/49 at 1.6, 26/49 at 2.0
     * rate multiplier > 1.3 in band -> q10-20 gets WORSE (6.14 at 1.6)
     * LOWER pitch integral gain (TAL_KITM).  This is the textbook fix -- the
       PI zero Ki/Kp climbs to 14-20 rad/s, well ABOVE the 4.18 rad/s ring, so
       the loop is nearly pure integral there and eats 90 deg of phase.  It
       DOES damp: KITM 0.5 cuts q10-20 rms 4.048 -> 2.096 (-48%).  But the
       integrator carries the pitch-program tracking, so it breaks the gate at
       EVERY value tried: 0.85 -> 47/49, 0.7 -> 47/49, 0.5 -> 42/49,
       0.25 -> 34/49, 0.1 -> 25/49.  Left at 1.0.
   The residual ringing is therefore phase-limited, not gain-limited: to go
   further the loop needs a genuine LEAD term or a faster rate path, not more
   of an existing gain. */

/* TAL_KWPF = 0.085 SHIPPED 2026-08-23 (tal_flag=1 only; tal=0 bit-identical).
   SWEEP -- rms stage-1 pitch rate over q 20-50, all 49:
     0.000  7.121  49/49  mean 10.98  max 22.60   (the defect)
     0.070  3.854  49/49  mean 11.50  max 27.01
     0.085  1.907  49/49  mean 11.97  max 28.21   <== SHIPPED
     0.095  1.281  49/49  mean 12.02  max 29.23
     0.105  0.995  48/49  T16n5 = 432 km, boost Az 52 g
     0.120  0.828  45/49  four cases 430-490 km
   RESULT AT 0.085, stage-1 rms pitch rate (deg/s) before -> after:
     q 20-30  7.06 -> 2.53 (-64%)   q 30-40  9.48 -> 2.15 (-77%)
     q 40-55  3.89 -> 0.85 (-78%)   theta-err rms 1.74 -> 0.47
   WHY 0.085 AND NOT 0.095.  The cliff is NOT a gradual gain margin, it is a
   SATURATION BIFURCATION.  At 0.105 on T16n5 the elevator command jumps to
   19.99 deg of the 20 deg budget (from 5.04 deg at 0.085), achieved
   deflections 25/26 deg, and rudder_limit sits on its 6 deg floor 56.5% of
   boost -- starving the yaw channel, hence the 52 g lateral and the 430 km
   miss.  The real margin is therefore FIN DEMAND, not the gain ratio: at
   0.085 the worst command over all 49 is 7.61 deg of 20, i.e. 2.6x margin,
   and no case puts the rudder on its floor for more than 1% of boost.
   COST, stated plainly: mean miss 10.98 -> 11.97 m, max 22.60 -> 28.21 m,
   against a 100 m gate.  Bought a 64-78% cut in pitch oscillation. */

/* PITCH CHANNEL AUDIT, 2026-08-23 -- CLEAN, with two exceptions below.
   Every pitch gain schedule was checked for poles and sign flips over the
   flown q range, the way the yaw and roll ones were:
     K_w_pitch1      (0.1582q^2-3.577q+85.85)/(q^2+36.29q+114.9)
                     denominator roots -3.5 and -32.8, numerator discriminant
                     NEGATIVE -> no pole, no sign change, positive everywhere
     K_p_integ_teta1 (0.04634q+3.203)/(q+1.359)   pole at q=-1.359, zero at -69
     K_i_integ_teta  (0.00164q^3-0.1727q^2+7.351q+7.19)/(q+52.74)  pole at -52.74
   ALL poles and zeros are at NEGATIVE q, i.e. outside the flown range.
   MEASURED over the 49: max |K_p_integ_teta1| 3.425, |K_w_pitch1| 3.631,
   |K_i_integ_teta| 15.03 -- bounded, no 1000x spikes, and ZERO samples with a
   negative gain.  Elevator clipping 0.0% of samples: the pitch channel is
   never saturated.  The sign/pole defects really are confined to the
   K_*_integ_acc and roll families.
   EXCEPTION 1 -- the stage-4 K_w_pitch clamp above (TAL_KWP4).
   EXCEPTION 2 -- TAL_AYLD6/TAL_AZLD6 (terminal feedback un-lag) are INERT and
   left at 0.  Measured: pitch-only mean 11.01->11.08, yaw-only 11.12,
   both 11.12, with terminal Ay unchanged at 21.31 in every case.  The reason
   is structural: TAL_KPA6P clamps the terminal pitch proportional gain to ZERO
   over the 11-83% of stage 6 where the schedule is negative, so the term is
   multiplied by ~0.  A phase correction cannot matter where the proportional
   path is switched off.  Do not re-sweep them unless KPA6P changes.
   TAL_KWP4 = 20.0 SHIPPED: measured effect is within trajectory noise
   (max hit 22.60 / 24.20 / 23.74 at clamps 20 / 5 / 1 vs 24.14 off,
   non-monotone; loads identical to 3 decimals).  It is shipped for ROBUSTNESS,
   not for the numbers: it converts an unbounded gain into a bounded one and
   uses the value the SAME gain already carries in stage 5, so it introduces no
   new tuning constant.  Post-clamp max K_w_pitch = 20.0 (was 100.2). */

/* SHIPPED 2026-08-23 at 1.0.  SWEEP (tal=1, all 49/49):
     0.00  bAz 3.913  mean 11.04  max 22.51  tAy 21.134
     0.25  bAz 3.792  mean 11.08  max 24.00  tAy 21.161
     0.50  bAz 3.655  mean 11.04  max 23.91  tAy 21.218
     1.00  bAz 3.466  mean 11.01  max 24.14  tAy 21.306   <== SHIPPED
   Monotone in Az, and 1.0 is the PRINCIPLED endpoint, not a tuned value: it
   feeds the P path exactly -ayc + az_b[k], the true current accelerometer.
   Going past 1.0 would be raw differentiation of a signal that steps at
   burnout -- do not.
   THE RATE-DAMPING CAVEAT WAS CHECKED AND IS FALSE.  Measured on T16n2 over
   boost: corr(_azl, wy_compensate) = -0.185 (weakly ANTI-correlated, not
   same-signed) and rms(_azl)/rms(du_yaw) = 0.06.  It is a small phase
   correction, not extra yaw rate damping, which is why it does not reproduce
   the TAL_KWY failure.  Cost: max hit 22.51 -> 24.14 m against a 100 m gate;
   mean IMPROVES 11.04 -> 11.01.  tal_flag=0 bit-identical. */

/* TAL_KPA6Y / TAL_KPA6P -- the SAME sign error, in the TERMINAL phase.
   K_p_integ_acc drives BOTH endgame loops: the yaw read (PILOT.CPP:1211, live
   for s_s>2) and the PITCH read (PILOT.CPP:1300, inside the
   s_s==3||5||6 block).  MEASURED over stage 6: the gain is NEGATIVE for
     T13n2 11%   T11n2 17%   T13n5 23%   T16n2 59%   T15n3 67%
     T16n5 74%   T10n4 80%   T12n6 80%   T13n4 82%   T11n0 83%
   of the terminal phase -- worst on the short, high-q reentries where stage 6
   spends half its time above q=18.  Stages 3 and 5 are clean (q ~ 0, gain is
   0.1 or 0).  Y floors the yaw read, P the pitch read, so they can be
   isolated.  Semantics as TAL_KPAF: <0 disables (bit-exact no-op), 0 clamps
   the gain at zero (removes the wrong sign only), >0 floors at K*0.125/q. */
#define TAL_KPA6Y  (-1.0)
#define TAL_KPA6P  0.0

/* POLE GUARDS -- three gain schedules have a POLE (denominator zero) INSIDE
   the flown q range, and all 49 cases fly through them at launch:
     K_p_integ_roll  pole q=0.1530   gain flips -1.44 <-> +1.44, normal ~0.012
     K_w_roll        pole q=0.8740   gain flips +17.2 <-> -17.2, normal ~0.024
     K_i_integ_acc   pole q=1.8623   gain flips -268  <-> +268 , normal ~0.03
   MEASURED IN FLIGHT: |K_i_integ_acc| reaches 461.7 on T13n6 at q=1.8623,
   t=4.02 s (that is ~15000x its normal value) and every one of the 49 cases
   traverses a pole neighbourhood.  out_integ_yaw is limited to +-0.0835, so a
   single step at that gain rails the integrator outright.
   SIGN, separately from the poles -- each fit is also WRONG-SIGNED over a band:
     K_i_integ_acc  NEGATIVE for q in (0.189, 1.862)  <- the whole early launch
     K_w_roll       NEGATIVE for q in (0.874, 1.195)  <- anti-damping in roll
     K_p_integ_roll NEGATIVE for q < 0.153
     K_i_integ_roll NEGATIVE for q < 0.366  (its q<30 polynomial branch)
   Guards are applied at the POINT OF USE (one site per gain), never by editing
   the 11 scheduling sites.  Bitmask: 1 = clamp negatives to zero (fix the
   sign), 2 = cap the magnitude (kill the pole spike), 3 = both, 0 = OFF and
   bit-exact.  ACC covers K_i_integ_acc (yaw AND pitch integrators), ROLL
   covers K_w_roll / K_p_integ_roll / K_i_integ_roll. */
#define TAL_PG_ACC   0
#define TAL_PG_ROLL  1
#define TAL_CAP_KIA  5.0   /* pole spike reaches 461; legit low-q values are ~0.1-1 */
#define TAL_CAP_KWR  2.0   /* pole spike reaches 60.9 */
#define TAL_CAP_KPR  0.5   /* pole spike reaches 1.08 */
#define TAL_CAP_KIR  0.5   /* pole spike reaches 1.09 */

/* ROLL-CHANNEL RESULT, SHIPPED 2026-08-23: TAL_PG_ROLL = 1, TAL_PG_ACC = 0.
   Measured spikes in flight, all 49 cases: |K_w_roll| 64.6 (normal ~0.02,
   i.e. ~3000x), |K_p_integ_roll| 1.079 (~2700x), |K_i_integ_roll| 1.089
   (~220x), |K_i_integ_acc| 461.7 (~15000x).  Worst on the flg==2 (n2) arms,
   which linger near the pole longest.
   KEY RESULT: THE POLE SPIKES ARE TRAVERSED BUT BENIGN.  Capping magnitude
   alone (PG=2, caps 2.0/0.5/0.5) is bit-for-bit indistinguishable from doing
   nothing on every metric -- the spikes last only a few samples.  It is the
   WRONG-SIGNED BANDS that cost, and clamping them is a strict improvement:
                     HITS  mean   max    boostAz  termAy  roll  phi
     baseline        49   11.08  23.33   3.959   21.345  18.8  33.9
     ROLL=1 (ship)   49   11.04  22.51   3.913   21.134  18.8  33.0   better x6
     ACC=1           49   11.07  22.25   4.252   20.412  17.8  40.7   mixed
     ACC=1 ROLL=1    49   11.05  21.38   4.232   20.412  17.8  39.4   mixed
   TAL_PG_ACC IS LEFT OFF: clamping the K_i_integ_acc sign improves terminal
   Ay and max miss but costs 7% on boost Az (3.959 -> 4.252) and 20% on peak
   roll attitude (33.9 -> 40.7 deg).  Given boost Az is the load we were asked
   to reduce, that trade is a step backwards.  Revisit only with a decision on
   which load dominates.
   DO NOT USE TIGHT CAPS.  The first attempt used caps from the q>=2 range
   (KWR 0.05 etc.), but these gains are legitimately ~1/q-large at low q, so
   the caps clipped real launch-phase control: 13/49 hits, phi = 180 deg
   (tumbling), termAy 56 g.  The caps above are set high enough to clip only
   the pole spike, and at that level they measure neutral. */

/* TERMINAL RESULT, SHIPPED 2026-08-23: KPA6Y OFF, KPA6P = 0 (zero-clamp).
   The two axes behave OPPOSITELY and must not be tied together:
     Y=0  P=off : 48/49 (T12n4 3633 m), termAz 13.7 -> 36.5, termAy 27.7 -> 56.1
                  -- flooring the terminal YAW gain is HARMFUL both ways. Keep off.
     Y=off P=0  : 49/49, meanHit 11.33 -> 11.08, maxHit 28.05 -> 23.33,
                  termAy 27.673 -> 21.345 g (-23%), termAz unchanged. <== SHIPPED
     Y=0   P=0  : 49/49 but the yaw damage dominates (termAz 40.6, termAy 52.5).
   ONLY THE ZERO-CLAMP IS VIABLE ON PITCH -- adding real positive gain
   overdrives the endgame, and fast:
     P=0.25  44/49 (5 failures 1.2-2.2 km)
     P=0.50   9/49 (sumFail 282 km)
     P=1.00   0/49        P=2.00  0/49
   That is consistent with the physics: the terminal loop was tuned while the
   scheduled gain sat slightly NEGATIVE, so removing the wrong sign is a small
   correction, whereas restoring 0.125/q is a large one the endgame was never
   designed for.  Do not sweep P upward again. */

/* SHIPPED 2026-08-23 at TAL_KPAF 1.5 / KPAE0 1.0 / KPAE1 3.0 (tal_flag=1 only;
   tal_flag==0 is bit-identical, verified 48/49 mean 10.40 max 13.49).
   FULL-49 RESULT   peak|Az| 5.884 -> 3.959 g (-33%)   p90 4.500 -> 3.225
                    median |Az| 2.798 -> 2.079 (-26%)
                    gate 49/49 held, meanHit 10.94 -> 11.33, maxHit 28.94 -> 28.05
   PER CASE on the arms the user flagged (n2 and n5 are both flg==2):
     T11n2 5.151 -> 3.247 (-37%)   T16n2 4.651 -> 3.130 (-33%)
     T10n2 4.116 -> 2.972 (-28%)   T13n2 4.012 -> 2.929 (-27%)
     T16n5 3.119 -> 2.483 (-20%)   T13n5 2.465 -> 2.079 (-16%)
     T13n4 5.884 -> 2.738 (-53%)   T10n4 4.618 -> 2.216 (-52%)
   FLAT-FLOOR SWEEP (KPAE1<=KPAE0, gain raised in trim as well as transient):
     0.25 49/49 pkAz 4.633 | 0.30 49/49 4.570 | 0.35 49/49 4.534 (best flat)
     0.40 49/49 but one case at 96.8 m -- no margin | 0.45 FAILS T11n2 191.9
     0.50 FAILS T11n2 250.4 | 1.00 FAILS T16n5 1968.8
   The flat floor runs out of accuracy because it also moves the steady trim,
   hence burnout velocity, hence every miss.  Gating on |du_yaw| leaves trim
   untouched and buys far more:
   TRANSIENT-GATED SWEEP (KPAE0 1.0, KPAE1 3.0):
     1.25 49/49 pkAz 4.045 | 1.50 49/49 3.959 <== SHIPPED | 2.00 49/49 3.819
     2.50 49/49 3.814 | 3.00 DIVERGES (pkAz 46.4 g, 45/49, 4 cases 250-430 km)
     4.00 43/49 | 6.00 39/49
   WHY 1.5 AND NOT 2.5: divergence starts between 2.5 and 3.0, so 1.5 keeps a
   2x (6 dB) gain margin to the cliff and costs only 3.5% more peak Az than
   2.0.  Shipping nearer the cliff buys almost nothing and loses the margin.
   The instability appears first on exactly the n2/n5 (flg==2) arms -- the same
   cases being treated -- so the margin matters there most. */
#define DIAG_ZERO_ETH  0   /* zero eth1/eth2 (nozzle cant) for flg==2   */
#define DIAG_ZERO_YZ   0   /* zero Yt/Zt (thrust offsets) for flg==2    */
#define DIAG_ZERO_XBC  0   /* zero xbc_y/xbc_z (lateral CG) for flg==2  */
/* same isolation for the flg==1 arms (n1, n4) */
#define DIAG_ZERO_ETH1 0
#define DIAG_ZERO_YZ1  0
#define DIAG_ZERO_XBC1 0

/* Terminal command cap.  GUIDANCE.cpp raises acc_limit x2.0 in fly mode on
   the strength of an "a_max ~ 4q" estimate that was later measured to be
   3-4x optimistic (real a_max ~ q).  2.0 = stock. */
#define TAL_ACCX  2.0

/* Donor HGV stage-6 glide correction (GUIDANCE.cpp, merge step 3).
   0 = donor behaviour, glide applies to every terminal.
   1 = restrict it to the LOW-ENERGY terminal (Vm_t < 675 at 50 km), which is
       the case it actually helps -- on hot arms it commands a 59 deg dive
       they do not need. */
#define GLIDE_LOWE_ONLY  0
/* Authority of the donor glide over ac_pitch: the donor blend saturates
   within ~4 km of a 50 km window, so it is effectively a hard switch.
   GLIDE_AUTH scales the blend weight; 1.0 = donor. */
#define GLIDE_AUTH  0.25

/* Donor stage-1 / stage-2 autopilot gain schedules (merge steps 11 and 12).
   Applied as an override at the END of each stage block, before the K_*33 /
   K_*22 captures, so the local block keeps its side effects (t_q_p1/t_q_p2,
   k_ramp) and only the gain VALUES are replaced.  0 = local, 1 = donor. */
#define DONOR_S1  1
/* Sub-flag: take the donor stage-1 ROLL gains too, or keep the local ones.
   The donor closes the low-q roll loop ~6x harder, which the local TAL_ROLLP
   work showed makes the flg==2 arms worse. */
#define DONOR_S1_ROLL  1
/* Two-tier LRG glide altitude (merge step 11c). */
#define LRG_DEEP_DV   (-65.0)
#define LRG_HC_DEEP   25.0e3
/* Stage-2 theta_p blend-in window, seconds (GUIDANCE.cpp M_teta_command).
   The ign2_stg!=2 path -- which is OURS, since reentry_ign==0 sets
   ign2_stg=0 -- ramps the (saturating, up to 5 deg) theta_p trim in over
   only 0.5 s, while the other paths use 5.5 s and 10.5 s. That is ~10 deg/s
   of commanded attitude rate and is the step behind the stage-2 transient. */
#define TAL_TSLOP  6.0
/* 2026-08-23, user asked for a 15 s theta_p ramp. 15 s DOES smooth it hard
   (stage-2 peak du_pitch 3.619 -> 0.862 deg, -76%) but costs SEVEN hits:
   49/49 -> 42/49, with T12n4/T14n1/T14n4/T15n1/T15n4/T16n1/T16n4 at
   3.5-10.7 km. Measured boundary, with TAL_AYCR 15 active:
     1.0 -> 49/49        4.0 -> 49/49 (du_pitch 2.396, max 25.6)
     6.0 -> 49/49 (1.960, max 28.1)  <-- largest that holds the gate
     8.0 -> 48/49 (T12n4 7158.6)     15.0 -> 42/49
   Shipped at 6.0: -46% on the command, gate intact. Set 15.0 to take the
   user value and accept 42/49. */
/* Same window for the STOCK vehicle. It is NOT the same optimum: tal_flag=0
   T16n2 is a knife-edge case that HITS at 2.0 and fails at 0.5/1.0, while the
   Edit-4 vehicle prefers 1.0. Per-vehicle values, measured on the full 49. */
#define TAL_TSLOP0 2.0
/* Blend half-width, in q, for the stage-2 pitch P-gain switch at q=31.
   PILOT.CPP applies K_p_integ_teta1 = 0.6523*(...) and then, for q>31,
   0.105126523*(...) -- a 6.205x STEP in the pitch proportional gain, taken
   while q decays through 31 at t = 23.4-25.2 s, i.e. exactly on the stage-2
   transient. 0 = hard step (stock); W blends over q in [31-W, 31+W]. */
#define TAL_Q31W  8.0
/* Blend half-width, in q, for the STAGE-1 branch switch at q=20, where the
   donor schedule swaps its entire gain set (K_w_yaw drops 2.18x and
   K_p_integ_acc changes SIGN across it). 0 = hard switch (stock). */
#define TAL_Q20W  1.0
/* Stage-1 pitch-loop damping trim. The stage-1 attitude loop carries a
   sustained ~0.8 Hz oscillation through max-q (wz peak-to-peak 48 deg/s on
   T11n1, tetam swinging +-3 deg about tetac). That is a DAMPING problem, not
   a discontinuity, so blending does nothing for it. S1KW scales the pitch
   RATE feedback (more damping), S1KP scales the attitude PROPORTIONAL gain
   (less loop gain). 1.0 = stock. */
#define TAL_S1KW  1.2801
#define TAL_S1KP  1

/* WASHOUT RATE FEEDBACK (stage 1).  Raising K_w_pitch1 damps the ~0.8 Hz
   max-q oscillation superbly (-64% at 1.5x) but costs accuracy at EVERY
   level, because that gain also feeds wz_compensate1 into delec0 and so
   moves the boost TRIM, not just the damping.
   The washout adds an EXTRA rate term built from the HIGH-PASSED pitch rate:
     wz_compensate1 += K_w_pitch1 * TAL_WOK * (wzr - lowpass(wzr))
   At steady state the high-passed term is ZERO, so the trim is untouched and
   the stock schedule is preserved exactly; at the oscillation frequency it
   acts like an extra TAL_WOK of rate gain.
   TAL_WOTAU is the washout time constant in seconds -- it must be well ABOVE
   the oscillation period (1.27 s) so 0.8 Hz passes while DC is blocked.
   TAL_WOK 0 = off (bit-identical to stock). */
#define TAL_WOK    0.05
#define TAL_WOTAU  2.0
/* Upper corner. A PURE washout keeps gaining with frequency and so amplifies
   the actuator band: measured at TAL_WOK 0.5 it damped the 0.8 Hz mode well
   (pk-pk 48 -> 15.2) but drove peak wz to 67.5 deg/s with the fins saturated,
   and at 1.0+ it diverged to 450 deg/s. TAL_WOLP low-passes the high-passed
   signal, making it a BAND-PASS centred on the oscillation: DC blocked by
   TAL_WOTAU, the actuator band rolled off by TAL_WOLP (seconds). */
#define TAL_WOLP   0.06

/* Yaw-command ramp at stage-2 entry, seconds. ayc reaches ~1.05 g in the
   first 6 s of stage 2 (T10n4 1.05, T16n2 1.04) against a 2.3 g limit, which
   is the yaw half of the entry transient. Ramps ayc in from zero over this
   window, keyed to t_vc (the stage-2 latch). 0 = off. */
/* TAL_AYCR -- REJECTED 2026-08-23. Ramping ayc up from ZERO at the stage-2
   latch minimises the value but ADDS a discontinuity: measured on T10n4, ayc
   went 1.0952 g (last stage-1 sample) -> 0.0000 g (first stage-2 sample), a
   1.055 g STEP at the boundary. Wrong shape. The cap below is continuous
   across the boundary and reaches BOTH the end of stage 1 and the start of
   stage 2, which is where the command actually is. 0 = off. */
#define TAL_AYCR  0.0
/* Yaw acceleration-command cap, g. M_acc_yaw_command_0 clamps ayc to 2.3 g,
   and late stage 1 SATURATES it: T10n4 reaches exactly 2.3000 g at t=20.6,
   T16n2 -1.856 g. Lowering the cap minimises the yaw command through the end
   of stage 1 and into stage 2 with no step anywhere. 2.3 = stock. */
#define TAL_AYCLIM  1.8
/* Cross-range VELOCITY feedback gain in M_acc_yaw_command_0 (local 25.5,
   donor 22.0). ROOT CAUSE of the railed yaw command: ayc = kg*(z_err +
   k_vz*vz_err), and the velocity term dominates 5-20x --
   measured kg*z_err 0.1-0.5 against kg*vz_err 2.2-2.6, which is what pins
   ayc to its cap. The raw velocity error is large in late stage 1 (T10n4
   commands vzm_c -8.7 m/s against an actual Vzm of -69.0; T16n2 -6.6 vs
   +49.6), so a gain of 25.5 on it saturates the command. Edit-4 only. */
#define TAL_KVZ  22.0
/* Yaw-trim release time constant at BURNOUT, seconds. The thrust-induced yaw
   moments are ALL proportional to thr (SUBSYS.CPP mtb0/mtb2), so they vanish
   when the motor stops -- but out_integ_yaw is only cleared at SEPARATION,
   which is several seconds later. In between the integrator holds trim
   against a force that no longer exists, and the fins over-control: measured
   at the T16n2 boundary, achieved Azb reaches -4.213 g while the command is
   only -0.978 g, i.e. the response is 4.3x the command and therefore
   DISTURBANCE-driven, not command-driven. Decays the trim through stage 2.
   0 = off. Edit-4 only. */
/* TAL_IYTAU -- MEASURED AND REJECTED 2026-08-23. Releasing the yaw trim at
   burnout is physically well-motivated (the thrust yaw moments are all
   proportional to thr and vanish there, while out_integ_yaw is only cleared
   at SEPARATION seconds later) -- but it does NOTHING here: peak achieved
   |Azb| stayed at 5.168 g at tau = 0.0 / 0.5 / 1.0 / 2.0, bit-identical, and
   0.5 and 1.0 each cost a hit (T11n2 2794, T12n4 6790). The trim is not the
   source. See the ROOT CAUSE note below. 0 = off. */
#define TAL_IYTAU  0.0

/* ================= ROOT CAUSE of the "big yaw acceleration" =================
   It is NOT the yaw command, and no yaw-side lever can fix it.
   The peak achieved |Azb| is 3.8-5.2 g on dispersed arms against 0.19-0.69 g
   on NOMINAL ones, and it lands 2.5-5.5 s BEFORE the stage 1->2 boundary, at
   q = 48-68 kPa with thrust still on. Sideslip there is only 1.0-2.4 deg,
   which accounts for well under 1 g -- the number never reconciled with a
   steady aerodynamic side force.
   A fine trace of T13n2 settles it. Over ~1 s the body-Z aero force fab[2]
   swings +6395.9 N -> +413.8 -> -6395.2 (a FULL SIGN REVERSAL) while phi
   rotates steadily at ~14 deg/s (-41.4 -> -54.9 deg). The vehicle is ROLLING,
   so a lateral force sweeps around the body axes and shows up in the yaw
   accelerometer as a large oscillation. Peak |Azb| is monotonic in ROLL,
   not in anything yaw:
     T11n0 phi  -0.14 deg -> 0.193 g      T10n4 phi -10.01 -> 4.172 g
     T11n1 phi  12.72     -> 3.767 g      T16n2 phi -22.78 -> 4.908 g
     T13n2 phi -55.52     -> 5.168 g
   So the source is the ROLL EXCURSION from the modelled nozzle misalignment
   (flg 1/2) -- the same disturbance documented in the boost-excursion work,
   which is authority-limited: the torque arrives where q is too low for the
   fins to counter it. Confirmed by exhaustion: capping ayc (2.3 -> 1.8 -> 1.5)
   and sweeping k_vz (25.5/22/15/10) both left peak |Azb| at exactly 5.168 g,
   and the achieved acceleration is 3-5x the command throughout.
   FIX THE ROLL, or accept it. Do not re-tune the yaw channel for this. */
/* Measured 2026-08-23, full 49 x both flags:
     2.30 (stock) -> end-S1 |ayc| 2.3000 SATURATED, 49/49, mean 11.64
     1.80 SHIPPED -> 1.8000, 49/49, mean 11.65, max 29.03  (-22% peak)
     1.50         -> 1.5000, 49/49, mean 11.65, max 28.94  (-35% peak)
     1.35         -> 48/49  (T10n1 3357)
     1.25         -> 47/49  (T10n1 27426, T11n4 3940)
     1.00         -> 46/49  |  0.70 -> 39/49
   1.80 CHOSEN over 1.50 (user decision, 2026-08-23) for MARGIN. 1.35 already
   fails, so 1.5 would sit only 0.15 g above a cliff with T10n1 breaking first;
   1.8 has 3x that margin for no measurable accuracy difference (mean 11.65
   either way, max 29.03 vs 28.94). It gives up 13% of the available command
   reduction: peak yaw 2.30 -> 1.80 g (-22%) rather than -> 1.50 (-35%). */
/* Fin-command SLEW LIMIT, deg/s. The servo is 2nd-order with
   speed_servo = 75 deg/s (SUBSYS.CPP), but the autopilot commands up to
   450 deg/s through the stage-2 transient -- 6x beyond what the actuator can
   deliver, so it sits rate-saturated. Limiting the COMMAND to something the
   servo can actually follow costs nothing physically and removes the
   saturation nonlinearity. 0 = off. */
/* MEASURED AND LEFT OFF 2026-08-23. The apparent 450 deg/s fin rate in
   stage 2 is a MEASUREMENT PHANTOM: sample spacing is a uniform 0.01 s and
   the 99th percentile fin rate is only 10.8 deg/s. The 450 is a SINGLE
   sample at the very end of stage 2, where Outputs() hard-zeroes
   delec/delrc/delac at separation (StopCtrlCmd && s_s<3). That is a COMMAND
   step the 75 deg/s servo already smooths into the real fin motion, and it
   happens downstream of this limiter, so slew limiting cannot see it:
   sweeping 300/150/75 deg/s left the metric at 450/449/449 and moved nothing.
   Kept (inert) in case a command rate limit is wanted later. */
#define TAL_SLEW  0.0
#define DONOR_S2  0

/* Incidence clamp policy for the aero lookup.  AERO.CPP stacks TWO clamps:
   the values are first clamped to +-20 deg (the edge of the tables, correct),
   and then a SECOND block tests the UNCLAMPED copies (alfaa/alfabb/bettaa)
   against 135 and, if they exceed it, overwrites the already-clamped value
   with 135.  So the lookup argument STEPS from 20 to 135 the moment true
   incidence crosses 135 deg -- a 6.75x jump that the interpolator then
   extrapolates ~30 segment-lengths past its last breakpoint.
   1 = stock (keep the 135 override), 0 = keep the 20 deg clamp only. */
#define TAL_A135  0

/* Roll to the NEAREST of 0 / +-180 instead of always to 0.  For this
   symmetric airframe a 180 deg bank is aerodynamically equivalent and the
   pitch/yaw resolution (delec0*cos(phim) - delrc0*sin(phim)) already handles
   the sign flip -- so the vehicle need never rotate THROUGH 90 deg, where the
   pitch demand resolves into a pure skid and beta hits the 20 deg table edge.
   0 = stock (always target 0).  Bit 1 = stage 6 only, 2 = all stages. */
#define TAL_ROLL180  0

/* Roll-rate cutout threshold on |delac| (deg).  The stock guard at
   PILOT.CPP:1094 demands |delac| > 14 deg, but delac is limited to at most
   8 deg (4 deg once the incidence clamp trips), so the protection is
   UNREACHABLE.  14.0 = stock/dead; a value below 4 makes it live. */
#define TAL_RCUT  14.0

/* Stage-6 roll alignment.  The coast leaves the vehicle banked (phi to 172
   deg on the flg==1 failures); stage 6 then flies the guidance pull as a
   SKID, because the frame resolution turns pitch demand into rudder, and
   incidence runs past the 20 deg edge of the aero tables.
   TAL_S6ROLL/TAL_S6SAT scale the roll gains and the roll-error clip while
   banked; TAL_S6HOLD scales the manoeuvre command by |cos(phi)| (floored)
   so the roll channel gets time to align first.
   Stock = 1.0 / 1.0 / 0. */
#define TAL_S6PHI    30.0
#define TAL_S6ROLL    1.0
#define TAL_S6SAT     1.0
#define TAL_S6HOLD    0
#define TAL_S6FLOOR   0.25

/* --- aerodynamic increments for the stretched body -----------------------
   The coefficient tables (RAJEH_AERO_S1.dat etc.) describe the STOCK
   airframe and cannot be regenerated here, so Edit 4's aerodynamics are
   modelled as first-order increments on top of them. Engineering-level, not
   CFD -- stated as such in the report.

   TAL_XAERO_S1 is the longitudinal station the stage-1 tables are referenced
   to. It matters because xbc (SUBSYS.CPP M_char) is built as xcg2_1 - Xcg,
   so xcg2_1 is doing DOUBLE DUTY: the stage-1 burnout CG *and* the aero
   reference. Edit 4 moves the burnout CG to 2.122592 but the tables still
   describe a body referenced to 1.995907, so the two must be decoupled or a
   mass change would silently shift the aerodynamic reference by 12.7 cm.

   TAL_DCA: skin friction over the added wetted area,
     Cf*(pi*D*L)/Aref = 0.0030*0.561246/0.123786 = 0.013603.
   TAL_KCROSS: viscous crossflow on the added cylinder,
     eta*Cdc*(D*L)/Aref = 0.7*1.2*0.17865/0.123786 = 1.2123, times sin^2(a).
   TAL_XARM_PLUG: that force acts at TAL_XPLUG, i.e.
     (3.976850 - 2.041007)/0.397 = 4.87618 calibres aft of the effective
     reference (TAL_XAERO_S1 + the s_s>1 xbc_er bias of 0.04510).           */
#define TAL_XAERO_S1   1.995907 /* m, stage-1 aero reference station        */
#define TAL_DCA        0.013603 /* added axial-force coefficient            */
#define TAL_KCROSS     1.2123   /* crossflow normal-force coefficient scale */
#define TAL_XARM_PLUG  4.878823 /* calibres, plug centroid aft of reference */

/* Tail share of total normal force. Moving the tail aft by TAL_DFIN moves the
   whole airframe's aerodynamic centre aft by TAL_DFIN*TAL_TAIL_SHARE, which
   is the difference between a stable and an unstable airframe here: Edit 4
   puts the full-vehicle CG at 2.695964 m while the stock AC sits near 2.5 m,
   so WITHOUT this shift the CG lies behind the AC and the model is unstable
   (measured: boost alpha pinned at the 20 deg clamp, fins saturated 60% of
   the time, burnout 2449.8 -> 2165.2 m/s despite +24% impulse).

   The shipped tables give only the TOTAL cma/cna and cannot resolve the
   tail's share, so this is a STATED ASSUMPTION, not a derived value. 0.55 is
   a standard figure for a tail-stabilised airframe; at that value the AC
   moves aft 0.2475 m against a full-vehicle CG shift of 0.2472 m, i.e. the
   static margin is very nearly preserved - which is the physically expected
   outcome of translating body and tail together. Sensitivity is reported
   rather than hidden: see the tal_flag report.
   Applied as dCm = -(dx/D)*cn on the TOTAL coefficients, so it is
   dimensionally consistent (an earlier attempt mixed the rate derivative Cnq
   into a total coefficient and was rejected). */
/* MEASURED 2026-08-22, not guessed. 0.55 (a textbook tail-stabilised figure)
   was tried first and DESTABILISED the vehicle: it moves the AC aft 0.2475 m
   while the EMPTY CG moves only 0.1295 m, so post-burnout static margin rises
   0.118 m. A stiffer airframe at unchanged damping means a lower damping
   ratio, and a dispersion kick at burnout -- where q collapses 45 -> 5 Pa in
   eight seconds -- then diverges instead of decaying. Measured at 0.55: fins
   ramp 0.30 -> -3.34 -> -9.18 -> +20.00 -> -20.00 deg, sign-flipping with
   growing amplitude from t=27 s, alpha and beta to the 20 deg clamp, roll to
   180 deg, misses of 189-267 km on T12/T14 at noise 4 and 6.
   At 0.35 the empty margin changes by only +0.028 m, so the airframe stays
   close to what the autopilot is tuned for: the same six cases fly at
   5.5-14.6 m with ZERO fin saturation and alpha ~2.7 deg. 0.20 also survives
   but flies worse (alpha 3.6-5.7, roll 2-3.5 deg), so 0.35 is the choice.
   NOTE this was an AERODYNAMIC modelling error, not an autopilot deficiency --
   the stock gains handle the Edit-4 airframe once its stability is right. */
#define TAL_TAIL_SHARE 0.35

/* TARGET BANDS. The two vehicles are validated over different envelopes,
   because they have different energy:
     tal_flag = 0  ->  300 .. 700 km
     tal_flag = 1  ->  500 .. 850 km
   This is what makes the launch-elevation solve well posed. Elevation rises
   with target range on this airframe (stock: 300 km -> 62.1 deg, 500 km ->
   64.1, 700 km -> 71.0), and the generator refuses any solve below 50 deg --
   breaking out BEFORE the laun.dat write, which silently leaves the target
   with another target's launch data. Edit 4 is more energetic and so needs
   LESS loft for a given range, which pushed the stock 300..730 km set down to
   43..50 deg and tripped that floor on every target but one. Re-basing the
   Edit-4 set to 500..850 km puts its solutions back above 50 deg.
   The whole Edit-4 model -- mass, thrust, aero AND inertia -- applies in every
   mode including generation, so the tables describe the vehicle that flies
   them. */
#include "math/mat_platform.h"
#define TAL_R(stock,tal) (sim_tal_flag ? (tal) : (stock))

extern int sim_tal_flag;

/* Defined in SUBSYS.CPP (where the mass globals live); called from
   Initilize_sim() in M273N.CPP once per run. Split in two because
   Total_Imp is not loaded until the once-per-process thrust-file read. */
#if !defined(MAT_PLATFORM_EMBEDDED)
void TalApplyMass(void);
void TalApplyImpulse(void);
#endif

#endif /* SIM_FIO_H */
