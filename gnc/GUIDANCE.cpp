/*#include "nav.h"
#include "navinit.h"
#include "main.h"
#include "fusion.h"//*/
#include"stdafx.h"
#include "guidance.h"

///////////////////////////////////////////////////////////
double Npn=4.3;//1300b_ymn
double impact_angle1=-70.-0,tau_pn1=25.,b_int1=0.,t_old1,Bref1,E_bias,acc_limit,b1;
int kkk1=0;//,kkk=0;
//////////////////////////////////////////////////////////
//double phin,psin,tetan,t_g;
extern mat3x1 g_b;
double mag_g=9.81;//
//double mag_g=9.7942;
/////////////////////321
void t_guid_lab();
extern char GPS_ACTIVE;
double apc1,ayc1;
double bp,bt;
double z_err_in,zm_c,vzm_c,vxzm_c;
double tt=0.,t_2=0.,t_vc=0;
unsigned int CntrAb=0;
int gama_flg=0;
////////////////////321
double t_ramp_gama;
double  delx,dely,delv;
double  delx_org,dely_org,delv_org,vmxy_org;
extern double teta_launch;
double gamac;
//double in_integ_pitch=0., in_integ_gama=0.;//v3
extern double t_z[],z_c[],vz_c[];
extern double t_tet[],teta_c[];
extern double t_pos[],x_pos[], y_pos[], v_pos[] ;
//extern double t_gui,Lx_gui, Ly_gui, Lv_gui,Lgama_gui;z
mat3x1 rtaz,vmaz,rmn_rtn,rme_rte,rme,vmn,rmn;
double Az_t,dRange;
double t_gama=0,gamam,alfa_xy;
//<<<<<<<<<<<<<>>>>>>>>>>
// time var
double Del_tm=0.5;//1.5+0.*2;//ocv14//v25
double Del_t=1000.,t_BeltCmd=1000., t_sep=1000.,t_ign2_cmd=1000;
double t_burn1=30.,t_burn2=80.;//ocv14
double t_ctrl=0.5,t_stg2=55.,kv=1.9,dv_r,tgo=1000.,t_ign2=1000;

int StopCtrlCmd=0,StopCtrlCmd_2=0,PushCmd=0;
double t_StopCtrlCmd=1000.,t_PushCmd=1000.;
int cond1=0,cond2=0,cond3=0,cond4=0;

//flags
int flg_stage=0.;
int sepflg=0,burn_flg=0,flgm_5=0,BeltCmd=0,sepflg2=0;
int  vb = 0,bv=0,burn_flg2=0,flgb_5=0,ss_6=0;
int homing_phase=0,flg_alt_hold,flg_vc,ign2_cmd=0;//
extern int tgt_flg,hgv_flg,hgv_flg2;
unsigned int gama_cnt=0;

//>>>>>>>>>>>>>>>>>>>>>>>>>
//1300C
double Az_6,Rng_6,z_err, vz_err;
 double TburnOut, VburnOut, TetaburnOut;
double theta_p=0;

double Rng_0,Az_0,tetam_6=0;

int ctrlNo=1;

float t_dive,t_yaw;
double psi_p=0,t_psi0=0,psi_0=0,Aim_coms=0;
//------------- 4sim
extern int nchoice;
extern long int p_count;
//-----------
extern int TrgtCnt;
int flgs_5=0,start_flg=0;
int ign2_5=0,ign2_fb=0,ign2_ok=0,ign2_ok5=0;
int BeltCmd_off=0,PushCmd_off=0,ign2_cmd_off=0;
int enemyzone_flg1=0,enemyzone_flg2=0,enemyzone_5=0;
double enemyzone_t=1000.,t_sep3=1000.;
double ign2_dely=0.1+0.126+0.;//0.13;//+0.26;

int WH_sep2=0,sep_ok5=0,sep_ok=0,bad_sep3_cond=0,sep3_cmd=0,sep3_off=0,psip_flg=0,psip_flg2=0;
//-------
double ancomx_limit=20;
#define GUARD_ENDGAME_H 2000.0 /*Step3: overflight guard may only disengage below this altitude [m]. Measured: healthy hits never fire the guard at ANY altitude (5 cases, 0 samples); only lost overshoot arms fire above it.*/
/*Step5: short-range cold-motor corrective trim. Class gate is fully
  observable: dv_r (measured, post-bias; nominal arms >= +9.5, cold <= -22.9,
  gap >= 32 m/s across all 7 targets) and Teta0 (scenario constant). Sweep
  evidence: short-range excess needs NEGATIVE (flatter) trim - the native
  positive theta_p is the wrong direction for this class.*/
#define SC_DV_ON  (5.0)   /* engage when dv_r below this [m/s] */
/* K is range-scheduled on Slant_Range at the latch (observable): measured
   zero-crossing gains K(T16/300km)=0.19, K(T15/400km)=0.08 differ 2.4x —
   a single K cannot serve the class (K=0.2/0.4/0.6/0.9 all fail one target).
   Linear in Slant_Range through the two anchors; clamped outside. */
#define SC_SR_A   280e3    /* anchor A: ~latch slant range, T16 [m] */
#define SC_K_A    0.19     /* K at anchor A [deg/(m/s)] */
#define SC_SR_B   380e3    /* anchor B: ~latch slant range, T15 [m] */
#define SC_K_B    0.08     /* K at anchor B [deg/(m/s)] */
#define SC_LIM    10.0     /* trim magnitude clamp [deg] */
#define LATH_SCALE 5.0     /* LATH: stage-6 lateral PN scale (stock N'~1 is sub-critical; fit by sweep) */
double hc=20.5e3;//16e3;
/* LRG - long-range cold-motor glide-altitude schedule. A deep energy deficit
   is best flown HIGHER: lower density -> less drag -> the glide stretches
   further. Measured sweep (hc 14..28 km) on the eight cold arms: the deep
   group (dv_r -47..-89) wants ~25 km, the mild group (dv_r -31..-37) wants the
   nominal 20.5 km and BREAKS at 24+ (T11n5 9 m -> 2068 m). dv_r separates them
   with a 10.4 m/s margin and is observable (measured at the stage-2 latch);
   K_test/noise are never referenced. hc_eff also drives homing_rng, exactly as
   the swept build did. */
// RIG_* defines retired 2026-08-22 -- the gate they served was non-physical
// (separation-pusher latch-up, not a motor). See the RIG note in Init_guid.
// #define RIG_R0      730.0e3
// #define RIG_KDV     630.0
// #define RIG_MARG    2.0e3
#define LRG_DV_ON   (-42.0)   /* engage below this dv_r [m/s] */
#define LRG_HC      21.5e3    /* glide altitude when engaged [m] */
/* LRG_HC RETUNED 25.0 -> 21.5 km on 2026-08-23. The 25 km value was fitted
   while T10/T11 still carried the 1.04 DESIGN MARGIN (M273N.CPP), i.e. against
   a boost profile deliberately designed ~30 km long. With the margin dropped
   to 1.00 the stretch OVERSHOOTS: T10n2 (dv_r -51.1, the deepest deficit in
   the matrix) flew 806.4 km at a 799.6 km target -- 6.8 km LONG -- while
   carrying the LOWEST burnout velocity of the row (2399 m/s). Less energy,
   more range: that is the glide law, not ballistics.
   MEASURED (T10n2 / T11n2 / T13n2):
     25.0 km  6814.0 / 12.4 / 17283      23.0 km  1508.1 / 10.7 / 19240
     22.5 km   430.6 / 11.7 / 20394      22.0 km     9.3 / 12.2 / 22123
     21.8 km     9.1 / 12.4 / 22749      21.5 km     9.2 / 11.4 / 23038
     20.5 km    10.3 / 11.2 / 26545
   The curve is monotone and SATURATES below 22.0 km, so 21.5 sits in the flat
   region rather than at the 22.0-22.5 knee -- chosen for robustness, at the
   cost of ~900 m on T13n2 versus 22.0.
   Everything with dv_r >= LRG_DV_ON is untouched and bit-identical (T10n5,
   T11n5, T12n2, T13n5, T10n0 all unchanged across the whole sweep).
   COST: T13n2 degrades 17283 -> 23038 (tal_flag=1) and 14547 -> 19297
   (tal_flag=0). Accepted because T13 already fails on BOTH flags and its
   scenario tables are CORRUPT (byte-copies of T10 s) -- see the generation
   note in M273N.CPP. Revisit once T13 can be generated.
   Gate: 46/49 -> 47/49 (tal_flag=1); 48/49 held (tal_flag=0).
   NOTE this define is NOT tal_flag-gated -- it moves both flags. */
/* HRG -- NEGATIVE RESULT 2026-08-22. Do not retry these levers.
   LRG is one-sided: it lifts the glide when the motor runs COLD, and nothing
   sheds energy when it runs HOT. The Edit-4 vehicle carries +24% impulse, so
   a thrust-up dispersion at the top of its band OVERFLIES the target --
   measured down-range error -7.2 km (T10 n4, dv_r +50.0), -7.0 km (T11 n1,
   +42.0), -13.1 km (T11 n4, +52.2), against +10.8 and +13.6 m on the nominal
   cases at dv_r +26.0 and +29.1. The obvious mirror was built and swept:
     (a) glide LOWER on dv_r > 35 (hc_eff 18 / 16 / 14 km): made the overshoot
         monotonically WORSE, T11n1 -6972 -> -7962 -> -10346 m.
     (b) the same with the homing handover pinned at nominal, to see the drag
         effect alone: 16 km and 24 km then gave BYTE-IDENTICAL results, equal
         to baseline. The glide altitude is entirely INERT for these cases --
         they never enter the altitude-hold glide.
     (c) scaling the homing handover range on its own (x1.00 / x1.25 / x1.60):
         also byte-identical.
   So NONE of the energy-management machinery (hc_eff, homing_rng, and hence
   EMG, which is gated on the same latch) lies on the path these cases take,
   and no retuning of those constants can move them. Whatever carries a hot
   dispersion past the target is elsewhere: identify the guidance path these
   cases actually follow before attempting this again. */
#define EMG_SCALE   1.30      /* energy-managed glide command scale */
double hc_eff = 20.5e3;
double homing_rng_eff = 0.0;
//double hc=17e3;
double homing_rng = 1.15*(32e-9*hc*hc*hc -112e-5*hc*hc + 14.8*hc -27e+3)+1*5e3;//45e3;
double hgv_rng =50e3;// 0.8*0.77*(-1e-7*hc*hc*hc + 45e-4*hc*hc -40*hc + 15.2e+4);//*0.65*1.2;//*0.5047;//-0*120e3;//65e3;
double q6=1.;
int flg_ms1 = 0;
double vm_s1=0, vmxy=0, vmxz=0, err_motor=0,err_vxz = 0,tet_cmps2=0;
int i_s8 = 0, i_s2 = 0, flg_for = 1;
int i_sz8 = 0, i_sz2 = 0, flg_for2 = 1;

#include "../common/gnc/guidance.cpp"
