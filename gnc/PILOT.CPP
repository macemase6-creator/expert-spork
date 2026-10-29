

//-----------------
#include"stdafx.h"
#include "m273n.h"
#include "aplt.h"


#include "guidance.h"

#include "K_ACC9.h"
#include "K_ACC61.h"
#include "K_ACC7.h"
#include "ACC_V8.h"
//#include "PSI_V51.H"
#include "PSI_V52.H"
#include "PSI_V6.H"
#include "PSI_V7.H"

#include "HGV_V1.H"
#include "qCond_V4.H"
//#include <cmath>
//#include <algorithm>

//#include "PSI_V52.H"
//#include "PSI_V6.H"

//-------------------------
extern int ign2_cmd;
int const H_n = 11;//3.;//2.;//11;
int const q_n = 11;
int const M_n = 13;
double q_table[q_n] ={ 0.3,7.,14.,21.,28.,35.,42.,49.,56.,63.,70} ;
double H_table[H_n] ={ 0.0, 5000.0, 10000.0, 15000.0, 20000.0, 25000.0, 30000.0, 35000.0, 40000.0,45000.0, 50000.0 } ;
double Mach_table[M_n] = { 0.5, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 15.0 };
unsigned int NUM_H_Mach[2]= { H_n-1, M_n-1 };
/////////////////////////////////////////
static const int N_QQ   = 13;
static const int N_MACH = 8;

static const double QQ_AXIS[N_QQ] = {
    1.0000, 2.0000, 3.5000, 5.0000, 7.0000, 10.0000, 15.0000, 20.0000, 30.0000, 40.0000, 55.0000, 70.0000, 80.0000
};

static const double MACH_AXIS[N_MACH] = {
    0.7000, 1.0000, 1.5000, 2.0000, 3.0000, 5.0000, 7.0000, 11.0000
};

struct GainSet {
    double K_p_integ_accI;
    double K_i_integ_accI;
    double K_w_pitchI;
    double Ki_w_pitchI;
};
static GainSet get_gains(double qq, double mach1);
/////////////////////////////////////////
int stride =H_n;
double look2_binlxpw(double _hm, double _Mach,double H_tab[],double Mach_tab[],
  float  table[],unsigned int NU_H_M[],unsigned int stride);
double gains_interpols(double x,int size,float  qMat[],float  gainsMat[]);
  
float interpol_MH(double m, double a, float mx[11][13]);
//-----------------------------
double finlim1;
int cmdFed;
void  SendFrame(unsigned int [],int);
//void SendFrame(unsigned int *,int);
//int  de2di(double t);

int SDmode1=0,SDmode2=0,SDmode3=0,SDmode4=0,SDmode5=0,TVC_sf=0;

extern float vOut[6];
extern unsigned int Dig[6];
extern unsigned int Dig2[6];
extern mat3x1 Aff;
double delec0, delrc0;
double x1_old=0., z1_old=0., y1_old=0.,t_stv0=0.;
double out_integ_pitch, out_integ_gama,integ_limit,out_integ_psi;  //sabzikar 90/5/4
double out_integ_roll, out_integ_yaw;
double out_integ_wz,out_integ_wy;
double du_comp_old,du_comp;

double Teta_sat,Teta_sat_033, Gama_sat,Psi_sat;
/* washout rate-feedback state -- see TAL_WOK in sim_fio.h. Reset per run in
   the AutoPilot pre-launch block, alongside the other integrator resets. */
double wo_wz_lp = 0., wo_wz_bp = 0., wo_t_prev = -1.;
double pl_u_prev = 0., pl_y_prev = 0., pl_t_prev = -1.;
/* TAL_IYTAU0 burnout yaw-trim release clock (flag-0). Reset per run in the
   AutoPilot pre-launch block, alongside the other integrator resets. */
double iy0_t_prev = -1.;
/* TAL_IYK0 one-shot trim knock-down latch (flag-0). Reset per run. */
int iy0_done = 0;
double pl_lp = 0.;
double Roll_sat, Acc_sat,delec_old;
double D_e1,D_a1;
double delex,delax,delrx,sat_err_a,sat_err_e,sat_err_r;
double K_p_integ_acc, K_i_integ_acc,K_p_integ_acc_p, K_i_integ_acc_p;
double K_p_integ_roll, K_i_integ_roll,K_p_integ_teta,K_p_integ_teta1,K_i_integ_teta;
double K_p_g, K_i_g,K_w_pitch,Ki_w_pitch,K_w_pitch1, K_w_yaw, K_w_roll, K_w_acc;

unsigned int t_state,t_stage;
int SelfDestructionMode=0;
double az_b, ay_b;
double q_old=0.,q_new = 0.,q_max=0.,q_max1=0,q_max2=0;
 int q_cunter=0,q_cunter1=0,q_cunter2=0,flg_q2=0;
extern double du_compensate1 , wy_acc_compensate;
double alfa_dot,dphi;
int pik_q=0,flg_q=0;
long  on_cntr=0;
/////////////////////////////////////////////////////////////////////////////
double K_w_pitch00,K_p_integ_teta00,K_i_integ_teta00,K_w_yaw00,K_i_integ_acc00;
double K_p_integ_acc00,K_w_roll00,K_p_integ_roll00,K_i_integ_roll00,t_q_p00;
double K_w_pitch11,K_i_integ_teta11,K_p_integ_teta11,K_w_yaw11,K_i_integ_acc11;
double K_p_integ_acc11,K_w_roll11,K_p_integ_roll11,K_i_integ_roll11,k_ramp;
double K_w_pitch22,K_i_integ_teta22,K_p_integ_teta22,K_p_integ_teta0,K_w_yaw22,K_w_yaw33;
double K_i_integ_acc22,K_p_integ_acc22,K_w_roll22,K_p_integ_roll22,K_i_integ_roll22;
double t_q_p1,t_q_p0,K_w_pitch33,K_p_integ_teta2,K_i_integ_acc33,K_p_integ_acc33,K_w_roll33;
double K_p_integ_roll33,K_i_integ_roll33,t_q_p2,K_w_pitch2,K_w_roll44,K_p_integ_roll44;
double K_i_integ_roll44,K_w_pitch44,t_q_p3,t_q_p4,K_w_roll55,K_p_integ_roll55;
double K_i_integ_roll55,K_w_pitch55,t_q_p5,acc_sat;
double K_w_psi00,K_p_integ_psi00,K_i_integ_psi00;
double K_w_psi22,K_p_integ_psi22,K_i_integ_psi22;
double K_w_psi11,K_p_integ_psi11,K_i_integ_psi11;
//////////////////////////

double du_compensate1, du_compensate2, wx_compensate, wy_compensate,
    wz_compensate, wz_compensate1,wy_compensate1, wz_compensate2;
 double apc_limit,ayc_limit,VzRespP=0,IntgVzRespP=0,AbsVzRespP=0;
////////////
//1300C
extern double t_ctrl;
double mach330;
int oi_flg=0;
double out_integ_w_limit,du_pitch_limit,out_integ_limit,in_integ_pitch_limit,out_integ_yaw_limit;
//float k_w_acc2[11][13],ki_w_acc2[11][13],k_i_acc2[11][13];
int flag_alfa=0;
double out_integ_psi_limit,out_integ_wy_limit,in_integ_psi_limit,du_psi_limit;
int first_stage_8 = 0, tvc_flg = 1;
double euler_d[3] = {10, 70., 0.};
double lambda_;
int flg_s8 = 0;
int flg_sign_s8 = 0;
double psi_command, teta_command;
double t_s8;
double rng_tvc, qold, defq = 0, del_tvc = 0;
double a_tab[3], tab_b[3];
double du_gama_lmt=0.2,out_integ_gama_lmt=0.4;
double K_w_psi,K_p_integ_psi,K_i_integ_psi;

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
//   This routine calculates the dynamic pressure.

#include "../common/gnc/pilot.cpp"
