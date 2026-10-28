/* Exact platform declarations from the immutable pre-merge headers. */
#include "gnc_platform.h"

#if GNC_IS_SIM
/*
#include "nav.h"
#include "navinit.h"
#include "main.h"
#include "fusion.h"//*/
#include "m273n.h"
/// Guidance Variable
//extern double phin,psin,tetan;
extern int SDmode1,SDmode2,SDmode3,SDmode4,SDmode5;
extern int q_cunter,q_cunter1,flg_q,q_cunter2,flg_q2;
extern double delx,dely,delv;
extern double  delx_org,dely_org,delv_org,vmxy_org;
extern float  dac[4];     //0 degree
extern double D_e1,D_a1,out_integ_roll;
extern double out_integ_pitch,out_integ_yaw,out_integ_gama,out_integ_wz,out_integ_wy,out_integ_psi;
extern double delax, delrx, delex;
extern double delec, delrc, delac;
extern double delec0, delrc0;
extern double alfa_xy, gamac;//change:extern shavad
extern double Acc_sat, Vm_t, Vm_t_2;
extern double Vxy,Vxm,Hme,ayc,ac_pitch,q,Gx_B,Vym,Vzm,Vxz;
extern double K_i_integ_acc_p,K_i_integ_acc,K_p_integ_acc;
extern double du_roll, K_i_integ_roll, K_p_integ_roll;
extern double K_i_integ_teta, K_i_g;
extern int    stage,s_s, loopflag,Loop_Test_Flg,SelfDestructionMode;
// extern double Slant_Range;
extern double Gama_corec, tetac, Wz, Wx_filter,Wy_filter, Wz_filter;
extern double Xtrgt, Ytrgt,Ztrgt;
extern double apc_limit, ayc_limit;
extern double Teta_sat, Teta_sat_033,Roll_sat;

// Pilot Variables
extern double du_pitch, du_gama, du_yaw, du_roll,t_g;
extern double in_integ_pitch, in_integ_gama;
extern double Ayb_filter, Azb_filter, beta;
 extern double mag_g;

//Seeker Guidance
extern int T_Flag_X,T_Flag_Z,TrcOkflg,cmdFed;
extern char WarHeadReady;
extern double bp,bt;
extern float tetas_Err,psis_Err;
extern double tetas_out,psis_out;
extern unsigned char Pan_SNR;
extern unsigned char Tilt_SNR;
extern unsigned int gotoSW,gama_cnt;
extern int BufferEmpty[4];
extern int F_Flag;
extern int gama_flg,pik_q;
extern float tetas_estimate,psis_estimate;
extern float tetas,psis;
//>>>>>>>>>>
extern double Del_tm, Del_t,gamam;
void NED_guide();
void Guidance();
void PilotGain();
void AutoPilot(char mod);
void Outputs();
void mib_mask();
double ramp(double t0,double tf,double t,double ramp);
unsigned char EightInone(unsigned char flg1,unsigned char flg2,unsigned char flg3,unsigned char flg4,unsigned char flg5,unsigned char flg6,unsigned char flg7,unsigned char flg8);//amr
//--------------
int Check_targets();
void Sep_func();
void Init_guid();
extern int ctrlNo,ign2_cmd,start_flg;
extern double mag_g,t_burn1,t_burn2,tt,t_2,t_vc,tetam_6,ancomx_limit;
int sign(double sg);
double limit(double var , double max_val , double min_val);
double max(double value1, double value2);
///
extern int StopCtrlCmd,PushCmd,ign2_ok,ign2_ok5;
extern int enemyzone_flg1,enemyzone_flg2,enemyzone_5;
extern double t_StopCtrlCmd,t_PushCmd,t_BeltCmd,enemyzone_t;
extern int WH_sep,sep_ok5,sep_ok,bad_sep3_cond,sep3_cmd,sep3_off;

extern int flg_ms1 ;
extern double vm_s1, vmxy, vmxz, err_motor;
extern int i_s8 , i_s2 , flg_for ;
extern int i_sz8 , i_sz2 , flg_for2 ;
extern double err_vxz ;
#else
/// Guidance Variable
extern double phin,psin,tetan;
extern int SDmode1,SDmode2,SDmode3,SDmode4,SDmode5;
extern int q_cunter,q_cunter1,flg_q,q_cunter2,flg_q2;
extern double delx,dely,delv;
extern double  delx_org,dely_org,delv_org,vmxy_org;
extern float  dac[4];     //0 degree
extern double D_e1,D_a1,out_integ_roll;
extern double out_integ_pitch,out_integ_yaw,out_integ_gama,out_integ_wz,out_integ_wy,out_integ_psi;
extern double delax, delrx, delex;
extern double delec, delrc, delac;
extern double delec0, delrc0;
extern double alfa_xy, gamac;//change:extern shavad
extern double Acc_sat, Vm_t, Vm_t_2;
extern double Vxy,Vxm,Hme,ayc,ac_pitch,q,Gx_B,Vym,Vzm,Vxz,apc;
extern double K_i_integ_acc_p,K_i_integ_acc,K_p_integ_acc;
extern double du_roll, K_i_integ_roll, K_p_integ_roll;
extern double K_i_integ_teta, K_i_g;
extern int    stage,s_s, loopflag,Loop_Test_Flg,SelfDestructionMode,psic_flg;
// extern double Slant_Range;
extern double Gama_corec, tetac, Wz, Wx_filter,Wy_filter, Wz_filter,psic;
extern double Xtrgt, Ytrgt,Ztrgt;
extern double apc_limit, ayc_limit;
extern double Teta_sat, Teta_sat_033,Roll_sat;
extern double Vx_trgt,Vy_trgt,Vz_trgt;

// Pilot Variables
extern double du_pitch, du_gama, du_yaw, du_roll,t_g;
extern double in_integ_pitch, in_integ_gama;
extern double Ayb_filter, Azb_filter, beta;
 extern double mag_g;

//Seeker Guidance
extern int T_Flag_X,T_Flag_Z,TrcOkflg,cmdFed;
extern char WarHeadReady;
extern double bp,bt;
extern float tetas_Err,psis_Err;
extern double tetas_out,psis_out;
extern unsigned char Pan_SNR;
extern unsigned char Tilt_SNR;
extern unsigned int gotoSW,gama_cnt;
extern int BufferEmpty[4];
extern int F_Flag;
extern int gama_flg,pik_q;
extern float tetas_estimate,psis_estimate;
extern float tetas,psis;
//>>>>>>>>>>
extern double Del_tm, Del_t,gamam;
void NED_guide();
void Guidance();
void PilotGain();
void AutoPilot(char mod);
void Outputs();
void mib_mask();
double ramp(double t0,double tf,double t,double ramp);
unsigned char EightInone(unsigned char flg1,unsigned char flg2,unsigned char flg3,unsigned char flg4,unsigned char flg5,unsigned char flg6,unsigned char flg7,unsigned char flg8);//amr
//--------------
int Check_targets();
void Sep_func();
void Init_guid();
extern int ctrlNo,ign2_cmd,start_flg,multi_trgts;
extern double mag_g,t_burn1,t_burn2,tt,t_2,t_vc,ancomx_limit;
int sign(double sg);
double limit(double var , double max_val , double min_val);
double max(double value1, double value2);
///
extern int StopCtrlCmd,PushCmd,ign2_ok,ign2_ok5;
extern int enemyzone_flg1,enemyzone_flg2,enemyzone_5;
extern double t_StopCtrlCmd,t_PushCmd,t_BeltCmd,enemyzone_t;
extern int WH_sep,sep_ok5,sep_ok,bad_sep3_cond,sep3_cmd,sep3_off,sepflg2;
extern int guid_flg,reentry_ign,ign2_stg;
#endif
