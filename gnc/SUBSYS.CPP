#include"stdafx.h"
#include "m273n.h"
#include<stdio.h>
#include "aplt.h"
#include "guidance.h"
extern double psi_command, teta_command;
extern double STHR[], TSTHR[], Total_Imp, angle_tvc_jad, del_tvc,Total_Imp_1,tgo_norm;
extern int dtt, freq;
double ay_new, az_new,ay_old, az_old;
extern double a_tab[3], tab_b[3];
extern double out_integ_wz;
extern double b_int1, Bref1,b1;
extern double psic_yaw,vmxy,vmxz;
extern double tet_cmps2, Aim_coms,forcc,ren;
extern double ch_df[], cb_df[], hing_mom[], bend_mom[];
extern double del_off[4];
double t_ay1, t_ay2, t_az1, t_az2, t_q1, t_q2, ay_m1, ay_m2, az_m1, az_m2,q_m1, q_m2;
double t_ax1, t_ax2, t_qa1, t_qa2, ax_m1, ax_m2, qa_m1, qa_m2, t_SX, t_SZ;
double wx_max, wy_max, wz_max;
float dac[4];
extern int tvc_flg ;
int flgto=0;

/*
// ashraf_220724
//double m_e_motor2=167.,m_fuel2=210., m_fuel = 5011.,m_liner = 45.;
double m_e_motor2=1*73.,m_fuel2=1*210., m_fuel = 5011.,m_liner = 55.;//45.;
double m_e_stg2 = 535 +m_e_motor2,m_f_stg2 =m_fuel2 + m_e_stg2;
double m_e_stg1 = 783.+m_f_stg2 + 0. ,m_f_stg1 = m_e_stg1+m_fuel+m_liner;

double ix1_1 = 554.3, ix2_1 = 166.1, ix1_2 = 51.33, ix2_2 = 42.5;
double iy1_1 = 45108.3, iy2_1 = 20642, iy1_2 = 566.8, iy2_2 = 525.3;
//double xcg1_1 = 6.407, xcg2_1 = 5.042, xcg1_2 = 2.154, xcg2_2 = 2.034+0.*50./1e3;//
double xcg1_1 = 6.407, xcg2_1 = 5.042, xcg1_2 = 2.320, xcg2_2 = 2.15;//*/

/*
// ashraf_280724
//double m_e_motor2=167.,m_fuel2=210., m_fuel = 5011.,m_liner = 45.;
double m_e_motor2=1*73.,m_fuel2=208.3, m_fuel = 5011.,m_liner = 55.;//45.;
double m_e_stg2 = 512.6 +m_e_motor2,m_f_stg2 =m_fuel2 + m_e_stg2;
double m_e_stg1 = 783.+m_f_stg2 + 0. ,m_f_stg1 = m_e_stg1+m_fuel+m_liner;//*/
/*
// ashraf_280724
//double m_e_motor2=167.,m_fuel2=210., m_fuel = 5011.,m_liner = 45.;
double m_e_motor2=1*73.,m_fuel2=210.6, m_fuel = 5016.63,m_liner = 55.;//45.;
double m_f_stg2 = 808.,m_e_stg2 =m_f_stg2-m_fuel2;//m_fuel2 + m_e_stg2;
double m_f_stg1 = 6674. ,m_e_stg1 = m_f_stg1-m_fuel-m_liner;//m_e_stg1+m_fuel+m_liner;//

double ix1_1 = 554.3, ix2_1 = 166.1, ix1_2 = 50.9, ix2_2 = 42.3;
double iy1_1 = 44351.4, iy2_1 = 20183., iy1_2 = 509., iy2_2 = 472.7;
double xcg1_1 = 6.425, xcg2_1 = 5.103, xcg1_2 = 2.202, xcg2_2 = 2.098-0*(50/1e3);//
//double xcg1_1 = 6.407, xcg2_1 = 5.042, xcg1_2 = 2.320, xcg2_2 = 2.15;//*/
//----------------------
/*
// t9-211124-
double m_fuel2=209.8, m_fuel = 4999.+0.,m_liner = 55.;//45.;
double m_f_stg2 = 817.0,m_e_stg2 =m_f_stg2-m_fuel2;//m_fuel2 + m_e_stg2;
double m_f_stg1 = 6686.-0. ,m_e_stg1 = m_f_stg1-m_fuel-m_liner;//m_e_stg1+m_fuel+m_liner;//

double ix1_1 = 554.3, ix2_1 = 166.1, ix1_2 = 50.9, ix2_2 = 42.3;
double iy1_1 = 44351.4, iy2_1 = 20183., iy1_2 = 509., iy2_2 = 472.7;
double xcg1_1 = 6.425, xcg2_1 = 5.103, xcg1_2 = 2.202, xcg2_2 = 2.098-0*(50/1e3);//
//double xcg1_1 = 6.407, xcg2_1 = 5.042, xcg1_2 = 2.320, xcg2_2 = 2.15;//*/
//----------------------
/*
// t13
double m_fuel2=209.9, m_fuel = 5004.+0.,m_liner = 55.;//45.;
double m_f_stg2 = 795.0,m_e_stg2 =m_f_stg2-m_fuel2;//m_fuel2 + m_e_stg2;
double m_f_stg1 = 6653.-0. ,m_e_stg1 = m_f_stg1-m_fuel-m_liner;//m_e_stg1+m_fuel+m_liner;

double ix1_1 = 554.3, ix2_1 = 166.1, ix1_2 = 50.9, ix2_2 = 42.3;
double iy1_1 = 44351.4, iy2_1 = 20183., iy1_2 = 509., iy2_2 = 472.7;
double xcg1_1 = 6.425, xcg2_1 = 5.103, xcg1_2 = 2.202, xcg2_2 = 2.098-0*(50/1e3);//
//double xcg1_1 = 6.407, xcg2_1 = 5.042, xcg1_2 = 2.320, xcg2_2 = 2.15;//*/
//----------------------
//
// double m_fuel2=100., m_fuel = 2003.8,m_liner = 25.;//45.;
// double m_f_stg2 = 358.,m_e_stg2 =m_f_stg2-m_fuel2;//m_fuel2 + m_e_stg2;
// double m_f_stg1 = 2693 ,m_e_stg1 = m_f_stg1-m_fuel-m_line

// double ix1_1 = 119.97, ix2_1 = 37.25, ix1_2 = 11.02, ix2_2 = 9.1;
// double iy1_1 = 8027.7, iy2_1 = 3875.6, iy1_2 = 106.24, iy2_2 = 97.2;
// double xcg1_1 = 4.307, xcg2_1 = 3.37756, xcg1_2 = 1.51265, xcg2_2 = 1.43;//

//double m_fuel2=100., m_fuel = 1963,m_liner = 25.;//45.;
//double m_f_stg2 = 366.,m_e_stg2 =m_f_stg2-m_fuel2;//m_fuel2 + m_e_stg2;
//double m_f_stg1 = 2678,m_e_stg1 = m_f_stg1-m_fuel-m_liner;//m_e_stg1+m_fuel+m_liner;

/* M480 mass properties from the detailed mass PDFs (not the rounded TXT
   summary): full 891.40 kg, empty 164.48 kg, propellant 726.92 kg. */
double m_fuel2=0.0, m_fuel = 726.92,m_liner = 0.0;
double m_f_stg2 = 107.745184-10,m_e_stg2 =m_f_stg2-m_fuel2;//m_fuel2 + m_e_stg2;
double m_f_stg1 = 891.40,m_e_stg1 = 164.48;

double ix1_1 = 24.99, ix2_1 = 5.33, ix1_2 = 1.833941, ix2_2 = 1.833941;
double iy1_1 =800.16, iy2_1 = 380.17, iy1_2 = 13.866809, iy2_2 = 13.866809;
double xcg1_1 = 2.56, xcg2_1 = 2.14, xcg1_2 = 1.026, xcg2_2 = 1.026;////
//*/

//double xcg1_1 = 6.407, xcg2_1 = 5.042, xcg1_2 = 2.320, xcg2_2 = 2.15;//*/
/*
double m_fuel2=0.0, m_fuel = 408+0,m_liner = 8.858;//45.;
double m_f_stg2 = 113.745184,m_e_stg2 =m_f_stg2-m_fuel2;//m_fuel2 + m_e_stg2;
double m_f_stg1 =  637.525450.+3.,m_e_stg1 = m_f_stg1-m_fuel-m_liner;//m_e_stg1+m_fuel+m_liner;

double ix1_1 = 13.434399, ix2_1 = 5.450438, ix1_2 = 1.833941, ix2_2 = 1.833941;
double iy1_1 =538.321029, iy2_1 = 311.273479, iy1_2 = 13.866809, iy2_2 = 13.866809;
double xcg1_1 = 2.456, xcg2_1 = 1.995907, xcg1_2 = 1.026, xcg2_2 = 1.026;////
//double xcg1_1 = 6.407, xcg2_1 = 5.042, xcg1_2 = 2.320, xcg2_2 = 2.15;//*/
//*/
/* M480 is the physical stage-1 model for both legacy TalFlag paths.  Keeping
   the assignment here prevents a previous run in the same process from
   leaking different mass properties into the next run. */
void TalApplyMass(void)
{
   m_fuel   = 726.92;  m_liner  =   0.0;
   m_f_stg1 = 891.40;  m_e_stg1 = 164.48;
   xcg1_1   =   2.56;  xcg2_1   =   2.14;
   ix1_1    =  24.99;  ix2_1    =   5.33;
   iy1_1    = 800.16;  iy2_1    = 380.17;
}

void TalApplyImpulse(void)
{
   /* M480_STHR_S1.dat already carries the integrated delivered impulse. */
   Total_Imp_1 = Total_Imp;
}
//----------------------
//#############################################################
extern FILE *sl_1; // ,*popof;
double sl, h_sl, sl_rx, sl_vm, angle_tvc_sim;
//int vb = 0,bv=0;
int motor_2_flg = 0;

int isthr_2 = 1000;
double bv_t;
double ftb0, ftb1, ftb2, mtb0, mtb1, mtb2, ftb0_2, ftb1_2, ftb2_2, mtb0_2,mtb1_2, mtb2_2;
double tberr_2 = 0., threrr_2 = 0., sthr_2, sthr_20, thr_2;
double eth1_2, eth2_2, Yt_2, Zt_2, Xt_2 = 0.25, xbc_y_2 = 0., xbc_z_2 = 0.;
//double Del_tm; // separation
//double Del_t; // starting wahab

extern double t_burn_2;
extern int sepflg, FTHR, pik_q, motor_flag, Aim;
double swx = 0., kkk;
extern double du_compensate2, du_compensate1, wz_compensate, q_max, q_new,K_w_pitch1;
double Lgps, lgps, hgps, vngps, vegps, vdgps, Re[3], me2n_[3][3], vme[3], vmn_[3];
double Ce2n[3][3]; // ,a_ecef_old[3],jerk_[3],am_,fb_old[3];

double STHR_max; // =31000./9.81;
//int flag_alfa = 0;
double  k_5; // 70
int dley_servo;
 //double wn = 128.23,zeta = 0.9 , speed_servo = 70.;//as_plstin
//
//double wn = 75.4,zeta = 0.9947 , speed_servo =75.;//zuhayr
//double wn = 27.14,zeta = 0.6617 , speed_servo =25.;//amp5
//double wn = 70.,zeta = 0.6617 , speed_servo =75.;//amp5_mybe_thr_rhit_values
//double wn = 21.47,zeta = 0.9625 , speed_servo =25.;//amp10
//double wn = 253.85,zeta = 0.2698 , speed_servo =12.;//amp1
double wn=67.297, zeta=0.8628,speed_servo=75;
double wn_t = 95,zeta_t = 0.7,speed_servo_t = 14.5;

extern int rocket;

extern int flg_q;
double tsep = 1000.;
double v_ecef[3], a_ecef[3], v_ned[3], a_ned[3];
extern double teta_l, out_integ_gama, K_p_g;
double d_ecef[3], d_fire[3], xtrgt, ytrgt, ztrgt;
double a_j, e_j, N_jm, N_jh, ecef_jm[3], ecef_jh[3], vmf_t[3], v_trgt[3];
double Az_j0, Az_j, d_Az_j, Range_j, Range_j0, head;
double fi, teta, si;
double Cf2e[3][3], Cf2t0[3][3];
double coef, am, ayz;
double dx_tr0, dy_tr0, rxy20, vxy0, qe_dot0;
extern double d_j, qe_dot, xbc_y, xbc_z;
extern double M_sfuel_Fire, t_burn_base, Isp_Fire, Total_Impulse, Total_Imp;
extern double Temp_static, TotalImpulse0, jerm;
double Ktemp_thr0, Km_thr0;
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
double vm_dot_new,LOS_ang[2];
extern double l_rail, x_laun,alfa_xy;
// int s_s;
double Cn2t[3][3];
double m1, m2, m3;
double ix1, ix2, ix3;
double iy1, iy2, iy3;
double xcg1, xcg2, xcg3;


extern int launch2, launch_mod;
extern double Teta_sat, vm_dot, l_launcher, delta_y_l, tipoff, zm_c, vzm_c,vxzm_c;
extern double cmde, cnde, ren, m_old, h_old, xbc_er, xbc_w, xbc_m, z_err,vz_err;
void Geo_Nav(double xx[9], double uu[6]);
//int isthr = 403.; // 307.;//449.;//390.;//411.;//389;

 int isthr = 1000.; // 307.;//449.;//390.;//411.;//389;
double axm_1, aym_1, azm_1, wxm_1, wym_1, wzm_1, wibb0[3];
extern int mk,pak_count, fr, f_100;
double G_ab[3], Ge_ab[3], Gf_ab[3], Gi_ab[3];
double jerk1, jerk2, jerk3;
int jav_count, jav_count2,f_tele=0;
double wwR[3], g[3], klm, Slant_Range,Slant_Range0;
double La, lo, h, vn, ve, vd, ffn[3], ffi[3];
double L2, ll2, l2, h2, vn2, ve2, vd2, an2, ae2, ad2;
double fb[3], fbb[3], fbn[3], wRR[3];
double Rfi[3],fngi[3], accb[3], acci[3], uu[6], xx[9], rf[3];
double Lmf, lmf,axi, ayi, azi, thr, wv, waz;
double sthr, rmfeb, mdot;
extern double TbStatic, cnqa, delec0, wv0, waz0,TbS_2;
extern double phi_dot, pak, qaz;
extern double out_integ_yaw, out_integ_roll, out_integ_pitch;
double stk1[12], stk2[12], stk3[12], stk4[12], stk5[12], stk6[12];
extern double K_i_integ_roll, K_p_integ_roll, K_w_roll, K_p_integ_acc,
    K_i_integ_acc, K_i_integ_teta, K_p_integ_teta, K_w_pitch, Ki_w_pitch,
    K_p_g, K_i_g,K_w_yaw;
	
extern double out_integ_psi_limit,out_integ_wy_limit,in_integ_psi_limit,du_psi_limit;
extern double K_w_psi,K_p_integ_psi,K_i_integ_psi;
///////////////////
double euler_f[3],euler_e[3],euler_n[3];
double erre[3],rmn_[3], Xcp,Xcg,xer, zer,SM,xbc_er0=0,rxz,rxy;
mat3x1 rmf_rtf,vmf_vtf,errm;
extern double delrc0;
int cond;
extern int cond2;

extern double Total_Imp2,Total_Imp_2;
extern int bdr_flg;
extern double sth_bdr;

int gama_cnt0=0,gama_flg0=0,tgo_cnt=0,tgo_flg=0;
double t_gama0=45.;
 double FAz,FRange,teta_xy,tgo_old=0;//,tgo=1000.;
 double MinMaxV[2];//,teta_yz[100];
 int MinMaxId[2];
extern double delrc0;
//////////////////////////////////
double RRR,tele_thr,tff; // total flight path (meter)
double angle_vgg, RR_m;
extern double gamma_path, Vg_lambert;
double accf[3], psimf, tetamf, phimf;
double _mf2b[3][3],rtf0[3],Range2;
//////////////////////
//extern int k;
//extern double T_togo;
//extern double gama_com;
double dot_r1r2 = 10000.;
//extern double phi_dist;
int   gama_flag = 0,sep_fb=0;

void Geo_Nav(double xx[9], double uu[6]);
void llhxyz(double lat, double lon, double h, double r[3]);
double fngb0,tt_tele0=0.,GGdt;
double t_bo,vm_bo,tetam_bo,vz_bo;

/* M480 nozzle data supplied with the stage-1 package. */
double Ae = 0.070686, Pe = 74700.0;
//if (thr_flg==2) Ae = 0.3957, Pe = 86500.;//IR
//double Ae2 = 0.424*0.424*pi/4, Pe2 = 74680.;
double Ae2 = 0.0*0.0*pi/4, Pe2 = 74680.;

int motor1_state=0,motor2_state=0,f_belt_flg=0;//fcu
	
void f6dm273na(void)
{
   double iyx, mx2;
   /* Calculate the components of position and velocity of missile in 'Zi' frame. */

   dy[0] = y[3];
   dy[1] = y[4];
   dy[2] = y[5];
   dy[3] = Gi[0] + fngi[0] / mass;
   dy[4] = Gi[1] + fngi[1] / mass;
   dy[5] = Gi[2] + fngi[2] / mass;
   /* Calculate the components of acceleration  coordinatized in 'Zb' frame. */
   axi = fngi[0] / mass / magg;
   ayi = fngi[1] / mass / magg;
   azi = fngi[2] / mass / magg;
   ai[0] = dy[3] / magg;
   ai[1] = dy[4] / magg;
   ai[2] = dy[5] / magg;
   twm(2, mb2i, ai, ab);
   /* Calculate the components of the angular velocity of the missile	in 'Zc' frame . */
   iyx = iyy - ixx;
   mx2 = 0.; // mass*xbc*xbc;


   dy[6] = (mab[0] + mtb[0]) / ixx; // normal
   dy[7] = (mab[1] + mtb[1] + y[6] * y[8] * iyx + xbc * fngb[2] - mx2 * y[6] * y[8]) / (iyy - mx2);
   dy[8] = (mab[2] + mtb[2] - y[6] * y[7] * iyx - xbc * fngb[1] + mx2 * y[6] * y[7]) / (iyy - mx2);

   // if(s_s>3 && s_s<6) y[8]=-.3*d2r;
   //if (l_launcher > (l_rail - 4.7) && l_launcher < l_rail && launch_mod == 1)
       if (l_launcher > (l_rail -3.5) && l_launcher < l_rail && launch_mod == 1)
      dy[8] = (mab[2] + mtb[2] - y[6] * y[7] * iyx - xbc * fngb[1] + (mass * x_laun * x_laun) * y[6] * y[7]) / (iyy + mass * x_laun * x_laun);

/*
   kkk = 0.;

   y[6] = y[6] + kkk;
   y[7] = y[7] + kkk;
   y[8] = y[8] + kkk;
   // */

   
	if(f_100==1 && (t<=tt_tele))
	{		
		//if(Ab(0,0)< 20.0 && t<5.)		{f_tele=0;tt_tele0=t;}
		//else 							
			f_tele=1;

	}else 	f_tele=0;
	
  if(f_tele==1  )//&& (t<=10))
  {
	//y[6]=tele_sns_interpol(5,t,tele_snsr)*d2r;
	//y[7]=tele_sns_interpol(6,t,tele_snsr)*d2r;	
	//y[8]=tele_sns_interpol(7,t,tele_snsr)*d2r;
  }
  //if (t<tt_tele)y[6]=tele_sns_interpol(5,t,tele_snsr)*d2r;//*/
  
   dy[9] = -0.5 * (y[10] * y[6] + y[11] * y[7] + y[12] * y[8]);
   dy[10] = 0.5 * (y[9] * y[6] + y[11] * y[8] - y[12] * y[7]);
   dy[11] = 0.5 * (y[9] * y[7] + y[12] * y[6] - y[10] * y[8]);
   dy[12] = 0.5 * (y[9] * y[8] + y[10] * y[7] - y[11] * y[6]);

   // if (stage == 4 && t > 200. && t < 210.)
   // y[8] = 10. * d2r;
   wx = (y[6] + gbx) * (1.0 + gsx);
   wy = (y[7] + gby) * (1.0 + gsy);
   wz = (y[8] + gbz) * (1.0 + gsz);
   
   if (fabs(wx) > 450.*d2r)	wx = 450.*d2r*sign(wx);
   if (fabs(wy) > 450.*d2r)	wy = 450.*d2r*sign(wy);
   if (fabs(wz) > 450.*d2r)	wz = 450.*d2r*sign(wz);//*/
   
   //Wb=Arr2Mat(y+6);
   Wb(0,0)=wx;
   Wb(1,0)=wy;
   Wb(2,0)=wz;

   wib[0] = y[6];
   wib[1] = y[7];
   wib[2] = y[8];
   fb[0] = fngb[0] / mass;
   fb[1] = fngb[1] / mass;
   fb[2] = fngb[2] / mass;
   am = mag(3, fb) / magg;
   ayz = (sqrt(fb[1] * fb[1] + fb[2] * fb[2])) / magg;

   dy[20] = -0.5 * (y[21] * wx + y[22] * wy + y[23] * wz);
   dy[21] = 0.5 * (y[20] * wx + y[22] * wz - y[23] * wy);
   dy[22] = 0.5 * (y[20] * wy + y[23] * wx - y[21] * wz);
   dy[23] = 0.5 * (y[20] * wz + y[21] * wy - y[22] * wx);
   // */
   /* Calculate the components of position and velocity of missile in 'Zi' frame. */
   dy[13] = y[16];
   dy[14] = y[17];
   dy[15] = y[18];
   dy[16] = Gi[0] + acci[0];
   dy[17] = Gi[1] + acci[1];
   dy[18] = Gi[2] + acci[2];
   if (t < 1.)			dy[19] = 0.;
   else if (t < 80.)	dy[19] = -magg * cos(tetam) / vm;
   /////////////

   /*
    mf2s[0][0]=1;  mf2s[0][1]=0;  mf2s[0][2]=0;
    mf2s[1][0]=0;  mf2s[1][1]=0;  mf2s[1][2]=1;
    mf2s[2][0]=0;  mf2s[2][1]=-1;  mf2s[2][2]=0;
    twm(1,mf2s,fb,fbb);
    twm(1,mf2s,wib,wibb0);

    for(int i=0;i<3;i++)
    {
    uu[i]=fbb[i];
    uu[i+3]=wibb0[i];
    }
    for(int i=0;i<9;i++)
    xx[i]=y[40+i];//mfp
    Geo_Nav(xx,uu);
    // */
   // ********************** Servo block **********************

   if (freq == 1)
   {
      //dley_servo = 1;
      dley_servo = 3;
		if (s_s == 6)	dley_servo = 5;
   }
   else {
      dley_servo = 5;
      if (s_s == 6)
         dley_servo = 5;

   }
   // */
  double err_naz = -0. * d2r;
   dy[37] = -wn_t * wn_t * y[36] - 2. * wn_t * zeta_t* y[37] +wn_t * wn_t * (stk5[0] + 0. * err_naz); // eth1_2;
   dy[36] = y[37];
   if (fabs(dy[36]) >= speed_servo_t  * d2r)	dy[36] = speed_servo_t  * d2r * fabs(dy[36]) / dy[36];
   if (fabs(y[36]) >= 2. * d2r) {
      y[36] = 2. * d2r * fabs(y[36]) / y[36];
      y[37] = 0.;
   }

   dy[39] = -wn_t * wn_t * y[38] - 2. * wn_t * zeta_t * y[39] +  wn_t * wn_t * (stk6[0] + err_naz); // eth2_2;
   dy[38] = y[39];
   if (fabs(dy[38]) >= speed_servo_t  * d2r)	dy[38] = speed_servo_t  * d2r * fabs(dy[38]) / dy[38];
   if (fabs(y[38]) >= 2. * d2r) {
      y[38] = 2. * d2r * fabs(y[38]) / y[38];
      y[39] = 0.;
   }

   // ##############################################
   // wn = -0.2 * q + 120;
   // speed_servo = -0.35 * q + 120;

   dy[30] = y[24];
   if (fabs(dy[30]) >= speed_servo * d2r)	dy[30] = speed_servo * d2r * fabs(dy[30]) / dy[30];
   // if(fabs(y[24])<10.*d2r) y[24]=0.;
   dy[24] = -2. * zeta * wn * y[24] - wn * wn * y[30] + wn * wn * stk1[dley_servo]; //

   dy[31] = y[25];
   if (fabs(dy[31]) >= speed_servo * d2r)	dy[31] = speed_servo * d2r * fabs(dy[31]) / dy[31];
   // if(fabs(y[24])<10.*d2r) y[24]=0.;
   dy[25] = -2. * zeta * wn * y[25] - wn * wn * y[31] + wn * wn * stk2[dley_servo]; //

   dy[32] = y[26];
   if (fabs(dy[32]) >= speed_servo * d2r)	dy[32] = speed_servo * d2r * fabs(dy[32]) / dy[32];
   // if(fabs(y[24])<10.*d2r) y[24]=0.;
   dy[26] = -2. * zeta * wn * y[26] - wn * wn * y[32] + wn * wn * stk3[dley_servo]; //

   dy[33] = y[27];
   if (fabs(dy[33]) >= speed_servo * d2r)	dy[33] = speed_servo * d2r * fabs(dy[33]) / dy[33];
   // if(fabs(y[24])<10.*d2r) y[24]=0.;
   dy[27] = -2. * zeta * wn * y[27] - wn * wn * y[33] +	wn * wn * stk4[dley_servo]; //
   // */

/*
   if ((sepflg == 0) && (fabs(ax) > fabs(ax_m1)))

   {
      ax_m1 = ax, t_ax1 = t;
   }
   if (sepflg == 1 && (fabs(ax) > fabs(ax_m2))) {
      ax_m2 = ax, t_ax2 = t;
   }
   if (sepflg == 0 && (fabs(ay) > fabs(ay_m1))) {
      ay_m1 = ay, t_ay1 = t;
   }
   if (sepflg == 1 && (fabs(ay) > fabs(ay_m2))) {
      ay_m2 = ay, t_ay2 = t;
   }

   if (sepflg == 0 && (fabs(az) > fabs(az_m1))) {
      az_m1 = az, t_az1 = t;
   }
   if (sepflg == 1 && (fabs(az) > fabs(az_m2))) {
      az_m2 = az, t_az2 = t;
   }
   if (sepflg == 0 && (dynpres / 10000. > q_m1)) {
      q_m1 = dynpres / 10000, t_q1 = t;
   }
   if (sepflg == 1 && (dynpres / 10000 > q_m2)) {
      q_m2 = dynpres / 10000, t_q2 = t;
   }

   if (sepflg == 0 && ((q * fabs(alfa)) > fabs(qa_m1))) {
      qa_m1 = q * alfa, t_qa1 = t;
   }
   if (sepflg == 1 && ((q * fabs(alfa)) > fabs(qa_m2))) {
      qa_m2 = q * alfa, t_qa2 = t;
   }

   if (fabs(y[6]) > fabs(wx_max))
      wx_max = y[6];
   if (fabs(y[7]) > fabs(wy_max))
      wy_max = y[7];
   if (fabs(y[8]) > fabs(wz_max))
      wz_max = y[8];//*/
   

}

void trmf(void) 
{
   unsigned int i;

   double wt, swt2, magr;
   /* Calculate  the cosine matrix from 'Zi" frame to 'Zf' frame */
   wt = wie * t;
   qif[0] = cos(wt / 2.);
   qif[1] = d1 * (swt2 = sin(wt / 2.));
   qif[2] = d2 * swt2;
   qif[3] = d3 * swt2;
   q2m(qif, mi2f);
   /* Calculate  the cosine matrix from 'Zi" frame to 'Zb' frame. */
   q2m(y + 9, mi2b);
   for (i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++)
         mb2i[j][i] = mi2b[i][j];
   }
   multmat(mi2f, mb2i, mb2f);
   /* Calculate the components of gravity  acceleration in  'Zi' frame. */
   Gacc(y, Gi);
   // if(t!=0.) for(int yi=0; yi<3; yi++)  Gi[yi]=Gi_ab[yi];
   magg = mag(3, Gi);

   /* Calculate  the  position and velocity of missile in  'Zf' frame  . */
   for (i = 0; i < 3; i++)	_Rii[i] = rf0[i] + y[13 + i];
   twm(1, mi2f, _Rii, _Rif);
   for (i = 0; i < 3; i++)	_rmf[i] = _Rif[i] - rf0[i];
   cross(wiff, _Rif, wR);
   for (i = 0; i < 3; i++)	 _vii[i] = y[16 + i];
   twm(1, mi2f, _vii, _vif);
   for (i = 0; i < 3; i++)	_vmf[i] = _vif[i] - wR[i];
   cross(wiff, wR, wwR);

   for (i = 0; i < 3; i++)	Rii[i] = rf0[i] + y[i];
   twm(1, mi2f, Rii, Rif);
   magr = mag(3, Rii);
   for (i = 0; i < 3; i++)	rmf[i] = Rif[i] - rf0[i];

   double r0_current = re * (1. - e2) / sqrt(1. - (2. * e2 - e2 * e2) * (1. - sinlatm * sinlatm));
   hm = rmf[1] + Hl + rmf[0] * rmf[0] / 2. / r0;
   cross(wiff, Rif, wR);
   twm(1, mi2f, y + 3, vif);
   for (i = 0; i < 3; i++)	vmf[i] = vif[i] - wR[i];
   // vm=mag(3,vmf);

   vm_dot_new = (mag(3, vmf) - vm) * 100.;
   vm = mag(3, vmf);

   twm(1, mf2e, Rif, Re);
   twm(1, mf2e, vmf, vme);

   if (t > 60.) 
   {
      if (fabs(hm) < fabs(h_old) && mk == 0) 
	  {
         m_old = h_old;
         mk = 1;
      };
      h_old = hm;
   }
//--------------------------------------------------------------
   Xm = rmf[0];	Ym = rmf[1];	Zm = rmf[2];
   r_l = Arr2Mat(rmf);
   v_l = Arr2Mat(vmf);
   Vxm=vmf[0];	Vym = vmf[1];	Vzm = vmf[2];
   Vm_t=vm;
   Vm_t_2 = vm*vm;

	rmf_rtf = -1*(Arr2Mat(rtf) - Arr2Mat(rmf));
	vmf_vtf = 1*(Arr2Mat(vmf));
	//vmxz = sqrt(vmf[0] * vmf[0] + vmf[2] * vmf[2]);
	//vmxy = sqrt(vmf[0] * vmf[0] + vmf[1] * vmf[1]);
	Vxy = sqrt(Vxm*Vxm + Vym*Vym) ;
	rxz = sqrt((rtf[0] - rmf[0]) * (rtf[0] - rmf[0]) + (rtf[2] - rmf[2]) *(rtf[2] - rmf[2]));
	rxy = sqrt((rtf[0] - rmf[0]) * (rtf[0] - rmf[0]) + (rtf[1] - rmf[1]) *(rtf[1] - rmf[1]));
	alfa_g =alfa*d2r;
	beta_g =betta*d2r;
	alfab_g =alfab*d2r;
	mat3x3 mbi;
	Arr2Mat3x3(mb2i,&mbi);
	mib=~mbi;

	Ffi=Ifi=rmn_[0];
	Flam=Ilam=rmn_[1];
	Fh=Ih=rmn_[2];
	FvN=IvN=vmn_[0];
	FvE=IvE=vmn_[1];
	FvD=IvD=vmn_[2];

	NED_Angles(0,0)=phim;//
	NED_Angles(1,0)=tetam;
	NED_Angles(2,0)=At-psim;//

	C_b_n = Rot_b_n(NED_Angles);
	q_b_n = RotMat2Quat(C_b_n);
	GGdt=t;


   /*
    if(Aim==1){
    fprintf(popof,"\n%3.2f      %6.4f    %6.4f    %5.4f   %5.4f    %5.4f    %5.4f 	  %5.4f    %5.4f	"
    ,t,mass,ax,cx,phim*r2d,tetam*r2d,hm,rmf[2],wz*r2d);
    }
    // */
}


///////////////////////////////////////////////////////////////////////////////////////
int om273na(void)
    
{
// PORPUSE :       To showing and saving the outputs of f6dtndr  routine .

   unsigned int i,idx;
   double magq, /*xer, zer,*/ magr, dt;
   trmf();
   if(t>10)	tgo = fabs(Slant_Range/vm);
   

   // ################# TVC #####################
    if (s_s > 3 && fabs(hm / Slant_Range) < 1)
      angle_tvc_sim = acos(hm / Slant_Range) * r2d;

   sl_rx = 250000.;
   // ############# save angle_tvc in firetable #########
   if (t > 100. && f_flag == 2 && sl == 0 && Slant_Range < sl_rx) {
      sl = 1;
      fprintf(sl_1, "%.3f	%3.3f ", tgo,angle_tvc_sim);
   }//*/
   // ###################################################
//*
   if (vmf[1] < 0. && vb == 0 && Slant_Range < sl_rx && nchoice < 5) {
      Del_t = t;
      vb = 1;

      motor_2_flg = 3; // normal

   }

   if (vmf[1] < 0. && vb == 0 && (Slant_Range < sl_rx) && nchoice > 4) {
      Del_t = t;
      vb = 1;

      if ((angle_tvc_sim - angle_tvc_jad) > 5.)
         motor_2_flg = 2; // down
      else if ((angle_tvc_sim - angle_tvc_jad) < -5.)
         motor_2_flg = 1; // up
      else
         motor_2_flg = 3; // normal
   }//*/
	if(tgt_flg>0)
	{
		rtf[0]=Xtrgt;
		rtf[1]=Ytrgt;
		rtf[2]=Ztrgt;
	}

   // #########################################################
  // NED_function();
   NED_function( Re,rmn_);
   hm = rmn_[2];//hgps;
   Hme = hm;

   M_char(&xbc, &mass, &ixx, &iyy, &izz);
   atmosp(hm, &temp, &press, &dens, &sound_speed, &rmu);
   m273na_thrust(ftb, mtb);
   wind();

   wv = 0.;

   if (flg == 1) // up
          wv = 10.;
   if (flg == 2) // down
          wv = -10.;
   waz = At + 45. * d2r;
   // */

   wv = wv0;
   waz = At + waz0 * d2r;
   if (launch2 == 1 || launch2 == 2) {
      wv = 20. * launch2 - 30.;
      waz = M_PI + At;
   }
   // */
   vbwf[0] = vmf[0] - wv * cos(waz - At);
   vbwf[1] = vmf[1];
   vbwf[2] = vmf[2] - wv * sin(waz - At);
   twm(2, mb2f, vbwf, vbwb);

   /* Calculate the components of aerodynamic forces and moments in 'Zb' frame. */
   M_aero(sound_speed, dens, vbwb, y + 6, fab, mab);
   /* Calculate the components of nongravititional  acceleration in 'Zi' frame. */
   fngb[0] = ftb[0] + fab[0];
   fngb[1] = ftb[1] + fab[1];
   fngb[2] = ftb[2] + fab[2];
   // Compute the Euler angles of missile .  */
   M_attitude(mi2b, &phim, &tetam, &psim);
   
   fngb0 = fngb[0];
	if(f_tele==1 )//&& (t<=tt_tele))
	{
	/*
	fngb[0]=tele_sns_interpol(2,t,tele_snsr)*mass;
	fngb[1]=tele_sns_interpol(3,t,tele_snsr)*mass;
	fngb[2]=tele_sns_interpol(4,t,tele_snsr)*mass;//*/
	
	//if(t<tt_tele-100.)
	fngb[0]=tele_sns_interpol(2,t,tele_snsr)*mass;
	//fngb[1]=tele_sns_interpol(3,t,tele_snsr)*mass;
	//fngb[2]=tele_sns_interpol(4,t,tele_snsr)*mass;//*/
	
   // fngb0=tele_sns_interpol(2,t,tele_snsr)*mass;
	

	//phim=tele_19_interpol(8,t,tele_3106)*d2r;
	//tetam=tele_19_interpol(10,t,tele_3106)*d2r;


  }//*/
  
   ///  if(thr_flg2==1 ) Ae2 =( 0.377*0.377*pi/4);

  //if (thr_flg==2) Ae = 0.71*0.71*pi/4, Pe = 66500.;//IR

  	
	if (ign2_cmd==0)	tele_thr=(fngb0-fab[0])-(Ae * (Pe - press));
	else 				tele_thr=(fngb0-fab[0])-(Ae2 * (Pe2 - press));
	if (l_launcher < l_rail)	tele_thr=fngb0-(-5.*9.81);
	//tele_thr -=(0.71*0.71*pi/4 * (76578.44 - press));
  //if( t<=34.5)	fngb[0]=tele_sns_interpol(2,t,tele_snsr)*mass;
   /*
    if(f_tele==1 && t>=35.22 && t<39.79)
    fngb[0]=(.65263*t*t-52.122*t+1039.9)*magg*mass;
    // */
   twm(1, mb2i, fngb, fngi);

   // ##########################
    Rot_e_n(rmn_[0], rmn_[1], Ce2n);
    //euler_f[2]=phim;euler_f[0]=tetam;euler_f[1]=psim;
    //twm(1,mf2e,euler_f,euler_e);
    //twm(1,Ce2n,euler_e,euler_n);
   // ##########################
   // phi_dot=(wz*sin(phim)-wy*cos(phim))*tan(tetam)+wx;

   // Calculate the outputs of accelerometers (I.M.U.) .
   accb[0] = (fngb[0] / mass + abx) * (1.0 + asx);
   accb[1] = (fngb[1] / mass + aby) * (1.0 + asy);
   accb[2] = (fngb[2] / mass + abz) * (1.0 + asz);
   
   if (fabs(accb[0]) > 40.*9.81)	accb[0] = 40.*9.81*sign(accb[0]);
   if (fabs(accb[1]) > 40.*9.81)	accb[1] = 40.*9.81*sign(accb[1]);
   if (fabs(accb[2]) > 40.*9.81)	accb[2] = 40.*9.81*sign(accb[2]);//*/
   
   ax = accb[0] / magg;
   ay = accb[1] / magg;
   az = accb[2] / magg;

   Ab=Arr2Mat(accb);

   /* Calculate  the cosine matrix from 'Zi" frame to 'Zb' frame. (with gyros output) */
   q2m(y + 20, _mi2b);
   for (i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++)
         _mb2i[j][i] = _mi2b[i][j];
   }
   multmat(mi2f, _mb2i, _mb2f);
   twm(2, _mi2b, accb, acci);
   // Obtain the stage of control system.
   // Solve the difference equations of autopilot.
	Guidance();
	//if(f_tele==1 )//&& (t<=tt_tele))
	/*{
		du_pitch=tele_19_interpol(8,t,tele_3103);
		ayc=tele_19_interpol(9,t,tele_3101)/9.81;
		du_yaw = -ayc + Azb_filter;

	}//*/
//	if (sim_count % 2 == 0)
 // {
	PilotGain();
    //if (sim_count % 2 == 0)
	AutoPilot(0);
	//}
	Outputs();
	// Sep_func();  // M480 is single-stage: physical separation is disabled.
   //    }
	//Del_t = t_BeltCmd+ign2_dely;//sim
	
	//if((motor_flag && sepflg) || (rocket && s_s>1) || sepflg2)
	if((StopCtrlCmd) )//|| (rocket && s_s>1) || sepflg2)
	{
	delc1=delc2=delc3 =delc4 =0.;
	dac[0]=dac[1]=dac[2]=dac[3]=0.;
	}//*/
	//if(motor_flag==0)ign2_cmd=0;

   magq = mag(4, y + 9);
   for (i = 11; i > 0; i--) {
      stk1[i] = stk1[i - 1];
      stk2[i] = stk2[i - 1];
      stk3[i] = stk3[i - 1];
      stk4[i] = stk4[i - 1];
      stk5[i] = stk5[i - 1];
      stk6[i] = stk6[i - 1];

   }
   stk1[0] = delc1=dac[0]*d2r;
   stk2[0] = delc2=dac[1]*d2r;
   stk3[0] = delc3=dac[2]*d2r;
   stk4[0] = delc4=dac[3]*d2r;
   stk5[0] = eth2_2;
   stk6[0] = eth1_2;

   /////////

   //if (t > .5 && t < 62. && p_count % 60 == 0 && f_flag == 1)
   if (t > .5 && t < 80. && p_count % 60 == 0 && f_flag == 1)
      fprintf(m_z, "%3.3f    %4.3f   %2.3f\n ", t, rmf[2], vmf[2]);

//   if (t > .5 && t < 62. && p_count % 60 == 0 && f_flag == 1)
   if (t >= .5 && t < 80. && p_count % 60 == 0 && f_flag == 1)
   {
      fprintf(m_tet, "%3.3f    %4.3f \n ", t, y[19]*r2d);
     fprintf(m_psi, "%3.3f    %4.3f \n ", t, psim*r2d);
   }

   if(f_flag == 2)
   {
	   //int gam_con =vb;// (t >= 32.0);//sepflg==1;//
	   int gam_con = (t >= 30.0);//sepflg==1;//
		//int  gam_con = burnoutflg2==1;

		if ((accb[0] < 140.) && (gam_con)) //140 oc_b
		//if ( (gam_con)) //140 oc_b
			 gama_cnt0 ++;
		else gama_cnt0 = 0;

		if ((3 < gama_cnt0) && (gama_flg0 == 0)) 
		{
			t_gama0 = t;
			gama_flg0 = 1;
			p_count=0;
		}
	if (t <= (t_gama0 +35.) &&  gama_flg0 && f_flag == 2. && p_count % 8 == 0)
		fprintf(m_pos, "%3.2f    %8.3f    %8.3f   %8.3f\n", t, rmf[0], rmf[1],sqrt(vmf[0]*vmf[0] + vmf[1]*vmf[1]));
   }//*/



   if (m_flag < 2)
      Slant_Range = sqrt((rtf[0] - rmf[0]) * (rtf[0] - rmf[0]) +
       (rtf[1] - rmf[1]) * (rtf[1] - rmf[1]) + (rtf[2] - rmf[2]) *
       (rtf[2] - rmf[2]));
   else
   {
		//Slant_Range = sqrt((rtf_6[0] - rmf[0]) * (rtf_6[0] - rmf[0]) +
		//(rtf_6[1] - rmf[1]) * (rtf_6[1] - rmf[1]) + (rtf_6[2] - rmf[2]) *
		//(rtf_6[2] - rmf[2]));

       Slant_Range = sqrt((rtf[0] - rmf[0]) * (rtf[0] - rmf[0]) +
       (rtf[1] - rmf[1]) * (rtf[1] - rmf[1]) + (rtf[2] - rmf[2]) *
       (rtf[2] - rmf[2]));
 
   }



   //if( t>(t_burn + tberr + Del_tm)  && burnoutflg1 == 0 && f_flag==2 && m_flag<2)
   if( s_s==2  && burnoutflg1 == 0 && f_flag==2 && m_flag<2)//fcu

   {
		burnoutflg1=1;
		if(nchoice==3)
		{
			//t_bo=t;	vm_bo=vm;	tetam_bo=tetam;
			
			burnoutvel = sim_fopen("BurnOutVelocity.dat", "wt");
			fprintf(burnoutvel, "%2.2f    %8.6f    %8.6f   \n ", t,vm,tetam);
			sim_fclose(burnoutvel);
		}
   }


   //	if(nchoice==5 && (((sim_count)%(4+dtt*2))==0) )
   	if(nchoice==5 && MinMaxRng_flg==0&& (((sim_count)%dtt)==0))//&& (sim_count % dtt == 0)/*&& ((sim_count%20)==0 || (Slant_Range<3e3 && sim_count%1==0))*/)
	{
		Telemetry();
		//PrintData();//pso
		PrintOutput2();

	}//*/

	//imperr(R0_L, Re, Ht_6 ,Latt_6, Longt_6,  erre);
	//imperr(R0_L, Re, Htf[TrgtCnt] ,Lattf[TrgtCnt], Longtf[TrgtCnt],  erre);

	 errm = imperr( Arr2Mat(R0_L), rmn,  rtn);
	 //  printf("\n\t\t\t  total time flight       = %.2f \n", t);

   // if(t>5. && rtf[1]>rmf[1])
   if (t > 100. && Ht > hm) 
	// if (t > 100. && (errm(0,0)<0)) 
	//if (t > 300. && (/*errm(0,0)<0 &&*/ errm(2,0)<0)) 
   {
	tff=t;
   //printf("\n\t\t\t  total time flight       = %.2f \n", tff);

	 // printf("\n\t\t\t\t done----------------------------\n ");
      if (m_flag < 2) {
         xer = rmf[0] - rtf[0];
         zer = rmf[2] - rtf[2];
      }
      else {
         xer = rmf[0] - rtf[0];
         zer = rmf[2] - rtf[2];
		
		 
		 if(rocket || motor_flag)
		 {
			 azimuth(Hl, Latl, Longl, hm, rmn_[0],  rmn_[1], &FAz, &FRange);
			 printf("\n\t final t = %.2f \t final range = %.2f km \t hl = %.2f\n ",t,FRange*1e-3,Hl);
			 printf(" latm = %.6f \t longm = %.6f km \t hm = %.2f\n ",rmn(0,0)*r2d,rmn(1,0)*r2d,rmn(2,0));
		 }

		// xer = -1*erre[0];	zer = erre[1];
      }
//	   sim_fclose(fd2);
      if (nchoice == 3 || nchoice == 5) 
	  {
         if (nchoice != 3)	show_data(6);

         if (nchoice == 5)	getch();
      }
      if (nchoice == 4 || nchoice == 6 || nchoice == 3)
         return 1;

      //if (nchoice == 5) {
      //   fprintf(fdiary, "\n   %7.2f %9.2f", t, thr);
      //}

      return 1;
   }

   hold = hm;

   if (g_flag == 1) 
   {
      int kkg = 100.; // floor(dtout/prmt[2]);
      // int kk =100.;
      int mod_count_g = sim_count_g % kkg;

      sim_count_g++;
   }
   // */
   if (nchoice == 4 || nchoice == 6 /* || nchoice == 3 */)
      return 0;
   // if(nchoice==4 || nchoice==6 )     return 0;


   jav_count++;

   int kk = floor(dtout / prmt[2]);
     kk =1000.;
   int mod_count = sim_count % kk;
   if (!mod_count) // inja .01*100 =1sec
   {
	 if (nchoice == 5)	show_data(5);
   }
   sim_count++;

   return 0;

}

double Interpol_1d(double tt) {
   int i, index;
   for (i = 0; i < isthr + 1; i++) {
      if (tt <= TSTHR[i]) {
         index = i - 1;
         break;
      }
   }
   if (tt == 0.)
      index = 0;
   if (tt >= TbStatic)
      return STHR[isthr - 1];
   else
      return (STHR[index] + (STHR[index + 1] - STHR[index]) *
       (tt - TSTHR[index]) / (TSTHR[index + 1] - TSTHR[index]));
}

double Interpol_2d(double tt) {
   int i, index;
   //isthr_2 = 99;
   for (i = 0; i < isthr_2 + 1; i++) {
      if (tt <= TSTHR_2[i]) {
         index = i - 1;
         break;
      }
   }
   if (tt == 0.)
      index = 0;
   if (tt >= TbS_2)
      return STHR_2[isthr_2 - 1];
   else
      return (STHR_2[index] + (STHR_2[index + 1] - STHR_2[index]) *
       (tt - TSTHR_2[index]) / (TSTHR_2[index + 1] - TSTHR_2[index]));
}

void m273na_thrust(double ftb[3], double mtb[3]) 
{
	 if(f_100)	flg=3.;
	 
	 //fcu
	 threrr = ((Total_Imp_1/Total_Imp)-1.);//-0.015;
	 threrr_2 = -0.000 + ((Total_Imp_2/Total_Imp2)-1.);
	 tberr = 0;//0.55;//1.3;
	  //eth1 = -7. / 60. * d2r;
      //eth2 = -0. / 60. * d2r;

   if (flg == 1) // up moslem
   {
      tberr += 1;//+1.3;
      threrr = 0.01+((Total_Imp_1/Total_Imp)-1.);//-0.015;
      eth1 = +8. / 60. * d2r;
      eth2 = -8. / 60. * d2r;
      Yt = -0.002;
      Zt = 0.002;
      xbc_y = 0.003;
      xbc_z = 0.003;
      if (DIAG_ZERO_ETH1 || (TAL_NOTVC)) { eth1 = eth2 = 0.; }
      if (DIAG_ZERO_YZ1)  { Yt = Zt = 0.; }
      if (DIAG_ZERO_XBC1) { xbc_y = xbc_z = 0.; }
   }
   if (flg == 2) // down moslem
   {
      tberr += -1;//+1.13;
      threrr = -0.01+((Total_Imp_1/Total_Imp)-1.);//-0.015;;
      eth1 = 8. / 60. * d2r;
      eth2 = 8. / 60. * d2r;
      Yt = 0.002;
      Zt = -0.002;
      xbc_y = 0.003;
      xbc_z = 0.003;
      if (DIAG_ZERO_ETH || (TAL_NOTVC)) { eth1 = eth2 = 0.; }
      if (DIAG_ZERO_YZ)  { Yt = Zt = 0.; }
      if (DIAG_ZERO_XBC) { xbc_y = xbc_z = 0.; }
   }

   if (flg == 1) // up vahab2
   {
      tberr_2 = 1.;
      threrr_2 = 0.01 -0.005+ ((Total_Imp_2/Total_Imp2)-1.);
      deth1_2 = -2. / 60. * d2r;
      deth2_2 = -2. / 60. * d2r;
      Yt_2 = -0.002;
      Zt_2 = 0.002;
      xbc_y_2 = 0.002;
      xbc_z_2 = 0.002;
   }
   if (flg == 2) // down vahab2
   {
      tberr_2 = -1.;
      threrr_2 = -0.01 -0.005+ ((Total_Imp_2/Total_Imp2)-1.);
      deth1_2 = 2. / 60. * d2r;
      deth2_2 = 2. / 60. * d2r;
      Yt_2 = 0.002;
      Zt_2 = -0.002;
      xbc_y_2 = 0.002;
      xbc_z_2 = 0.002;
   }//*/
   //tberr_2=t_burn_2-TbS_2;
//   xbc_y_2   = 0;
//   xbc_z_2   = 0.000;
//-------------

   double scale_tb = TbStatic / (t_burn + tberr);
   double tt = t * scale_tb;

   double scale_tb_2 = TbS_2 / (t_burn_2 + tberr_2);
   double tt_2 = (t - Del_t) * scale_tb_2; // mfp
   

   if (t <= t_burn + tberr)//+(0.75*(thr_flg==1))) 
   {
      /* Use the full thrust curve, matching M4-RAJEH_2-V_1. */
      sthr = Interpol_1d(tt) / 9.81;
	  //sthr *=Total_Imp_1/Total_Imp;//fcu
      sthr *= (1 + threrr) * scale_tb;
	  

      dy[34] = sthr;


      thr =sthr * 9.81 + Ae * (Pe - press);
	  
      
	  thr_2=0;
	  motor1_state=1;//fcu
	  
   }
	else if (ign2_cmd && (t>=Del_t && t <= (Del_t + t_burn_2 + tberr_2))&& motor2_state<2 &&motor_flag==0) 
	//else if (BeltCmd && (t>Del_t && t < (Del_t + t_burn_2 + tberr_2))&& motor_flag==0) 
   {
      sthr_2 = 1. * Interpol_2d(tt_2) / 9.81;
	  //sthr_2 *=Total_Imp_2/Total_Imp2;
      sthr_2 *= (1 + threrr_2) * scale_tb_2;
      dy[35] = sthr_2;
	  thr_2 = sthr_2 * 9.81 + Ae2 * (Pe2 - press);
      thr = 0.;
	  motor2_state=1;//fcu

   }
   else
   {
      thr = thr_2 = 0.;
	 // tele_thr=0.;
	 /*
	  	 if(burnoutflg2==0 && t>Del_t)
	 {
	   printf(" InpTotImp : %.2f \t TeleTotImp : %.2f\n ",Total_Imp,y[34]);
	   printf(" InpTotImp2 : %.2f \t TeleTotImp2 : %.2f\n ",Total_Imp2,y[35]);
	 }//*/
	 
	 if (ign2_cmd && t>Del_t)	 burnoutflg2=1;
	 
	 //fcu
	  if (motor1_state ==1)	motor1_state=2;
	  if (motor2_state ==1)	motor2_state=2;


   }

   if (thr < 0.)	thr = 0;
   if (thr_2 < 0.)	thr_2 = 0;

   double f_belt = 0.;
   if (PushCmd && (t < t_PushCmd + 0.04))
   {
      f_belt = 2500.;
	  //sep_fb=1;
   }
   else if(BeltCmd && t>t_BeltCmd+0.2 && sep_fb==0)
	   sep_fb=3;//*/

   ftb0 = thr * cos(eth1) * cos(eth2) + 4. * f_belt;
   ftb1 = thr * sin(eth1) * cos(eth2);
   ftb2 = -thr * sin(eth2) * cos(eth1);
  mtb0 = thr * (-(Yt - xbc_y) * cos(eth1) * sin(eth2) - (Zt - xbc_z) * sin(eth1)) - 0.0 * f_belt * (0.6 / 2.);
   mtb1 = -thr * cos(eth1) * sin(eth2) * Xt + thr * (Zt - xbc_z) * cos(eth1)* cos(eth2) +  0.0 * f_belt * (0.6 / 2.);
   mtb2 = -thr * sin(eth1) * cos(eth2) * Xt - thr * (Yt - xbc_y) * cos(eth1)* cos(eth2) +0. * f_belt * (0.6 / 2.);
#if DIAG_TELE
   O_vec[138]=mtb2; O_vec[139]=mtb1; O_vec[140]=thr; O_vec[141]=ftb2;
#endif
 //if (m_flag < 2 || s_s != 8)
      if ((m_flag < 2) || (s_s != 2&&t_ign2 <t) ||(guid_flg==1 && s_s == 2) )
      y[36] = y[38] = 0.;


   ftb0_2 = thr_2 * cos(y[36] + deth2_2) * cos(y[38] + deth1_2);
   ftb1_2 = -thr_2 * sin(y[38] + deth1_2) * cos(y[36] + deth2_2);
   ftb2_2 = thr_2 * sin(y[36] + deth2_2) * cos(y[38] + deth1_2);

   mtb0_2 = thr_2 * (-(Yt_2 - xbc_y_2) * sin(y[36] + deth2_2) * cos
       (y[38] + deth1_2) - (Zt_2 - xbc_z_2) * sin(y[36] + deth1_2));

   mtb1_2 = thr_2 * cos(y[38] + deth1_2) * sin(y[36] + deth2_2) * (Xt_2 + xbc) +
       thr_2 * (Zt_2 - xbc_z_2) * cos(y[36] + deth1_2) * cos(y[38] + deth2_2);

   mtb2_2 = thr_2 * sin(y[38] + deth1_2) * cos(y[36] + deth2_2) * (Xt_2 + xbc) +
       thr_2 * (Yt_2 - xbc_y_2) * cos(y[36] + deth1_2) * cos(y[38] + deth2_2);

   ftb[0] = ftb0 + ftb0_2 * (1. - bal_flag);
   ftb[1] = ftb1 + ftb1_2;
   ftb[2] = ftb2 + ftb2_2;
   mtb[0] = mtb0 + mtb0_2;
   mtb[1] = mtb1 + mtb1_2;
   mtb[2] = mtb2 + mtb2_2;

}


void M_char(double *xbc, double *mass, double *ixx, double *iyy, double *izz) {
   

    /////////////////////// 1300c V09
	// if(!sepflg)						xbc_er = (5.224-xcg2_1);
	// else								xbc_er = (2.219-xcg2_2);

	if(!sepflg)							xbc_er = (2.45-xcg2_1);
 else								xbc_er = (1.05-xcg2_1);

   double coef_1 = y[34] / Total_Imp / (1. + threrr);
   double xbc0_1 = xcg2_1 - xcg1_1;
   double coef_2 = y[35] / Total_Imp2 / (1. + threrr_2);
   double xbc0_2 = xcg2_2 - xcg1_2;
   double m2 = m_e_stg1;//m_f_stg1 - m_fuel - m_liner;


   //if (t < t_burn + tberr) 
 if (sepflg==0 )//&& (miss_type!=23))
  {
   //if (t < t_burn + tberr) 
	//if (motor1_state==1)//fcu
	if (motor1_state!=2)//fcu
   
{
       *mass = m_f_stg1 - (m_f_stg1-m_e_stg1)*coef_1;
      //*mass = m_f_stg1 - (m_fuel + m_liner) * coef_1;
      *xbc = xbc0_1 + (xcg1_1 - xcg2_1) * coef_1 + 0;//0.097;//0.091;
      *ixx = ix1_1 - (ix1_1 - ix2_1) * coef_1;
      *izz = *iyy = iy1_1 - (iy1_1 - iy2_1) * coef_1;
	  Xcg = xcg1_1 - (xcg1_1 - xcg2_1) * coef_1;
	  
	  	   //printf(" t : %.2f \n ",t);

   }
   else if (!sepflg)//
   {
      *mass = m_e_stg1;//m2;
      *xbc = 0;//0. + 0.097;//.091;
      *ixx = ix2_1;
      *izz = *iyy = iy2_1;
	  Xcg = xcg2_1;
   }
  }
   else if (sepflg==1 && sepflg2==0)
   {
		//if (t > Del_t && t < (t_burn_2 + tberr_2 + Del_t)) 
		if  (motor2_state==1)//(burn_flg2==0) //fcu
		{
		  *mass = m_f_stg2 - (m_f_stg2 - m_e_stg2) * coef_2;
		  *xbc = xbc0_2 + (xcg1_2 - xcg2_2) * coef_2 ;//0.069;
		  *ixx = ix1_2 - (ix1_2 - ix2_2) * coef_2;
		  *izz = *iyy = iy1_2 - (iy1_2 - iy2_2) * coef_2;
		  Xcg = xcg2_2 - (xcg2_2 - xcg1_2) * coef_2;
	   }
	   else if (motor2_state==2)//
		//else if(t > (t_burn_2 + tberr_2 + Del_t)) 
	   {
		  *mass = m_e_stg2;
		  *xbc = 0;
		  *ixx = ix2_2;
		  *izz = *iyy = iy2_2;
		  Xcg = xcg2_2;
	   

     }
	   else 
	   {
		  *mass = m_f_stg2;
		  *xbc =0.0;// -0.01;//0.018;//(xcg2_2-xcg1_2) ;
		  *ixx = ix1_2;
		  *izz = *iyy = iy1_2;
		  Xcg = xcg2_2;//xcg1_2;
	   }
	   
	   	if(motor_flag)
		{
			*mass = m_e_stg1-m_f_stg2;//744.5996;//m2-m1_2-5.21;
			//*mass = 744.5996;//m2-m1_2-5.21;
			*xbc = 0.4280;
			*ixx = 110.4;
			*izz = *iyy = 5749;
		}
   }
//#############################################
  

  // SM = ((cna)?(cma*D_body/(cna)-xbc_er):0);///D_body;// static margin in meter 

//--------------------------------------------------------------
	  xbc_er=0;
	
	if(motor_flag==0)
	{
	   
	
	   if ( rocket  ||m_flag < 2) 
	   {   
   
           if (!sepflg) {
		   
		   if(s_s>1)	xbc_er = 0.850000000000;// M480 remains the same body; separation is disabled.
		   else	if (guid_flg==1)		xbc_er = 0.850000000000;// Passive stability through the full DATCOM Mach range.
	       else     xbc_er = 0.850000000000;
	     }
	       else         { 
		   
		   
	    
      if (guid_flg==0 && motor2_state!=2 ){

       *xbc = (xcg2_2-xcg1_2) ;
      *ixx = ix1_2*1;
      *izz = *iyy = iy1_2*10;
       //*mass =500;
           //     xbc_er = -0.05713;
             xbc_er = -(xcg2_2-xcg1_2);// 0.08265;
                //////////
   // *mass = m_e_stg1;//m2;
     // *xbc = 0;//0. + 0.097;//.091;
  //*ixx = ix2_1;
      //*izz = *iyy = iy2_1;
   // Xcg = xcg2_1;


        }

        else{
          xbc_er = 0.02;//xcg2_2-2.25;//;
      *xbc = -0.00;
      *ixx = ix2_2;
      *izz = *iyy = iy2_2*10;



       

           }


         }
	   } 
	   else             xbc_er  = 0 ;
	

     
//
//
//
//if (s_s>2) xbc_er = 0.12;
 
/*  if ( s_s>4) xbc_er = -0.0;
 else xbc_er = 0; */
   /* tal_flag: keep the AERODYNAMIC reference where the tables put it.
      xbc is built as (xcg2_1 - Xcg), so xcg2_1 serves as both the stage-1
      burnout CG and the station the stage-1 coefficient tables are referenced
      to. Edit 4 moves the burnout CG to 2.122592, but the tables still
      describe a body referenced to 1.995907 -- without this line the mass
      change would silently drag the aero reference 12.7 cm aft, which is an
      aerodynamic change masquerading as a mass change.
      flag=0 is a strict no-op: TAL_XAERO_S1 == xcg2_1 exactly there.
      Pre-separation only; stage 2 is the unchanged payload. */
   if (sim_tal_flag && sepflg == 0) *xbc = TAL_XAERO_S1 - Xcg;

   *xbc += xbc_er;
}

}
void wind(void) {
   if (hm < Hl + 1000.) {
      wv = Wv1 + (hm - Hl) * (Wv2 - Wv1) / 1000.;
      waz = Waz1 + (hm - Hl) * (Waz2 - Waz1) / 1000.;
   }
   else if (hm < Hl + 2000.) {
      wv = Wv2 + (hm - Hl - 1000.) * (Wv3 - Wv2) / 1000.;
      waz = Waz2 + (hm - Hl - 1000.) * (Waz3 - Waz2) / 1000.;
   }
   else if (hm < Hl + 5000.) {
      wv = Wv3 + (hm - Hl - 2000.) * (Wv4 - Wv3) / 3000.;
      waz = Waz3 + (hm - Hl - 2000.) * (Waz4 - Waz3) / 3000.;
   }
   else if (hm < Hl + 10000.) {
      wv = Wv4 + (hm - Hl - 5000.) * (Wv5 - Wv4) / 5000.;
      waz = Waz4 + (hm - Hl - 5000.) * (Waz5 - Waz4) / 5000.;
   }
   else if (hm < Hl + 20000.) {
      wv = Wv5 - (hm - Hl - 10000.) * Wv5 / 10000.;
      waz = Waz5 - (hm - Hl - 10000.) * Waz5 / 10000.;
   }
   else {
      wv = 0.;
      waz = 0.;
   }
}

void M_attitude(double mi2b[3][3], double *phim, double *tetam, double *psim) 
{
   *phim = -atan2(mi2b[2][1], mi2b[1][1]);
   *tetam = asin(mi2b[0][1]);
   *psim = -atan2(mi2b[0][2], mi2b[0][0]);

   if (fabs(*tetam) > 80. * d2r) {
      *phim = atan2(mi2b[1][2], mi2b[2][2]);
      *tetam = atan2(mi2b[0][1], mi2b[0][0]);
      *psim = -asin(mi2b[0][2]);
   }

}

void MATRIXe2n_(double L, double l) {
   me2n_[0][0] = -sin(L) * cos(l);
   me2n_[1][0] = -sin(l);
   me2n_[2][0] = -cos(L) * cos(l);
   me2n_[0][1] = -sin(L) * sin(l);
   me2n_[1][1] = cos(l);
   me2n_[2][1] = -cos(L) * sin(l);
   me2n_[0][2] = cos(L);
   me2n_[1][2] = 0;
   me2n_[2][2] = -sin(L);

}
void NED_function(double Re[3],double rn[3]) 
{
   double s, b0, u0, b1, u1, N2;
   double f = 0.0033528106647812;
   double e = 0.081819190842622;
   rn[1] = atan2(Re[1], Re[0]);//lgps
   s = sqrt(Re[1] * Re[1] + Re[0] * Re[0]);
   b0 = atan2(Re[2], (1 - f) * s);
   for (int i = 0; i < 10; i++) {
      u0 = atan2((Re[2] + (e * e * (1 - f) * re * sin(b0) * sin(b0) * sin(b0)) /
          (1 - e * e)), (s - e * e * re * cos(b0) * cos(b0) * cos(b0)));
      b1 = atan2((1 - f) * tan(u0), 1);
      b0 = b1;
      if (i == 1)
         u1 = u0;
      if (i == 1 && (u1 - u0) < (1e-10))
         break;
   }
   N2 = re / sqrt(1 - e * e * sin(u0) * sin(u0));
   rn[2] = s * cos(u0) + (Re[2] + e * e * N2 * sin(u0)) * sin(u0) - N2;////hgps
   rn[0] = u0;//Lgps

   MATRIXe2n_(rn[0], rn[1]);
   twm(1, me2n_, vme, vmn_);
   vngps = vmn_[0];
   vegps = vmn_[1];
   vdgps = vmn_[2];
   vmn = Arr2Mat(vmn_);
   rmn = Arr2Mat(rn);
}


void Find_LOS_Ang()
{
	double temp;
   mat3x1 rts;
  // mat3x3 C_b_n = Rot_b_n(NED_Angles);

	rts= ~C_b_n*rmn_rtn;
	temp=sqrt(rts(0,0)*rts(0,0)+rts(1,0)*rts(1,0));
	LOS_ang[0] = atan2(rts(1,0),rts(0,0))*r2d;//simulateX
	LOS_ang[1] = -atan2(rts(2,0),temp)*r2d;//simulateY
}

void MakeZero()
{
    //=====Integer
	sepflg = sepflg2=pik_q = flg_q = flg_stage = sim_count = sim_count_g = p_count = flag_alfa = 0.;
	burn_flg=gama_flg=0;//flgm_5=0;
	kkk1=b_int1=Bref1=b1=BeltCmd=sep_fb=0;//flgm_1=flgm_2=burn_flg2=0;
	burnoutflg2=homing_phase=flg_vc=burnoutflg1=0;
	flg_alt_hold=1;
	vb=cond2=cond=motor_2_flg=tgo_cnt=tgo_flg=0;
	tgo_old=0;
	flg_stage=0;
	//f_flag = m_flag = 0;
	ign2_5=ign2_fb=0;
	oi_flg=0;
	motor1_state=motor2_state=StopCtrlCmd=StopCtrlCmd_2=0;//fcu
	sep_ok5=sep_ok=bad_sep3_cond=sep3_cmd=sep3_off=0;
	f_belt_flg=psip_flg=0;
	out_cnt=ct=cnt1=0;
	t_psi0=psi_0=t_ramp_gama=0;
	ign2_ok=ign2_ok5=0.;
	
			flg_ms1=i_s2=i_s8=i_sz2=i_sz8=0;
	err_motor=err_vxz=vm_s1=vmxy=vmxz=0;
	flg_for=flg_for2=1;
	psip_flg=psip_flg2=0;
    //========double

    Gama_corec=ren=Teta_launch_rok=delta_gama=0.0;
    vm_old=vm_f_old=xbc=ixx=iyy=izz=Wx_filter=Wy_filter=Wz_filter=V_save=Yt=Zt=thr=0.0;

    Ldot=ldot=rl=rL=axn=ayn=azn=tetamn=phimn=psimn=0.0;
    delx=dely=delv=0.0;
    vm=mach=hm=hold=q=du_pitch=du_gama=du_yaw=du_roll=0.0;
    t_dive=Ayb_filter=Azb_filter=cln=cy=cz=cll=cll_t=cllda=alfab=cx=cm=0.0;
	k_5=1;
    alfa=betta=gamam=gamac=dela=delr=dele=ax=ay=az=wx=wy=wz=0.0;
    delac=delrc=delec=delc1=delc2=delc3=delc4=phim=phic=tetam=tetac=teta_burn=psim=ayc=ac_pitch=0.0;
    Wv1=Waz1=Wv2=Waz2=Wv3=Waz3=Wv4=Waz4=Wv5=Waz5=Jr_star=0.0;
    d1=d2=d3=s1=s2=s3=sinlatm=mass=0.0;
    dynpres=t=totalt=0;//t_burn=t_burn_2=0.0;
    teterr= tetraterr=azerr=azraterr=tberr=maserr=threrr=eth1=eth2=0.0;
    gbx=gby=gbz=abx=aby=abz=asx=asy=asz=gsx=gsy=gsz=meanx=sigmax=meanz=sigmaz=cep=0.0;
    L_x=L_y=L_v=L_gama=V_star=d0=r0=temp=press=dens=sound_speed=rmu=magg=0.0;
    ca=cmqb=cmqa=cllp=cma=cna=0.0;
	K_w_pitch=K_p_integ_teta=K_i_integ_teta=K_w_yaw=K_p_integ_acc=K_i_integ_acc=K_w_roll=K_p_integ_roll=K_i_integ_roll=K_p_g=K_i_g=0.0;

	//khr=1;
   l_launcher=q_max=t_old1=b1=0;
  tsep=t_BeltCmd=t_ign2_cmd=1000.;

    //========arrays
    int i=0,j=0,k=0;
   // for(i=0;i<2;i++)rmf_save[i]=vmf_save[i]=0.0;
    for(i=0;i<3;i++)
    {
        wib[i]=wibb[i]=win[i]=winn[i]=wnb[i]=rni[i]=0.0;
        w_fir[i]=w_ini[i]=rf0[i]=rtf[i]=wiff[i]=wifb[i]=0.0;
        fab[i]=fngb[i]=ftb[i]=ai[i]=ab[i]=mab[i]=mtb[i]=ri[i]=Rii[i]=Rif[i]=rmf[i]=0.0;
        _ri[i]=_Rii[i]=_Rif[i]=_rmf[i]=_vii[i]=_vif[i]=_vmf[i]=0.0;
        vii[i]=vif[i]=vmf[i]=vbwb[i]=vbwf[i]=wR[i]=Gi[i]=accb[i]=0.0;
		Re[i]=vme[i]=rmn_[i]=0.0;
			stk1[i] =stk2[i] =stk3[i] = stk4[i] = stk5[i] = stk6[i] =0;
    }
    for(i=0;i<4;i++)_qib[i]=qib[i]=qbi[i]=qif[i]=0.0;

    for(i=0;i<3;i++)for(j=0;j<3;j++)
    {
        mf2s[i][j]=mb2n[i][j]=mb2i[i][j]=mi2b[i][j]=mb2f[i][j]=_mb2i[i][j]=_mi2b[i][j]=_mb2f[i][j]=0.0;
        mi2f[i][j]=_mi2f[i][j]=mf2b[i][j]= 0.0;

    }

    for(i=0;i<50;i++)dy[i]=y[i]=y[i]=dy[i]=prmt[i]=0.0;
    //for(i=0;i<35;i++)data_laun[i]=0.0;

	   tberr = -0;
      threrr = -0;
      eth1 = 0 / 60. * d2r;
      eth2 = 0 / 60. * d2r;
      Yt = 0;
      Zt = -0;
      xbc_y = 0;
      xbc_z = 0;

      tberr_2 = 0;
      threrr_2 = 0;
      deth1_2 = -0 / 60. * d2r;
      deth2_2 = -0 / 60. * d2r;
      Yt_2 = -0;
      Zt_2 = 0;
      xbc_y_2 = 0;
      xbc_z_2 = 0;
	 delc1=0;
	 delc2=0;
	 delc3=0;
	 delc4=0;
	 eth2_2=0;
	 eth1_2=0;
	 
	 	  	out_integ_yaw= out_integ_roll=out_integ_psi=0;
	K_i_integ_roll= K_p_integ_roll= K_w_roll= K_p_integ_acc= K_i_integ_acc=
	K_i_integ_teta= K_p_integ_teta= K_p_integ_teta1= K_p_integ_teta2=
	K_w_pitch1= K_w_pitch2= K_w_pitch= K_p_g= K_i_g=0;
	out_integ_gama=K_w_yaw=0;
	 Ki_w_pitch=0;
 k_ramp= K_w_pitch11= K_i_integ_teta11= K_p_integ_teta11= t_q_p0= t_q_p1=
    t_q_p2= t_q_p3= t_q_p4= t_q_p5=0;
 K_w_pitch00= K_i_integ_teta00= K_p_integ_teta00= t_q_p00=0;
 K_w_yaw00= K_i_integ_acc00= K_p_integ_acc00=0;
 K_w_roll00= K_i_integ_roll00= K_p_integ_roll00=0;
 K_w_yaw11= K_i_integ_acc11= K_p_integ_acc11=0;
 K_w_roll11= K_i_integ_roll11= K_p_integ_roll11=0;
 K_w_yaw22= K_i_integ_acc22= K_p_integ_acc22=0;
 K_w_roll22= K_i_integ_roll22= K_p_integ_roll22=0;
 K_w_pitch22= K_i_integ_teta22= K_p_integ_teta22=0;
 K_w_roll33= K_i_integ_roll33= K_p_integ_roll33= K_w_pitch33=
    K_i_integ_acc33= K_p_integ_acc33=0;
 K_w_roll44= K_i_integ_roll44= K_p_integ_roll44= K_w_pitch44=0;
// =K_i_integ_acc33=K_p_integ_acc33;
 K_w_roll55= K_i_integ_roll55= K_p_integ_roll55= K_w_pitch55=0;
 K_w_psi00=K_p_integ_psi00=K_i_integ_psi00=K_w_psi22=K_p_integ_psi22=K_i_integ_psi22=0;
  K_w_psi11=K_p_integ_psi11=K_i_integ_psi11=0;
// =K_i_integ_acc33=K_p_integ_acc33;
 //K_p_integ_roll66=0;//*/
 psic=0;
tvc_flg=0;

}

void MakeZero3()
{
    //=====Integer
	sepflg = sepflg2=pik_q = flg_q = flg_stage = sim_count = sim_count_g = p_count = flag_alfa = 0.;
	burn_flg=gama_flg=0;//flgm_5=0;
	kkk1=b_int1=Bref1=b1=BeltCmd=sep_fb=0;//flgm_1=flgm_2=burn_flg2=0;
	burnoutflg2=homing_phase=flg_vc=burnoutflg1=0;
	flg_alt_hold=1;
	vb=cond2=cond=motor_2_flg=tgo_cnt=tgo_flg=0;
	tgo_old=0;
	flg_stage=0;
	//f_flag = m_flag = 0;
	ign2_5=ign2_fb=0;
	oi_flg=0;
	motor1_state=motor2_state=StopCtrlCmd=0;//fcu
	sep_ok5=sep_ok=bad_sep3_cond=sep3_cmd=sep3_off=0;
	f_belt_flg=psip_flg=0;
	//out_cnt=ct=cnt1=0;
	t_psi0=psi_0=0;
    //========double

	
   l_launcher=q_max=t_old1=b1=0;
  tsep=t_BeltCmd=t_ign2_cmd=1000.;

}

void Telemetry()
{
//	if (pid_tune)	return;
		
	double stg2_guidc=psic,oiy=out_integ_yaw,apc_lim=apc_limit;
	if (gamac_flg)	stg2_guidc=gamac;
	if (psic_flg)	oiy=out_integ_psi*r2d;
	if (hgv_flg&&homing_phase==0)	apc_lim=ancomx_limit;
   	O_vec[0] =t ;
	O_vec[1]  =rmf[0];		O_vec[2] =rmf[1];		O_vec[3] =rmf[2];
	O_vec[4]  =vmf[0];		O_vec[5] =vmf[1];		O_vec[6] =vmf[2];
	O_vec[7]  =accb[0];		O_vec[8] =accb[1];		O_vec[9] =accb[2];//10
	O_vec[10] =y[6];		O_vec[11] =y[7];		O_vec[12] =y[8] ;
	O_vec[13] =phim*r2d;	O_vec[14] =psim*r2d;	O_vec[15] =tetam*r2d;
	O_vec[16] =tetac *r2d;
	O_vec[17] =Azb_filter;	O_vec[18] =ayc;			O_vec[19] =Ayb_filter;	O_vec[20] =ac_pitch;
	O_vec[21] =dela;		O_vec[22] =delr;		O_vec[23] =dele;
	O_vec[24] =hm;			O_vec[25] =vm;			O_vec[26] =alfa;		O_vec[27] =betta ;
	O_vec[28] =mass;//y[36] ;	
	O_vec[29] =xbc;			O_vec[30] =Teta0 ;//ixx ;	
	O_vec[31] =y[38] ;
	O_vec[32] =thr-(Ae * (Pe - press));
	O_vec[33] =mach ;		O_vec[34] =dynpres;		O_vec[35] =q;
	O_vec[36] =s_s;
	O_vec[37] =gamac*r2d;	O_vec[38] =gamam*r2d;
	O_vec[39] =rmn(0,0)*r2d;	O_vec[40] =rmn(1,0)*r2d;	O_vec[41] =rmn(2,0);
	O_vec[42] =delc1*r2d;	O_vec[43] =delc2*r2d;	O_vec[44] =delc3*r2d;	O_vec[45] =delc4*r2d;
	// O_vec[42] =y[30]*r2d; O_vec[43] =y[36]*r2d; O_vec[44] =y[32]*r2d; O_vec[45] =y[38]*r2d;
  O_vec[46] =ca;		O_vec[47] =fab[1];		O_vec[48] =fab[2];		O_vec[49] =fab[1]/mass*9.81;	
	O_vec[50] =(ftb[0]-thr_2);	O_vec[51] =tele_thr;///9.81;//fngb[0]-fab[0];

	O_vec[52] =du_roll;		O_vec[53] =du_pitch;	O_vec[54] =du_yaw;		O_vec[55] =du_gama;
	O_vec[56] =Gama_corec;
	O_vec[57] =Wx_filter;	O_vec[58] =Wy_filter;	O_vec[59] =Wz_filter;//60
	O_vec[60] =Slant_Range;
	O_vec[61] =sthr_2;
	O_vec[62] =alfab;
	O_vec[63] =apc_lim;//apc_limit;//sepflg;
	O_vec[64] =tgo;//SM;//alfa_xy*r2d;
	O_vec[65] =sound_speed;
	//O_vec[66] =rtf[0];		O_vec[67] =rtf[1];		O_vec[68] =rtf[2];
	O_vec[66] =Xtrgt;		O_vec[67] =Ytrgt;		O_vec[68] =Ztrgt;
	O_vec[69] =Xcg;			O_vec[70] =Xcp;			O_vec[71] =SM;
	//O_vec[72] =erre[0];		O_vec[73] =erre[1];		O_vec[74] =erre[2];
	O_vec[72] =errm(0,0);		O_vec[73] =errm(1,0);		O_vec[74] =errm(2,0);

	O_vec[75] =K_w_pitch1;	O_vec[76] =K_p_integ_teta;	O_vec[77] =K_i_integ_teta;
	O_vec[78] =K_w_yaw;		O_vec[79] =K_p_integ_acc;	O_vec[80] =K_i_integ_acc;
	O_vec[81] =K_w_roll;	O_vec[82] =K_p_integ_roll;	O_vec[83] =out_integ_roll;
	O_vec[84] =out_integ_pitch;		O_vec[85] =oiy;//out_integ_yaw;
	O_vec[86] =delac*r2d;	O_vec[87] =delrc0*r2d;		O_vec[88] =delec0*r2d;
	O_vec[89] =delrc*r2d;	O_vec[90] =delec*r2d; 

	O_vec[91] =hing_mom[0];	O_vec[92] =hing_mom[1];	O_vec[93] =hing_mom[2];	O_vec[94] =hing_mom[3];
	O_vec[95] =bend_mom[0];	O_vec[96] =bend_mom[1];	O_vec[97] =bend_mom[2];	O_vec[98] =bend_mom[3];

	O_vec[99] =rtaz(0,0);		O_vec[100] =rtaz(1,0);		O_vec[101] =rtaz(2,0);
	O_vec[102] =vmaz(0,0);		O_vec[103] =vmaz(1,0);		O_vec[104] =vmaz(2,0);
	
	//sep
	O_vec[105] =BeltCmd;	O_vec[106] =ign2_fb;	O_vec[107] =sepflg;
	O_vec[108] =t_BeltCmd;	O_vec[109] =t_ign2;	O_vec[110] =t_sep;	O_vec[111] =Del_t;
	O_vec[112] =t_burn1-(t_burn + tberr);
	O_vec[113] =(t_burn2-Del_t)-(t_burn_2+ tberr_2);

	O_vec[114] =fab[0];		O_vec[115] =sim_dbg_dvr;		O_vec[116] =(double)sim_dbg_guard;/*Step2: were dup fab[1]/fab[2] (cols 48/49)*/
	O_vec[117] =mab[0];		O_vec[118] =mab[1];		O_vec[119] =sim_dbg_theta_p;/*Step2: was dup mab[1]*/
	O_vec[120] =zm_c;	O_vec[121] =vzm_c;	
	
	/* Guidance-phase instrumentation (telemetry ONLY; never read by any control
	   law). These three channels previously carried K_w_psi/K_p_integ_psi/
	   K_i_integ_psi, which are declared at PILOT.CPP:138 and never assigned
	   anywhere -- they are read only inside the psic_flg branch, and psic_flg
	   (M273N.CPP:110) is never written, so that branch is dead and the three
	   channels were permanently zero. Repurposed to expose which guidance path
	   a case actually flies. */
	O_vec[122] =(double)homing_phase;	O_vec[123] =(double)guid_flg;	O_vec[124] =(double)flg_vc;
	O_vec[125] =out_integ_psi_limit;	O_vec[126] =out_integ_wy_limit;	O_vec[127] =in_integ_psi_limit;
	O_vec[128] =du_psi_limit;	O_vec[129] =psic*r2d;
	
	//saveit(O_vec);
}
