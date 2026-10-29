/*--------------------------- Global-variables ---------------------------*/
//#include"stdafx.h"

#pragma once
#include "aero_tables.h"   /* AERO_R / AERO_C / AERO_K */
/*--------------------------- Global-variables ---------------------------*/
char *option[]={
		"  1 )   Initialize  the characteristic data. ",
		"  2 )   Initialize  the launch data.  ",
		"  3 )   Initialize  the error data. ",
		"  4 )   Guidance parameters calculation. ",
		"  5 )   Ballistic   parameters calculation.",
		"  6 )   Simulation of M-325 Missile. ",
		"  7 )   C.E.P.  Calculation.    ",
		"  8 )   Calculation Coefficient of Transfer Function.  ",
		"  9 )   Graphic plot.  " ,
		"  10)   Exit.  " };


typedef struct {
	long type;   /* type */
	long mrows;  /* row dimension */
	long ncols;  /* column dimension */
	long imagf;  /* flag indicating imag part */
	long namlen; /* name length (including NULL) */
} Fmatrix;
Fmatrix pls;
//#########################################################
// ###################### missile ##########################
double R_t, R_f, r2[3], r1[3], R_m[3];
float cmalfam[AERO_R][AERO_C],cnalfam[AERO_R][AERO_C],cmqalfam[AERO_R][AERO_C],cnqalfam[AERO_R][AERO_C],clpm[AERO_R][AERO_C];
float cmdeltam[AERO_R][AERO_C],cndeltam[AERO_R][AERO_C],cadeltam[AERO_R][AERO_C],catotalm[AERO_R][AERO_C],caalfam[AERO_R][AERO_C];
// roll
float AlfaArrayrm[AERO_R], MachArrayrm[AERO_C], DelaArrayrm[AERO_K], DeltaArrayrm[AERO_R][AERO_C][AERO_K];
int AlfaDataNorm, MachDataNorm, DelaDataNorm;
// drag
float  AlfaArraycm[AERO_R],MachArraycm[AERO_C],RenArraycm[AERO_K],CxArrayoncm[AERO_R][AERO_C][AERO_K],CxArrayofcm[AERO_R][AERO_C][AERO_K];
int AlfaDataNocm,MachDataNocm,RenDataNocm;
//#########################################################
//###################### warhead ##########################
float cmalfaw[AERO_R][AERO_C], cnalfaw[AERO_R][AERO_C], cmqalfaw[AERO_R][AERO_C], cnqalfaw[AERO_R][AERO_C],clpw[AERO_R][AERO_C];

float cmdeltaw[AERO_R][AERO_C],cndeltaw[AERO_R][AERO_C],cadeltaw[AERO_R][AERO_C],catotalw[AERO_R][AERO_C];
float cH_d[AERO_R][AERO_C], cB_d[AERO_R][AERO_C],cH_a[AERO_R][AERO_C], cB_a[AERO_R][AERO_C];

// roll
float  AlfaArrayrw[AERO_R],MachArrayrw[AERO_C],DelaArrayrw[AERO_K],DeltaArrayrw[AERO_R][AERO_C][AERO_K];
int AlfaDataNorw,MachDataNorw,DelaDataNorw;
// drag
float  AlfaArraycw[AERO_R],MachArraycw[AERO_C],RenArraycw[AERO_K],CxArrayoncw[AERO_R][AERO_C][AERO_K];
int AlfaDataNocw,MachDataNocw,RenDataNocw;

float  AlfaArraycwf[AERO_R],MachArraycwf[AERO_C],RenArraycwf[AERO_K],CxArrayofcw[AERO_R][AERO_C][AERO_K];
int AlfaDataNocwf,MachDataNocwf,RenDataNocwf;
// ###################### warhead2 ##########################
 float cmalfaw2[AERO_R][AERO_C], cnalfaw2[AERO_R][AERO_C], cmqalfaw2[AERO_R][AERO_C], cnqalfaw2[AERO_R][AERO_C],clpw2[AERO_R][AERO_C];
 float cmdeltaw2[AERO_R][AERO_C], cndeltaw2[AERO_R][AERO_C], cadeltaw2[AERO_R][AERO_C], catotalw2[AERO_R][AERO_C];
 float cmalfaw2_[AERO_R][AERO_C], cnalfaw2_[AERO_R][AERO_C], cmqalfaw2_[AERO_R][AERO_C], cnqalfaw2_[AERO_R][AERO_C];
// drag
 float CxArrayofcw2[AERO_R][AERO_C][AERO_K];

// #########################################################
//###################### motor ##########################
//double re,wie,J,e,Mu,pi,A,B ;
float cmalfamo[AERO_R][AERO_C],cnalfamo[AERO_R][AERO_C],cmqalfamo[AERO_R][AERO_C],cnqalfamo[AERO_R][AERO_C],clpmo[AERO_R][AERO_C];
float cmdeltamo[AERO_R][AERO_C],cndeltamo[AERO_R][AERO_C],cadeltamo[AERO_R][AERO_C],catotalmo[AERO_R][AERO_C],clldelamo[AERO_R][AERO_C];
// roll
//float  AlfaArrayrmo[6],MachArrayrmo[13],DelaArrayrmo[3],DeltaArrayrmo[6][13][3];
//int AlfaDataNormo,MachDataNormo,DelaDataNormo;
// drag
float  AlfaArraycmo[AERO_R],MachArraycmo[AERO_C],RenArraycmo[AERO_K],CxArrayoncmo[AERO_R][AERO_C][AERO_K],CxArrayofcmo[AERO_R][AERO_C][AERO_K];
int AlfaDataNocmo,MachDataNocmo,RenDataNocmo;
//#########################################################
// #########################################################


//##############################
double mf2s[3][3],mb2n[3][3],wib[3],wibb[3],win[3],winn[3],wnb[3],rni[3];
float MDOT[404];
float CB[13][16],CH[13][16];

double cb_1,ch_1,bending1,hing1,cb_2,ch_2,bending2,hing2,cb_3,ch_3,bending3,hing3,cb_4,ch_4,bending4,hing4;
double bending1_old,bend1,hing1_old,hin1;
double bending2_old,bend2,hing2_old,hin2;
double bending3_old,bend3,hing3_old,hin3;
double bending4_old,bend4,hing4_old,hin4;
double mach1,mach2,mach3,mach4,mach5,mach6,mach7,mach8;
double alde1,alde2,alde3,alde4,alde5,alde6,alde7,alde8;
double R0_L[3],mf2e[3][3];
double w_fir[3],w_ini[3];//

double Ldot,ldot,rl,rL,axn,ayn,azn,tetamn,phimn,psimn;
int nchoice,tune_flag=0,stage,s_s,flag_hmax=0.;
int m_flag=0,f_flag=0;
double r_flag=0.,stg2;
int dvd;
extern double ren;
double mi2l[3][3],rml[3],vml[3];
//double t_tet[103],teta_c[103],t_z[103],z_c[103],vz_c[103],t_tet5[75],teta_c5[75],t_pos[341],x_pos[341],y_pos[341],v_pos[341],t_gui,Lx_gui,Ly_gui,Lv_gui,Lgama_gui;
//double t_gui,Lx_gui,Ly_gui,Lv_gui,Lgama_gui;
double t_gui[30],Lx_gui[30],Ly_gui[30],Lv_gui[30],Lgama_gui[30];

//double t_tet[200],teta_c[200],t_z[200],z_c[200],vz_c[200],t_pos[529],x_pos[529],y_pos[529],v_pos[529],TburnOut, VburnOut, TetaburnOut;
double t_tet[200],teta_c[200],t_z[200],z_c[200],vz_c[200],Vxz_c[200],t_pos[529],x_pos[529],y_pos[529],v_pos[529];//,TburnOut, VburnOut, TetaburnOut;
double t_psi[200],psi_c[200];
double xcg0,ixx_end,iyy_end,xcg_end,t_stage_2;
double vm,mach,hm,hold,q,du_pitch,du_gama,du_yaw,Hme;
double delta_gama=0.,vm_old=0. ,vm_f_old=0.;
double Ayb_filter=0.,azi_filter=0.,Azb_filter=0.;
double tet,cllda,alfab,cx,cm,k2,cln,cy,cz,cll,cll_t;
double alfa,betta,lambdac,dela,delr,dele,ax,ay,az,wx,wy,wz;
double delac,delrc,delec,delc1,delc2,delc3,delc4;
double phim,phic,tetam,tetac,teta_burn,psim,ayc,ac_pitch,apc;//ac_pitch=apc
double Wv1,Waz1,Wv2,Waz2,Wv3,Waz3,Wv4,Waz4,Wv5,Waz5,Jr_star;
double d1,d2,d3,s1,s2,s3,sinlatm,vdot_0_flag,t_vdot_0 ;
double L_body,D_body,Aref,Ix0,Iy0,Wx_filter,Wy_filter,Wz_filter;
double M_body,M_sfuel,mass,Xt,Yt,Zt,xbc,ixx,iyy,izz,M_total;
double Longt,Latt,Ht,rt,Longl,Latl,Hl,At,Aiming_Angle,Teta_launch,Gama_corec,Teta_launch_rok;
double delt,dtout,t,t_burn,totalt,tterm;
double dynpres,hmax;
double Range,t_guidance,acmin,wmax,finlim,tservo,Teta0;
double teterr, tetraterr,azerr,azraterr,tberr,maserr,threrr,eth1,eth2;
double gbx,gby,gbz,abx,aby,abz,asx,asy,asz,gsx,gsy,gsz;
double meanx,sigmax,meanz,sigmaz,cep,sig[25];
double xerr,zerr;
double y[50],dy[50],prmt[50];
double d0,r0,rf0[3],rtf[3],rtdf[3],wiff[3],wifb[3];
double fab[3],fngb[3],ftb[3],ai[3],ab[3];
double mab[3],mtb[3];
double ri[3],Rii[3],Rif[3],rmf[3];
double _ri[3],_Rii[3],_Rif[3],_rmf[3],_vii[3],_vif[3],_vmf[3];
double vii[3],vif[3],vmf[3],vbwb[3],vbwf[3],wR[3],v;
double mb2i[3][3],mi2b[3][3],mb2f[3][3],_mb2i[3][3],_mi2b[3][3],_mb2f[3][3];
double mi2f[3][3],_mi2f[3][3],mf2b[3][3],mf2i[3][3];
double _qib[4],qib[4],qbi[4],qif[4],Gi[3];
double teta_dot_burn,temp,Temp0,press,dens,sound_speed,rmu,magg;

int burnoutflg1,burnoutflg2=0,tgt_flg=0;
double  t_BurnOut_save,V_BurnOut_save,Teta_BurnOut_save;
//float cmalfa[7][10],cnalfa[7][10],cadelta[7][10],cndelta[7][10],caalfa1[7][10],caalfa2[7][10],clldela[7][10];
//float catotal[7][10],cmqalfa[7][10],cnqalfa[7][10],cmdelta[7][10],clp[7][10],cll_tail[7][10];
//float  AlfaArray[11],MachArray[10], RenArray[10],CxArrayon[11][10][10],CxArrayoff[11][10][10];
//float  Alfa1Array[6],Mach1Array[10], Delta1Array[3],DeltaArray[6][10][3];
//int AlfaDataNo,MachDataNo,RenDataNo,Alfa1DataNo,Mach1DataNo,Delta1DataNo ;




double ca,cmqb,cmqa,cllp;
 double Xtrgt, Ytrgt,Ztrgt,Xm,Ym,Zm,Vxm,Vym,Vzm,Vm_t,Vm_t_2;
double L_x,L_y,L_v,L_gama,V_star;
long int sim_count=-1,sim_count_g=-1,p_count=-1;
double re,A=re=6378140.,B=6356755.0; /* The Earth's equator radius(m).[Krassowski model] */
//double re,A=re=6378137.,B=6356752.3; /* The Earth's equator radius(m).[Krassowski model] */

double wie=7.29211533e-5; /* The Earth's angular velocity ,rad/s */
double J=1.623945e-3; /* The oblateness constant of the earth gravitational
					field . */
double e2=3.352813178e-3; /*  Ellipticity */
double Mu=3.98600513257472e14; /*  mu=f*M=gravitation constant * Earth mass. */
double pi =3.1415926535897932;
//double sqrt2=1.4142135620;
#include "m273n.h"
FILE *fout,*fdiary, *fd,*fcep,*fecep,*fd2,*fd3,*fd4,*m_tet,*m_pos,*m_pos2,*m_gui,*m_z,*m_zc,*m_psi;
FILE *m_tetc,*m_psic,*m_posc,*m_guic,*mfp2,*mfp3,*mfp4,*mfp5,*mfp6,*m_VburnOut,*burnoutvel,*Outfile;
FILE *Ka,*hgv_fd,*wp;
float k_w_acc8[11][13],ki_w_acc8[11][13],k_i_acc8[11][13];

#ifdef __cplusplus
	//double max (double value1, double value2);
	//double max(double value1, double value2)
	//{
	//	return ( (value1 > value2) ? value1 : value2);
	//}
#endif
void Initilize(char *filename1,char *filename2,char *filename,int dim);
void Initilize_sim(void);
void simulate(void);
void Print_plot(void);
void Print_diar(void);
void Print_sim(void);
void Print_cep(void);
void Print_ballistic(void);
void Ballistic_parameter (void);
void Guidance_parameter(void);
void CEP_Calculation(void);
 FILE *f1,*f2,*f3,*f4;
 double dy00[50],y00[50],t0,Teta_save,V_save,rmf_save[2],vmf_save[2];
 int nsim,dim=7,dim0=13,dim1=35,dim2=24,choice,j_cep,num=-1,l_flag;
 char laun_file[80];
 char def0[260]="char.dat";
 char def1[260]="laun.dat";
 char def2[260]="error.dat";
 char defout[]="out.dat\0    ";
 char defd[]="diary.dat\0    ";
 char read0[]="Read Characteristic Data From.\0  ";
 char read1[]="Read Launch Data From.\0  ";
 char read2[]="Read Variance of Errors Data From.\0  ";
 double r2d=180.0/pi;
 double d2r=pi/180.0;
 double M_PI=pi;
 double O_vec[N_to_save];
/********************************Dont Remove***************************************/
//void getch();
void randomize();
void clrscr();
void clrscr()
{
}
void gotoxy(int x,int y);
void gotoxy(int x,int y)
{
}
void randomize()
{
}
//void getch()
//{
//}
int plot_count=0;
void main1(void);
void Plss(double buff[]);
 //double buff_pls[100000000];
 void Initilize(int number);
 int m_launn(double d[],char ch[],int dim);

 double Xer,Zer;
 void show_data(int num1);
 int an=0,tf=0,Place=0,Place2=0;

//using namespace simulator_2;
//auto sim3 = gcnew MyForm1();
 ////////////
 double du_roll;
 float k_w_acc2[11][13],ki_w_acc2[11][13],k_i_acc2[11][13];
 float k_w_acc3[143],ki_w_acc3[143],k_i_acc3[143];
 //float k_w_acc4[143],ki_w_acc4[143],k_i_acc4[143];
 float k_w_acc_hgv[143],ki_w_acc_hgv[143],k_i_acc_hgv[143];
 double Longtf[wp_max],Lattf[wp_max],Htf[wp_max],Longt0[wp_max],Latt0[wp_max],Ht0[wp_max];
 double Longlf[100],Latlf[100],Hlf[100],Rng0,az0,t2go,Avail_trgt[wp_max],ratio_m[wp_max],R0_t[3];
 //double Latt0[2000], Longt0[2000], Ht0[2000],R0_t[3];
mat3x1 r_l,v_l,rt_l,rtn,rte,NED_Angles,Ab,Wb,R0_E;
 mat3x3 mAz2n,me2n,C_b_n,mib,C_e_l;
 mat4x1 q_b_n,q_b_n_ins;
 
 double alfa_g,beta_g,alfab_g,Ll,Lonl,Vxy,Vxz;
 double Ffi,Flam,Fh,FvN,FvE,FvD,Ifi,Ilam,Ih,IvN,IvE,IvD;
 /////////////////////////
 double phin,tetan,psin,psic;
 ///////// only for OC
 int ConvFlag=1;
 char GPS_ACTIVE=3;
 unsigned char Exteraflg=0;
 unsigned char CheckInput_flg=0xFF;//@!@
 int destruct_flg=0,out_cnt=0,ct=0,cnt1=0;//@!@
 unsigned char CWarHead_Cmd=127;

 void SetDriverBoxCmd(unsigned char cmdMode)
{
}
 //void WarHead(WarHeadCmnd Cmnd)
 void WarHead(unsigned char Cmnd)
{
 }
// #pragma once
// enum CmdDriverBox
//{
//	SEP_Save_On          =1,
//	SEP_Save_Off         =2,
//	SEP_Cmd_On           =3,
//	SEP_Cmd_Off          =4,
//	PAYM_Save_On         =5,
//	PAYM_Save_Off        =6,
//	PAYM_Cmd_On          =7,
//	PAYM_Cmd_Off         =8,
//	igni_Save_On         =9,
//	igni_Save_Off        =10,
//	igni_Cmd_On          =11,
//	igni_Cmd_Off         =12
//	
//};
/// tele
const int rows=23000,col=8;
float tele_snsr[rows][col],
	tele_31010[rows][30],
	tele_3101[rows][19],
	tele_3103[rows][19],
	tele_3108[rows][19],
	tele_3106[rows][19],
	tele_3107[rows][20];

int tele_sns_sz=rows,tele_50hz_sz=rows/2,tele_5hz_sz=rows/20;//
