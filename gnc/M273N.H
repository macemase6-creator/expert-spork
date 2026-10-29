//#include"MyForm1.h"

#include"string.h"
#include "stdio.h"
#include "aero_tables.h"   /* AERO_R / AERO_C / AERO_K */
#include <dos.h>
#include <io.h>
#include <stdlib.h>
#include <conio.h>
#include "math.h"
#include "MAT_DEF.H"

#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#define	SGN(A)   	((A) < 0   ? (-1): (1))
#define 	ABS(A)          ((A) > 0   ? (A) :(-A))
#define	LIMIT(A, B)	((ABS(A)) > (B) ? (SGN(A)*B) : (A))
#define	SAT(A,MN,MX)	((MN)<(MX)?((A) > (MX) ? (MX) : (A) < (MN) ? (MN) : (A)):(A))


#define N_to_save 176//91

#define  EARTH_RADIUS_A  6378137.0
#define  EARTH_RADIUS_B  6356752.3
#define PI 3.1415926536

#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#define	MIN(A, B)	((A) < (B) ? (A) : (B))

//#define wp_max 300//2000
#define wp_max 500//2000
#define  Ts				0.01             //Sampling Time
//#########################################################
//###################### missile ##########################
extern char *FoP;

extern float cmalfam[AERO_R][AERO_C], cnalfam[AERO_R][AERO_C], cmqalfam[AERO_R][AERO_C], cnqalfam[AERO_R][AERO_C],
    clpm[AERO_R][AERO_C];
extern float cmdeltam[AERO_R][AERO_C], cndeltam[AERO_R][AERO_C], cadeltam[AERO_R][AERO_C], catotalm[AERO_R][AERO_C],
    caalfam[AERO_R][AERO_C];
// roll
extern float AlfaArrayrm[AERO_R], MachArrayrm[AERO_C], DelaArrayrm[AERO_K],
    DeltaArrayrm[AERO_R][AERO_C][AERO_K];
extern int AlfaDataNorm, MachDataNorm, DelaDataNorm;
// drag
extern float AlfaArraycm[AERO_R], MachArraycm[AERO_C], RenArraycm[AERO_K],
    CxArrayoncm[AERO_R][AERO_C][AERO_K], CxArrayofcm[AERO_R][AERO_C][AERO_K];
extern int AlfaDataNocm, MachDataNocm, RenDataNocm;
// #########################################################
// ###################### warhead ##########################
extern float cmalfaw[AERO_R][AERO_C], cnalfaw[AERO_R][AERO_C], cmqalfaw[AERO_R][AERO_C], cnqalfaw[AERO_R][AERO_C],
    clpw[AERO_R][AERO_C];
extern float cmdelta[AERO_R][AERO_C], cndeltaw[AERO_R][AERO_C], cadeltaw[AERO_R][AERO_C], catotalw[AERO_R][AERO_C];
extern float cH_d[AERO_R][AERO_C], cB_d[AERO_R][AERO_C], cH_a[AERO_R][AERO_C], cB_a[AERO_R][AERO_C];
// roll
extern float AlfaArrayrw[AERO_R], MachArrayrw[AERO_C], DelaArrayrw[AERO_K],
    DeltaArrayrw[AERO_R][AERO_C][AERO_K];
extern int AlfaDataNorw, MachDataNorw, DelaDataNorw;
// drag
extern float AlfaArraycw[AERO_R], MachArraycw[AERO_C], RenArraycw[AERO_K],
    CxArrayoncw[AERO_R][AERO_C][AERO_K];
extern int AlfaDataNocw, MachDataNocw, RenDataNocw;

extern float AlfaArraycwf[AERO_R], MachArraycwf[AERO_C], RenArraycwf[AERO_K],
    CxArrayofcw[AERO_R][AERO_C][AERO_K];
extern int AlfaDataNocwf, MachDataNocwf, RenDataNocwf;

// #########################################################
// ###################### warhead2 ##########################
extern float cmalfaw2[AERO_R][AERO_C], cnalfaw2[AERO_R][AERO_C], cmqalfaw2[AERO_R][AERO_C], cnqalfaw2[AERO_R][AERO_C],clpw2[AERO_R][AERO_C];
extern float cmdeltaw2[AERO_R][AERO_C], cndeltaw2[AERO_R][AERO_C], cadeltaw2[AERO_R][AERO_C], catotalw2[AERO_R][AERO_C];
extern float cmalfaw2_[AERO_R][AERO_C], cnalfaw2_[AERO_R][AERO_C], cmqalfaw2_[AERO_R][AERO_C], cnqalfaw2_[AERO_R][AERO_C];
// drag
extern float CxArrayofcw2[AERO_R][AERO_C][AERO_K];


// #########################################################
//###################### motor ##########################
//double re,wie,J,e,Mu,pi,A,B ;
extern float cmalfamo[AERO_R][AERO_C],cnalfamo[AERO_R][AERO_C],cmqalfamo[AERO_R][AERO_C],cnqalfamo[AERO_R][AERO_C],clpmo[AERO_R][AERO_C];
extern float cmdeltamo[AERO_R][AERO_C],cndeltamo[AERO_R][AERO_C],cadeltamo[AERO_R][AERO_C],catotalmo[AERO_R][AERO_C],clldelamo[AERO_R][AERO_C];
// roll
//extern float  AlfaArrayrmo[],MachArrayrmo[],DelaArrayrmo[],DeltaArrayrmo[][13][3];
//extern int AlfaDataNormo,MachDataNormo,DelaDataNormo;
// drag
extern float  AlfaArraycmo[AERO_R],MachArraycmo[AERO_C],RenArraycmo[AERO_K],CxArrayoncmo[AERO_R][AERO_C][AERO_K],CxArrayofcmo[AERO_R][AERO_C][AERO_K];
extern int AlfaDataNocmo,MachDataNocmo,RenDataNocmo;
//#########################################################
//motor
//extern float huge cmalfam[6][12],cnalfam[6][12],cmqalfam[6][12],cnqalfam[6][12],clpm[6][12],cmdeltam[6][12],cndeltam[6][12],cadeltam[6][12],cldeltam[6][12];
//####################################################

extern double R0_L[3],mf2e[3][3];
void MATRIXf2e();
extern float MDOT[404];
extern float CB[13][16],CH[13][16];

//extern double vmf_t[],d_fire[];
extern double cb_1,ch_1,bending1,hing1,cb_2,ch_2,bending2,hing2,cb_3,ch_3,bending3,hing3,cb_4,ch_4,bending4,hing4;
extern double bending1_old,bend1,hing1_old,hin1;
extern double bending2_old,bend2,hing2_old,hin2;
extern double bending3_old,bend3,hing3_old,hin3;
extern double bending4_old,bend4,hing4_old,hin4;
extern double mach1,mach2,mach3,mach4,mach5,mach6,mach7,mach8;
extern double alde1,alde2,alde3,alde4,alde5,alde6,alde7,alde8;

extern double mf2s[3][3],mb2n[3][3],wib[3],wibb[3],win[3],winn[3],wnb[3],rni[3];
extern double Ldot,ldot,rl,rL,axn,ayn,azn,tetamn,phimn,psimn;
extern double w_fir[3],w_ini[3],stg2;
extern int m_flag,f_flag;
extern int dvd;
extern double delx,dely,delv;
//extern double t_tet[103],teta_c[103],t_z[103],z_c[103],vz_c[103],t_tet5[75],teta_c5[75],t_pos[341],x_pos[341],y_pos[341],v_pos[341],t_gui,Lx_gui,Ly_gui,Lv_gui,Lgama_gui;
//extern double t_gui,Lx_gui,Ly_gui,Lv_gui,Lgama_gui;
extern double t_gui[],Lx_gui[],Ly_gui[],Lv_gui[],Lgama_gui[];

extern double t_tet[],teta_c[],t_z[],z_c[],vz_c[],Vxz_c[],t_pos[],x_pos[],y_pos[],v_pos[],TburnOut, VburnOut, TetaburnOut;
extern double t_psi[],psi_c[];

extern double  ixx_end,iyy_end,xcg_end,xcg0,flg,t_stage_2;
extern double Ayb_filter,azi_filter,Teta_launch_rok,Azb_filter;
extern double delta_gama,vm_old,vm_f_old,cln,cy,cz,cll,cll_t,dxcg0;
extern double gamam,gamac,lambdac,du_gama,Temp0,cllda,alfab,cx,cm,xcg_end,xcg_endo;
extern int    nchoice ,tune_flag,stage,s_s,l_flag1,flag_hmax,l_flag,choice,laun_flag,teta_flag,launch2;
extern double Wv1,Waz1,Wv2,Waz2,Wv3,Waz3,Wv4,Waz4,Wv5,Waz5,Jr_star;
extern double d1,d2,d3,s1,s2,s3,sinlatm,vdot_0_flag,t_vdot_0,V_save,rmf_save[2],vmf_save[2] ;
extern double L_body,D_body,Aref,Ix0,Iy0,xbc,ixx,iyy,izz;
extern double M_body,M_sfuel,mass,M_total;
extern double phim,phic,tetam,teta_burn,psim,ayc,apc,ac_pitch,dynpres;//apc=ac_pitch
extern double vm,mach,hm,hold,q,du_pitch,du_yaw,Hme;
extern double delc1,delc2,delc3,delc4,delac,delrc,delec;
extern double Longt,Latt,Ht,Longl,Latl,Hl,At,Aiming_Angle,Teta_launch,Gama_corec;
extern double delt,dtout,t,t_burn,t_guidance,totalt,tterm;
extern double Range,acmin,wmax,finlim,tservo,hmax;
extern double teterr, tetraterr,azerr,azraterr,tberr,maserr,threrr,eth1,eth2,Xt,Yt,Zt;
extern double gbx,gby,gbz,abx,aby,abz,asx,asy,asz,gsx,gsy,gsz;
extern double meanx,sigmax,meanz,sigmaz,cep,sig[25];
extern double xerr,zerr;
extern double Teta0,tetac,alfa,betta,gama,dela,delr,dele,ax,ay,az,wx,wy,wz;
extern double r2d,d2r,mx2,L_x,L_y,L_v,L_gama,V_star,Wx_filter,Wy_filter,Wz_filter;
extern long  int sim_count,sim_count_g,p_count;
extern double re,wie,J,e2,Mu,pi,A,B ;
extern double y[50],dy[50],prmt[50];
extern double d0,r0,rf0[3],rtf[3],wiff[3],wifb[3];
extern double fab[3],fngb[3],ftb[3],ai[3],ab[3],mab[3],mtb[3];
extern double ri[3],Rii[3],Rif[3],rmf[3];
extern double _ri[3],_Rii[3],_Rif[3],_rmf[3],_vii[3],_vif[3],_vmf[3];
extern double vii[3],vif[3],vmf[3],vbwb[3],vbwf[3],wR[3],v;
extern double mb2i[3][3],mi2b[3][3],mb2f[3][3],_mb2i[3][3],_mi2b[3][3],_mb2f[3][3];
extern double mi2f[3][3],_mi2f[3][3],mf2b[3][3],mf2i[3][3];
extern double mi2l[3][3],rml[3],vml[3];
extern double _qib[4],qib[4],qbi[4],qif[4],Gi[3];
extern double teta_dot_burn,temp,press,dens,sound_speed,rmu,magg;
extern double ca,cmqb,cmqa,cllp,cma,cna;
extern double Xtrgt, Ytrgt,Ztrgt,Xm,Ym,Zm,Vxm,Vym,Vzm,Vm_t,Vm_t_2;
extern int burnoutflg1,burnoutflg2,sepflg,tgt_flg,motor_2_flg;
extern double    t_BurnOut_save,V_BurnOut_save,Teta_BurnOut_save;
//extern float cmalfa[7][10],cnalfa[7][10],cadelta[7][10],cndelta[7][10],caalfa1[7][10],caalfa2[7][10],clldela[7][10];
//extern float catotal[7][10],cmqalfa[7][10],cnqalfa[7][10],cmdelta[7][10],clp[7][10],cll_tail[7][10];
//extern float  AlfaArray[11],MachArray[10], RenArray[10],CxArrayon[11][10][10],CxArrayoff[11][10][10];
//extern float  Alfa1Array[6],Mach1Array[10], Delta1Array[3],DeltaArray[6][10][3];
//extern int Alfa1DataNo,Mach1DataNo,Delta1DataNo ;
//extern int AlfaDataNo,MachDataNo,RenDataNo ;





//extern float  AlfaArray[11],MachArray[10], RenArray[10],CxArrayon[11][10][10],CxArrayoff[11][10][10];
//extern float  Alfa1Array[6],Mach1Array[10], Delta1Array[3],DeltaArray[6][10][3];
//extern int Alfa1DataNo,Mach1DataNo,Delta1DataNo ;
//extern int AlfaDataNo,MachDataNo,RenDataNo ;
extern double K_w_pitch00,K_p_integ_teta00,K_i_integ_teta00,K_w_yaw00,K_i_integ_acc00;
extern double K_p_integ_acc00,K_w_roll00,K_p_integ_roll00,K_i_integ_roll00,t_q_p00;
extern double K_w_pitch11,K_i_integ_teta11,K_p_integ_teta11,K_w_yaw11,K_i_integ_acc11;
extern double K_p_integ_acc11,K_w_roll11,K_p_integ_roll11,K_i_integ_roll11,k_ramp;
extern double K_w_pitch22,K_i_integ_teta22,K_p_integ_teta22,K_p_integ_teta0,K_w_yaw22;
extern double K_i_integ_acc22,K_p_integ_acc22,K_w_roll22,K_p_integ_roll22,K_i_integ_roll22;
extern double t_q_p1,t_q_p0,K_w_pitch33,K_p_integ_teta2,K_i_integ_acc33,K_p_integ_acc33,K_w_roll33;
extern double K_p_integ_roll33,K_i_integ_roll33,t_q_p2,K_w_pitch2,K_w_roll44,K_p_integ_roll44;
extern double K_i_integ_roll44,K_w_pitch44,t_q_p3,t_q_p4,K_w_roll55,K_p_integ_roll55;
extern double K_i_integ_roll55,K_w_pitch55,t_q_p5,acc_sat;//,t_q_p6,K_w_roll66,K_p_integ_roll66;
extern double K_p_integ_roll, K_i_integ_roll,K_p_integ_teta,K_p_integ_teta1,K_i_integ_teta;
extern double K_w_psi00,K_p_integ_psi00,K_i_integ_psi00,K_w_psi22,K_p_integ_psi22,K_i_integ_psi22;
extern double K_w_psi11,K_p_integ_psi11,K_i_integ_psi11;
/*---------------------------------Functions -----------------------------*/
extern FILE *fout,*fdiary,*fd,*fecep,*fd2,*fd3,*fd4,*m_tet,*m_pos,*m_pos2,*m_gui,*m_z,*m_zc,*m_psi;
extern FILE *m_tetc,*m_psic,*m_posc,*m_guic,*mfp2,*mfp3,*mfp4,*mfp5,*mfp6,*m_VburnOut,*burnoutvel,*Outfile;
extern FILE *Ka;
extern FILE *f1,*hgv_fd,*wp;
extern float k_w_acc8[11][13],ki_w_acc8[11][13],k_i_acc8[11][13];

void amar(int n,double xx[],double *mean,double *sigma);
double arcsin(double xx);
void atmosp(double h,double *temp,double *press,double *dens,double *sound,double *rmu);
void azimuth(double h1,double phi1,double lon1,double h2,double phi2,double lon2,double *paz,double *prange);
void cross(double a[3],double b[3],double acb[3]);
void drkg4(int ndim,void (*fun)(void),int (*out)(void));
void dyn_press(void);
void e2q(double teta,double psi,double phi,double q[4]);
void err(int ierr);
void e2m(double psi,double teta,double phi,double m[3][3]);
void e2q(double teta,double psi,double phi,double q[4]);
void m2q(double C[3][3],double q[4]);
void f110a_pilot(void);
void f6df110a(void);
void f110a_thrust(double ftb[3],double mtb[3]);
void Gama_correction(void);
void Gama_correction2(void);
void Gacc(double ri[],double Gi[]);
void gaus(double mean,double sigma,double *x);
void get_data(int ndim,char *filename,double data[]);
void Initat(int season,double lat,double hl,double temp0);
void vreckon(double lat,double lon,double dist_m,double bearing_rad, double points[3]);
 void vreckon(double lat1,double lon1,double dist_m,double bearing_rad, double *lat2,double *lon2);
void traject(double lat1, double long1, double alt1, double Az, double R, double x_er, double z_er,double *lat22, double *long22);
void Ft_Convert(double Launch_lat,double Launch_lon, double L_virtual,double Aztv,double total_t,double *lat2,double *lon2);

double  limit(double var , double max ,double min);
double mag(int num,double r[]);
void multmat(double a[3][3],double b[3][3],double ab[3][3]);
void M_char(double *xbc,double *mass,double *ixx,double *iyy,double *izz);
void M_aero(double sound_speed,double dens,double vbwb[3],double wibb[3],double fab[3],double mab[3]);
double max (double value1, double value2);
double M_teta_command(double t);
double M_teta_dot_command(double t,double ax,double vm);
double M_acc_yaw_command(double d_fire[3],double vmf_t[3]);
double M_acc_yaw_command_0(double _rmf[3],double _vmf[3]);
double M_acc_pitch_command(double _rmf[3],double _vmf[3]);
void M_attitude(double mi2b[3][3],double *phim,double *tetam,double *psim);
int M_stage(void);
int  of110a(void);
int  popup(char *matn[],char *keys,int count,int x,int y);
void quaternion(double wx,double wy,double wz,double _qib[4]);
void qi2f(double qif[4]);
void q1q2(double q1[4],double q2[4],double q12[4]);
void q2m(double q[4],double m[3][3]);
void readfile(char piam[],char def[],int dim,char ans[],double data[],int mod);
int sign(float sg);
void twm(int mod,double m[3][3],double v[3],double mv[3]);
double teta_limit(double q);
void Transe2(void) ;
void wind(void);
void Telemetry();
void AutoPilot(char mod);
int ReadFile3Drm(char *filename, float ax[AERO_R], float ay[AERO_C], float az[AERO_K],
    float f_xyz[AERO_R][AERO_C][AERO_K], int *nx, int *ny, int *nz);
int ReadFile3Dcm(char *filename, float ax[AERO_R], float ay[AERO_C], float az[AERO_K],
    float f_xyz[AERO_R][AERO_C][AERO_K], int *nx, int *ny, int *nz);
int ReadFile3Drw(char *filename, float ax[AERO_R], float ay[AERO_C], float az[AERO_K],
    float f_xyz[AERO_R][AERO_C][AERO_K], int *nx, int *ny, int *nz);
int ReadFile3Dcw(char *filename, float ax[AERO_R], float ay[AERO_C], float az[AERO_K],
    float f_xyz[AERO_R][AERO_C][AERO_K], int *nx, int *ny, int *nz);
int ReadFile3Dcwf(char *filename, float ax[AERO_R], float ay[AERO_C], float az[AERO_K],
    float f_xyz[AERO_R][AERO_C][AERO_K], int *nx, int *ny, int *nz);
int ReadFile3D_7x12x5 (char *filename,float ax[AERO_R],float ay[AERO_C],float az[AERO_K]
		,float f_xyz[AERO_R][AERO_C][AERO_K],int *nx,int *ny,int *nz);

int ReadFile3D_6x13x9 (char *filename,float ax [6],float ay [13],float az [9],float f_xyz[6][13][9],int *nx,int *ny,int *nz);
void TVC();
void saveit(double* O_vec);
void saveit2(double* O_vec,FILE *Outfile );
void r2dms(double acc, double R, int* pd, int* pm, double* ps);
void MakeZero();
void MakeZero3();

void nav2cart(double r_n[3],double r_e[3]);
void nav2cart(double lat,double lon,double h,double r_e[3]);
void nav2local(double r_n[3],double r_l0[3],double r_l[3]);
void nav2local(double lat,double lon,double h,double r_l0[3],double r_l[3]);

double look2_gw2(double _hm, double _Mach,double H_tab[],double Mach_tab[],double * gainsMat,unsigned int NU_H_M[],unsigned int stride);
///////////////
void Guidance(void);
 void PilotGain(void) ;
 void Outputs(void);
 void f6dm273na(void);
 int om273na(void);
 void m273na_thrust(double ftb[3], double mtb[3]) ;
	void m273na_thrust2(double ftb[3], double mtb[3]) ;

	void MATRIXe2n(double L, double l);
	void MATRIXAz2n(double Az);
	 void NED_guide();
	 void Find_LOS_Ang();
void NED_function();

	double dot(int n, double *a, double *b);
	void  imperr(double rl[3], double rm[3], double ht, double latt,double longt, double erre[3]);
	mat3x1 imperr(mat3x1 rl, mat3x1 rmn, mat3x1 rtn);
	mat3x1 imperr(mat3x1 rl, mat3x1 rmn, double ht, double latt,double longt);
	mat3x1 Arr2Mat(double a,double b,double c);
	void AlfaObtain();

	void NED_function(double Re[3],double rn[3]);
	void Rot_e_n(double phi, double lam,double C_e_n[3][3]);
	double guidance_arc(double wp_lonx,double wp_latx,double wp_alt);
	double guidance_arc2(double wp_lonx,double wp_latx,double wp_alt);
	double M_acc_pitch_alti(double altcom);
	double control_altitude(double altcom);
	double control_heading(double psivdcomx);
	mat3x1 guidance_AGL( mat3x1 STBIK,mat3x1 VTBIK);
	void aerodynamics_der();
	double control_roll(double phicomx);
	double control_normal_accel(double ancomx,double int_step);
	float difdatw11(float mx1[][9],float mx2[][9]);
	float interpolwd(float m, float a, float mx[AERO_R][AERO_C]);
	float interpolw2(float m, float a, float mx[AERO_R][AERO_C]) ;
	void Min_Max( int mod,int no,double inp[],double val[],int idx[]);
	int Check_targets();
	double calc_hit_prob3(mat3x1 pm0,mat3x1 vm0,mat3x1 am0,mat3x1 pt0,mat3x1 vt0,mat3x1 sigma_xyz);
	double calc_hit_prob(mat3x1 pm0,mat3x1 vm0,mat3x1 am0,mat3x1 pt0,mat3x1 vt0,double sigma_cep);
	double  guss_prp(mat3x1 dxyz,mat3x1 sig);
	int ReadWayPoints(const char* filename);
	int ReadWayPoints_2(const char* filename);
	char * cearte_dir(int folderN, char *fileN);
	//int CopyFile(char *sourceFilePath,char *destFolderPath,int mod);
	int CopyFile(int foldername,char *filename,int mod);
	int CopyFile(char *destfoldername,char *sourcefilename,int mod);
	//char * CreateForderPath(int foldername);
	char * CreateForderPath(int Lfoldername,int Tfoldername,int hgvflg);
	char * CearteFilePath( char *filename, char *folderpath);
	char * CearteFoFiPath(int foldername,char *filename);
	char * getFilePath( char *filename, char *folderpath);
	void Array_Min(int no,double inp[],double val,int idx);
	int MinId(int no,double inp[]);
	void PrintOutput2();
	void Init_guid();
///-----------------
	extern int sepflg,motor_flag,burn_flg,gama_flg,sim_no,noise,flg_stage,kkk1;
	extern int pik_q, flag_alfa,Place,Place2;
	extern double t_old1,k_5,M_PI,tsep;
	extern double euler_d[3], sl_hm, defq;
	extern double deth1_2, deth2_2,t_sep;
	extern double TSTHR_2[], STHR_2[], gama_, lambda_, bal_flag, dgama,dgama_m, rng;
	extern double rtf_6[], g_flag, rtf_6[], Latt_6, Longt_6,Ht_6;
	extern int FTrc_PackNum,micro_switch_cnt,micro_switch,BeltCmd,sep_fb,f_belt_cnt,hgv_flg,hgv_flg2;
	extern double rmn_[],vmn_[];
	extern mat3x1 rmf_rtf,vmf_vtf;
/********************************Dont Remove********************/
    extern	int isthr,isthr_2;
	extern	int plot_count,num;
	//extern double buff_pls[100000000];
	void Plss(double buff[] );
	void print_F();
	extern void main1(void);
	extern int an,tf; 
	extern void Initilize(int number);
	extern void gotoxy(int x,int y);
//    extern void getch();
    extern void randomize();
    extern void clrscr();
	void print_SIM();
	extern void show_data(int num1);
	extern double Xer,Zer;
	extern double O_vec[],rtf_1[],rtf_2[],rtf_3[];

	/////////
extern double du_roll,Slant_Range,FRange,tgo,tgo_norm,hgv_rng,err_motor,tet_cmps2,dRange;
extern int homing_phase,flg_alt_hold,flg_vc,cg_flg,TrgtCnt;
extern int burn_flg2,flgb_5,rocket;
extern float k_w_acc2[11][13],ki_w_acc2[11][13],k_i_acc2[11][13];
extern float k_w_acc3[],ki_w_acc3[],k_i_acc3[];
//extern float k_w_acc4[],ki_w_acc4[],k_i_acc4[];
extern float k_w_acc_hgv[],ki_w_acc_hgv[],k_i_acc_hgv[];
extern double Longtf[],Lattf[],Htf[],Longt0[],Latt0[],Ht0[],Longlf[],Latlf[],Hlf[],Rng0,az0,t2go,Avail_trgt[],R0_t[],ratio_m[];
//extern double Latt0[], Longt0[], Ht0[];
extern int  PointsCnt,ctrlNo,ign2_cmd;
extern int 	  flg_for,flg_for2,flg_ms1,i_s2,i_sz2;
///////////////////////////////////
extern double alfa_g,beta_g,alfab_g,Ll,Lonl,t_BeltCmd,t_ign2_cmd,t_burn1,t_burn2,Vxy,alfa_xy,t_burn_2;
extern double Ffi,Flam,Fh,FvN,FvE,FvD,Ifi,Ilam,Ih,IvN,IvE,IvD;

extern mat3x1 r_l,rt_l,vmn,rmn,rtn,rte,rme,rme_rte,rmn_rtn,rtaz,vmaz,NED_Angles,Ab,Wb,R0_E;
extern mat3x3 mAz2n,me2n,C_b_n,mib,C_e_l;
extern mat4x1 q_b_n,q_b_n_ins;
extern double phin,tetan,psin;
///////////
extern int sepflg2;
extern double Del_tm, Del_t;
void Sep_func();
/////////// only on sim
extern int ss_6,flgm_5,flg_stage,flg_q;
extern float t_dive;
extern double t_ctrl,tt,t_2,t_vc,psic;
extern int vb, bv,Avail_trgt_c,ctrlNo,thr_flg,thr_flg2;
extern int ign2_5,ign2_fb;
///////// ----------------------- only for OC
extern int ConvFlag,destruct_flg;
extern unsigned char Exteraflg;
unsigned int de2di(double t_t);
void OutVolt(int chnl, float volt);
void SetDriverBoxCmd(unsigned char cmdMode);
//extern int BeltCmd;
extern unsigned char CheckInput_flg;
extern unsigned char CWarHead_Cmd;
void WarHead(unsigned char WarHeadCmnd );

//unsigned int de2di(double t_t)
// {
// }

//// tele
void input_tele(char* s,int col,int rows,float tele_out[][8]);
void input_tele19(char* s,int col,int rows,float tele_out[][19]);
double tele_sns_interpol(int m,double tt,float mx[][8]);
double tele_19_interpol(int m,double tt,float mx[][19]);

extern const int rows,col;
extern float tele_snsr[][8],
	tele_31010[][30],
	tele_3101[][19],
	tele_3103[][19],
	tele_3108[][19],
	tele_3106[][19],
	tele_3107[][20];
extern int tele_sns_sz,tele_50hz_sz,tele_5hz_sz,oi_flg,psip_flg,psip_flg2;//
extern double tt_tele,tt_tele0,t_ign2,ign2_dely,tff,khr;
;

extern int StopCtrlCmd,StopCtrlCmd_2,PushCmd,aero_flg,psic_flg,var_idx,traject_up;
extern int multi_trgts,gamac_flg,stg6_guid,reentry_ign;
extern double t_StopCtrlCmd,t_PushCmd,GGdt,t_psi0,psi_0,t_ramp_gama,MaxR_stg1;
extern int motor1_state,motor2_state,ign2_stg,guid_flg,ign2_ok5,ign2_ok,MinMaxRng_flg;