#include"stdafx.h"
#include "M273NH.H"
#include <stdio.h>
#include <float.h>
#include "aero_tables.h"
#include "aplt.h"
//#include "matric.h"
/* ---------------------------   Main   Program ------------------------- */
FILE *mfp1, *mfp_vah2, *sl_1, *sl_2,*f5,*f6,*lan,*trgt; // ,*popof;
mat3x1 erre0;
// ########################
//double STHR[403], TSTHR[403], angle_tvc_jad;
double STHR[10000], TSTHR[10000], angle_tvc_jad,tgo_norm;
double TSTHR_2[1000], STHR_2[1000],TbS_2, t_burn_2;
double deth1_2 = 0, deth2_2 = 0, sl_hm;
double g_flag = 0, bal_flag = 0, rtf_6[3], Latt_6, Longt_6, Ht_6,rtf_1[3],rtf_2[3],rtf_3[3];

extern double m_fuel,m_fuel2;
double points[3],points0[3],points1[3],points2[3],points_6[3];
//double TSTHR_2[99], STHR_2[99];
extern double t_ay1, t_ay2, t_az1, t_az2, t_q1, t_q2, ay_m1, ay_m2, az_m1,
    az_m2, q_m1, q_m2;
extern double h_f[], b_f[];
extern double t_ax1, t_ax2, t_qa1, t_qa2, ax_m1, ax_m2, qa_m1, qa_m2,
    t_SX, t_SZ;
extern double wx_max, wy_max, wz_max, xer, zer;
extern double eth1_2, eth2_2, Yt_2, Zt_2, Xt_2, xbc_y_2, xbc_z_2, tberr_2,threrr_2;
extern double eth1_2, eth2_2, Yt_2, Zt_2, Xt_2, xbc_y_2, xbc_z_2, tberr_2,threrr_2;

double er_lat, er_long;
double del_off[4];
int num_sim,Avail_trgt_c=0,input_tele_f=0,read_aero_flg=0,read_coord_flg=0;
int loaded_aero_tal_flag=-1;

/* Status exported to the MEX and CLI wrappers.  It reports the latest
   nchoice==3 profile-generation attempt without changing its physics. */
int trajectory_generation_success = 0;
int trajectory_generation_iterations = 0;
double trajectory_generation_xerr_m = 0.0;
double trajectory_generation_zerr_m = 0.0;

/* Single source of truth for the Teta0 additions used by flight simulation
   (nchoice==5).  The CLI also calls this function when reporting the value
   that must be sent to the FCU. */
double Teta0AfterNchoice5Additions(double raw_teta0, int hgv2)
{
   if (hgv2 != 1) return raw_teta0;
   if (raw_teta0 < 52.5) return raw_teta0 + 2.2;
   if (raw_teta0 < 54.0) return raw_teta0 + 2.8 / 1.5;
   if (raw_teta0 < 56.7) return raw_teta0 + 3.2 / 1.7;
   if (raw_teta0 < 58.0) return raw_teta0 + 3.8 / 1.2;
   if (raw_teta0 < 60.0) return raw_teta0 + 4.8 / 1.1;
   if (raw_teta0 < 63.0) return raw_teta0 + 6.0 / 1.1;
   return raw_teta0 + 6.2 / 1.27;
}

extern double ecef_jm[3],erre[];
extern double vn, ve, vd, La, lo, h;
int dtt;
double d_j = 0.5;//1.;//0.5;//12.;

double M_sfuel0, Isp, TotalImpulse0, Temp_static, Mdot_base, t_burn_base;
double M_sfuel_Fire, Total_Imp,Total_Imp_1;
double t_burn0, Mdot_fuel, Isp_Fire, K_rate, var_Temp0, xbc_er;

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
int freq = 1;
int FTHR = 0; // 1:normal,2:hot,3:cold

double rng = 1; // 0:min  1:max
double flg =3; // 1:up 2:down 3:normal
double K_test = 1.;
double xbc_m = -0.0;
double xbc_w = 0.0;

int motor_flag = 0.;


double wv0, waz0, xbc_y, xbc_z, jerm,Az0;
double t_s4 = 0.;
double m_old, h_old;
int Aim = 0,mk = 0, wp_cnt=0,wp_i;//1;
// ######## launcher ###################
int launch_mod = 1; // 0 : double  1 : mono
double l_rail = 4.5;//7.5;// 4.8; // 10.4;//11.3;//11.3;
int launch2 = 0; // 0:normal  1:tipoff:up  2:tipoff:down
double dtl;
// #####################################
double delta_r = 100.,delta_v = 10.;
int pak_count = 0, fr = 0;
double TbStatic; // =36.;//37.9;
double pak = 1., qaz = 1.;

double Total_Imp2,Total_Imp_2, Rng_err;

int TrgtCnt,launp,PointsCnt,res_r,MinIdx[2];//13;//5;
int  launp_no=28,wp_no0 = 194;//409;//64;//13;//12;//12;
//---------------------------------------
double Teta_launch_roket =76;//64.5;
int rocket = 0;

double tfinal =1000.;//1000.;


int cg_flg = 2;	//1: fattah cg	2: our design cg

int noise=0;//4;
int sim_no = 0*6;//6;

//------
int launp0 =1 ,TrgtCnt0=3;
double max_z=35e3;
//------------

double Range0 = 450e3 ;//,	bearing=(329.)*d2r ;

int choose_point=0;	//1: choose launch point base on Range0	2: choose targe	base on Range0	0 : depends on launp & TrgtCnt
//-----------------------------------
//int ctrlNo = 1;	//1: multi-Trgts	4:one trgt	5: multi-Trgts moh
//----------------------
//double khr=1;
double tt_tele=22.4;//36.72+100.;//140.;//50.;//36.72+0*100;//43.;//34.81;//43;//34.81;
int f_100 =0; // 0:normal 1:tele 100Hz

int hgv_flg = 1;
int hgv_flg2 = 1;


int guid_flg=0,ign2_stg=2;
int MinMaxRng_flg=0;
int thr_flg = 0;	//0: modified static data	1: t9 tele  2:low_burn_rate  3:baramel_data   4:binALI_DATA 
int thr_flg2=0;      // 0:yemn_basala  1:binali_basala
int aero_flg = 1;
int WH_sep = 0;
int psic_flg =0;
int gamac_flg = 0;
int multi_trgts = 0;
int stg6_guid=0;  // 0 => LOS 	1 => PN
int reentry_ign=0;// 0 => No ignition in stg6	1=> stg6 ignition
int traject_up=0;// 0 => No rising the trajectoy	1=> rising the trajectoy

double MaxR_stg1=1050e3;
//----------------------------------------------
void Initilize_sim(void)
{
#ifdef SIM_CLI_HOOK
   fprintf(stderr, "[sim602_cli] init: begin\n");
#endif

    double /*Az0,*/ data[40], Csd, Xsd;
   int Season_flag, i, j, cnt0=0;;
   char piam[80], s[30], ans[80];

   /* tal_flag: set the airframe BEFORE anything reads a mass constant.
      Initilize_sim() runs once per run (main1 case 3/4/5/6), so this tracks
      the flag across repeated MEX calls in one MATLAB session. */
   TalApplyMass();


   if(rocket)	{noise=0;	sim_no=1;}

    if(nchoice==5 )	
  {
	if (noise==0)		{flg = 3;		K_test=1.;}
	else if (noise<4)	{flg = noise;	K_test=0.75;}
	else if(noise<7)	{flg = noise-3;	K_test=1.3;}
	
	//{flg = 1;	K_test=0.6;}

  }//*/


    if(nchoice==5 || hgv_flg2 ==0 )	 hgv_rng = 0;
	if(f_100==1 && input_tele_f==0)
	{
		/*
		input_tele( "31011_t6",col,rows,tele_snsr);
		input_tele19("3101_t6",19,rows,tele_3101);
		input_tele19("3106_t6",19,rows,tele_3106);
		input_tele19("3103_t6",19,rows,tele_3103);//*/
		//input_tele( "31011_t25",col,rows,tele_snsr);
		input_tele( "31011_ghdt2",col,rows,tele_snsr);

		//input_tele( "31011_t24",col,rows,tele_snsr);
		//input_tele19("3101_t24",19,rows,tele_3101);
		//input_tele19("3106_t15",19,rows,tele_3106);
		//input_tele19("3103_t15",19,rows,tele_3103);
		/*input_tele19("3106_t5",19,rows,tele_3106);
		input_tele20("3107_t5",20,rows,tele_3107);
		input_tele30("31010_t5",30,rows,tele_31010);//*/
		
		//t_burn =34.8;
		input_tele_f=1;
	}

//###########################################
	/* Reload when TalFlag changes in the same MEX/CLI process.  The baseline
	   and stretched airframes intentionally use independent aero datasets. */
	if(read_aero_flg==0 || loaded_aero_tal_flag != sim_tal_flag)
	{
  //###################### thrust file #########################

	
		/* M480 is the stage-1 vehicle for both legacy TalFlag paths. */
		isthr = 0;
		mfp1 = sim_fopen("M480_STHR_S1.dat", "r");
	//{isthr = 708;	mfp1 = sim_fopen("stc_t9.dat", "r");}//1300b local
	//if (thr_flg==1){isthr = 708;	mfp1 = sim_fopen("sthrv3_t9.dat", "r");}//1300b local

		while (isthr < 10000 && fscanf(mfp1, "%lf %lf", &TSTHR[isthr], &STHR[isthr]) == 2)
			isthr++;
		t_burn=TbStatic = TSTHR[isthr-2];
		Total_Imp_1=Total_Imp = STHR[isthr-1];
		
   


			//Total_Imp_1 =1*Total_Imp*(m_fuel/2003.8);//fcu//t1

		// The M480 footer already contains the delivered total impulse.

		//t_burn=TbStatic+1.;
		isthr -=2;
		sim_fclose(mfp1);//*/

	// stage 2
      //	if (thr_flg2==0)	{isthr_2 = 193;	mfp_vah2 = sim_fopen("sthr2v2_t7.dat", "r");}//Exit 424
		//{isthr_2 = 436;	mfp_vah2 = sim_fopen("sthr2v1_t9.dat", "r");}//Exit 424
	//else {isthr_2 = 372;	mfp_vah2 = sim_fopen("AT9_2.dat", "r");}//Exit 424
	{isthr_2 = 369;	mfp_vah2 = sim_fopen("STHR_M591_STG2.dat", "r");}//Exit 424
		for (int j= 0; j < isthr_2; j++) fscanf(mfp_vah2, "%lf %lf", &TSTHR_2[j], &STHR_2[j]);
		t_burn_2=TbS_2 =TSTHR_2[isthr_2-2];
		Total_Imp_2=Total_Imp2 = STHR_2[isthr_2-1];
		
		Total_Imp_2 =Total_Imp2*(m_fuel2/100.6);//fcu

		 //Total_Imp_2 =Total_Imp2*(m_fuel2/210.);//fcu
		//Total_Imp_2 =47615.392;	t_burn_2 +=1.;
		isthr_2 -=2;//
		sim_fclose(mfp_vah2);//*/
	
	//###################### aero files #########################
	
	   /* ---- M480 stage-1 tables ----------------------------------------
	    * Generated directly from the supplied Missile DATCOM coefficient files
	    * by tools/generate_m480_data.py. Dimensions and axes come from the file
	    * وتُسجَّل مقرونةً بمؤشّر كل مصفوفة، فتقرأها دوال الاستيفاء وحدها
	    * دون تغيير أي موضع استدعاء.
	    * ------------------------------------------------------------- */
	   {
	      const char *stage1_aero = "aero/m480_stage1_aero.dat";
	      int aerr = aero_load(stage1_aero);
	      aerr |= aero_fill2("cmalfam",  &cmalfam[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cnalfam",  &cnalfam[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cmqalfam", &cmqalfam[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cnqalfam", &cnqalfam[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cmdeltam", &cmdeltam[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cndeltam", &cndeltam[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("clpm",     &clpm[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("catotalm", &catotalm[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("caalfam",  &caalfam[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill3("DeltaArrayrm", &DeltaArrayrm[0][0][0], AERO_R, AERO_C, AERO_K);
	      if (aerr) printf("\n*** aero: %s load/fill FAILED ***\n", stage1_aero);
	   }

	   /* cadeltam مشتقّ حسابياً من catotalm، فيرث محاوره */
	   {
	      const AeroTableInfo *T = aero_axes_for(&catotalm[0][0]);
	      int nca = T ? T->n1 : 7, ncm = T ? T->n2 : 10;
	      for (i = 0; i < nca; i++)	for (j = 0; j < ncm; j++)
	         cadeltam[i][j] = catotalm[i][j] - catotalm[0][j];
	      aero_alias(&cadeltam[0][0], &catotalm[0][0], AERO_C, 0);
	   }

	   /* المحاور التي كان ReadFile3Drm يضعها -- من السجلّ الآن */
	   {
	      const AeroTableInfo *T = aero_axes_for(&DeltaArrayrm[0][0][0]);
	      if (T) {
	         AlfaDataNorm = T->n1;  MachDataNorm = T->n2;  DelaDataNorm = T->n3;
	         for (i = 0; i < T->n1; i++) AlfaArrayrm[i] = T->ax1[i];
	         for (i = 0; i < T->n2; i++) MachArrayrm[i] = T->ax2[i];
	         for (i = 0; i < T->n3; i++) DelaArrayrm[i] = T->ax3[i];
	      }
	   }
	   ReadFile3Dcm("M480_CA_ONS1.dat",
	      AlfaArraycm, MachArraycm, RenArraycm,CxArrayoncm,
	      &AlfaDataNocm, &MachDataNocm, &RenDataNocm);
	   ReadFile3Dcm("M480_CA_OFS1.dat",
	      AlfaArraycm, MachArraycm, RenArraycm,CxArrayofcm,
	      &AlfaDataNocm, &MachDataNocm, &RenDataNocm);

	   // ############ warhead
	  // f2 = fopen("nv_wv12.dat", "r");
	   //f2 = fopen("nv_wv14.dat", "r");//old

	   /* ---- جداول المرحلة الثانية -------------------------------------
	    * تُملأ آلياً من aero/stage2.xlsx، بدل قراءة nv_wv600.dat.
	    * كل كتل ذلك الملف مغطّاة الآن، فحلّ aero_fill2 محلّ قراءته،
	    * و aero_fill3 محلّ ReadFile3Drw.
	    * ------------------------------------------------------------- */
	   {
	      const char *stage2_aero = sim_tal_flag ?
	         "aero/stage2_aero_tal.dat" : "aero/stage2_aero.dat";
	      int aerr = aero_load_add(stage2_aero);
	      aerr |= aero_fill2("cmalfaw",  &cmalfaw[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cnalfaw",  &cnalfaw[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cmqalfaw", &cmqalfaw[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cnqalfaw", &cnqalfaw[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cmdeltaw", &cmdeltaw[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cndeltaw", &cndeltaw[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("clpw",     &clpw[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("catotalw", &catotalw[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cH_d",     &cH_d[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cB_d",     &cB_d[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cH_a",     &cH_a[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cB_a",     &cB_a[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill3("DeltaArrayrw", &DeltaArrayrw[0][0][0], AERO_R, AERO_C, AERO_K);
	      if (aerr) printf("\n*** aero: %s load/fill FAILED ***\n", stage2_aero);
	   }

	   /* cadeltaw مشتقّ من catotalw، فيرث محاوره */
	   {
	      const AeroTableInfo *T = aero_axes_for(&catotalw[0][0]);
	      int ndw = T ? T->n1 : 7, nmw = T ? T->n2 : 12;
	      for (i = 0; i < ndw; i++)	for (j = 0; j < nmw; j++)
	         cadeltaw[i][j] = catotalw[i][j] - catotalw[0][j];
	      aero_alias(&cadeltaw[0][0], &catotalw[0][0], AERO_C, 0);
	   }

	   /* المحاور التي كان ReadFile3Drw يضعها -- من السجلّ الآن */
	   {
	      const AeroTableInfo *T = aero_axes_for(&DeltaArrayrw[0][0][0]);
	      if (T) {
	         AlfaDataNorw = T->n1;  MachDataNorw = T->n2;  DelaDataNorw = T->n3;
	         for (i = 0; i < T->n1; i++) AlfaArrayrw[i] = T->ax1[i];
	         for (i = 0; i < T->n2; i++) MachArrayrw[i] = T->ax2[i];
	         for (i = 0; i < T->n3; i++) DelaArrayrw[i] = T->ax3[i];
	      }
	   }
	   ReadFile3Dcw("nv_won600.dat", AlfaArraycw, MachArraycw, RenArraycw, CxArrayoncw, &AlfaDataNocw, &MachDataNocw, &RenDataNocw);
	  // ReadFile3Dcwf("RAJEH_CA_OFF_S2.dat", AlfaArraycwf, MachArraycwf, RenArraycwf,CxArrayofcw, &AlfaDataNocwf, &MachDataNocwf, &RenDataNocwf);
	  
	  ReadFile3Dcwf(sim_tal_flag ? "CA_OFS2_TAL.dat" : "CA_OFS2.dat",
	      AlfaArraycwf, MachArraycwf, RenArraycwf,CxArrayofcw,
	      &AlfaDataNocwf, &MachDataNocwf, &RenDataNocwf);

     // ############----------------  motor
	   /* ---- جداول المحرّك ---------------------------------------------
	    * من aero/motor.xlsx بدل PMOTOR.dat. ملاحظة: ذلك الملف لا يحوي
	    * محاور، فمحاور motor.xlsx مأخوذة من interpolmo في AERO.CPP --
	    * أي أن الهجرة تسجّل الافتراض القائم لا تكتشف جديداً.
	    * ------------------------------------------------------------- */
	   {
	      int aerr = aero_load_add("aero/motor_aero.dat");
	      aerr |= aero_fill2("cnalfamo",  &cnalfamo[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cndeltamo", &cndeltamo[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cmalfamo",  &cmalfamo[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cmdeltamo", &cmdeltamo[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("catotalmo", &catotalmo[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("clpmo",     &clpmo[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cnqalfamo", &cnqalfamo[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("cmqalfamo", &cmqalfamo[0][0], AERO_R, AERO_C);
	      aerr |= aero_fill2("clldelamo", &clldelamo[0][0], AERO_R, AERO_C);
	      if (aerr) printf("\n*** aero: motor_aero.dat load/fill FAILED ***\n");
	   }

	   /* cadeltamo مشتقّ من catotalmo، فيرث محاوره */
	   {
	      const AeroTableInfo *T = aero_axes_for(&catotalmo[0][0]);
	      int ndo = T ? T->n1 : 6, nmo = T ? T->n2 : 12;
	      for (i = 0; i < ndo; i++)	for (j = 0; j < nmo; j++)
	         cadeltamo[i][j] = catotalmo[i][j] - catotalmo[0][j];
	      aero_alias(&cadeltamo[0][0], &catotalmo[0][0], AERO_C, 0);
	   }
		ReadFile3D_7x12x5("CAMOTOR.dat", AlfaArraycmo, MachArraycmo, RenArraycmo,CxArrayofcmo, &AlfaDataNocmo, &MachDataNocmo, &RenDataNocmo);

	
	
	    //Ka = sim_fopen("k_acc9.dat", "r");
    Ka = sim_fopen("k_acc8.dat", "r");
    fscanf(Ka, "%s", s);
    for (i = 0; i < 11; i++)
		for (j = 0; j < 13; j++)
    fscanf(Ka, "%f", &k_w_acc8[i][j]);
    
	fscanf(Ka, "%s", s);
    for (i = 0; i < 11; i++)
		for (j = 0; j < 13; j++)
    fscanf(Ka, "%f", &ki_w_acc8[i][j]);

    fscanf(Ka, "%s", s);
    for (i = 0; i < 11; i++)
		for (j = 0; j < 13; j++)
    fscanf(Ka, "%f", &k_i_acc8[i][j]);
   
    sim_fclose(Ka);
		read_aero_flg=1;
		loaded_aero_tal_flag=sim_tal_flag;
 }
#ifdef SIM_CLI_HOOK
   fprintf(stderr, "[sim602_cli] init: propulsion and aero loaded\n");
#endif
   /* OUTSIDE the read_aero_flg guard above on purpose: that block runs ONCE
      per process, so Total_Imp_1 must be re-derived on every run or the first
      case of a 49-case session would fix it for all the rest. */
   TalApplyImpulse();
//----------------------------------------
//------------------------------------------------------------
   strcpy(piam, "Enter Characteristic Data Filename. ");
   readfile(piam, def0, dim0, ans, data, 0);

//#######################################

   /* M480 DATCOM reference geometry.  Xt is tail station minus dry CG:
      4.334 - 2.140 = 2.194 m; M_char applies the live CG translation. */
   L_body = 4.334;
   D_body = 0.486;
   Aref = 0.185508;
   Xt = 2.194;

   double Mdot_fuel = data[12];
   strcpy(piam, " Enter Launch  Data Filename.  ");
   readfile(piam, def1, dim1, ans, data, 0);
   strcpy(laun_file, ans);
#ifdef SIM_CLI_HOOK
   fprintf(stderr, "[sim602_cli] init: launch data loaded\n");
   /* Explicit-coordinate requests bypass the catalogue file reader. */
   { extern int cli_use_catalog_coordinates(void);
     if (!cli_use_catalog_coordinates())
     {
        extern void cli_seed_coordinates(void);
        cli_seed_coordinates();
        read_coord_flg = 1;
     }
   }
#elif defined(SIM_MEX_MANUAL_HOOK)
   if (mex_manual_coordinates_enabled())
   {
      mex_manual_seed_coordinates();
      read_coord_flg = 1;
   }
#endif
//printf("\n main 332 \t  \n");
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   double Rng2;

   	//-----------------------------------------------
	double az,R1,MinMaxV[2];
	TrgtCnt=TrgtCnt0;
	launp=launp0;
	PointsCnt = wp_no0;//=280;


//------------------------
	if(read_coord_flg==0)
	{
		trgt = sim_fopen("target3.dat", "r");
		lan = sim_fopen("launchp.dat", "r");
		//f5 = sim_fopen("targets_data.dat", "wt");
		//f6 = sim_fopen("Available_targets.dat", "wt");
		
			
		for (int i = 0; i < launp_no; i++)	//9
		{
			fscanf(lan, "%lf %lf %lf", &Latlf[i], &Longlf[i], &Hlf[i]);
			Latlf[i]  *=d2r;	Longlf[i] *=d2r;
		}

		for (int i = 0; i < PointsCnt; i++)	
		{
			fscanf(trgt, "%lf %lf %lf", &Latt0[i], &Longt0[i], &Ht0[i]);
			Latt0[i]  *=d2r;	Longt0[i] *=d2r;

		}
		
		sim_fclose(trgt);
		//sim_fclose(f5);
		//sim_fclose(f6);
		sim_fclose(lan);
		read_coord_flg=1;
	}
//------------

//t4
	Hlf[launp-1]	=	500;//Hlf[launp-1];
	Latlf[launp-1]	=	15.333333*d2r;//Latlf[launp-1];//Hl;
	Longlf[launp-1]	= 	44.183333*d2r;//Longlf[launp-1];//
	
	Ht0[TrgtCnt0] = 40.;
	Latt0[TrgtCnt0] = 16.003601*d2r;//
	Longt0[TrgtCnt0] = 40.872898*d2r;//*/
	
	/*Hlf[launp-1]	=	1000;//Hlf[launp-1];
	Latlf[launp-1]	=	17.056578*d2r;//Latlf[launp-1];//Hl;
	Longlf[launp-1]	= 	43.875156*d2r;//Longlf[launp-1];//*/

	/*Ht0[TrgtCnt0] = 40.;
	Latt0[TrgtCnt0] = 22.003601*d2r;//
	Longt0[TrgtCnt0] = 41.872898*d2r;//*/
//------------

//------------

	Hl		=	Hlf[launp-1];
	Latl	=	Latlf[launp-1];//Hl;
	Longl	= 	Longlf[launp-1];//
	Hl		=	2000;//1829.;//

//------------/// read  targets points

	// PointsCnt = ReadWayPoints("points.dat");


//-------------------------------------------------------------------
	//------------/// check Available targets
	mat3x1 rtn0 = Arr2Mat(Latt0[TrgtCnt0],Longt0[TrgtCnt0], Ht0[TrgtCnt0]);
	nav2cart( Latl, Longl, Hl, R0_L);
	if(choose_point==2)	nav2cart( Latt0[TrgtCnt], Longt0[TrgtCnt], Ht0[TrgtCnt], R0_t);
	else 			nav2cart( Latt0[TrgtCnt0], Longt0[TrgtCnt0], Ht0[TrgtCnt0], R0_t);

//---------------------------
	Lattf[0]  =Latt0[TrgtCnt0];	Longtf[0] =Longt0[TrgtCnt0];	Htf[0] =Ht0[TrgtCnt0];
	//Avail_trgt[0]=1;
	TrgtCnt=0;

	nav2cart( Lattf[0], Longtf[0], Htf[0], R0_t);

	for (int i = 0; i < PointsCnt; i++)	
	{
			erre0 = imperr(Arr2Mat(R0_L),rtn0 , Ht0[i], Latt0[i],Longt0[i]);
			azimuth(Hl, Latl, Longl, Ht0[i], Latt0[i], Longt0[i], &az0, &Rng2);

			if( (( (fabs(erre0(1,0))<max_z) || (hgv_flg && fabs(erre0(1,0))<2.*max_z) ) 
				&& (erre0.Norm())>3e3) )
			{
				cnt0++;
				Lattf[cnt0]  =Latt0[i];		
				Longtf[cnt0] =Longt0[i];	
				Htf[cnt0]	=Ht0[i];
				//Avail_trgt[cnt0]=1;
				//fprintf(f6,"%d  %d  %2.6f  %3.6f  %5.2f  %5.3f  %3.3f\n",cnt0, i,Lattf[cnt0]*r2d, Longtf[cnt0]*r2d, Htf[cnt0],Rng2/1e3,az0*r2d);
			}

		//fprintf(f5,"%d  %2.6f  %3.6f  %5.2f  %5.3f  %3.3f\n", i,Latt0[i]*r2d, Longt0[i]*r2d, Ht0[i],Rng2/1e3,az0*r2d);
	}//*/
	
	PointsCnt=cnt0; //Available targets numbers

	if (PointsCnt<3 || multi_trgts==0)	
		ctrlNo=4;
	else ctrlNo=1;
	//printf("\n ctrlNo : %d\t  multi_trgts : %d\t  ",ctrlNo,multi_trgts);
//------------------------------------------------------------------
	Ht		=	Htf[TrgtCnt];
	Latt 	=	Lattf[TrgtCnt];//Hl;
	Longt  = 	Longtf[TrgtCnt];//*/
	rtn(0,0)=Lattf[TrgtCnt];	rtn(1,0)=Longtf[TrgtCnt];	rtn(2,0)=Htf[TrgtCnt];
	azimuth(Hl, Latl, Longl, Ht, Latt, Longt, &az, &R1);
//////////////////
 double bearing = az,dist_m=1913e3 ;//- (2*d2r)*(nchoice==5);
//vreckon(Latl, Longl,500e3,az,points0);//for_test
		vreckon(Latl, Longl, R1,az ,points0);//for_real
	az=67.29 * d2r;
/*	if ( TrgtCnt0==1)  vreckon(Latl, Longl,1150e3-hgv_rng,az,points0);//for_test
else if ( TrgtCnt0==2)  vreckon(Latl, Longl,1250e3-hgv_rng,az,points0);//for_test
 else if ( TrgtCnt0==3)  vreckon(Latl, Longl,1350e3-hgv_rng,az,points0);//for_test
else if ( TrgtCnt0==4)  vreckon(Latl, Longl,1450e3-hgv_rng,az,points0);//for_test
else if ( TrgtCnt0==5)  vreckon(Latl, Longl,1550e3-hgv_rng,az,points0);//for_test
else if ( TrgtCnt0==6)  vreckon(Latl, Longl,1650e3-hgv_rng,az,points0);//for_test
else if ( TrgtCnt0==7)  vreckon(Latl, Longl,1710e3-hgv_rng,az,points0);//for_test
else if ( TrgtCnt0==8)  vreckon(Latl, Longl,1840e3-hgv_rng,az,points0);//for_test
else if ( TrgtCnt0==9)  vreckon(Latl, Longl,1860e3-hgv_rng,az,points0);//for_test

else //*/
	/* DESIGN MARGIN: during table GENERATION only (nchoice 3/4), design the
	   boost/mid-course profile for a longer range than the actual target, so
	   the profile carries energy margin for a cold motor. Fly mode (nchoice 5)
	   always uses the true target, so the miss is measured honestly. */
	/* ENABLED and PERMANENT (user decision 2026-08-18). T10 (700 km) and T11
	   (686 km) sit within ~25 km of the measured COLD-MOTOR range ceiling of
	   673-675 km, so their boost/mid-course profile is designed for 4% more
	   range than the target and therefore carries energy margin for a
	   thrust-down motor. Measured: T11n2 -29%, T10n2 -7%, every other arm
	   unchanged and still hitting; full 49 gate has zero regressions.
	   *** THIS IS NOT A BUILD FLAG ON PURPOSE. *** The HL2T10/HL2T11 tables on
	   disk were generated WITH this margin; changing the value here does
	   NOTHING until those tables are REGENERATED (one target and one pass per
	   MATLAB session -- generation state is once-per-process).
	   1.08 is NOT usable: the launch-angle solver bails at Teta_launch 92.9deg.

	   *** REVISED TO 1.00 ON 2026-08-23 -- the paragraph above is the ORIGINAL
	   2026-08-18 rationale and is now SUPERSEDED. ***
	   The margin buys cold-arm (flg==2) range at the cost of hot-arm (flg==1)
	   OVERSHOOT: at 1.04 the profile is designed ~30 km long, and a +1% thrust
	   dispersion then puts the vehicle past the target with a terminal geometry
	   steeper than the 20 deg aero envelope can fly -- T10n1/T10n4/T11n1/T11n4
	   all departed. The cold-arm problem the margin was protecting against was
	   independently fixed by the RAJEH_CO_ROLL_S2.dat repair (see AERO.CPP), so
	   the protection is no longer needed and only the cost remains.
	   MEASURED, T10 row (7 noises), regenerating at each value:
	     1.00 -> 6/7 (only n2, 6764 m)      1.01 -> 6/7 (n2 7031)
	     1.02 -> 5/7 (n2, n4)               1.03 -> 4/7 (n1, n2, n4)
	     1.04 -> 5/7 (n1 4622, n4 9028; n2 hits at 9.0)
	   T11 at 1.00 is 7/7. Full 49: 43/49 -> **46/49** (tal_flag=1), all four
	   flg==1 departures recovered, T10n2 lost as the direct trade.
	   tal_flag=0 unaffected (48/49) -- only the _TAL tables were regenerated.
	   CONTROL RUN: regenerating at 1.04 reproduces the shipped tables almost
	   exactly (n1 4622.0 vs 4619.4, n4 9027.5 vs 9125.1), which both validates
	   the regeneration path and shows the guidance.dat vintage is inert here.
	   NOTE the regenerated guidance.dat is 62 bytes and that is CORRECT -- only
	   its first line is read (M273N.CPP, five values). The ~37 kB files still
	   sitting in HL2T12/14/15/16_TAL are stale pass-1 dumps whose first line is
	   all ZEROS, i.e. those four targets fly with zeroed terminal guidance
	   gains. Regenerating them was NOT attempted; they currently hit. */
/* PER-TARGET margins, ORIGINAL 2026-08-18 measurements (superseded above):
   T10 peaked at 1.04 (1.05+ made its cold arm worse); T11 peaked at 1.05
   (a further -19%). 1.07 is unusable -- it breaks T10n5 (9 m -> 7656 m). */
#ifndef DESIGN_MARGIN_T10
#define DESIGN_MARGIN_T10 1.00
#endif
/* T11 = 1.00 as of 2026-08-23 (was 1.04). The original note below is still
   the best EVIDENCE for why 1.00 works: raising T11's margin was already known
   to break its HOT arm -- at 1.05, "T11 n4 fell 6.0 m -> 4172.5 m, a lost hit".
   Lowering it to 1.00 does the reverse and recovers BOTH hot arms; T11 is 7/7
   at 1.00. Screen the affected target across ALL noises.
   ORIGINAL 2026-08-18 NOTE: T11 = 1.04, NOT 1.05. 1.05 looked better on a
   6-case screen (cold arm -19%, several others improved) but the FULL 49
   rejected it: T11 n4 fell 6.0 m -> 4172.5 m, a lost hit. */
#ifndef DESIGN_MARGIN_T11
#define DESIGN_MARGIN_T11 1.00
#endif
	int   dgen  = (nchoice==3 || nchoice==4);
	double dm10 = dgen ? (DESIGN_MARGIN_T10) : 1.0;
	double dm11 = dgen ? (DESIGN_MARGIN_T11) : 1.0;
/* Target ranges follow the flag: tal_flag=0 spans 300..650 km, tal_flag=1
   spans 500..850 km (see TAL_R in sim_fio.h for why the bands differ).
   Index ordering is preserved -- T16 shortest, T13 longest.
        index    stock km    Edit-4 km
         T16        300         500
         T15        350         550
         T14        400         600
         T12        450         700
         T11        500         750
         T10        550         800
         T13        650         810   <- lowered 850->810, 2026-08-23        */
	if ( TrgtCnt0==10)  vreckon(Latl, Longl,(TAL_R(400e3,800e3)-hgv_rng)*dm10,az,points0);//for_test
else if ( TrgtCnt0==11)  vreckon(Latl, Longl,(TAL_R(600e3,750e3)-hgv_rng)*dm11,az,points0);//for_test
else if ( TrgtCnt0==12)  vreckon(Latl, Longl,TAL_R(550e3,700e3)-hgv_rng,az,points0);//for_test
/* T13 tal-mode band top LOWERED 850 -> 810 km on 2026-08-23 (user decision).
   At 850 km the target was NOT REACHABLE and the scenario could not even be
   generated: generation solves for (target - hgv_rng) = 800 km, and the
   launch-elevation coarse scan returned xerr NEGATIVE at all nine elevations
   in [50,85] deg -- best -2956 m at 67.5 deg, i.e. the range-vs-elevation
   curve PEAKS about 3 km short of the target. With no sign change the solver
   set Teta_launch = 90 deg, tripped the guard, and broke out BEFORE writing
   the tables -- leaving HL2T13_TAL holding byte-copies of HL2T10_TAL
   (position/tetac) and HL2T10_CMP (laun.dat). At 810 km generation solves for
   760 km, inside what T10 already solves at 750 km. */
 else if ( TrgtCnt0==13)  vreckon(Latl, Longl,TAL_R(520e3,650e3)-hgv_rng,az,points0);//for_test
else if ( TrgtCnt0==14)  vreckon(Latl, Longl,TAL_R(400e3,600e3)-hgv_rng,az,points0);//for_test
else if ( TrgtCnt0==15)  vreckon(Latl, Longl,TAL_R(350e3,550e3)-hgv_rng,az,points0);//for_test
else if ( TrgtCnt0==16)  vreckon(Latl, Longl,TAL_R(320e3,500e3)-hgv_rng,az,points0);//for_test
//*

	Latt  =	rtn(0,0)=	points0[0];//Hl;
	Longt =rtn(1,0)=	points0[1];
//////////////////////////

	azimuth(Hl, Latl, Longl, Ht, points0[0], points0[1], &az, &R1);


	Ll=Latl,Lonl=Longl;
	//rtn(0,0)=Lattf[TrgtCnt];	rtn(1,0)=Longtf[TrgtCnt];	rtn(2,0)=Htf[TrgtCnt];
	//printf("\n Latt : %.6f\t  Longt : %.6f\t  ",Latt*r2d,Longt*r2d);

//-----------------------------------------------
//////////////////
	//printf("\n TrgtCnt =%d  \t  \n",TrgtCnt);
   Range = data[6];
   At =az*d2r;// data[7] * d2r;
   Aiming_Angle = data[8] * d2r;
   //if(f_100) Aiming_Angle=0.25883171*d2r;//-2.5474891*d2r;
  Teta_launch =55.*d2r;// data[9] * d2r;
   Teta0 = data[10] - 0.0 + sim_teta0_bias;/*pre-launch pitch bias probe*/
 
 
  // if ((guid_flg==0 &&Teta0>55))Teta0 =Teta0-1;
      //
	  /*   if(Teta0>65)  Teta0 =Teta0-3.;
       else  if (Teta0>62 && Teta0<65) Teta0 =Teta0-2.;
	   else  if (Teta0>61 && Teta0<65) Teta0 =Teta0-1.;
    else if (Teta0>59.9 && Teta0<61) Teta0 =59.5;
      else if ((Teta0>55 && Teta0<60))Teta0 =Teta0-1; //*/
     // else 
    
         // double  Teta0_1; 
	      // Teta0_1 = (Teta0-53.5)/2.5;
		  // if (R1 < MaxR_stg1 &&Teta0 <59.) Teta0 = Teta0-Teta0_1;
		  // else if  ( R1 < MaxR_stg1 && Teta0 >=59.)  Teta0 = Teta0-Teta0_1-1;

  // //  if (Teta0>55 &&Teta0<57  && R1 < MaxR_stg1 ) Teta0 =Teta0-0.8;
    // //   else if (Teta0>57 &&Teta0<59.8  && R1 < MaxR_stg1 ) Teta0 =Teta0-1.7;
   // else if (Teta0>59.8 && R1 < MaxR_stg1 ) Teta0 =Teta0-1.5;
 // //if (Teta0>55 && Teta0<60 && R1 >MaxR_stg1 ) Teta0 =Teta0-1;
 // //  if (Teta0>63 && Teta0<65 && R1 >MaxR_stg1) Teta0 =Teta0-1.1;   
  // //else 
     	// if (Teta0>59.0  && R1 >MaxR_stg1)  Teta0 =Teta0-2.2; 
  // else	if (Teta0>57.5  && R1 >MaxR_stg1) 
 // Teta0 =Teta0+4.150; 
 Teta0 = Teta0AfterNchoice5Additions(Teta0, hgv_flg2);
    // else if (R1 >MaxR_stg1)  Teta0 =Teta0-0.5; 
   // //if(hgv_flg
 // //if (traject_up==0)  	
 //Teta0 -=0.52;
  if(rocket==0&&nchoice==5)     Teta_launch = 70*d2r;// (Teta0+5.)*d2r;// data[9] * d2r;
 // // Teta_launch =(60)*d2r;
    if(rocket==1) 
    {
	    Teta0=69.;
     // //  Teta0=64.1188;
	   // //Hl=1900.;
    }
    L_x = data[11];
   L_y = data[12];
   L_v = data[13];
   L_gama = data[14];
   V_star = data[15];
   //Jr_star = data[16];
   //if (nchoice==5)	MaxR_stg1 = data[16]*1e3;
   dtt = 1 * freq;
   delt = 1. / (dtt * 100); // data[17];
   dtout = data[18];
   totalt = 1000.; // data[19];
   tterm = 1000.; // data[20];
   if(nchoice==5) tterm = tfinal;
 //if(nchoice==5&& rocket==0) Teta_launch =(Teta0+5.)*d2r;

   
   if (FTHR == 1) {
      Temp0 = 25 + 273.15;
   }
   if (FTHR == 2) {
      Temp0 = 55 + 273.15;
   }
   if (FTHR == 3) {
      Temp0 = -10. + 273.15;
   }
   ///*/

   Temp0 = 25 + 273.15;

   Season_flag = data[22];
   Wv1 = -data[23];
   Waz1 = data[24] * d2r;
   Wv2 = -data[25];
   Waz2 = data[26] * d2r;
   Wv3 = -data[27];
   Waz3 = data[28] * d2r;
   Wv4 = -data[29];
   Waz4 = data[30] * d2r;
   Wv5 = -data[31];
   Waz5 = data[32] * d2r;
   Csd = data[33];
   Xsd = data[34];
  

   // Correct the burning time  of motor for the reason of variation of
   // temperature and burning rate of fuel.      1380 / 11 /28

   t_guidance = 1.2;
   acmin = 6.;
   wmax = 5. * d2r;
   tservo = .07;
   finlim = 24. * d2r;

  strcpy(piam,"Enter Variance of Error Data Filename. ");
  readfile(piam,def2,dim2,ans,data,0);
  sig[0]=data[0]*d2r/60.0;
  sig[1]=data[1]*d2r;
  sig[2]=data[2]*d2r/60.0;
  sig[3]=data[3]*d2r;
  sig[4]=data[4];
  sig[5]=data[5];
  sig[6]=data[6]/100.;
  sig[7]=data[7]*d2r/60.;
  sig[8]=data[8]*d2r/60.;
  sig[9]=data[9]*1e-3;
  sig[10]=data[10]*1e-3;
  sig[11]=data[11]*d2r/3600.;
  sig[12]=data[12]*d2r/3600.;
  sig[13]=data[13]*d2r/3600.;
  sig[14]=data[14]*1e-6;
  sig[15]=data[15]*1e-6;
  sig[16]=data[16]*1e-6;
  sig[17]=data[17]*.00981;
  sig[18]=data[18]*.00981;
  sig[19]=data[19]*.00981;
  sig[20]=data[20]*1e-6;
  sig[21]=data[21]*1e-6;
  sig[22]=data[22]*1e-6;
  nsim  =10.;//data[23];

  clrscr();
   azimuth(Hl, Latl, Longl, Ht, Latt, Longt, &Az0, &Range);
  At = Az0;
   //if (nchoice > 4)	azimuth(Hl, Latl, Longl, Ht_6, Latt_6, Longt_6, &Az0, &Range);
 

   d0 = e2 * sin(2.0 * Latl);
   k2 = 2.0 * e2 - e2 * e2;
   r0 = re * (1.0 - e2) / sqrt(1.0 - k2 * cos(Latl - d0) * cos(Latl - d0));

  Initat(Season_flag,Latl,Hl,Temp0);
 // fd2 = sim_fopen("plsfile.dat","wt");
 // fd = sim_fopen("input_tf.dat", "wt");
  //hgv_fd = sim_fopen("hgv_fd.dat", "wt");

  if(nchoice==5&&sim_no>1)
  {
  if(noise==1)					{fd2 = sim_fopen("noise1.dat","wt");	Outfile = sim_fopen("noise1.mat", "wb");}
  else if (noise==2)			{fd2 = sim_fopen("noise2.dat","wt");	Outfile = sim_fopen("noise2.mat", "wb");}
  else if (noise==3)			{fd2 = sim_fopen("noise3.dat","wt");	Outfile = sim_fopen("noise3.mat", "wb");}
  else if (noise==4)			{fd2 = sim_fopen("noise4.dat","wt");	Outfile = sim_fopen("noise4.mat", "wb");}
  else if (noise==5)			{fd2 = sim_fopen("noise5.dat","wt");	Outfile = sim_fopen("noise5.mat", "wb");}
  else if (noise==6)			{fd2 = sim_fopen("noise6.dat","wt");	Outfile = sim_fopen("noise6.mat", "wb");}
  else							{fd2 = sim_fopen("plsfile.dat","wt");	Outfile = sim_fopen("plsfile.mat", "wb");}
  }//
else //
{
	fd2 = sim_fopen("plsfile.dat","wt");
//////////////////
	Outfile = sim_fopen("plsfile.mat", "wb");
	hgv_fd = sim_fopen("hgv_fd.mat", "wb");

}
	char *matrix_name = "R2000";
	pls.type = 0;	pls.ncols = 1;	pls.imagf = 0;
	pls.mrows = N_to_save;
	pls.namlen = strlen(matrix_name) + 1;
	fwrite(&pls, sizeof(Fmatrix), 1, Outfile);
	fwrite(matrix_name, sizeof(char), (int)pls.namlen, Outfile);//*/

#ifdef SIM_CLI_HOOK
	/* Reapply CLI mission inputs after the legacy initializer has loaded its
	   compiled scenario and recompute all coordinate-derived quantities. */
	{ extern void cli_param_hook(void); cli_param_hook(); }
#elif defined(SIM_MEX_MANUAL_HOOK)
	mex_manual_apply_coordinates();
#endif




}
void simulate(void)
{
   double ndim = 50, vf0[3], er[23], x, t_s4 = 0.;
   sepflg = sepflg2=pik_q = flg_q =0;
   flg_stage = sim_count = sim_count_g = p_count = flag_alfa = 0.;
   t =0;
   s_s = 1;
   vb = bv = 0;
   //Del_tm = d_j;//12.;
   Del_t = tsep =t_sep=t_BeltCmd=t_ign2_cmd=t_ign2=1000.;
   Place=Place2=0;
   BeltCmd=sep_fb=ign2_cmd=ign2_fb=0;
   burn_flg=flgm_5=burn_flg2=flgb_5=0;
   gama_flg=tgt_flg=0;
   burnoutflg2=burnoutflg1=0;
   tgo=1000.;
   t_burn1=25;
   ss_6=0;
   tt=t_2=t_vc=0.;
	StopCtrlCmd=PushCmd=StopCtrlCmd_2=0;
	t_StopCtrlCmd=t_PushCmd=1000.;
	//gama_cnt=0;
	motor1_state=motor2_state=0;
	ign2_ok=ign2_ok5=0.;
	ign2_stg=0;
	guid_flg=0;
	ign2_stg=2;
 
  int i;
   if (nchoice == 3) {
      for (i = 0; i < 23; i++)	er[i] = 0.;
      tune_flag = 1;
   }
   else if (nchoice == 5) {
	    //tune_flag = 0;
      for (i = 0; i < 23; i++)	er[i] = sig[i];
   }
   else if (nchoice == 6) {
      for (int i = 0; i < 21; i++) {gaus(0.0, sig[i], &x);	er[i] = x;
         if (fabs(er[i]) > 3. * sig[i])		er[i] = 3. * sig[i] * fabs(sig[i]) / sig[i];
      }
   }
   else if (nchoice == 4) {
      for (i = 0; i < 23; i++)	er[i] = 0.;
      Wv1 = Wv2 = Wv3 = Wv4 = Wv5 = Waz1 = Waz2 = Waz3 = Waz4 = 0.;
      tune_flag = 1;
   }
   teterr = er[0];
   tetraterr = er[1];
   azerr = er[2];
   azraterr = er[3];
   tberr = er[4]; // 1 sec
   maserr = er[5];
   threrr = er[6]; // 1 %
   eth1 = er[7]; // 1 min
   eth2 = er[8]; // 1 min
   Yt = er[9]; // 1 mm
   Zt = er[10]; // 1 mm
   gbx = er[11];
   gby = er[12];
   gbz = er[13];
   gsx = er[14];
   gsy = er[15];
   gsz = er[16];
   abx = er[17];
   aby = er[18];
   abz = er[19];
   asx = er[20];
   asy = er[21];
   asz = er[22];

      if (nchoice == 6) {
      teterr = tetraterr = azerr = azraterr = tberr = maserr = threrr = eth1 = eth2 = Yt = Zt = gbx = gby = 0.;

      xbc_y = er[0] / 1000.; // 1 mm
      wv0 = fabs(er[1]); // 7.5 m/s
      waz0 = er[2]; // 15 deg
      xbc_er = er[3] / 100.; // 2 cm
      tberr = er[4]; // 1. sec
      K_test = er[5]; // .6~2
      K_test = fabs(K_test);
		if (K_test <= .6)	K_test = 0.6;
		if (K_test >= 2.)	K_test = 2.;
         K_test = 2.;
      threrr = er[6]; // 0.66 %
      eth1 = er[7]; // 1 min
      eth2 = er[8]; // 1 min
      Yt = er[9]; // 1 mm
      Zt = er[10]; // 1 mm
      xbc_z = er[11] / 1000.; // 1 mm
      jerm = er[12]; // 20 kg
   }
	if (nchoice == 3 || nchoice == 4)
     {
		K_test = 1.;	motor_flag = 0;	flg=3;	 rocket=0;	hgv_flg=0;
#if defined(SIM_MEX_MANUAL_HOOK)
		if (!mex_manual_coordinates_enabled()) hgv_flg2=0;
#elif !defined(SIM_CLI_HOOK)
		hgv_flg2=0; /* Preserve the explicit CLI value in console builds. */
#endif
		sim_no=1;	reentry_ign=0;
	}

   if (nchoice == 5) {
      teterr = tetraterr = azerr = azraterr = tberr = maserr = threrr = eth1 = eth2 = Yt = Zt = gbx = gby = 0.;

      xbc_y = 0.0;
      wv0 = 0.0;
      waz0 = 0.0;
      xbc_er = .0;
      tberr = .0;
      // K_test=1.;
      threrr = .0;
      eth1 = .0;
      eth2 = .0;
      Yt = .0;
      Zt = .0;
      xbc_z = .0;
      jerm = .0;
      // if(K_test0!=1.) K_test=K_test0;
      // if(xbc_er0!=0.) xbc_er=xbc_er0;

   }
	//Init_guid();
	//tberr = -1.;
   /* Calculate the directional cosines of the rotation axis of the earth with respect to the 'F' frame. */

   d1 = cos(Latl) * cos(At);
   d2 = sin(Latl);
   d3 = -cos(Latl) * sin(At);
   /* Calculate  the angular velocity  of earth in 'F' frame */
   wiff[0] = wie * d1;
   wiff[1] = wie * d2;
   wiff[2] = wie * d3;
   /* Compute the missile attitude quaternions from the  "aiming OK" inertial
    coordinate system Zt to the  body coordinate system Zb before launching. */
   
   e2q(Teta_launch + teterr, Aiming_Angle + azerr, 0. *d2r, qib);

   e2q(Teta_launch + teterr, azerr, 0.0, _qib);
   q2m(qib, mi2b);
   q2m(qib, mi2l);
   twm(1, mi2b, wiff, wifb);
   /* Compute the components of the "qbi" from "qib" . */
   qbi[0] = qib[0];
   qbi[1] = -qib[1];
   qbi[2] = -qib[2];
   qbi[3] = -qib[3];
   /* Calculate the components of the  vector of the missile firing point
    coordinatized in the Geocentric inertial coordinate system . */
   rf0[0] = -r0 * sin(d0) * cos(At);
   rf0[1] = r0 * cos(d0) + Hl;
   rf0[2] = r0 * sin(d0) * sin(At);

   /* Calculate the components of the  vector of the target point coordinatized in the 'F' frame . */
   rt = Ht + r0; // Ht+re*(1.0-e)/sqrt(1.0-k2*cos(Latt)*cos(Latt));
   rtf[0] = rt * sin(Range / r0);
   rtf[1] = rt * cos(Range / r0) - r0 - Hl;
   rtf[2] = 0.;
   MATRIXf2e();

    mat3x1 rtn0 = Arr2Mat(Latt0[TrgtCnt0],Longt0[TrgtCnt0], Ht0[TrgtCnt0]);
	mat3x1 R0_N = Arr2Mat(Latl, Longl, Hl);

	R0_E = nav2cart(R0_N);
	rte = nav2cart(rtn);
	C_e_l  = Rot_e_l(At, R0_N(0,0), R0_N(1,0));

	rt_l = C_e_l * (rte - R0_E);

   //double rtn0[3],r_e0[3];

    nav2cart(Latl, Longl, Hl, R0_L);

	nav2local(Latt,Longt,Ht, R0_L, rtf);
	//nav2local(Latt_6,Longt_6,Ht_6, R0_L, rtf_6);

	//Xtrgt=rtf_6[0];	Ytrgt=rtf_6[1];	Ztrgt=rtf_6[2];
	Xtrgt=rtf[0];	Ytrgt=rtf[1];	Ztrgt=rtf[2];
	//
	mat4x1 qu;
	for (i = 0; i < 4; i++)
		qu(i,0) = qib[i];

	mat3x1 Angles = Quat2Euler_l(qu);
	Angles(2,0) = At - Angles(2,0);
	C_b_n = Rot_b_n(Angles);
	q_b_n = RotMat2Quat(C_b_n);//qu;
	q_b_n_ins = q_b_n;
//-----------------------

   /* Calculate the components of the  vector of the initial missile velocity coordinized in the  'F' frame . */
   cross(wiff, rf0, vf0);
   hold = Hl;
   for (i = 0; i < ndim; i++)	dy[i] = y[i] = 0.;

   y[3] = vf0[0];
   y[4] = vf0[1];
   y[5] = vf0[2];
   /*
    if(flg==1)//error up
    {
    y[6]=-3.*d2r+wifb[0];
    y[7]=-3.*d2r+wifb[1]+azraterr*0.;
    y[8]=3.*d2r+wifb[2]+tetraterr*0.;
    }
    else if(flg==2)//error down
    {
    y[6]=3.*d2r+wifb[0];
    y[7]=3.*d2r+wifb[1]+azraterr*0.;
    y[8]=-3.*d2r+wifb[2]+tetraterr*0.;
    }
    else//normal// */ {
      y[6] = 0. * d2r + wifb[0];
      y[7] = 0. * d2r + wifb[1] + azraterr * 0.;
      y[8] = -0. * d2r + wifb[2] + tetraterr * 0.;

   }

   y[9] = qib[0];
   y[10] = qib[1];
   y[11] = qib[2];
   y[12] = qib[3];

   y[16] = vf0[0];
   y[17] = vf0[1];
   y[18] = vf0[2];

   y[20] = qib[0];
   y[21] = qib[1];
   y[22] = qib[2];
   y[23] = qib[3];

   y[24] = atan2(rtf[1], rtf[0]);

   y[40] = La = Latl;
   y[41] = lo = Longl;
   y[42] = h = Hl;

   y[43] = 0.;
   y[44] = 0.;
   y[45] = 0.;

   y[46] = 0.;
   y[47] = Teta_launch;
   y[48] = At - Aiming_Angle;

   double _a = 6378137.0;
   double _b = 6356752.3;

   double _c = _a * _a * cos(La) * cos(La) + _b * _b * sin(La) * sin(La);
   double _N = _a * _a / sqrt(_c);

   double _f = (_a - _b) / _a;
   double _e = sqrt(2 * _f - _f * _f);

   ecef_jm[0] = (_N + h) * cos(La) * cos(lo);
   ecef_jm[1] = (_N + h) * cos(La) * sin(lo);
   ecef_jm[2] = (_N * (1 - _e * _e) + h) * sin(La);

   t_dive = 0.;
   Ayb_filter = Azb_filter=azi_filter = 0.;
   sim_count = 0.;
   sim_count_g = 0.;
   if (nchoice == 5) {
      Print_diar();
      Print_plot();
   }
   // if(nchoice==6)   				Print_diar();
   if (nchoice == 3 || nchoice == 5)
      Print_sim();
   if (nchoice == 6)
      Print_cep();
   if (nchoice == 3 || nchoice == 4) {
      if (nchoice == 4)
         Print_ballistic();
      if (l_flag == 0) {
		 burnoutflg1=0;
		 tgt_flg=0;
         t = 0.;
         prmt[0] = t;
         prmt[1] = stg2;
         prmt[2] = delt;
         om273na();
         drkg4(ndim, f6dm273na, om273na);
         for (int k = 0; k < ndim; k++) {
            dy00[k] = dy[k];
            y00[k] = y[k];
         }
         Teta_save = tetam;
         rmf_save[0] = rmf[0];
         rmf_save[1] = rmf[1];
         vmf_save[0] = vmf[0];
         vmf_save[1] = vmf[1];
         V_save = vm;
         t0 = t;
         prmt[0] = t;
         prmt[1] = tterm;
         prmt[2] = delt;
         om273na();

         drkg4(ndim, f6dm273na, om273na);
      }
      else if (l_flag == 1) {
         for (int k = 0; k < ndim; k++) {
            dy[k] = dy00[k];
            y[k] = y00[k];
         }
         vdot_0_flag = 1;
         t = t0;
         om273na();
         double rif[3], rii[3], Vif[3];
         switch (choice) {
         case 0:	rmf[0] += delta_r * num;	break;
         case 1:	rmf[1] += delta_r * num;	break;
         case 2:	vmf[0] += delta_v * num;	break;
         case 3:	vmf[1] += delta_v * num;	break;
         default:	break;
         }
         for (i = 0; i < 3; i++)	rif[i] = rmf[i] + rf0[i];
         twm(2, mi2f, rif, rii);
         for (i = 0; i < 3; i++)	 y[i] = rii[i] - rf0[i];
         cross(wiff, Rif, wR);
         for (i = 0; i < 3; i++)	Vif[i] = vmf[i] + wR[i];
         twm(2, mi2f, Vif, y + 3);
         prmt[0] = t;
         prmt[1] = tterm;
         prmt[2] = delt;
         om273na();
         drkg4(ndim, f6dm273na, om273na);
      }
   }
   else {
      t = 0.;
      prmt[0] = t;
      prmt[1] = tterm;
      prmt[2] = delt;
      om273na();
      drkg4(ndim, f6dm273na, om273na);
   }
   
   /*
	double FAz,FRange;
	azimuth(Hl, Latl, Longl, hm, rmn_[0],  rmn_[1], &FAz, &FRange);
	printf("\n t  = %.2f  final range = %.2f km \t hm = %.2f\n ",t,FRange*1e-3,hm);//*/

}

void Ballistic_parameter(void)
{
   double rmf_save1[7], L[4], data[40], delta;
   int iii, jj3j, k, i;
   g_flag = 0;
   l_flag = 1;
   for (iii = 0; iii < 4; iii++) {
      choice = iii;
      if (iii < 2)
         delta = delta_r;
      else
         delta = delta_v;
      for (jj3j = 0; jj3j < 3; jj3j++) {
         if (jj3j != 1) {
            num = jj3j - 1;
            simulate();
            rmf_save1[jj3j] = rmf[0];
         }
         else
            rmf_save1[jj3j] = 0.;
      }
      L[iii] = (rmf_save1[2] - rmf_save1[0]) / (2. * delta);// 10. is delta_r or delta_v
   }
   L_x = L[0];
   L_y = L[1];
   L_v = L[2] * cos(Teta_save) + L[3] * sin(Teta_save);
   L_gama = (L[3] * cos(Teta_save) - L[2] * sin(Teta_save)) * V_save;
   Jr_star = rmf_save[0] * L[0] + rmf_save[1] * L[1];

	clrscr();
	//  gotoxy(30,11);
	show_data(4);
	fprintf(m_gui, "%2.2f    %8.6f    %8.6f   %8.6f    %8.6f\n ", stg2, L_x, L_y,L_v, L_gama);

	tune_flag = 0;
	bal_flag = 0;
	getch();
}
void Guidance_parameter(void)
{


   double data[40];
   int cnt=0;
   int k, i;
   Teta0 = 55;//67.; // test1
  // Teta0 = 55.; // fcu
  		//printf("\n Teta0 = %.2f \n",Teta0);
double RR=40.;
	if(Range<1500001) RR=15.;
	else if(Range<1700001) RR=20.;
	else if(Range<1900001) RR=30.;
	else if(Range<2200001) RR=55.;
	int p=0;
   do {
	   cnt++;
      simulate();
	//  printf(":%8.3f   :%8.3f   :%8.3f\n",rmf[0],rtf[0],fabs(rmf[0]-rtf[0]));
	  // if(p==0){showdattta();p=1;}
	 // 	 printf("\n  cnt = %d and  Teta0 = %.4f  \t xerr = %.2f \n",cnt,Teta0,(rmf[0] - rtf[0]));

      if (fabs(rmf[0] - rtf[0]) > 100.)
	  {
		  
         //Teta0 += (3.e-5)*(rtf[0] - rmf[0]);
         //Teta0 += (5.e-7)*(rtf[0] - rmf[0]);
		if(Range<MaxR_stg1)  Teta0 +=(10 / Range)*(rtf[0] - rmf[0]);
		else	     Teta0 +=(RR / Range)*(rtf[0] - rmf[0]);
	  }

	 //cnt++;
	 //printf("\n  iter = %d and  Teta0 = %.2f  \t xerr = %.2f \n",cnt,Teta0,(rmf[0] - rtf[0]));

   }
   while (fabs(rmf[0] - rtf[0]) > 100. && cnt<150.&& Teta0<80.);
	double guidance_xerr_m = rmf[0] - rtf[0];
	double guidance_zerr_m = rmf[2] - rtf[2];
	if(!_finite(guidance_xerr_m) || fabs(guidance_xerr_m) >= 100.0)
	{
		trajectory_generation_success = 0;
		trajectory_generation_iterations = cnt;
		trajectory_generation_xerr_m = guidance_xerr_m;
		trajectory_generation_zerr_m = guidance_zerr_m;
		printf("\n guidance trajectory did not converge: iter=%d xerr=%.3f m "
		       "(required |xerr| < 100 m), Teta0=%.3f deg Range=%.3f km hl=%.2f m\n",
		       cnt, guidance_xerr_m, Teta0, Range/1e3, Hl);

		nchoice=9;
		//sim_fcloseall();
		//exit(0);
		return;
	}
   // */
/*
    if (launch_mod == 0)	dtl = 3.;
    else					dtl = 7.;
   if (fabs(floor(Teta0) - Teta0) < 0.5)
      Teta_launch = (floor(Teta0) + dtl) * d2r;
   else
      Teta_launch = (ceil(Teta0) + dtl) * d2r; // */

	//azimuth(Hl, Latl, Longl, Ht0[TrgtCnt], Latt0[TrgtCnt], Longt0[TrgtCnt], &az0, &Range);
	//azimuth(Hl, Latl, Longl, rtn(2,0), rtn(0,0), rtn(1,0), &az0, &Range);

   tune_flag = 0;
   f1 = sim_fopen(laun_file, "r+");
   for (i = 0; i < dim1; i++)
      fscanf(f1, "%d%lf", &k, &data[i]);
   data[6] = Range;
   data[7] = At * r2d;
   data[8] = Aiming_Angle * r2d;
   data[9] = Teta_launch * r2d;
   data[10] = Teta0;
   data[16] = MaxR_stg1/1e3;

   rewind(f1);
   for (i = 0; i < dim1; i++)
      fprintf(f1, "%3d  %14.8e\n", i + 1, data[i]);
   sim_fclose(f1);
   gotoxy(12, 10);
   printf("\n\t\t\t  Elevation          = %8.4f ", Teta_launch*r2d);
   printf("\n\t\t\t  Azimuth of Target  = %8.4f ", At*r2d);
   printf("\n\t\t\t  Azimuth of Missile = %8.4f ",
       At*r2d - Aiming_Angle*r2d);
   printf("\n\t\t\t  Range              = %8.4f ", Range);
      printf("\n\t\t\t  MaxR_stg1           = %8.4f ", MaxR_stg1/1e3);
   printf("\n\t\t\t  Teta0              = %8.4f ", Teta0);
   printf("\n\t\t\t  Hl,Ht              = %8.4f  %8.4f ", Hl,Ht);
   printf("\n\t\t\t  cnt              = %d ", cnt);
   printf("\n\t\t\t  TrgtCnt              = %d ", TrgtCnt);
   printf("\n\t\t\t  launp              = %d ", launp);
   printf("\n\t\t\t  total time flight       = %.2f \n", tff);
   getch();


}


void CEP_Calculation(void)
{
 if (nsim < 1)
      nsim = 2;
   clrscr();
   for (j_cep = 0; j_cep < nsim; j_cep++) {
      simulate();
      xerr = rmf[0] - rtf[0];
      zerr = rmf[2] - rtf[2];

      long int u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11, u12, u13;
      u1 = xbc_y * 1000000.;
      u2 = wv0 * 1000000.;
      u3 = waz0 * 1000000.;
      u4 = xbc_er * 1000000.;
      u5 = tberr * 1000000.;
      u6 = K_test * 1000000.;
      u7 = threrr * 1000000.;
      u8 = eth1 * 1000000.;
      u9 = eth2 * 1000000.;
      u10 = Yt * 1000000.;
      u11 = Zt * 1000000.;
      u12 = xbc_z * 1000000.;
      u13 = jerm * 1000000.;

   }
   sim_fclose(fcep);

   //MinMaxR();
   getch();

  
}
//void MinMaxR()
//{
//	flg_sh=1;
//	flg_c=0;
//	Teta_launch=50.*d2r;
//	simulate();
//	minR=rmf[0];
//    Teta_launch=75.*d2r;
//    simulate();
//    maxR=rmf[0];
//	show_data(7);
//}

void Print_diar(void)
{
  // struct date d;
//   getdate(&d);
}
void Print_plot(void)
{
	float indata[8];
    indata[0]=Range;
    indata[1]=At;
    indata[2]=Aiming_Angle;
    indata[3]=Latl;
    indata[4]=Hl;
    indata[5]=Ht;
    indata[6]=delt;
  //  write(fout->fd,&indata,4*8);
  //  write(fout->fd,&indata,4*19);//dumy only for match
    clrscr();
    randomize();
}


void Print_cep(void)
{

}


void main1(void)
{




 // fcep= sim_fopen("cep.dat","wt");
 // fecep=sim_fopen("ercep.dat","wt");

  char ans[80];
  if(nchoice==3)
  {
	trajectory_generation_success = 0;
	trajectory_generation_iterations = 0;
	trajectory_generation_xerr_m = 0.0;
	trajectory_generation_zerr_m = 0.0;
  }
  do
  {
	// clrscr();
	// nchoice=popup(option,"cho1",9,7,18);
	 switch(nchoice)
	 {
	   case 0: //Initilize(0);   //break;
	   case 1: //Initilize(def1,read1,"matn1",dim1);   break;
	   case 2: //Initilize(def2,read2,"matn2",dim2);   break;
	   case 3:
	   case 4:
	   case 5:
	   case 6: Initilize_sim();
	   if(nchoice==5)
	   {
		 strcpy(ans,defd);
		 fdiary=sim_fopen(ans,"w");

	   }
	   if(nchoice==5)
	   {
		 strcpy(ans,defout);
		 fout=sim_fopen(ans,"wb");
	   }
//--------------------------------
/*
	   if(nchoice==3 && MinMaxRng_flg==0)
	   {
		 nchoice=5;
		Teta_launch_roket = 73.;
		Teta0 =70;
		rocket=1;
		MinMaxRng_flg=1;
		ign2_stg=6;
		MaxR_stg1=2500e3;
		// printf("\n ign2_stg = %d \t noise=%d\n",ign2_stg,noise);
	   }//*/
//--------------------------------
	   if(nchoice==3)
	   {
		   int tl_cnt=0;
		  //m_flagm=1;
		 	Aiming_Angle =0;// 0.44 * d2r;
			//if(Teta_launch<50.*d2r &&Teta_launch>75.*d2r)
				//Teta_launch = 72.*d2r;//69.4 * d2r;
                 //Teta_launch = 55.*d2r;//69.4 * d2r;
			double R=15.;
			double RA=20.;
			Teta_launch =65.*d2r;//69.4 * d2r;
			if(Range<1300001){ R=5.;  RA=20.;}
            else if(Range<1650001){ R=5.;  RA=20.;}//{ R=12.;  RA=25.;}
			else if(Range<1800001){ R=15.;	RA=30.;}//{ R=25.;	RA=40.;}
            else if(Range<2000001){ R=15.;	RA=25.;}//*/

            Aim = 1;

            /* ---- launch-elevation solve: BISECTION over [50,90] deg -------
               Replaces a fixed-step gradient walk from a hard-coded 65 deg
               start. That walk only converged when the answer lay ABOVE its
               start; otherwise it stepped monotonically down through the
               50 deg floor and bailed -- and the bail breaks out BEFORE the
               laun.dat write further down, so the target silently kept
               another target's launch data. It was path-dependent rather than
               range-dependent (T10 and T11 both sit at 780 km yet solved at
               66.33 and 49.45 deg), and it could not reproduce the shipped
               stock tables: a fresh stock generation diverged on T12/T14/T15/T16.

               Bisection converges whenever a root exists in the bracket and
               honours the 50 deg floor by construction. Azimuth keeps the
               original gradient update, applied at every evaluation; that
               dimension was always well behaved. Applies to BOTH flags. */
            {
               /* Range is UNIMODAL in launch elevation, not monotonic: it is
                  near zero at both ends of [50,85] and peaks in between.
                  Measured (stock T10, 676 km target): 50 deg reaches 47 km,
                  85-90 deg lands BEHIND the launcher. So a plain bisection on
                  the full interval brackets nothing, and there are in general
                  TWO roots - a depressed one below the peak and a lofted one
                  above it. Take the LOWER root, below the peak: that is the
                  branch the shipped tables sit on (stock T16 62.10, T14 64.15,
                  T10 70.99 deg, against a peak near 72), and it is the
                  efficient one - the upper root reaches the same range by
                  throwing energy into loft. Measured scan, stock T14 (500 km):
                  xerr runs -404 km at 50 deg, through zero near 64.2, peaks
                  +235 km at 71.9, back through zero near 82.3, to -151 km at
                  85. Coarse-scan first to find the bracket, then bisect it. */
               const double TL_MIN = 50.0*d2r, TL_MAX = 85.0*d2r;
               const int    NSCAN  = 8;
               double sc_t[9], sc_e[9];
               double tl_lo = TL_MIN, tl_hi = TL_MAX, e_lo = 0., e_hi = 0., e_mid;
               double kaz = (Range < MaxR_stg1) ? 30. : RA;
               int bracketed = 0, is;

               for (is = 0; is <= NSCAN; is++)
               {
                  sc_t[is] = TL_MIN + (TL_MAX - TL_MIN)*is/(double)NSCAN;
                  Teta_launch = sc_t[is];
                  simulate();
                  sc_e[is] = rmf[0] - rtf[0];
                  Aiming_Angle -= (kaz / Range) * (rtf[2] - rmf[2]) * d2r;
                  tl_cnt++;
                  printf("\n  scan = %d and  Teta_launch = %.4f  \t xerr = %.2f \t zerr = %.2f\n",
                         tl_cnt,Teta_launch*r2d,(rmf[0] - rtf[0]),(rmf[2] - rtf[2]));
               }
               /* lowest-elevation sign change = the efficient root */
               for (is = 0; is < NSCAN; is++)
                  if ((sc_e[is] > 0.) != (sc_e[is+1] > 0.))
                  {
                     tl_lo = sc_t[is];   e_lo = sc_e[is];
                     tl_hi = sc_t[is+1]; e_hi = sc_e[is+1];
                     bracketed = 1;
                     break;
                  }
               if (!bracketed)
               {
                  printf("\n  no launch elevation in [50,85] reaches Range=%.3f km\n", Range/1e3);
                  Teta_launch = 90.*d2r;   /* trip the guard below: do NOT
                                              write a bogus laun.dat */
               }

               while (bracketed && tl_cnt < 45)
               {
                  Teta_launch = 0.5*(tl_lo + tl_hi);
                  simulate();
                  e_mid = rmf[0] - rtf[0];
                  Aiming_Angle -= (kaz / Range) * (rtf[2] - rmf[2]) * d2r;
                  tl_cnt++;
                  printf("\n  iter = %d and  Teta_launch = %.4f  \t xerr = %.2f \t zerr = %.2f\n",
                         tl_cnt,Teta_launch*r2d,(rmf[0] - rtf[0]),(rmf[2] - rtf[2]));

                  if (fabs(e_mid) <= 100. && fabs(rmf[2] - rtf[2]) <= 100.) break;
                  if ((e_mid > 0.) == (e_lo > 0.)) { tl_lo = Teta_launch; e_lo = e_mid; }
                  else                             { tl_hi = Teta_launch; e_hi = e_mid; }
                  if ((tl_hi - tl_lo) < 1.e-5*d2r) break;       /* elevation converged */
               }

               /* azimuth-only refinement at the solved elevation */
               while (bracketed && tl_cnt < 50 && fabs(rmf[2] - rtf[2]) > 100.)
               {
                  simulate();
                  Aiming_Angle -= (kaz / Range) * (rtf[2] - rmf[2]) * d2r;
                  tl_cnt++;
               }
            }

            if (0) do {
				tl_cnt++;
               simulate();
               // sim_fclose(popof);
             /*  if (fabs(rmf[0] - rtf[0]) > 100. || fabs(rmf[2] - rtf[2]) > 100.)
               {
                  Teta_launch += (10. / Range) * (rtf[0] - rmf[0]) * d2r;
                  Aiming_Angle -= (10. / Range) * (rtf[2] - rmf[2]) * d2r;
               }//*/
			   	printf("\n  iter = %d and  Teta_launch = %.4f  \t xerr = %.2f \t zerr = %.2f\n",tl_cnt,Teta_launch*r2d,(rmf[0] - rtf[0]),(rmf[2] - rtf[2]));

               if (fabs(rmf[0] - rtf[0]) > 100. || fabs(rmf[2] - rtf[2]) > 100.)
               {
                if(Range<MaxR_stg1)
                {
					Teta_launch += (7. / Range) * (rtf[0] - rmf[0]) * d2r;
					Aiming_Angle -= (30. / Range) * (rtf[2] - rmf[2]) * d2r;
				}

              else
              	{  
					//Teta_launch += (15. / Range ) * (rtf[0] - rmf[0]) * d2r;
					//Aiming_Angle -= (20. / Range) * (rtf[2] - rmf[2]) * d2r;

					Teta_launch += (R / Range) * (rtf[0] - rmf[0]) * d2r;
					Aiming_Angle -= (RA / Range) * (rtf[2] - rmf[2]) * d2r;
                             }
               }//*/
				//printf("\n  iter = %d and  Teta_launch = %.2f  \t xerr = %.2f \t zerr = %.2f\n",tl_cnt,Teta_launch*r2d,(rmf[0] - rtf[0]),(rmf[2] - rtf[2]));

            }
             while ((fabs(rmf[0] - rtf[0]) > 100. || fabs(rmf[2] - rtf[2]) > 100.)
                    && tl_cnt<50 && Teta_launch<90.*d2r && (Teta_launch)>50.*d2r);//*/

           // while ((fabs(rmf[0] - rtf[0]) > 100./* || fabs(rmf[2] - rtf[2]) > 100.*/)
			//	&& tl_cnt<50 && Teta_launch<90.*d2r && (Teta_launch)>50.*d2r);//*/
           				
				//printf("\n max iteration = %d  of Teta_launch =%.3f calc  Range=%.3f   hl=%.2f\n",tl_cnt,Teta_launch*r2d,Range/1e3,Hl);

			trajectory_generation_iterations = tl_cnt;
			trajectory_generation_xerr_m = rmf[0] - rtf[0];
			trajectory_generation_zerr_m = rmf[2] - rtf[2];

			/* NOTE: tripping this guard breaks out BEFORE the laun.dat write
			   further down, so the target silently keeps whatever laun.dat was
			   last left in the working directory. Teta_launch must stay above
			   50 deg -- the solve has to land on the lofted branch. */
			if(!_finite(trajectory_generation_xerr_m) ||
			   !_finite(trajectory_generation_zerr_m) ||
			   fabs(trajectory_generation_xerr_m) >= 100.0 ||
			   fabs(trajectory_generation_zerr_m) >= 100.0 ||
			   Teta_launch>85.*d2r || Teta_launch<50.*d2r)
			{
				printf("\n max iteration = %d  of Teta_launch =%.3f calc  Range=%.3f   hl=%.2f\n",tl_cnt,Teta_launch*r2d,Range/1e3,Hl);

				/* Make this failure impossible to miss.  Silently keeping the
				   previous target's files is what produced the corrupt
				   HL2T13_TAL (position/tetac byte-identical to HL2T10_TAL,
				   laun.dat to HL2T10_CMP, a 62-byte guidance.dat stub) and
				   cost a whole investigation to contaminated data, 2026-08-22. */
				printf("\n*** GENERATION ABORTED for TrgtCnt0=%d: no launch-elevation root in [50,85] deg.\n*** tetac/psic/zm/laun/guidance were NOT written -- this target still holds the PREVIOUS target's files.\n*** DO NOT FLY IT. Lower the target range or fix the solve.\n", TrgtCnt0);
				nchoice=9;
				sim_fcloseall();
				break;

				exit(0);
				return;
			}

		   Aim = 2;
            f_flag = 1.;
			
            //m_tet = sim_fopen(cearte_dir( TrgtCnt, "tetac.dat"), "wt");
			//m_z = sim_fopen(cearte_dir( TrgtCnt, "zm.dat"), "wt");
			
            m_tet = sim_fopen("tetac.dat", "wt");
            m_psi = sim_fopen("psic.dat", "wt");
            m_z = sim_fopen("zm.dat", "wt");
			
            simulate();
            sim_fclose(m_z);
            sim_fclose(m_tet);
            sim_fclose(m_psi);
			
            m_tetc = sim_fopen("tetac.dat", "r");
            m_psic = sim_fopen("psic.dat", "r");
           //m_tetc = sim_fopen(cearte_dir( TrgtCnt, "tetac.dat"), "r");
            //for (int i = 0; i < 103; i++)
            for (int i = 0; i < 133; i++)
               fscanf(m_tetc, "%lf %lf", &t_tet[i], &teta_c[i]);
            sim_fclose(m_tetc);

            for (int i = 0; i < 133; i++)
               fscanf(m_psic, "%lf %lf", &t_psi[i], &psi_c[i]);
            sim_fclose(m_psic);
            m_flag = 1.;
            Guidance_parameter();
			if(nchoice==9)
			{
				if (fd2 != NULL) { sim_fclose(fd2); fd2 = NULL; }
				if (Outfile != NULL) { sim_fclose(Outfile); Outfile = NULL; }
				if (hgv_fd != NULL) { sim_fclose(hgv_fd); hgv_fd = NULL; }
				break;
			}

            m_pos = sim_fopen("position.dat", "wt");
           // m_pos2 = sim_fopen("position2.dat", "wt");
            //m_pos = sim_fopen(cearte_dir( TrgtCnt, "position.dat"), "wt");
            f_flag = 2.;
            sl_1 = sim_fopen("sl1.dat", "wt");
            //sl_1 = sim_fopen(cearte_dir( TrgtCnt, "sl1.dat"), "wt");
            simulate();
            sim_fclose(m_pos);
           // sim_fclose(m_pos2);
            sim_fclose(sl_1);
			trajectory_generation_success = 1;
			//-----------

		  nchoice=9;
	   }
	   else if(nchoice==4)
	   {
		   //f_flag=2;

            if (launch_mod == 0)	dtl = 3.;
            else					dtl = 7.;

            if (fabs(floor(Teta0) - Teta0) < 0.5)
               Teta_launch = (floor(Teta0) + dtl) * d2r;
            else
               Teta_launch = (ceil(Teta0) + dtl) * d2r;
//*/
            
            m_tetc = sim_fopen("tetac.dat", "r");
            //for (int i = 0; i < 103; i++)
            for (int i = 0; i < 133; i++)
               fscanf(m_tetc, "%lf %lf", &t_tet[i], &teta_c[i]);
            sim_fclose(m_tetc);
            // */

            m_gui = sim_fopen("guidance.dat", "wt");
            //stg2 = 50.;//t_burn + d_j;
            stg2 =70;// t_burn + 30;
            l_flag = 0;
            choice = -1;
            m_flag = 1.;
            bal_flag = 1;
            simulate();
            Ballistic_parameter();

            // */
            sim_fclose(m_gui);
		  nchoice=9;
	   }
	   else if(nchoice==5)
	   {
		   
			m_zc = sim_fopen(getFilePath("zm.dat",FoP), "r");
			m_tetc = sim_fopen(getFilePath("tetac.dat",FoP), "r");
			m_psic = sim_fopen(getFilePath("psic.dat",FoP), "r");
			m_posc = sim_fopen(getFilePath("position.dat",FoP), "r");
			m_VburnOut = sim_fopen(getFilePath("BurnOutVelocity.dat",FoP), "r");
			sl_2 = sim_fopen(getFilePath("sl1.dat",FoP), "r");
			
			if (m_zc==NULL || m_tetc==NULL ||m_psic==NULL ||m_posc==NULL ||m_VburnOut==NULL || sl_2==NULL)
			{
			//printf("\n %s file not found \n ",getFilePath("zm.dat",FoP));
			printf("\n Guidance_parameter files not found \n ");
			return;
			}
			
			Aim=0;
		//	Teta_launch = 72.*d2r;
			/*
            if (launch_mod == 0)	dtl = 3.;
            else					dtl = 7.;

            if (fabs(floor(Teta0) - Teta0) < 0.5)
               Teta_launch = (floor(Teta0) + dtl) * d2r;
            else
               Teta_launch = (ceil(Teta0) + dtl) * d2r;//*/

            m_flag = 2;
			//Teta_launch = 72.*d2r;

            //m_zc = sim_fopen("zm.dat", "r");
 //           for (int i = 0; i < 103; i++)
            for (int i = 0; i < 133; i++)
               fscanf(m_zc, "%lf %lf %lf", &t_z[i], &z_c[i], &vz_c[i]);
		     //  fscanf(m_zc, "%lf %lf %lf %lf", &t_z[i], &z_c[i], &vz_c[i], &Vxz_c[i]);

            sim_fclose(m_zc);

           // m_tetc = sim_fopen("tetac.dat", "r");
			
           // for (int i = 0; i < 103; i++)
            for (int i = 0; i < 133; i++)
               fscanf(m_tetc, "%lf %lf", &t_tet[i], &teta_c[i]);
            sim_fclose(m_tetc);

            //m_posc = sim_fopen("position.dat", "r");
            //for (int i = 0; i < 175; i++)
            //   fscanf(m_posc, "%lf %lf %lf %lf", &t_pos[i], &x_pos[i],&y_pos[i], &v_pos[i]);
            //sim_fclose(m_posc);
        for (int i = 0; i < 133; i++)
               fscanf(m_psic, "%lf %lf", &t_psi[i], &psi_c[i]);
            sim_fclose(m_psic);
            //m_guic = sim_fopen("guidance.dat", "r");
            //fscanf(m_guic, "%lf %lf %lf %lf %lf", &t_gui, &Lx_gui, &Ly_gui,&Lv_gui, &Lgama_gui);
            //sim_fclose(m_guic);

			//m_VburnOut = sim_fopen("BurnOutVelocity.dat", "r");
			
            fscanf(m_VburnOut, "%lf %lf %lf ", &TburnOut, &VburnOut, &TetaburnOut);
            sim_fclose(m_VburnOut);

            if (rocket == 1)	Teta_launch = Teta_launch_roket * d2r;
           // sl_2 = sim_fopen("sl1.dat", "r");
			
            fscanf(sl_2, "%lf %lf ", &tgo_norm,&angle_tvc_jad);
            simulate();
            sim_fclose(sl_2);

		  if (MinMaxRng_flg) 
		  {
			nchoice=3;
			m_flag=0;
			rocket=0;
			MaxR_stg1=FRange-20e3;
			printf("\n\t final t = %.2f \t MaxR_stg1 = %.2f km \t hl = %.2f\n ",t,FRange*1e-3,Hl);

		  }
		  else nchoice=9;
//		   MinMaxR();
	   }
	 /*  else if(nchoice==6)
	   {
            if (launch_mod == 0)	dtl = 3.;
            else					dtl = 7.;

            if (fabs(floor(Teta0) - Teta0) < 0.5)
               Teta_launch = (floor(Teta0) + dtl) * d2r;
            else
               Teta_launch = (ceil(Teta0) + dtl) * d2r;

            m_flag = 2;
            m_zc = sim_fopen("zm.dat", "r");
            for (int i = 0; i < 133; i++)
               fscanf(m_zc, "%lf %lf %lf", &t_z[i], &z_c[i], &vz_c[i]);
            sim_fclose(m_zc);

            m_tetc = sim_fopen("tetac.dat", "r");
            for (int i = 0; i < 133; i++)
               fscanf(m_tetc, "%lf %lf", &t_tet[i], &teta_c[i]);
            sim_fclose(m_tetc);

            m_posc = sim_fopen("position.dat", "r");
            for (int i = 0; i < 391; i++)
               fscanf(m_posc, "%lf %lf %lf %lf", &t_pos[i], &x_pos[i],&y_pos[i], &v_pos[i]);
            sim_fclose(m_posc);

            m_guic = sim_fopen("guidance.dat", "r");
            // for(i=0;i<16;i++)
            fscanf(m_guic, "%lf %lf %lf %lf %lf", &t_gui, &Lx_gui, &Ly_gui,&Lv_gui, &Lgama_gui);
            sim_fclose(m_guic);

            CEP_Calculation();
		   nchoice=9;
//		   MinMaxR();
	   }//*/
	  // printf("\n\t end nchoice:%d \n",nchoice);
	   sim_fcloseall();
	   break;
	  // case 7:   /*Transe2() ;*/  nchoice=9;    
	  // case 8:     break;
	  // case 9:
	   default:exit(0);
			  // break;
		   //exit(0);
	 }
  }while(nchoice!=9);
}
 int m_launn(double d[],char ch[],int dim)
           {
               int i , j, p = 0, h = 0;
			  
			   char ans[80];
			  strcpy(ans,ch);
			  f1=sim_fopen(ans,"r"); for(i=0;i<dim1;i++)fscanf(f1,"%d%lf",&j,&d[i]);
		      sim_fclose(f1);
			
			  return 0;

           }

  void Print_sim(void)
{

	if(nchoice!=5) 
	{ 
		Xer=rmf[0]-rtf[0];//d_m_[0];//
		Zer=rmf[2]-rtf[2];//d_m_[2];////*/
		//show_data(1);
	}
}
  void Print_ballistic(void)
{
	show_data(3);


}
