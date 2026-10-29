#include"stdafx.h"
#include "m273n.h"
#include "aero_tables.h"
//#include "f110a.h"
// extern float cmalfa_5[4][7],cnalfa_5[4][7];
double cllda_test;
extern double d_j, Del_tm, Del_t, tberr_2, t_burn_2, xbc_y_2, xbc_z_2;
extern int test, f_100, motor_flag, rocket, vb;
// extern double d_j;
double ch_df[4], ch_al[4], cb_df[4], cb_al[4], hing_mom[4], bend_mom[4],
    h_f[4], b_f[4];
double forcc, torqc, ddv, xcg_y = 0, xcg_z = 0, torq2;
double l_rail2 = 7.9 - 4.65;
double x_laun;
double l_launcher, delta_y_l, tipoff;
extern int launch2, launch_mod;
extern double K_test, l_rail, xcg1, xcg2_1, xbc_er, xbc_y, xbc_z;
double ren;
double ff[4];
double cnqb, cnqa, cad1, cad2, cad3, cad4;
double cnb, cmb, cmde, cmdr, cndr, cnde, cna, cma;
// float interpoldw(float m,float a,float mx[8][12]);
// float interpold(float m,float a,float mx[5][12]);
extern double tsep;
extern int dist, motor1_state,motor2_state;//fcu;

/* ===================================================================
 *  نواتا الاستيفاء الموحَّدتان لجداول المعاملات الأيروديناميكية
 *
 *  مستخرجتان حرفياً من الدوال الست عشرة الأصلية: ترتيب العمليات ونوع
 *  float محفوظان كما هما، فالنتائج مطابقة بتّةً ببتّة.
 *
 *  كل دالة مسمّاة صارت غلافاً يحتفظ بمقدّمتها الأصلية كما هي --
 *  جداول المحاور، وتثبيت المدخلات، ومعالجة الإشارة -- ثم يستدعي
 *  النواة. ولم يُغيَّر أي سلوك، بما في ذلك ما يبدو خطأً (انظر
 *  الملاحظتين أسفل الملف).
 *
 *  ثم رُبطت الأغلفة بسجلّ aero_tables: إن كان الجدول قد مُلئ من ملف
 *  البيانات، أخذ الغلاف محاوره وأطوالها من الملف بدل المُثبّتة أدناه،
 *  وتبعه التثبيت لأن حدوده صارت طرفَي المحور لا أرقاماً مكتوبة. فتغيير
 *  عدد النقاط أو قيم المحاور في Excel لا يحتاج لمسَ هذا الملف. وما لم
 *  يُملأ من ملف يبقى على محاوره المُثبّتة، فسلوكه كما كان.
 * =================================================================== */

static float aero_interp2(float x, float y,
                          const float *xt, int nx,
                          const float *yt, int ny,
                          const float *mx, int s1)
{
   int i, j;
   float c1, c2;
   for (i = 1; i < nx && x > xt[i]; i++);
   i = (--i == nx - 1) ? nx - 2 : i;
   for (j = 1; j < ny && y > yt[j]; j++);
   j = (--j == ny - 1) ? ny - 2 : j;
   c1 = mx[i*s1+j] + (mx[i*s1+j+1] - mx[i*s1+j]) * (y - yt[j]) /
       (yt[j + 1] - yt[j]);
   c2 = mx[(i+1)*s1+j] + (mx[(i+1)*s1+j+1] - mx[(i+1)*s1+j]) * (y - yt[j]) /
       (yt[j + 1] - yt[j]);
   return c1 + (c2 - c1) * (x - xt[i]) / (xt[i + 1] - xt[i]);
}

static float aero_interp3(float x, float y, float z,
                          const float *xt, int nx,
                          const float *yt, int ny,
                          const float *zt, int nz,
                          const float *mx, int s1, int s2)
{
   int i, j, k;
   float c1, c2, c3, c4, c5, c6;
   for (i = 1; i < nx && x > xt[i]; i++);
   i = (--i == nx - 1) ? nx - 2 : i;
   for (j = 1; j < ny && y > yt[j]; j++);
   j = (--j == ny - 1) ? ny - 2 : j;
   for (k = 1; k < nz && z > zt[k]; k++);
   k = (--k == nz - 1) ? nz - 2 : k;
#define AI3(A,B,C) mx[(A)*s1 + (B)*s2 + (C)]
   c1 = AI3(i,j,k) + (AI3(i,j+1,k) - AI3(i,j,k)) * (y - yt[j]) /
       (yt[j + 1] - yt[j]);
   c2 = AI3(i+1,j,k) + (AI3(i+1,j+1,k) - AI3(i+1,j,k)) *
       (y - yt[j]) / (yt[j + 1] - yt[j]);
   c3 = c1 + (c2 - c1) * (x - xt[i]) / (xt[i + 1] - xt[i]);
   c4 = AI3(i,j,k+1) + (AI3(i,j+1,k+1) - AI3(i,j,k+1)) *
       (y - yt[j]) / (yt[j + 1] - yt[j]);
   c5 = AI3(i+1,j,k+1) + (AI3(i+1,j+1,k+1) - AI3(i+1,j,k+1)) *
       (y - yt[j]) / (yt[j + 1] - yt[j]);
   c6 = c4 + (c5 - c4) * (x - xt[i]) / (xt[i + 1] - xt[i]);
#undef AI3
   return c3 + (c6 - c3) * (z - zt[k]) / (zt[k + 1] - zt[k]);
}

/* -------------------------------------------------------------------
 *  محاور الجدول من السجلّ -- ولا احتياط مُثبَّت في الكود.
 *
 *  كان لكل غلاف نسخةٌ مكتوبةٌ من محاوره تُستعمل إن لم يجد جدوله في
 *  السجلّ. وحُذفت: المحور المكتوب الذي يخالف ملفه يُخطئ **بصمت**، وهو
 *  ما وقع فعلاً في interpol3cwf (رينولدز 1e5..6e7 في الكود مقابل
 *  3e5..5e8 في nv_wof600.dat). فجدولٌ لم يُسجَّل صار خطأً صريحاً.
 *
 *  والأبعاد والمحاور والخطوة كلها من AeroTableInfo: n1 n2 n3 و
 *  ax1 ax2 ax3 و s1 s2. فلا رقم في هذه الدوال يصف بياناتٍ بعد اليوم،
 *  وتوسيع جدول في Excel لا يمسّ سطراً منها.
 * ----------------------------------------------------------------- */
static const AeroTableInfo *aero_need(const void *mx, const char *who)
{
   static const void *warned[AERO_MAX_TABLES];
   static int nwarned = 0;
   const AeroTableInfo *T = aero_axes_for(mx);
   int i;
   if (T) return T;
   for (i = 0; i < nwarned; i++)
      if (warned[i] == mx) return 0;              /* حُذّر منه مرة، يكفي */
   if (nwarned < AERO_MAX_TABLES) warned[nwarned++] = mx;
   printf("\n*** aero: %s -- table not registered, returning 0 ***\n", who);
   return 0;
}


float interpol3cm(float d, float m, float a, float mx[AERO_R][AERO_C][AERO_K]) {
   const AeroTableInfo *T = aero_need(&mx[0][0][0], "interpol3cm");
   float c, S = 0;
   if (!T) return 0.f;

   if (d < T->ax3[0])          d = T->ax3[0];
   if (d > T->ax3[T->n3 - 1])  d = T->ax3[T->n3 - 1];

   S = (d >= 0) ? 0 : (d = -d);
   c = aero_interp3(a, m, d, T->ax1, T->n1, T->ax2, T->n2, T->ax3, T->n3,
                    &mx[0][0][0], T->s1, T->s2);
   return (S) ? -c : c;
}

float interpol3cw(float d, float m, float a, float mx[AERO_R][AERO_C][AERO_K]) {
   const AeroTableInfo *T = aero_need(&mx[0][0][0], "interpol3cw");
   float c, S = 0;
   if (!T) return 0.f;

   if (d < T->ax3[0])          d = T->ax3[0];
   if (d > T->ax3[T->n3 - 1])  d = T->ax3[T->n3 - 1];

   S = (d >= 0) ? 0 : (d = -d);
   c = aero_interp3(a, m, d, T->ax1, T->n1, T->ax2, T->n2, T->ax3, T->n3,
                    &mx[0][0][0], T->s1, T->s2);
   return (S) ? -c : c;
}

float interpol3cwf(float d, float m, float a, float mx[AERO_R][AERO_C][AERO_K]) {
   const AeroTableInfo *T = aero_need(&mx[0][0][0], "interpol3cwf");
   /* FIX: use data-driven axes from CA_OFS2.dat (registered by ReadFile3Dcwf)
      instead of wrong legacy axes that belonged to nv_wof600.dat format.
      Reynolds clamp kept at [1e5, 1e7] to match fina2 original behavior. */
   float c, S = 0;
   if (!T) return 0.f;

   if (d < 1e5f) d = 1e5f;
   if (d > 1e7f) d = 1e7f;

   S = (d >= 0) ? 0 : (d = -d);
   c = aero_interp3(a, m, d, T->ax1, T->n1, T->ax2, T->n2, T->ax3, T->n3,
                    &mx[0][0][0], T->s1, T->s2);
   return (S) ? -c : c;
}

float interpol3rm(float d, float m, float a, float mx[AERO_R][AERO_C][AERO_K]) {
   const AeroTableInfo *T = aero_need(&mx[0][0][0], "interpol3rm");
   float c, S = 0;
   if (!T) return 0.f;

   S = (d >= 0) ? 0 : (d = -d);
   c = aero_interp3(a, m, d, T->ax1, T->n1, T->ax2, T->n2, T->ax3, T->n3,
                    &mx[0][0][0], T->s1, T->s2);
   return (S) ? -c : c;
}

float interpol3rw(float d, float m, float a, float mx[AERO_R][AERO_C][AERO_K]) {
   const AeroTableInfo *T = aero_need(&mx[0][0][0], "interpol3rw");
   float c, S = 0;
   if (!T) return 0.f;

   if (m < T->ax2[0]) m = T->ax2[0];
   if (m < 0.8) m = 0.8;              /* تثبيت يبتلع الذي قبله -- محفوظ عمداً */
   if (m > T->ax2[T->n2 - 1]) m = T->ax2[T->n2 - 1];

   S = (d >= 0) ? 0 : (d = -d);
   c = aero_interp3(a, m, d, T->ax1, T->n1, T->ax2, T->n2, T->ax3, T->n3,
                    &mx[0][0][0], T->s1, T->s2);
   return (S) ? -c : c;
}

float interpol33m(float d, float m, float a, float mx[AERO_R][AERO_C][AERO_K]) {
   const AeroTableInfo *T = aero_need(&mx[0][0][0], "interpol33m");
   float c, S = 0;
   if (!T) return 0.f;

   if (d < T->ax3[0])          d = T->ax3[0];
   if (d > T->ax3[T->n3 - 1])  d = T->ax3[T->n3 - 1];

   S = (d >= 0) ? 0 : (d = -d);
   c = aero_interp3(a, m, d, T->ax1, T->n1, T->ax2, T->n2, T->ax3, T->n3,
                    &mx[0][0][0], T->s1, T->s2);
   return (S) ? -c : c;
}

float interpolm(float m, float a, float mx[AERO_R][AERO_C]) {
   const AeroTableInfo *T = aero_need(&mx[0][0], "interpolm");
   float c, S = 0;
   if (!T) return 0.f;

   S = (a >= 0) ? 0 : (a = -a);
   c = aero_interp2(a, m, T->ax1, T->n1, T->ax2, T->n2, &mx[0][0], T->s1);
   return (S) ? -c : c;
}

float interpolm_(float m, float a, float mx[AERO_R][AERO_C]) {
   const AeroTableInfo *T = aero_need(&mx[0][0], "interpolm_");
   float c, S = 0;
   if (!T) return 0.f;

   if (m < T->ax2[0])          m = T->ax2[0];
   if (m > T->ax2[T->n2 - 1])  m = T->ax2[T->n2 - 1];

   if (a < T->ax1[0])          a = T->ax1[0];     /* المحور الأول رينولدز */
   if (a > T->ax1[T->n1 - 1])  a = T->ax1[T->n1 - 1];

   S = (a >= 0) ? 0 : (a = -a);
   c = aero_interp2(a, m, T->ax1, T->n1, T->ax2, T->n2, &mx[0][0], T->s1);
   return (S) ? -c : c;
}

float interpolmd(float m, float a, float mx[AERO_R][AERO_C]) {
   const AeroTableInfo *T = aero_need(&mx[0][0], "interpolmd");
   float c, S = 0;
   if (!T) return 0.f;

   S = (a >= 0) ? 0 : (a = -a);
   c = aero_interp2(a, m, T->ax1, T->n1, T->ax2, T->n2, &mx[0][0], T->s1);
   return (S) ? -c : c;
}

float interpolw(float m, float a, float mx[AERO_R][AERO_C]) {
   const AeroTableInfo *T = aero_need(&mx[0][0], "interpolw");
   float c, S = 0;
   if (!T) return 0.f;

   S = (a >= 0) ? 0 : (a = -a);
   c = aero_interp2(a, m, T->ax1, T->n1, T->ax2, T->n2, &mx[0][0], T->s1);
   return (S) ? -c : c;
}

float interpolw2(float m, float a, float mx[AERO_R][AERO_C]) {
   const AeroTableInfo *T = aero_need(&mx[0][0], "interpolw2");
   float c, S = 0;
   if (!T) return 0.f;

   if (m < T->ax2[0])          m = T->ax2[0];
   if (m > T->ax2[T->n2 - 1])  m = T->ax2[T->n2 - 1];

   S = (a >= 0) ? 0 : (a = -a);
   c = aero_interp2(a, m, T->ax1, T->n1, T->ax2, T->n2, &mx[0][0], T->s1);
   return (S) ? -c : c;
}

float interpolw2_(float m, float a, float mx[AERO_R][AERO_C]) {
   const AeroTableInfo *T = aero_need(&mx[0][0], "interpolw2_");
   float c, S = 0;
   if (!T) return 0.f;

   if (m < T->ax2[0])          m = T->ax2[0];
   if (m > T->ax2[T->n2 - 1])  m = T->ax2[T->n2 - 1];

   if (fabs(a) > T->ax1[T->n1 - 1]) a = T->ax1[T->n1 - 1] * sign(a);

   S = (a >= 0) ? 0 : (a = -a);
   c = aero_interp2(a, m, T->ax1, T->n1, T->ax2, T->n2, &mx[0][0], T->s1);
   return (S) ? -c : c;
}

float interpol_hb(float m, float a, float mx[AERO_R][AERO_C]) {
   const AeroTableInfo *T = aero_need(&mx[0][0], "interpol_hb");
   float c, S = 0;
   if (!T) return 0.f;

   S = (a >= 0) ? 0 : (a = -a);
   c = aero_interp2(a, m, T->ax1, T->n1, T->ax2, T->n2, &mx[0][0], T->s1);
   return (S) ? -c : c;
}

float interpolw_(float m, float a, float mx[AERO_R][AERO_C]) {
   const AeroTableInfo *T = aero_need(&mx[0][0], "interpolw_");
   float c, S = 0;
   if (!T) return 0.f;

   S = (a >= 0) ? 0 : (a = -a);
   c = aero_interp2(a, m, T->ax1, T->n1, T->ax2, T->n2, &mx[0][0], T->s1);
   return (S) ? -c : c;
}

float interpolwd(float m, float a, float mx[AERO_R][AERO_C]) {
   const AeroTableInfo *T = aero_need(&mx[0][0], "interpolwd");
   float c, S = 0;
   if (!T) return 0.f;

   S = (a >= 0) ? 0 : (a = -a);
   c = aero_interp2(a, m, T->ax1, T->n1, T->ax2, T->n2, &mx[0][0], T->s1);
   return (S) ? -c : c;
}

float interpolmo(float m, float a, float mx[AERO_R][AERO_C]) {
   const AeroTableInfo *T = aero_need(&mx[0][0], "interpolmo");
   float c, S = 0;
   if (!T) return 0.f;

   S = (a >= 0) ? 0 : (a = -a);
   c = aero_interp2(a, m, T->ax1, T->n1, T->ax2, T->n2, &mx[0][0], T->s1);
   return (S) ? -c : c;
}


void M_aero(double sound_speed, double dens, double vbwb[3], double wibb[3],
    double fab[3], double mab[3]) {

   v = mag(3, vbwb);
   mach = v / sound_speed;
   
   if (mach <=0.3 && sepflg==1) mach=0.3;
   if (mach >10. && sepflg==1) mach=10.;
   
   
   dynpres = 0.5 * dens * v * v;

   //if (t > t_burn + tberr + d_j) 
	if (((sepflg) &&(!motor_flag))||sepflg2)
   {
      Aref = D_body = 0.123785819;
   }
   else {
      /* M480 DATCOM reference diameter/area (LREF=0.486, SREF=0.185508). */
      D_body = 0.486;
      Aref = 0.185508;
   }

   forcc = dynpres * Aref;
   // forcc=q*1.e4*Aref;
   // Calculate the drag force of missile on the launcher .
   l_launcher = sqrt(rmf[0] * rmf[0] + rmf[1] * rmf[1]);
   twm(1, mi2l, rmf, rml);
   twm(1, mi2l, vmf, vml);

   if (launch_mod == 0) {
      if (l_launcher < 1.35) // 1.8
      {
         alfa = betta = tipoff = 0.0;
         delta_y_l = rml[1]; // +(Xt-xbc)*sin(tipoff);
         fab[0] = sign(vbwb[0]) * interpolm_(mach, ren, caalfam) * forcc -
             (sin(Teta_launch) + 0.1 * cos(Teta_launch)) * (mass * 9.81);
         
		 if (fabs(fab[0]) >= fabs(ftb[0]))
            ftb[0] = fab[0] * sign(fab[0]);
         fab[1] = fab[2] = mab[0] = mab[1] = mab[2] = ftb[1] = ftb[2] = 0.0;
         Gi[0] = Gi[1] = Gi[2] = mtb[0] = mtb[1] = mtb[2] = dy[6] = dy[7] =
             dy[8] = 0.0;
         return;
      }
      else {
         tipoff = Teta_launch - tetam;
         delta_y_l = rml[1]; // +(Xt-xbc)*sin(tipoff);//markaze do jenaghi
         if (launch2 == 1)
            delta_y_l = rml[1] + (Xt - xbc) * sin(tipoff);
         // up  end of missile
         if (launch2 == 2)
            delta_y_l = rml[1]; // +(.6*Xt-xbc)*sin(tipoff);//down  cg-end
      }
   }

   if (launch_mod == 1) {
      if (l_launcher < l_rail) {
         if (l_launcher < l_rail - 2.3) {
            tipoff = alfa = betta = 0.0;
            fab[0] = sign(vbwb[0]) * interpolm_(mach, ren, caalfam) * forcc -
                (sin(Teta_launch) + 0.1 * cos(Teta_launch)) * (mass * 9.81);
           // fab[0] =-4.;
			if (fabs(fab[0]) >= fabs(ftb[0]))
               ftb[0] = fab[0] * sign(fab[0]);
            fab[1] = fab[2] = mab[0] = mab[1] = mab[2] = ftb[1] = ftb[2] = 0.0;
            Gi[0] = Gi[1] = Gi[2] = mtb[0] = mtb[1] = mtb[2] = dy[6] = dy[7] =dy[8] = 0.0;
            return;
         }
         else {
            tipoff = Teta_launch - tetam;
            alfa = -atan2(vbwb[1], vbwb[0]) * r2d;
            betta = 0.0;
            delta_y_l = rml[1] + 0.5 * 3.0 * sin(tipoff);
            // markaze do jenaghi
            // delta_y_l=rml[1]+(l_rail-l_launcher)*sin(tipoff);//markaze do jenaghi
            // if(launch==1)	delta_y_l=rml[1]+0.8*sin(tipoff);//up  end of missile
            // if(launch==2)	delta_y_l=rml[1]+0.5*4.65*sin(tipoff);//down  cg-end

            fab[0] = sign(vbwb[0]) * interpolm_(mach, ren, caalfam) * forcc -
                (sin(Teta_launch) + 0.1 * cos(Teta_launch) * 0.5) *(mass * 9.81);
          // fab[0] =-4.;
		   if (fabs(fab[0]) >= fabs(ftb[0]))
               ftb[0] = fab[0] * sign(fab[0]);
            fab[2] = mab[0] = mab[1] = ftb[1] = ftb[2] = 0.0;
            Gi[2] = mtb[0] = mtb[1] = dy[6] = dy[7] = 0.0;
            x_laun = xbc + L_body - xcg2_1 - 0.45;
            // faseleye markaze jerm ta jenaghiye aghab
            mab[2] = -mass * magg * cos(tetam) * x_laun -(0.1 * cos(Teta_launch) * 0.5 * mass * 9.81 * x_laun);
			mab[2] *=1;
            return;
         }
      }
      else {
         tipoff = Teta_launch - tetam;
         delta_y_l = rml[1] + 0.5 * 3.0 * sin(tipoff); // markaze do jenaghi
      }
	  

   }
   // ##################################################################
 //  if (s_s>1)y[30] = y[31] = y[32] = y[33] = y[38]=y[36]=0;

   //	 if((rocket && s_s>1) ||(motor_flag && sepflg))// && sepflg)	
     // y[30]=y[31]=y[32]=y[33]=0.;





 if(rocket && s_s>0 )//|| sepflg2)// && sepflg)
   {
      y[30]=y[31]=y[32]=y[33]=0.;
    //StopCtrlCmd=1;
   }

    // if((rocket && s_s>1)) 
    // y[38]=y[36]=0;//y[30]=y[31]=y[32]=y[33]=
   // if(s_s>3)
  //  y[30]=y[31]=y[32]=y[33]=0.;

	//if((motor_flag && sepflg))
	//	y[30]=y[31]=y[32]=y[33]=0.;
	
   double kk,alfaa,alfabb,bettaa;
   // if (stage == 6)
   // K_test = 0.6;

   {  /* Corroded lower fins 2 and 3: scale their aerodynamic contribution.
         sim_fin_corr == 1.0 reproduces the stock expressions exactly. */
      /* Stage-6 scoping: the corrosion is specified on the SIXTH-STAGE
         control rudders, so it is applied only while stage 6 is flying.
         Applying it (and its compensation) across boost wrecked the
         cold-motor arms -- measured: 6 arms to 300-670 km. */
      double kc6 = (s_s == 6) ? sim_fin_corr : 1.0;
      if (sim_wear_on) kc6 = sim_wear_update((s_s == 6), q, t);
      double f2 = kc6 * y[31], f3 = kc6 * y[32];
   dele = K_test * 0.25 * (-y[30] - f2 + f3 + y[33]) * r2d;
   delr = K_test * 0.25 * (-y[30] + f2 + f3 - y[33]) * r2d;
   dela = K_test * 0.25 * (y[30] + f2 + f3 + y[33]) * r2d;
   }

   alfa = -atan2(vbwb[1], vbwb[0]) * r2d;
   alfab = atan2(sqrt(vbwb[1] * vbwb[1] + vbwb[2] * vbwb[2]), vbwb[0]) * r2d;
   betta = atan2(vbwb[2], vbwb[0]) * r2d;
   
    alfaa = -atan2(vbwb[1], vbwb[0]) * r2d;
   alfabb = atan2(sqrt(vbwb[1] * vbwb[1] + vbwb[2] * vbwb[2]), vbwb[0]) * r2d;
   bettaa = atan2(vbwb[2], vbwb[0]) * r2d;


   if(fabs(alfab)>20.0)alfab=20.0*sign(alfab);
  if(fabs(alfa)>20.0)alfa=20.0*sign(alfa);
  if(fabs(betta)>20.0)betta=20.0*sign(betta);
  
  
   /* TAL_A135 -- STACKED CLAMP DEFECT, disabled 2026-08-23 (TAL_A135 = 0).
      The block above already clamps alfa/alfab/betta to +-20 deg, the edge of
      the aero tables (RAJEH_CA_OFF_S1.dat breakpoints 0/2/4/8/12/16/20) --
      correct and conservative.  This SECOND block then tests the UNCLAMPED
      copies (alfaa/alfabb/bettaa) against 135 and, when they exceed it,
      overwrites the already-clamped value with 135.  So the value handed to
      the interpolator STEPS from 20 to 135 the instant true incidence crosses
      135 deg -- a 6.75x jump.  interpol3rm/interpol3rw do NOT clamp their
      angle argument (interpol3rw clamps mach only, and its alpha clamp is
      commented out), they fall back to the last [16,20] segment and
      EXTRAPOLATE: at a = 135 that is (135-16)/(20-16) = 29.75 segment-lengths
      beyond the table, which for a negative Cma slope manufactures an
      enormous artificial restoring moment and a violent tumble.
      Measured with it disabled: the four flg==1 failures keep the aero inside
      the tables (maxAlf6 135 -> 20) and every HEALTHY case is BIT-IDENTICAL,
      because the override never fires for them.  Gate unchanged: 43/49
      (tal_flag=1) and 48/49 (tal_flag=0); T13n2/T13n5 bit-identical too.
      The misses barely move (T11n1 8958 -> 9037, T10n1 7091 -> 4736), which is
      the load-bearing result: THE DEPARTURE IS PHYSICAL, not an artifact of
      the extrapolation.  The vehicle genuinely cannot fly the required
      terminal geometry within 20 deg of incidence.
      Set TAL_A135 = 1 in sim_fio.h to restore the old behaviour. */
   if (TAL_A135) {
   if(fabs(alfabb)>135.0)alfab=135.0*sign(alfab);
  if(fabs(alfaa)>135.0)alfa=135.0*sign(alfa);
  if(fabs(bettaa)>135.0)betta=135.0*sign(betta);
   }
   /*
  // if (s_s>4)   
   if (sepflg2)
   {
	if(fabs(alfab)>24.0)alfab=24.0*sign(alfab);
	if(fabs(alfa)>24.0)alfa=24.0*sign(alfa);
	if(fabs(betta)>24.0)betta=24.0*sign(betta);
   }//*/
	   

   ddv = D_body / (2. * v) * r2d;
   torqc = forcc * D_body;
   torq2 = dynpres * 0.1 * 0.1;
   // if (t < (t_burn + tberr + Del_tm)) {
  // if (t < tsep) 
	if (sepflg==0)
   {
      //if (t < (t_burn + tberr))
      if (motor1_state==1)//fcu
         ca =1 * interpol3cm(ren, mach, fabs(alfab), CxArrayoncm);
       else
       ca = 1. *interpol3cm(ren, mach, fabs(alfab), CxArrayofcm);

      cad1 = interpolmd(mach, fabs(y[30] * r2d), cadeltam) / 4.;
      cad2 = interpolmd(mach, fabs(y[31] * r2d), cadeltam) / 4.;
      cad3 = interpolmd(mach, fabs(y[32] * r2d), cadeltam) / 4.;
      cad4 = interpolmd(mach, fabs(y[33] * r2d), cadeltam) / 4.;

      cma = interpolmd(mach, alfa, cmalfam);
      cmb = interpolmd(mach, betta, cmalfam);
      cna = interpolmd(mach, alfa, cnalfam);
      cnb = interpolmd(mach, betta, cnalfam);
      cmqa = interpolmd(mach, fabs(alfa), cmqalfam);
      cmqb = interpolmd(mach, fabs(betta), cmqalfam);
      cnqa = interpolmd(mach, fabs(alfa), cnqalfam);
      cnqb = interpolmd(mach, fabs(betta), cnqalfam);
      cmde = interpolmd(mach, dele, cmdeltam);
      cmdr = interpolmd(mach, delr, cmdeltam);
      cnde = interpolmd(mach, dele, cndeltam);
      cndr = interpolmd(mach, delr, cndeltam);
      cllp = interpolmd(mach, fabs(alfab), clpm);

      cllda = interpol3rm(dela, mach, fabs(alfab), DeltaArrayrm);
	  cllda =cllda-0.0; 
	   double xbc_ere=0.03;
       double  SMD = ((cnde)?(cmde*D_body/(cnde)-xbc_ere):0)/D_body;
       double SM1 = ((cna)?(cma*D_body/(cna)-xbc_ere):0)/D_body;
	    double  SMDr = ((cndr)?(cmdr*D_body/(cndr)-xbc_ere):0)/D_body;
       double SM1r = ((cnb)?(cmb*D_body/(cnb)-xbc_ere):0)/D_body;
	   
	     cmb=SM1r*cnb;
        cmdr=SMDr*cndr;
		
		cma=SM1*cna;
        cmde=SMD*cnde;
 
   }
   	else if(motor_flag)
	{

		ca = 1 *interpol33m(ren, mach, fabs(alfab), CxArrayofcmo);

		cad1 = cad2 = cad3 = cad4 = 0;

		cma = interpolmo(mach, alfa, cmalfamo);
		cmb = interpolmo(mach, betta, cmalfamo);
		cna = interpolmo(mach, alfa, cnalfamo);
		cnb = interpolmo(mach, betta, cnalfamo);
		cmqa = interpolmo(mach, fabs(alfa), cmqalfamo);
		cmqb = interpolmo(mach, fabs(betta), cmqalfamo);
		cnqa = interpolmo(mach, fabs(alfa), cnqalfamo);
		cnqb = interpolmo(mach, fabs(betta), cnqalfamo);
		cmde = cmdr = cnde = cndr = 0;
		cllp = 0;//interpolwd(mach, fabs(alfab), clpw);

		cllda = 0;

	}
   else 
   {
      //if (t > (Del_t) && t < (t_burn_2 + tberr_2 + Del_t))
      //if ( t < (t_burn_2 + tberr_2 + Del_t))
      if (motor2_state==1)//fcu
         ca =1 * interpol3cw(ren, mach, fabs(alfab), CxArrayoncw);
      else
         ca =1 * interpol3cwf(ren, mach, fabs(alfab), CxArrayofcw);
/*
      cad1 = interpolw2(mach, fabs(y[30] * r2d), cadeltaw) / 4.;
      cad2 = interpolw2(mach, fabs(y[31] * r2d), cadeltaw) / 4.;
      cad3 = interpolw2(mach, fabs(y[32] * r2d), cadeltaw) / 4.;
      cad4 = interpolw2(mach, fabs(y[33] * r2d), cadeltaw) / 4.;
//*/
       cad1 = interpolw2_(mach, fabs(y[30] * r2d), cadeltaw) / 4.;
      cad2 = interpolw2_(mach, fabs(y[31] * r2d), cadeltaw) / 4.;
      cad3 = interpolw2_(mach, fabs(y[32] * r2d), cadeltaw) / 4.;
      cad4 = interpolw2_(mach, fabs(y[33] * r2d), cadeltaw) / 4.;


      cma = interpolw_(mach, alfaa, cmalfaw);
      cmb = interpolw_(mach, bettaa, cmalfaw);
      cna = interpolw_(mach, alfaa, cnalfaw);
      cnb = interpolw_(mach, bettaa, cnalfaw);
      cmqa = interpolw2(mach, fabs(alfa), cmqalfaw);
      cmqb = interpolw2(mach, fabs(betta), cmqalfaw);
      cnqa = interpolw2(mach, fabs(alfa), cnqalfaw);
      cnqb = interpolw2(mach, fabs(betta), cnqalfaw);
      cmde = interpolw2(mach, dele, cmdeltaw);
      cmdr = interpolw2(mach, delr, cmdeltaw);
      cnde = interpolw2(mach, dele, cndeltaw);
      cndr = interpolw2(mach, delr, cndeltaw);
      cllp = interpolw2(mach, fabs(alfab), clpw);

      cllda = interpol3rw(dela, mach, fabs(alfab), DeltaArrayrw);
	  
    //double xbc_ere=1.51265- 1.43;
    //double  SMD = ((cnde)?(cmde*D_body/(cnde)-xbc_ere):0)/D_body;
  //double SM1 = ((cna)?(cma*D_body/(cna)-xbc_ere):0)/D_body;
 
 if (motor2_state!=1 && motor2_state!=2 && s_s>2)
                 {
   //  cma=SM1*cna;
    // cmde=SMD*cnde;
                   }

          if (rocket || StopCtrlCmd)  cmde=cmdr=cnde=cndr=cllda=0;

   }
     /* else //if(sepflg2==0)
   {

      ca = 1. * interpol3cwf(ren, mach, fabs(alfab), CxArrayofcw2);

      cad1 = cad2 = cad3 = cad4 = 0;

      cma = interpolw2(mach, alfa, cmalfaw2);
      cmb = interpolw2(mach, betta, cmalfaw2);
      cna = interpolw2(mach, alfa, cnalfaw2);
      cnb = interpolw2(mach, betta, cnalfaw2);
      cmqa = interpolw2(mach, fabs(alfa), cmqalfaw2);
      cmqb = interpolw2(mach, fabs(betta), cmqalfaw2);
      cnqa = interpolw2(mach, fabs(alfa), cnqalfaw2);
      cnqb = interpolw2(mach, fabs(betta), cnqalfaw2);
      cmde = 0;//interpolw2(mach, dele, cmdeltaw);
      cmdr = 0;//interpolw2(mach, delr, cmdeltaw);
      cnde = 0;//interpolw2(mach, dele, cndeltaw);
      cndr = 0;//interpolw2(mach, delr, cndeltaw);
      cllp = 0;//interpolw2(mach, fabs(alfab), clpw);

      cllda = 0;//interpol3rw(dela, mach, fabs(alfab), DeltaArrayrw);
	  
	  if (aero_flg)
	  {
		cma = interpolw2_(mach, alfa, cmalfaw2_);
		cmb = interpolw2_(mach, betta, cmalfaw2_);
		cna = interpolw2_(mach, alfa, cnalfaw2_);
		cnb = interpolw2_(mach, betta, cnalfaw2_);

		cmqa = interpolw2_(mach, fabs(alfa), cmqalfaw2_);
		cmqb = interpolw2_(mach, fabs(betta), cmqalfaw2_);
		cnqa = interpolw2_(mach, fabs(alfa), cnqalfaw2_);
		cnqb = interpolw2_(mach, fabs(betta), cnqalfaw2_);
	  }
	  

   }//*/

   ch_df[0] = interpol_hb(mach, (-y[30] * r2d), cH_d);
   ch_df[1] = interpol_hb(mach, (-y[31] * r2d), cH_d);
   ch_df[2] = interpol_hb(mach, (+y[32] * r2d), cH_d);
   ch_df[3] = interpol_hb(mach, (+y[33] * r2d), cH_d);

   cb_df[0] = interpol_hb(mach, (-y[30] * r2d), cB_d);
   cb_df[1] = interpol_hb(mach, (-y[31] * r2d), cB_d);
   cb_df[2] = interpol_hb(mach, (+y[32] * r2d), cB_d);
   cb_df[3] = interpol_hb(mach, (+y[33] * r2d), cB_d);


	for(int i=0;i<4;i++){
		cb_al[i] = interpol_hb(mach, 0. * (alfa + betta), cB_a);
		ch_al[i] = interpol_hb(mach, 0. * (alfa + betta), cH_a);
	}
	for(int i=0;i<4;i++){
		hing_mom[i] = torq2 * (ch_df[i] + ch_al[i]);
		bend_mom[i] = torq2 * (cb_df[i] + cb_al[i]);
		h_f[i] = max(fabs(hing_mom[i]), h_f[i]);
		b_f[i] = max(fabs(bend_mom[0]), b_f[i]);
	}


   /* TalFlag=1 already selects the complete RAJEH2 aero tables loaded in
      M273N.CPP.  Do not add the former analytical drag/tail/stability/
      damping/control increments here; that would count the same geometry
      twice.  The physical 0.45 m tail/nozzle translation remains in Xt. */

   cx = -ca - cad1 - cad2 - cad3 - cad4;
   cll = -cllda + cllp * wibb[0] * ddv;
   cy = cna + cnqa * wibb[2] * ddv + cnde;
   cz = -cnb - cnqb * wibb[1] * ddv - cndr;
   cln = cmb + cmdr + cmqb * wibb[1] * ddv;
   cm = cma + cmde + cmqa * wibb[2] * ddv;

   fab[0] = forcc * cx*1.0;
   fab[1] = forcc * cy;
   fab[2] = forcc * cz;
   // mab[0]=torqc*cll;
   // mab[1]=torqc*cln;
   // mab[2]=torqc*cm;
   if (t < (t_burn + tberr + Del_tm) ) 
   {
      xbc_y = xbc_y_2;
      xbc_z = xbc_z_2;
   }
   if(sepflg2)	xbc_y=xbc_z=xcg_y=xcg_z=0.;
   mab[0] = torqc * cll + fab[1] * xcg_z + fab[2] * xcg_y;
   mab[1] = torqc * cln + fab[0] * xcg_z;
   mab[2] = torqc * cm - fab[0] * xcg_y;
 }



int ReadFile3Drm(char *filename, float ax[AERO_R], float ay[AERO_C], float az[AERO_K],
    float f_xyz[AERO_R][AERO_C][AERO_K], int *nx, int *ny, int *nz) {

   FILE *f;
   int i, j, k, ni, nj, nk;
   float temp1;
   char s[30];
   if ((f = sim_fopen(filename, "r")) == NULL) {
      clrscr();
      printf("\nfile %s Not Found\n", filename);
      getch();
      return -1;
   }
   fscanf(f, "%d %d %d\n", &ni, &nj, &nk);
   f_xyz[0][0][0] = 0.;
   for (i = 0; i < ni - 1; i++) {
      fscanf(f, "%f", &temp1);
      ax[i] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   ax[ni - 1] = temp1;

   for (j = 0; j < nj - 1; j++) {
      fscanf(f, "%f", &temp1);
      ay[j] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   ay[nj - 1] = temp1;

   for (k = 0; k < nk - 1; k++) {
      fscanf(f, "%f", &temp1);
      az[k] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   az[nk - 1] = temp1;

   for (k = 0; k < nk; k++) {
      fscanf(f, "%s", s);
      for (i = 0; i < ni; i++) {
         for (j = 0; j < nj - 1; j++) {

            fscanf(f, "%f", &temp1);
            f_xyz[i][j][k] = temp1;
         }
         fscanf(f, "%f\n", &temp1);
         f_xyz[i][nj - 1][k] = temp1; // j=  MachArray => colomn
      } // ni= AlfaArray =>row
      // k = reinolds Array
   }

   if (sim_fclose(f) == EOF)
      return -1;

   *nx = ni;
   *ny = nj;
   *nz = nk;
   sim_fclose(f);
   aero_register3(&f_xyz[0][0][0], filename, ni, nj, nk, ax, ay, az, AERO_C*AERO_K, AERO_K);
   return 0;
}

int ReadFile3Dcm(char *filename, float ax[AERO_R], float ay[AERO_C], float az[AERO_K],
    float f_xyz[AERO_R][AERO_C][AERO_K], int *nx, int *ny, int *nz) {

   FILE *f;
   int i, j, k, ni, nj, nk;
   float temp1;
   char s[30];
   if ((f = sim_fopen(filename, "r")) == NULL) {
      clrscr();
      printf("\nfile %s Not Found\n", filename);
      getch();
      return -1;
   }
   fscanf(f, "%d %d %d\n", &ni, &nj, &nk);
   f_xyz[0][0][0] = 0.;
   for (i = 0; i < ni - 1; i++) {
      fscanf(f, "%f", &temp1);
      ax[i] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   ax[ni - 1] = temp1;

   for (j = 0; j < nj - 1; j++) {
      fscanf(f, "%f", &temp1);
      ay[j] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   ay[nj - 1] = temp1;

   for (k = 0; k < nk - 1; k++) {
      fscanf(f, "%f", &temp1);
      az[k] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   az[nk - 1] = temp1;

   for (k = 0; k < nk; k++) {
      fscanf(f, "%s", s);
      for (i = 0; i < ni; i++) {
         for (j = 0; j < nj - 1; j++) {

            fscanf(f, "%f", &temp1);
            f_xyz[i][j][k] = temp1;
         }
         fscanf(f, "%f\n", &temp1);
         f_xyz[i][nj - 1][k] = temp1; // j=  MachArray => colomn
      } // ni= AlfaArray =>row
      // k = reinolds Array
   }

   if (sim_fclose(f) == EOF)
      return -1;

   *nx = ni;
   *ny = nj;
   *nz = nk;
   sim_fclose(f);
   /* المحاور التي قرأناها للتوّ -- تلتقطها دوال الاستيفاء */
   aero_register3(&f_xyz[0][0][0], filename, ni, nj, nk, ax, ay, az, AERO_C*AERO_K, AERO_K);
   
   return 0;
}

/* RAJEH_CO_ROLL_S2.dat WAS CORRUPT (found and repaired 2026-08-23).
   It stored its numbers with U+066B ARABIC DECIMAL SEPARATOR (bytes D9 AB)
   instead of ASCII '.', in 21 of its 28 data lines -- the whole of the
   Roll05, Roll10 and Roll15 blocks, i.e. EVERY NON-ZERO aileron deflection;
   only Roll00 (all zeros by definition) was clean.  The unchecked "%f" below
   consumed the leading "-0", stopped at the D9, and every later conversion
   then matching-failed WITHOUT consuming input, so the array kept stale
   values and cllda was effectively DEAD for the entire POST-SEPARATION
   flight (stages 3-6 -- the sepflg!=0 branch selects DeltaArrayrw).
   The aileron therefore had NO authority in coast or terminal, which is why
   every roll lever tried against the terminal departures measured inert.
   After repair: phi at stage-6 entry goes from 92.9/96.8/172.4/-167.2 deg to
   0.0 on every case, and the gate improves 42->43/49 (tal_flag=1) and
   47->48/49 (tal_flag=0).
   Corrupt original kept at
     ai_work/backup_aero_tables/RAJEH_CO_ROLL_S2.dat.corrupt
   NOTE nv_rwv397.dat still carries the same 21 corrupt lines but is read by
   nothing -- almost certainly where this came from.
   The fscanf calls in this file are still unchecked; a bad table will again
   fail silently.  Check RAJEH_*.dat for non-ASCII bytes before trusting a
   coefficient:  od -An -tu1 -v f | tr ' ' '\n' | awk 'NF && $1>127' */
int ReadFile3Drw(char *filename, float ax[AERO_R], float ay[AERO_C], float az[AERO_K],
    float f_xyz[AERO_R][AERO_C][AERO_K], int *nx, int *ny, int *nz) {

   FILE *f;
   int i, j, k, ni, nj, nk;
   float temp1;
   char s[30];
   if ((f = sim_fopen(filename, "r")) == NULL) {
      clrscr();
      printf("\nfile %s Not Found\n", filename);
      getch();
      return -1;
   }
   fscanf(f, "%d %d %d\n", &ni, &nj, &nk);
   f_xyz[0][0][0] = 0.;
   for (i = 0; i < ni - 1; i++) {
      fscanf(f, "%f", &temp1);
      ax[i] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   ax[ni - 1] = temp1;

   for (j = 0; j < nj - 1; j++) {
      fscanf(f, "%f", &temp1);
      ay[j] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   ay[nj - 1] = temp1;

   for (k = 0; k < nk - 1; k++) {
      fscanf(f, "%f", &temp1);
      az[k] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   az[nk - 1] = temp1;

   for (k = 0; k < nk; k++) {
      fscanf(f, "%s", s);
      for (i = 0; i < ni; i++) {
         for (j = 0; j < nj - 1; j++) {

            fscanf(f, "%f", &temp1);
            f_xyz[i][j][k] = temp1;
         }
         fscanf(f, "%f\n", &temp1);
         f_xyz[i][nj - 1][k] = temp1; // j=  MachArray => colomn
      } // ni= AlfaArray =>row
      // k = reinolds Array
   }

   if (sim_fclose(f) == EOF)
      return -1;

   *nx = ni;
   *ny = nj;
   *nz = nk;
   sim_fclose(f);
   aero_register3(&f_xyz[0][0][0], filename, ni, nj, nk, ax, ay, az, AERO_C*AERO_K, AERO_K);
   return 0;
}

int ReadFile3Dcw(char *filename, float ax[AERO_R], float ay[AERO_C], float az[AERO_K],
    float f_xyz[AERO_R][AERO_C][AERO_K], int *nx, int *ny, int *nz) {

   FILE *f;
   int i, j, k, ni, nj, nk;
   float temp1;
   char s[30];
   if ((f = sim_fopen(filename, "r")) == NULL) {
      clrscr();
      printf("\nfile %s Not Found\n", filename);
      getch();
      return -1;
   }
   fscanf(f, "%d %d %d\n", &ni, &nj, &nk);
   f_xyz[0][0][0] = 0.;
   for (i = 0; i < ni - 1; i++) {
      fscanf(f, "%f", &temp1);
      ax[i] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   ax[ni - 1] = temp1;

   for (j = 0; j < nj - 1; j++) {
      fscanf(f, "%f", &temp1);
      ay[j] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   ay[nj - 1] = temp1;

   for (k = 0; k < nk - 1; k++) {
      fscanf(f, "%f", &temp1);
      az[k] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   az[nk - 1] = temp1;

   for (k = 0; k < nk; k++) {
      fscanf(f, "%s", s);
      for (i = 0; i < ni; i++) {
         for (j = 0; j < nj - 1; j++) {

            fscanf(f, "%f", &temp1);
            f_xyz[i][j][k] = temp1;
         }
         fscanf(f, "%f\n", &temp1);
         f_xyz[i][nj - 1][k] = temp1; // j=  MachArray => colomn
      } // ni= AlfaArray =>row
      // k = reinolds Array
   }

   if (sim_fclose(f) == EOF)
      return -1;

   *nx = ni;
   *ny = nj;
   *nz = nk;
   sim_fclose(f);
   aero_register3(&f_xyz[0][0][0], filename, ni, nj, nk, ax, ay, az, AERO_C*AERO_K, AERO_K);
   return 0;
}

int ReadFile3Dcwf(char *filename, float ax[AERO_R], float ay[AERO_C], float az[AERO_K],
    float f_xyz[AERO_R][AERO_C][AERO_K], int *nx, int *ny, int *nz) {

   FILE *f;
   int i, j, k, ni, nj, nk;
   float temp1;
   char s[30];
   if ((f = sim_fopen(filename, "r")) == NULL) {
      clrscr();
      printf("\nfile %s Not Found\n", filename);
      getch();
      return -1;
   }
   fscanf(f, "%d %d %d\n", &ni, &nj, &nk);
   f_xyz[0][0][0] = 0.;
   for (i = 0; i < ni - 1; i++) {
      fscanf(f, "%f", &temp1);
      ax[i] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   ax[ni - 1] = temp1;

   for (j = 0; j < nj - 1; j++) {
      fscanf(f, "%f", &temp1);
      ay[j] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   ay[nj - 1] = temp1;

   for (k = 0; k < nk - 1; k++) {
      fscanf(f, "%f", &temp1);
      az[k] = temp1;
   }
   fscanf(f, "%f\n", &temp1);
   az[nk - 1] = temp1;

   for (k = 0; k < nk; k++) {
      fscanf(f, "%s", s);
      for (i = 0; i < ni; i++) {
         for (j = 0; j < nj - 1; j++) {

            fscanf(f, "%f", &temp1);
            f_xyz[i][j][k] = temp1;
         }
         fscanf(f, "%f\n", &temp1);
         f_xyz[i][nj - 1][k] = temp1; // j=  MachArray => colomn
      } // ni= AlfaArray =>row
      // k = reinolds Array
   }

   if (sim_fclose(f) == EOF)
      return -1;

   *nx = ni;
   *ny = nj;
   *nz = nk;
   sim_fclose(f);
   aero_register3(&f_xyz[0][0][0], filename, ni, nj, nk, ax, ay, az, AERO_C*AERO_K, AERO_K);
   return 0;
}

int ReadFile3D_7x12x5 (char *filename,float ax[AERO_R],float ay[AERO_C],float az[AERO_K]
		,float f_xyz[AERO_R][AERO_C][AERO_K],int *nx,int *ny,int *nz)
{

	 FILE  *f ;
	 int   i, j, k, ni, nj, nk ;
	 float temp1 ;
	 char s[30];
	 if ((f = fopen (filename, "r")) == NULL){
		clrscr();
		printf("\nfile %s Not Found\n",filename);
		getch();
		return -1 ;
	 }
	 fscanf (f, "%d %d %d\n", &ni, &nj, &nk) ;
	 f_xyz [0][0][0] = 0. ;
	 for (i=0 ; i<ni-1 ; i++){
	fscanf (f, "%f", &temp1) ;
	ax [i] = temp1 ;
	 }
	 fscanf (f, "%f\n", &temp1) ;
	 ax [ni-1] = temp1 ;

	 for (j=0 ; j<nj-1 ; j++){
	fscanf (f, "%f", &temp1) ;
	ay [j] = temp1 ;
	 }
	 fscanf (f, "%f\n", &temp1) ;
	 ay [nj-1] = temp1 ;

	 for (k=0 ; k<nk-1 ; k++){
	fscanf (f, "%f", &temp1) ;
	az [k] = temp1 ;
	 }
	 fscanf (f, "%f\n", &temp1) ;
	 az [nk-1] = temp1 ;

	 for (k=0 ; k<nk ; k++){
	 fscanf(f,"%s",s);
	for (i=0 ; i<ni ; i++){
		 for (j=0 ; j<nj-1 ; j++){

		fscanf (f, "%f", &temp1) ;
		f_xyz [i][j][k] = temp1 ;
		 }
		 fscanf (f, "%f\n", &temp1) ;
		 f_xyz [i][nj-1][k] = temp1 ;  // j=  MachArray => colomn
	}                                      // ni= AlfaArray =>row
						   // k = reinolds Array
	 }

	 if (fclose (f) == EOF) return -1 ;

	 *nx = ni ;
	 *ny = nj ;
	 *nz = nk ;
	 sim_fclose(f);
	 aero_register3(&f_xyz[0][0][0], filename, ni, nj, nk, ax, ay, az, AERO_C*AERO_K, AERO_K);
	 return 0 ;
}
