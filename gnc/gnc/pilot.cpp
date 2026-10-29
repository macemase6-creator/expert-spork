/* Shared M4-RAJEH / STM743-M397 autopilot implementation. */
#include "gnc_platform.h"
#include "sdre/SDRE.h"
#include "sdre/SDRE_M4_Model.h"

/* Shared dynamic-pressure logic for simulation and firmware. */
void DynPress()
{
  double      rhot2,rhot3,rhot1;
  double    para1_1=1.450427886e-4 ;
  double    para2_1=13.99548426;
  double    para1_2=1.672038698e-4;
  double    para2_2=13.90685586;
  double    para1_3=2.2792106e-4;
  double    para2_3=13.238985471+.003;

  if(Hme<4000)    rhot1=(para2_1+Hme*para1_1);
  else if(Hme<11.e3)  rhot1=(para2_2+Hme*para1_2);
  else            rhot1=(para2_3+Hme*para1_3);
  rhot2=floor(rhot1);
  rhot3=rhot1-rhot2;
  rhot3=pow(2,rhot3)*pow(2,rhot2);
// Find the dynamic pressure .
  q_new=Vm_t_2/rhot3;
  
   if (s_s > 5. && q_new < q_old)	q_cunter++;
   else								q_cunter = 0.;

   if (q_cunter > 3. && flg_q == 0) 
   {
      flg_q = 1;
      q_max = q_new;
   }

   if (s_s < 3 && q_new < q_old)	q_cunter1++;
   else								q_cunter1 = 0.;

	if (100 < q_cunter1) q_cunter1 = 100;
	if (100 < q_cunter) q_cunter = 100;

   if ( q_cunter1 > 5. && pik_q == 0)
      pik_q = 1;

   q_old = q_new;

   q = q_new;


}

/* Shared gain schedule for simulation and firmware. */
void PilotGain(void) 
 {
   DynPress();


   if (q <= 0.0001)				q = 0.0001;
   if ((s_s < 2) && (q < 0.03))	q = 0.03;

    mach330 = Vm_t / 330.; // mach;//in oc mach isn't modeling
   // hm = Hme;

   //aplt_step();

	out_integ_limit=0.35;
	out_integ_w_limit=q/40.;
	
	   //if (q > 220)				q = 220.;

  apc_limit = q / 2.;
  ayc_limit = q / 3.;
  if(hgv_flg==0)
  {   
    ayc_limit = q / 3.5;//1.8;
  }//
  else
  {
    ayc_limit = q / 3.;
    //ayc_limit = q / 10.;
    apc_limit = q / 3.;
  }

    
   // ******************************Gains**********************************
   // ----------------------------stage1 Gains-----------------------------
if(s_s == 1 )//||s_s == 2) 
{
		if(pik_q == 0)		
		{
		

      if (q <4) 
			  {
				K_p_integ_teta1 = 1.0836*(4.5 / q + 0.35 )* 1;
			  
				K_w_pitch1 = 0.275*(1.5 / q + 0.125);  
					  
				K_i_integ_teta= 3.*(0.00164*q*q*q + -0.1727*q*q + 7.351*q + 7.19) / (q +  52.74);
        
       
        K_p_integ_roll=  0.000085*( -0.0002884 *q*q*q + 0.5219*q*q + -2.197*q + 13.58) /(q*q*q + -12.75*q*q + 95.65*q + -14.34);
		/* TAL_ROLLP -- close the roll proportional loop below q=4.
		   This branch and the q>=4 branch below DISAGREE BY 12754x at their
		   shared boundary: at q=4 this gives 4.887e-6 while the q>=4 branch
		   gives 6.233e-2. Below q=4 the roll P loop is therefore effectively
		   OPEN, and the gain then slams shut by four orders of magnitude at
		   the switch.
		   It is invisible on a nominal flight because flg==3 injects no roll
		   torque, so there is nothing to excite: n0 holds max |p| = 0.198 deg/s
		   and |phi| = 0.134 deg through boost. With the flg==1/2 nozzle
		   misalignment the vehicle rolls open-loop below q=4, the loop then
		   slams shut, and it RINGS: max |p| = 19.15 deg/s at t=6.40 and again
		   6.10 deg/s near max-q, at 0.276 Hz, |phi| to 11.06 deg. That is the
		   rate which later freezes in at low q and integrates to ~90 deg of
		   bank over the coast, entering stage 6 rolled and departing.
		   The repair is a RAMP, not a flat scale: the defect is the STEP at
		   q=4, not the low gain itself, and a flat scale also closes the loop
		   hard at q~0 where there is no dynamic pressure to work against.
		   (q/4)^n reaches the q>=4 value exactly at the boundary and leaves
		   q~0 near stock. n = TAL_ROLLP_N; TAL_ROLLP_K = 1 disables.

		   MEASURED, THEN LEFT OFF (K = 1.0), 2026-08-22.  The fix works on
		   the ring and does nothing for the misses -- they are DECOUPLED.
		     flat K=12754   boost |p| 19.15/23.86/20.32/24.33 -> 9.30/8.34/
		                    9.71/8.34 (halved), n0 0.198 -> 0.068 (3x).
		     ramp N=2       -> 13.50/13.07/14.29/13.32 (-40%).
		   In EVERY variant the end-of-stage-2 roll rate is INVARIANT:
		   0.279/0.282/0.456/0.501 deg/s at K=1, at K=12754, and at N=1/2/3.
		   phi at stage-6 entry likewise moves <1 deg (92.90 -> 93.66 flat,
		   92.88 ramp).  So the residual rate that banks the vehicle through
		   the coast is NOT laid down by the boost transient -- it is built
		   AFTER burnout by the low-q drift (mab0 0.001-0.008 N.m walking p
		   from 0.04 to 0.33 as q decays).  Damping boost cannot reach it.
		   Full 49, tal_flag=1: stock 42/49, flat 42/49 (T10n2 1377 -> hit but
		   T11n2 hit -> 218 m, T13n2 41394 -> 58168, T13n5 10062 -> 25674),
		   ramp N=2 41/49 (T16n5 -> 117.7).  tal_flag=0 held 47/49 in all.
		   No variant improves the gate, and both cost a hit or a case, so the
		   schedule is left as found.  Re-enable only WITH a post-burnout roll
		   fix, never alone.
		   SEPARATE, UNADDRESSED: T13n2 is not in this regime at all -- it
		   rolls at 222-233 deg/s through boost with the fins SATURATED
		   (maxFin_s1 = 20.00) and ends stage 2 at p = -0.565.  That is a
		   boost roll departure, an order of magnitude past the hot arms, and
		   it is untouched by any gain tried here. */
		if (sim_tal_flag && TAL_ROLLP_K != 1.0)
		{
			double r_rp = q / 4.0;
			if (r_rp > 1.0) r_rp = 1.0;
			if (r_rp < 0.0) r_rp = 0.0;
			K_p_integ_roll *= 1.0 + (TAL_ROLLP_K - 1.0) * pow(r_rp, TAL_ROLLP_N);
		}

        K_p_integ_acc = 1*(0.5 / q)*0.506; 
				K_i_integ_acc =0.04026*(1.0 / q)*1; 
        K_w_yaw =1.85 * K_w_pitch1;   
        
		
			}
			 else 
			{
				k_ramp = ramp(t_q_p00, t_q_p00 + 3.0, t, 1)*0.1;
				
				K_p_integ_teta1 = 0.3*0.180*1.89* 1.3611*1.40257*(4.5 / q + 0.35 )*0.15 * (1. - k_ramp) + k_ramp * 40. / (q);
				
			//	K_w_roll = 0.8*K_w_roll00 * (1. - k_ramp) + k_ramp * 1.5 * (0.015 * q + 1.024) / (q + 0.0);
				K_p_integ_roll = 1.9*0.1*0.9*0.9 *(1.62 / q) * (1. - k_ramp) + k_ramp * (0.0687 * q + 3.0) / (q + 0.0);
			//	K_i_integ_roll = 0.34*K_i_integ_roll00 * (1. - k_ramp) + k_ramp * (0.094 * q + 0.58) / (q + 0.0);
	 
			   
			  
				K_p_integ_acc = 2.5*(0.5 / q)*0.1;
				K_i_integ_acc = 0.080251135030716*(1.0 / q)*1;   
			  
				K_w_pitch1 = 1.35*0.2*1.18382*1.10638*(1.5 / q + 0.125)* 1.2;       //1.5        
        
        K_w_yaw = 1.2*1 * K_w_pitch1;
				
				K_i_integ_teta=0.435*(0.00164*q*q*q + -0.1727*q*q + 7.351*q + 7.19) / (q +  52.74);//
		
		//		K_p_integ_roll=0.6*2.5*( -0.0002884 *q*q*q + 0.5219*q*q + -2.197*q + 13.58) /(q*q*q + -12.75*q*q + 95.65*q + -14.34);
			  
				//if(q>26)
			//	{
				//   K_w_roll *= 0.6* 0.9;
			//	   K_w_yaw *= 0.6* 2;
			//	}
        
			//	if (q>60 && guid_flg==1)  K_w_roll *=0.7;   
		
		 
			   
				K_w_roll11 = K_w_roll;
				K_p_integ_roll11 = K_p_integ_roll;
				K_i_integ_roll11 = K_i_integ_roll;

				K_w_yaw11 = K_w_yaw;
				K_i_integ_acc11 = K_i_integ_acc;
				K_p_integ_acc11 = K_p_integ_acc;

				K_w_pitch11 = K_w_pitch1;
				K_i_integ_teta11 = K_i_integ_teta;
				K_p_integ_teta11 = K_p_integ_teta1;
				t_q_p0 = t;
				k_ramp = 0.;

			}
		}
		  else
		{
         k_ramp = ramp(t_q_p0, t_q_p0 + 3.0, t, 1)*0.1;

         K_w_pitch1 =K_w_pitch11 *1.3;// (1. - k_ramp) + k_ramp * 3.5 * 7. / (q);
         K_i_integ_teta = K_i_integ_teta11 *0.128;// (1. - k_ramp) + k_ramp * 100. / (q);
         K_p_integ_teta1 = K_p_integ_teta11 *0.8;// (1. - k_ramp) + k_ramp * 30. * 3. / (q);

////////////////////////////////////////////yaw////////////////////////////////////////////////////////

         K_w_yaw = K_w_yaw11 * (1. - k_ramp) + k_ramp * 1.5 * 8. / q;
         K_i_integ_acc = K_i_integ_acc11 * (1. - k_ramp) + k_ramp * 1.31 / q;
         K_p_integ_acc = K_p_integ_acc11 *1;// (1. - k_ramp) + k_ramp * 2.4 / q;
     //	K_w_yaw = 0.2889*(1.0 / q)*1.23;
       
        

if (q>30&&q < 55.)	
		 {    
		// K_i_integ_acc = K_i_integ_acc11 ;//* (1. - k_ramp) + k_ramp * 4.1 / q;
         K_w_pitch1 =K_w_pitch11 *2.3;// (1. - k_ramp) + k_ramp * 3.5 * 7. / (q);
         K_i_integ_teta = K_i_integ_teta11 *0.328;// (1. - k_ramp) + k_ramp * 100. / (q);
         K_p_integ_teta1 = K_p_integ_teta11 *0.58;// (1. - k_ramp) + k_ramp * 30. * 3. / (q);
          // K_w_yaw = K_w_yaw11 * (1. - k_ramp) + k_ramp * 1.25 * 8. / q;
		  }

		K_w_roll22 = K_w_roll;
		
        K_p_integ_roll22 = K_p_integ_roll;
        K_i_integ_roll22 = K_i_integ_roll;

        K_w_yaw22 = K_w_yaw;
        K_i_integ_acc22 = K_i_integ_acc;
        K_p_integ_acc22 = K_p_integ_acc;

        K_w_pitch22 = K_w_pitch1;
        K_i_integ_teta22 = K_i_integ_teta;
        K_p_integ_teta22 = K_p_integ_teta1;
        t_q_p1 = t;
        k_ramp = 0.;
    }
  K_w_roll = 0.432*(0.5 / q);
	//	K_p_integ_roll =0.12329*(1.62 / q);// fcu 1.2    1.262
		K_i_integ_roll = 1.130*(0.15 / q);
		/* MERGE step 11 (2026-08-23): donor STAGE-1 gain schedule, applied as a
		   whole-block override so the local branches keep their side effects and
		   only the gain VALUES change. Donor breakpoint is q<20, local is q<4. */
		if (DONOR_S1)
		{
		  double _lrw=K_w_roll, _lrp=K_p_integ_roll, _lri=K_i_integ_roll;
		  if(pik_q == 0)
		  {
		    /* TAL_Q20W -- the donor schedule swaps its ENTIRE gain set at q=20.
		       Both branch value-sets are computed here and interpolated over
		       q in [20-W, 20+W]; W<=0 reproduces the hard switch EXACTLY.
		       Note K_p_integ_acc uses a different functional form on each side
		       (and changes sign near q=20), so the VALUES must be blended, not
		       the leading coefficients. */
		    {
		      double _pit=(0.1582*q*q + -3.577*q + 85.85) / (q*q + 36.29*q +  114.9);
		      double _tet=( 0.04634*q + 3.203) / (q + 1.359);
		      double _cub=(0.00164*q*q*q + -0.1727*q*q + 7.351*q + 7.19) / (q +  52.74);
		      double _iac=( 0.0009912*q*q*q + -0.1064*q*q + 5.66*q + -1.067 ) /(q*q + 64.13*q + -122.9);
		      double _rol=( 0.0002963*q*q*q + -0.05156*q*q + 2.956*q + -3.459 ) /(q*q + 85.51*q + -75.5);
		      double _prl=( -0.0002884 *q*q*q + 0.5219*q*q + -2.197*q + 13.58) /(q*q*q + -12.75*q*q + 95.65*q + -14.34);
		      double _w20;
		      if ((TAL_Q20W) <= 0.) _w20 = (q < 20) ? 0. : 1.;
		      else { _w20 = (q - (20. - (TAL_Q20W))) / (2.*(TAL_Q20W));
		             if (_w20 < 0.) _w20 = 0.;  if (_w20 > 1.) _w20 = 1.; }

		      double A_kwp = 1*_pit,      B_kwp = 0.93*_pit;
		      double A_kpt = 1.142*_tet,  B_kpt = 0.82*_tet;
		      double A_kit = (q>4 ? 1.13 : 2.3)*_cub, B_kit = 1.3*_cub;
		      double A_kia = 0.1055*_iac, B_kia = 0.355*_iac;
		      double A_kpa = 0.58*( -2.228e-05*q*q*q +  0.003546*q*q + -0.2357*q +  3.247) /(q*q +  7.828*q +  21.85);
		      double B_kpa = 0.15*( 0.7268) / (q*q + 0.2468*q +  5.923);
		      double A_kwy = 1.85 * 0.275*(1.5 / q + 0.125);
		      double B_kwy = 0.85 * 0.275*(1.5 / q + 0.125);
		      double A_kwr = 0.82*_rol,   B_kwr = 1.2*_rol;
		      double A_kpr = 0.0005*_prl, B_kpr = 0.5*_prl;

		      K_w_pitch1     = (1.-_w20)*A_kwp + _w20*B_kwp;
		      K_p_integ_teta1= (1.-_w20)*A_kpt + _w20*B_kpt;
		      K_i_integ_teta = (1.-_w20)*A_kit + _w20*B_kit;
		      K_i_integ_acc  = (1.-_w20)*A_kia + _w20*B_kia;
		      K_p_integ_acc  = (1.-_w20)*A_kpa + _w20*B_kpa;
		      K_w_yaw        = (1.-_w20)*A_kwy + _w20*B_kwy;
		      K_w_roll       = (1.-_w20)*A_kwr + _w20*B_kwr;
		      K_p_integ_roll = (1.-_w20)*A_kpr + _w20*B_kpr;
		      /* identical in both donor branches */
		      if(q<30)K_i_integ_roll =0.08*(2.567e-06*q*q*q + -0.0003011*q*q + 0.008972*q + -0.003241);
		      else    K_i_integ_roll =2.*(0.0004788*q + 0.006476) / (q + -0.01023);

		    // if(q>50&&q<90)
				    if(q>50)
		      {
		        K_p_integ_acc = 2.5*(0.5 / q)*0.1;
		        K_i_integ_acc = (1.-_w20)*A_kia + _w20*B_kia*0.7;//0.100251135030716*(1.0 / q)*1;
		        K_w_pitch1 =0.424*(1.5 / q + 0.125)* 1.2;
		        K_p_integ_teta1=3.52*( 0.04634*q + 3.203) / (q + 1.359);
		      //  if(q>80) K_w_pitch1 =0.65424*(1.5 / q + 0.125)* 1.2;
		        K_w_yaw = 1.2*0.8 * K_w_pitch1;
		        K_i_integ_teta= ((1.-_w20)*A_kit + _w20*B_kit)*0.5;//0.35*(0.00164*q*q*q + -0.1727*q*q + 7.351*q + 7.19) / (q +  52.74);
		      }
			  	K_p_integ_acc = 2.5*(0.5 / q)*0.091;
			if(sim_tal_flag)	K_i_integ_acc = 0.080251135030716*(1.0 / q)*1;   
			  
				K_w_pitch1 = 1.1*0.2*1.08382*1.10638*(1.5 / q + 0.125)* 1.2;       //1.5   
   if( q < 10 )		{
	   K_w_pitch1=K_w_pitch1/1.55;
	//   if( q < 3.5 ) K_w_pitch1/2.55;	
     K_p_integ_teta1=K_p_integ_teta1*1.6;
//	 if(q>3)K_p_integ_teta1=K_p_integ_teta1*2.06;
	 K_i_integ_teta =K_i_integ_teta*1.8;
   }	 
        
        K_w_yaw = 1.5*0.799 * K_w_pitch1;
		    }
		  }
		  else if((q>60&&sim_tal_flag)||(q>60&&sim_tal_flag==0))
		  {
		   if(sim_tal_flag) K_w_pitch1=1.8*(0.1582*q*q + -3.577*q + 85.85) / (q*q + 36.29*q +  114.9);
		    else K_w_pitch1=1.05*(0.1582*q*q + -3.577*q + 85.85) / (q*q + 36.29*q +  114.9);
		    K_p_integ_teta1=4.42*( 0.04634*q + 3.203) / (q + 1.359);
	//	   if ( sim_tal_flag)K_p_integ_teta1 = 0.284523*(4.5 / q + 0.35 )* 1;
		    K_i_integ_teta=0.34*(0.00164*q*q*q + -0.1727*q*q + 7.351*q + 7.19) / (q +  52.74);
			 K_p_integ_acc =0.05*( -2.228e-05*q*q*q +  0.003546*q*q + -0.2357*q +  3.247) /(q*q +  7.828*q +  21.85);
		    if(sim_tal_flag)
			{   
			 K_w_yaw=1.151*(0.1074*q*q + -0.6039 *q +  12.36 ) / (q*q + 1.024 *q + 8.029);
		    K_i_integ_acc=0.004955*( 0.0009912*q*q*q + -0.1064*q*q + 5.66*q + -1.067 ) /(q*q + 64.13*q + -122.9);
		   }
		   else
		   {
		           	K_w_yaw=1.751*(0.1074*q*q + -0.6039 *q +  12.36 ) / (q*q + 1.024 *q + 8.029);
						if(q>100){	K_w_yaw=0.551*(0.1074*q*q + -0.6039 *q +  12.36 ) / (q*q + 1.024 *q + 8.029);
						            K_p_integ_teta1=2.42*( 0.04634*q + 3.203) / (q + 1.359);
									K_i_integ_teta=0.54*(0.00164*q*q*q + -0.1727*q*q + 7.351*q + 7.19) / (q +  52.74);
									 }
		                           K_i_integ_acc=0.3254955*( 0.0009912*q*q*q + -0.1064*q*q + 5.66*q + -1.067 ) /(q*q + 64.13*q + -122.9);
		    }
			
		    K_w_roll=0.82*( 0.0002963*q*q*q + -0.05156*q*q + 2.956*q + -3.459 ) /(q*q + 85.51*q + -75.5);
		    K_p_integ_roll=0.005*( -0.0002884 *q*q*q + 0.5219*q*q + -2.197*q + 13.58) /(q*q*q + -12.75*q*q + 95.65*q + -14.34);
		    if(q<30)K_i_integ_roll =0.08*(2.567e-06*q*q*q + -0.0003011*q*q + 0.008972*q + -0.003241);
		    else    K_i_integ_roll =2.*(0.0004788*q + 0.006476) / (q + -0.01023);
		  }
		  else
		  {
		    K_w_pitch1=1.785*(0.1582*q*q + -3.577*q + 85.85) / (q*q + 36.29*q +  114.9);
		    if(sim_tal_flag)K_p_integ_teta1=3.42*( 0.04634*q + 3.203) / (q + 1.359);
			 else K_p_integ_teta1=5.542*( 0.04634*q + 3.203) / (q + 1.359);
		    K_i_integ_teta=0.358*(0.00164*q*q*q + -0.1727*q*q + 7.351*q + 7.19) / (q +  52.74);
		    K_w_yaw=1.851*(0.1074*q*q + -0.6039 *q +  12.36 ) / (q*q + 1.024 *q + 8.029);
		 if(sim_tal_flag)  K_i_integ_acc=0.01955*( 0.0009912*q*q*q + -0.1064*q*q + 5.66*q + -1.067 ) /(q*q + 64.13*q + -122.9);

		 else   K_i_integ_acc=0.32955*( 0.0009912*q*q*q + -0.1064*q*q + 5.66*q + -1.067 ) /(q*q + 64.13*q + -122.9);
		    K_p_integ_acc =0.18*( -2.228e-05*q*q*q +  0.003546*q*q + -0.2357*q +  3.247) /(q*q +  7.828*q +  21.85);
		    K_w_roll=0.82*( 0.0002963*q*q*q + -0.05156*q*q + 2.956*q + -3.459 ) /(q*q + 85.51*q + -75.5);
		    K_p_integ_roll=0.005*( -0.0002884 *q*q*q + 0.5219*q*q + -2.197*q + 13.58) /(q*q*q + -12.75*q*q + 95.65*q + -14.34);
		    if(q<30)K_i_integ_roll =0.08*(2.567e-06*q*q*q + -0.0003011*q*q + 0.008972*q + -0.003241);
		    else    K_i_integ_roll =2.*(0.0004788*q + 0.006476) / (q + -0.01023);
		  }
		  /* NEGATIVE RESULT 2026-08-23 -- stage 1 does NOT smooth the way
		     stage 2 did, and these are left at 1.0.
		     The stage-1 transient is NOT a discontinuity: blending the q=20
		     branch switch (TAL_Q20W, 3/6/10) was measured INERT -- wz peak
		     24.55 -> 24.69/24.81/24.33, dupitch 4.82, dele p99 9.7, alfa 9.82,
		     all unchanged. The peak sits at q=34-36, BETWEEN switches.
		     It is a sustained ~0.8 Hz OSCILLATION through max-q: wz peak-to-peak
		     48.0 deg/s on T11n1 (19 zero crossings, period 1.27 s), tetam
		     swinging +-3 deg about tetac. Damping it works beautifully and costs
		     accuracy at EVERY level tried:
		       S1KW 1.05 -> pk-pk 42.0, screen 1 fail -> 2, sum 237 -> 333
		       S1KW 1.10 -> 36.0, 2 fails      S1KW 1.20 -> 27.8, 2 fails
		       S1KW 1.50 -> 17.4 (-64%!), 4 fails, one miss of 654 km
		       S1KW 2.50 -> DIVERGES (wz pk-pk 898, peak 450 deg/s), 9 fails
		       S1KP 0.7  -> oscillation WORSE (53.9) and 3 fails
		     K_w_pitch1 also feeds wz_compensate1 into delec0, so it moves the
		     boost TRIM, not just the damping -- which is why a 5% change can
		     cost a hit. The schedule sits at a local optimum. Do not re-tune
		     these for smoothness without a way to separate trim from damping. */
		  if ((TAL_KWY)  != 1.0) K_w_yaw        *= (TAL_KWY);
		  if ((TAL_S1KW) != 1.0) K_w_pitch1      *= (TAL_S1KW);
		  if ((TAL_S1KP) != 1.0) K_p_integ_teta1 *= (TAL_S1KP);
		  if (!DONOR_S1_ROLL)
		  { K_w_roll=_lrw; K_p_integ_roll=_lrp; K_i_integ_roll=_lri; }
	  
	       if(sim_tal_flag)   if ((TAL_S1KW) != 1.0) K_w_pitch1      /= (TAL_S1KW);
		

     
			 
			  
	// K_w_roll=0.82*( 0.0002963*q*q*q + -0.05156*q*q + 2.956*q + -3.459 ) /(q*q + 85.51*q + -75.5);
		    K_p_integ_roll=1.2005*( -0.0002884 *q*q*q + 0.5219*q*q + -2.197*q + 13.58) /(q*q*q + -12.75*q*q + 95.65*q + -14.34);
		    //if(q<30)
			K_i_integ_roll =0.098*(2.567e-06*q*q*q + -0.0003011*q*q + 0.008972*q + -0.003241);
 //K_i_integ_roll= 30*(0.00477) / (q  -0.03721);
if(q<5)K_p_integ_roll =0.012 *(1.62 / q);

					K_p_integ_acc =2.95*( 0.7268) / (q*q + 0.2468*q +  5.923);//

		if(q<2)				K_i_integ_acc=0.1*(0.01969*q*q + -0.5062*q + 10.88) / (q*q + -2.414*q + 147.8);
		else if(q<5)		K_i_integ_acc=0.15*(0.01969*q*q + -0.5062*q + 10.88) / (q*q + -2.414*q + 147.8); 
		else if(pik_q==1)	K_i_integ_acc=0.53*(-0.06126 *q + 7.234) / (q +  51.15);
		else				K_i_integ_acc=0.55*( 0.0009912*q*q*q + -0.1064*q*q + 5.66*q + -1.067 ) /(q*q + 64.13*q + -122.9);//
//
if(pik_q==0&&q<10) K_w_yaw=0.6581*(0.1074*q*q + -0.6039 *q +  12.36 ) / (q*q + 1.024 *q + 8.029);	
		
}	
	        
			
   }
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
///////////////////////////////////////////////������� �������////////////////////////////////////////////
   
   else if (s_s == 2) 
   {
	   //  double PsoVar[11] ={1.19888,   1.96301, 0.729759,  0.86838,   2.41807,  1.14524,   1.3611,  1.04705,  2.13715,  1.18382,};
	k_ramp =0.21* ramp(t_q_p1, t_q_p1 + 1., t, 1);
			K_w_roll =0.271*(0.5 / q);
			K_p_integ_roll =0.0032*(1.62 / q);// fcu 1.2    1.262
			K_i_integ_roll = 0.135*(0.15 / q);
			K_p_integ_teta1 =  0.6523*(4.5 / q + 0.35 )* 1;//* 0.605;//;//
		 K_i_integ_teta =1.585*(4.0 / q + 0.1) ;//* 5.5;//* 3.5;//;
	//
		if ((TAL_Q31W) <= 0. || !sim_tal_flag)
		{
			if (q > 31.)K_p_integ_teta1 =  0.105126523*(4.5 / q + 0.35 )* 1;//* 0.605;//;//
		}
		else
		{	/* blended form of the same switch -- see TAL_Q31W in sim_fio.h */
			double w31 = (q - (31. - (TAL_Q31W))) / (2.*(TAL_Q31W));
			if (w31 < 0.) w31 = 0.;  if (w31 > 1.) w31 = 1.;
			K_p_integ_teta1 = (1. - w31) * (0.6523*(4.5 / q + 0.35))
			                +       w31  * (0.105126523*(4.5 / q + 0.35));
		}
		//	if (q < 9.)K_p_integ_teta1 =  0.05126523*(4.5 / q + 0.35 )* 1;//* 0.605;//;//
			K_w_pitch1 =0.15*K_w_pitch11 * (1. - k_ramp) + k_ramp * 1.5 * 25. / (q);
			K_p_integ_teta1 *= 1;

				 K_w_yaw = K_w_yaw11 * (1. - k_ramp) + k_ramp * 1.5 * 5. / q;
	

			//	K_w_yaw = 1*(1 * 1.5 * 0.2) /(q+0.1) ;
		

  	
	K_w_pitch1 = 4.75 / (q + 0.1);

//	K_p_integ_acc =1.7428*(0.5 / q)*0.350510000121;
//		K_i_integ_acc = 0.0889*(1.0 / q)*0.05123;
	//	K_w_pitch1 = 1.45 / (q + 0.1);

	K_p_integ_acc =1.7428*(0.5 / q)*0.1000021;
		K_i_integ_acc = 0.0889*(1.0 / q)*0.2085123;
	//	K_w_pitch1 = 1.45 / (q + 0.1);


		
		
		
			 K_w_roll = 0.232*(0.5 / q);
		K_p_integ_roll =0.42329*(1.62 / q);// fcu 1.2    1.262
		K_i_integ_roll = 1.130*(0.15 / q);
//
//	K_w_yaw =K_w_pitch1;
	if (q>25)
			{
		//	  K_p_integ_teta1 *= 0.12;
K_w_pitch1 = 4.5 / (q + 0.1);
			 //  K_w_yaw *=0.0125*q;
	//K_p_integ_acc =1.7428*(0.5 / q)*0.000021;
//if (q>35)	
//K_p_integ_acc =1.7428*(0.5 / q)*0.1000021;
K_p_integ_acc =0.70*(-0.1551*q + 11.31) / (q*q + -4.458*q + 33.71);
//K_p_integ_acc =2.5*( 0.7268) / (q*q + 0.2468*q +  5.923);//
K_w_yaw=0.4700*(-0.04209 *q + 11.52) / (q +  1.866);		
			K_i_integ_acc=0.0981050*(0.001395*q*q + -0.1079*q +  4.807) / (q + 9.728);//
			K_p_integ_acc   = 1.428*(0.5 / q)*0.085000021;
			K_i_integ_acc   = 0.0889*(1.0 / q)*0.0185123;
	K_w_yaw         = 1.5*K_w_pitch1;
			}
		//	K_w_yaw=0.4900*(-0.04209 *q + 11.52) / (q +  1.866);
	//	K_p_integ_acc =0.70*(-0.1551*q + 11.31) / (q*q + -4.458*q + 33.71);
	
//K_w_yaw=0.4900*(-0.04209 *q + 11.52) / (q +  1.866);
	//	K_p_integ_acc =0.0070*(-0.1551*q + 11.31) / (q*q + -4.458*q + 33.71);
//	if(q<0.5)K_p_integ_acc =1.7428*(0.5 / q)*0.0135000021;
//if(q>10&&q<25) 
//K_i_integ_acc=0.1571*(0.001395*q*q + -0.1079*q +  4.807) / (q + 9.728);//
if ( !sim_tal_flag)	K_p_integ_teta1 = 0.4523*(4.5 / q + 0.35 )* 1;
// if ( sim_tal_flag&&q>60)K_p_integ_teta1 = 0.084523*(4.5 / q + 0.35 )* 1;
 			K_w_roll = 0.28*1.96301*1.05983*(0.5 / q);
			K_p_integ_roll =1.2* 0.729759*0.124562*(1.62 / q);// fcu 1.2    1.262
			K_i_integ_roll = 1.5*0.86838*0.1*(0.15 / q);
      if(q<2.5)
		{
			K_w_roll = 0.26 / (q + 1.0);
			K_p_integ_roll = 0.14 / (q + 1.0);
			/* TAL_ROLLQ -- NEGATIVE RESULT 2026-08-22. Raising the roll gains
			   in this low-q branch does NOT remove the residual roll rate.
			   Context: this branch replaces the 0.5825/q schedule above (which
			   holds LOOP gain constant, since control power scales with q) by
			   0.26/(q+1), which does not -- 3x lower loop gain at the q=2.5 switch,
			   6x by q=0.59. T11 n1 leaves the atmosphere with 0.271 deg/s of roll
			   frozen in, which integrates to 87 deg of bank over the 315 s coast;
			   it enters stage 6 at phi=92.9 deg against a healthy -1.85, the
			   de-rotation then swaps pitch and yaw, and the case departs.
			   MEASURED, scaling both gains by 1/2/4/8: the residual rate falls only
			   0.279 -> 0.262 -> 0.243 (13% for 4x gain) and the misses get WORSE;
			   at 4x, T11n4 and T10n4 destabilise outright (p to 15 deg/s, fins to
			   the 20 deg clip) and at 8x every case does, healthy ones included.
			   DECISIVE detail: the fin deflection does not move at all (6.48 ->
			   6.47 deg) under a 4x gain. A gain-limited loop would deflect more.
			   So this loop is NOT gain-limited here -- it is balancing an
			   UNCOMMANDED ROLL TORQUE. Find that torque; do not raise these gains. */

		}
		if(q<2.6 )
			{ 
				K_w_yaw =  1.15/q;//*(1 * 1.5 * 0.2) /(q+0.1) ;//ocv14
				K_w_pitch1 = 1.75 / (q + 0.1);
				if( guid_flg==1)K_p_integ_teta1 =1.025/q;//(3.5 / q);
				else K_p_integ_teta1 =0.025/q;//(3.5 / q);
				K_i_integ_teta = 1.4*(3 / q);//*/
			}
		/* MERGE step 12 (2026-08-23): donor stage-2 gain schedule. Placed here so
		   the local block above keeps its side effects and only the VALUES change;
		   the K_*33 captures below then carry the donor values into stage 3/4. */
		if (DONOR_S2)
		{
			K_p_integ_teta1 = 0.4523*(4.5 / q + 0.35 )* 1;
			K_i_integ_teta  = 0.585*(4.0 / q + 0.1) ;
			K_w_pitch1      = 4.45 / (q + 0.1);
			K_p_integ_acc   = 1.428*(0.5 / q)*0.085000021;
			K_i_integ_acc   = 0.0889*(1.0 / q)*0.0185123;
			K_w_roll        = 0.32*(0.5 / q);
			K_p_integ_roll  = 0.12329*(1.62 / q);
			K_i_integ_roll  = 1.130*(0.15 / q);
			K_w_yaw         = 1.5*K_w_pitch1;
		}
	    t_q_p2 = t;
		k_ramp = 0.0;
		K_p_integ_acc33 = K_p_integ_acc;
		K_i_integ_acc33 = K_i_integ_acc;
		K_p_integ_roll33 = K_p_integ_roll;
		K_i_integ_roll33 = K_i_integ_roll;
		K_w_pitch33 =1* K_w_pitch1;
		K_w_roll33 = K_w_roll;//*/

   }
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
//////////////////////////////////////////������� ������� ///////////////////////////////////////////////////
   
   else if (s_s == 3) 
   {
	   	k_ramp = ramp(t_q_p2, t_q_p2 + 1.0, t, 1);

		K_w_roll = (1.0 - k_ramp) * K_w_roll33 + (k_ramp * 0.1) / (q + 0.01);
		K_p_integ_roll =(1.0 - k_ramp) * K_p_integ_roll33 + (k_ramp * 0.12 * 0.19 * 0.5) / (q + 0.1);
		K_i_integ_roll =(1.0 - k_ramp) * K_i_integ_roll33 + (k_ramp * 0.12 * 0.25) / (q + 3.0);
		K_w_pitch = 0.0*(1.0 - k_ramp) * K_w_pitch33 +(k_ramp * 1.5 * 0.2) / q ;
	  
		K_w_pitch = (3 * 0.5 * 0.2) / q ;//ocv14
		/* TAL_KWP4 -- only unclamped branch of this gain.  See sim_fio.h. */
		if ((TAL_KWP4) > 0. && sim_tal_flag && K_w_pitch > (TAL_KWP4)) K_w_pitch = (TAL_KWP4);

	
		K_i_integ_acc = 0.20;
		K_p_integ_acc = 0.10;
			
		K_w_yaw = 1*K_w_pitch;
	 
		K_w_roll44 = K_w_roll;
		K_p_integ_roll44 = K_p_integ_roll;
		K_i_integ_roll44 = K_i_integ_roll;
		K_w_pitch44 = K_w_pitch;

      t_q_p3 = t;
      k_ramp = 0.;
   }
   else if (s_s == 4) 
   {
		k_ramp = ramp(t_q_p2, t_q_p2 + 1.0, t, 1);

		K_w_roll = (1.0 - k_ramp) * K_w_roll33 + (k_ramp * 0.1) / (q + 0.01);
		K_p_integ_roll =(1.0 - k_ramp) * K_p_integ_roll33 + (k_ramp * 0.2 * 0.9 * 0.5) / (q + 0.1);
		K_i_integ_roll =(1.0 - k_ramp) * K_i_integ_roll33 + (k_ramp *0.2 * 0.25) / (q + 3.0);
		K_w_pitch = 1.*(1.0 - k_ramp) * K_w_pitch33 +(k_ramp * 1.5 * 0.2) / q ;//ocv14
	//	if (guid_flg==1) K_w_roll = 0.18 / (q + 0.1);
		K_p_integ_acc = 0. * 0.02;
		K_i_integ_acc = 0. * 1. / (q + 0.1);
		K_w_pitch = 0. * 4. / (q + 0.1);
		out_integ_pitch = 0.0;
		out_integ_yaw = 0.0;
		out_integ_roll = 0.;

      K_w_yaw = K_w_pitch;

      t_q_p4 = t;
      k_ramp = 0.;
   }
   else if (s_s == 5.) 
   {
		k_ramp = ramp(t_q_p4, t_q_p4 + 3.0, t, 1);
		K_w_roll = 0. * (1. - k_ramp) + k_ramp * 0.25 / (q + 0.1);
		K_p_integ_roll = 0. * (1. - k_ramp) + k_ramp * 0.25 / (q + 0.1);
		K_i_integ_roll = 0. * (1. - k_ramp) + k_ramp * 0.15 / (q + 0.1);
		K_w_pitch = 0. * (1. - k_ramp) + k_ramp * .4 / (q);
		if (K_w_pitch > 30.)
		 K_w_pitch = 30.;

		K_i_integ_acc = 0.;
		K_p_integ_acc = 0.;

		//---------------
		if (guid_flg==0)    K_w_roll = 0.0798 / (q + 0.1);
		else K_w_roll = 0.18 / (q + 0.1);
		K_p_integ_roll = 0.109 / (q + 0.1);
		K_i_integ_roll =  0.11 / (q + 0.1);

		K_w_pitch = 0.4 / q;
		if (20.0 < K_w_pitch) K_w_pitch = 20.0;

      K_w_yaw = K_w_pitch;


      K_w_roll55 = K_w_roll;
      K_p_integ_roll55 = K_p_integ_roll;
      K_i_integ_roll55 = K_i_integ_roll;
      K_w_pitch55 = K_w_pitch;

      t_q_p5 = t;
      k_ramp = 0.;

   }
//-----------------------s_s 6 Gains---------------------
   else if (s_s == 6.) 
   {
  //----------------- org
  k_ramp = ramp(t_q_p5, t_q_p5 + 2.0, t, 1);

		out_integ_limit=0.35;
		out_integ_w_limit=q/40.;
		du_pitch_limit=2. * q;//
		K_p_integ_acc = 0.;
	 
    K_w_roll = 0.064/ q;
  if (flg_q == 1)  K_p_integ_roll =0.0018671364/ q;
  else if (flg_q == 0&&q>80)  K_p_integ_roll =0.00038671364/ q;
		else K_p_integ_roll = 0.254123*0.5 / q; 
		K_i_integ_roll = 0.0538499/ q;//

//============================  HGV =======================
  
    k_ramp = ramp(t_q_p5, t_q_p5 + 2.0, t, 1);
    
   // K_w_pitch = 1*look2_binlxpw(Hme, mach330,H_table,Mach_table,k_w_acc61,NUM_H_Mach,stride);//
    K_i_integ_acc = 1*look2_binlxpw(Hme, mach330,H_table,Mach_table,k_i_acc61,NUM_H_Mach,stride);//
    Ki_w_pitch = 1*look2_binlxpw(Hme, mach330,H_table,Mach_table,ki_w_acc6,NUM_H_Mach,stride);//
   // K_w_yaw = K_w_pitch;
    
		if (guid_flg==0)
		{
			if(q>2) Ki_w_pitch =-0.322 ;
			else        Ki_w_pitch *= -0.320;//
		} 
		K_i_integ_acc=0.12*(0.6042) / (q -1.045);


if(q<2)
    {

			K_w_roll = 0.108 / (q + 0.1);
			K_p_integ_roll = 0.15 / (q + 0.1);
			K_i_integ_roll =  0.1 / (q + 0.1);
		}
		if((flg_q==1)||(q>10 && flg_q==0))
		{
			K_i_integ_acc = 1.*look2_binlxpw(Hme, mach330,H_table,Mach_table,k_i_acc61,NUM_H_Mach,stride);//
		}
			
		if((q<20&&flg_q == 1))K_w_pitch = 0.10980591000035*(-0.06362*q + 4.241) / (q + 2.03);
		else K_w_pitch = 0.21125*(-0.06362*q + 4.241) / (q + 2.03);//

		K_i_integ_acc = (0.5 / q + 0.004) *0.3;
			if(q>5)			K_p_integ_acc = (-0.18547*q+0.1774)/(q*q-2.167*q+1.464);/*DEAD: overwritten 2 lines below, and again by the guid_flg==1 block*/
		K_p_integ_acc =(-0.263*q + 1.285) / (q + 0.007431);//

		K_w_yaw =1*K_w_pitch;

		if (guid_flg==1)
		{  
			/* MERGE step 10 -- REJECTED 2026-08-23. Donor guid_flg==1 stage-6
			   set (Ki_w_pitch 0.97 -> 0.197, single 0.16265 K_i_integ_acc form
			   replacing the q>30 branch, K_p_integ_acc 0.00657 -> 0.0057,
			   K_w_yaw 0.8 -> 2.0 x K_w_pitch) destroys terminal margin:
			   3 screen failures (T10n1 508.3, T13n1 250.2) with T11n1 83.3 and
			   T10n4 73.4 against a 100 m gate. Consistent with the AP1 note
			   below -- this terminal is not autopilot-limited, so a foreign
			   gain schedule only costs. Local set kept. */
			if (flg_q == 1)   Ki_w_pitch *= 0.97;//
			else        Ki_w_pitch *= 0.0;//
			if( flg_q==1&&q>30)	 K_i_integ_acc =(-0.00026*q+1.44)/(q+0.3751)*0.125;// *1.1;//0.8;
			else K_i_integ_acc = (0.3 / q + 0.004) * 0.2265;//else K_i_integ_acc = 1*(0.5 / q + 0.004) * 0.2065;

			K_p_integ_acc = 0.00657*(-0.263*q + 1.285) / (q + 0.007431);//
//	K_i_integ_acc=0.312*(0.6042) / (q -1.045);
			K_w_yaw =0.8*K_w_pitch;
		}

		if( q>70)
		{
			K_w_pitch *=0.8;
		}
		/* AP1 (2026-08-18) — autopilot stiffness campaign, NEGATIVE result.
		   guid_flg==1 is the LIVE HGV path (set GUIDANCE.cpp:917), so every
		   guid_flg==0 assignment above is dead for the HGV matrix, and the
		   guid_flg==1 block reassigns BOTH acc gains — any trim must go AFTER it.
		   Placed correctly, K_p_integ_acc x2.5 and K_i_integ_acc x2.0 moved the
		   T13n0 miss by 0.04 m (10.85 -> 10.89). The loop already tracks its
		   command (err rms 0.03-0.36) with fins 0% saturated at ~4% of available
		   slew rate: terminal accuracy is NOT autopilot-limited. Do not re-tune
		   these for accuracy without new evidence. */
if( flg_q==1&&q<7)	 K_p_integ_acc =K_p_integ_acc*1.5;
		
   }
  
}
// ------------------------End of PilotGain-----------------------------
/////////////////////////////////////////////////////////////////////////////
/* TAL_PLEAD -- lead compensator on the pitch PI output.  See sim_fio.h.
   C(s) = (1 + a*tau*s)/(1 + tau*s), Tustin-discretised on the actual dt.
   DC gain is exactly 1, so trim and steady-state tracking are unchanged. */
double PitchLead(double u, double tnow)
{
   if ((TAL_PLEAD) <= 1.) return u;
   double dtl = (pl_t_prev < 0.) ? 0. : (tnow - pl_t_prev);
   if (dtl < 0. || dtl > 0.5) dtl = 0.;
   pl_t_prev = tnow;
   if (dtl <= 0.) { pl_u_prev = u; pl_y_prev = u; return u; }
   double aa  = (TAL_PLEAD);
   double tau = 1. / ((TAL_PLEADW) * sqrt(aa));
   double A = aa * tau, B = tau;
   double y = ((2.*A + dtl) * u + (dtl - 2.*A) * pl_u_prev
               - (dtl - 2.*B) * pl_y_prev) / (2.*B + dtl);
   pl_u_prev = u;
   pl_y_prev = y;
   if ((TAL_PLEADLP) > 0.)          /* roll the lead off above the ring band */
   {
      double tlp = 1. / ((TAL_PLEADLP) + 1.e-12);
      pl_lp += (dtl / (tlp + dtl)) * (y - pl_lp);
      return pl_lp;
   }
   return y;
}

/* Shared AutoPilot implementation for simulation and firmware. */
void AutoPilot(char mod)
 {
  static double x1_old, z1_old, y1_old, x2_old, z2_old, y2_old;
   double x1_new, z1_new, y1_new, x2_new, z2_new, y2_new;
  // double Ts = 0.01;
   double dphi;

   double Gama_sat,in_integ_pitch, in_integ_gama,in_integ_yaw;
   double cos_phi, sin_phi, wzr, wyr;
   delec0= delrc0=0.0;
  if ((fabs(Xm)<=20.) && (t <= t_ctrl)) 
  //if (l_launcher < 20. && t <= t_ctrl)//1.) // For C
  {
      Ayb_filter = Azb_filter = 0.0;
      delec = delrc = delac = 0.0;
    delec0 = delrc0 = 0.0;
      z1_old = y1_old = 0.0;
      in_integ_pitch = out_integ_wz = out_integ_wy = 0.0;
    out_integ_pitch = out_integ_psi = 0.0;
      out_integ_roll = out_integ_yaw = 0.0;
    out_integ_gama=0;
    gama_flg=gama_cnt=0;

      sepflg = sepflg2=flg_stage=oi_flg=BeltCmd=sep_fb=0;
    k_ramp=q_old=0;
    q_cunter=q_cunter1=q_cunter2=pik_q=flg_q=flg_q2=q_max=0;
    tt=t_2=t_vc=0.;
    ss_6=ign2_ok=ign2_ok5=0.;
    IntgVzRespP=0;
    wo_wz_lp = 0.; wo_wz_bp = 0.; wo_t_prev = -1.;
    pl_u_prev = 0.; pl_y_prev = 0.; pl_t_prev = -1.; pl_lp = 0.;
    iy0_t_prev = -1.; iy0_done = 0;
      
      return;
    }

   Teta_sat = 4. * (.03 + 0.008 * q);
   Psi_sat = Teta_sat;

   if (q < 15)	Roll_sat = 3. * (0.1396 + 0.03499 * q);
   else			Roll_sat = 3. * (0.6632);

   /* TAL_S6ALIGN -- see sim_fio.h.  Stage 6 is entered banked (phi 92.9 to
      172.4 deg on the flg==1 failures, against <=35 deg on every case that
      hits), so the pitch pull is resolved into rudder and the vehicle skids
      until incidence leaves the 20 deg aero table.  Give the roll channel
      authority to align before that happens.  Stock at 1.0 / 1.0.
      NEGATIVE RESULT 2026-08-23.  Swept roll gain x4/x10 with Roll_sat x2/x3,
      and separately TAL_S6HOLD (scaling the manoeuvre by |cos phi|): NONE
      stop the departure -- all four failures keep maxAlf6 = maxBet6 = 135 deg
      in every configuration, and the misses move erratically (T11n1 6973 ->
      9985 -> 11643 -> 15361) because the post-departure trajectory is chaotic
      AND outside the aero database.  Left at stock. */
   if (sim_tal_flag && s_s == 6 && fabs(phim) > TAL_S6PHI * d2r)
   {
      K_w_roll       *= TAL_S6ROLL;
      K_p_integ_roll *= TAL_S6ROLL;
      Roll_sat       *= TAL_S6SAT;
   }

   // --------------angular velocities--------------------------

 if (s_s == 3 || s_s == 4 || s_s == 5) 
  {
    Wx_filter = (1. * Wb(0,0) + 9. * Wx_filter) / 10.;
    Wy_filter = (1. * Wb(1,0) + 20. * Wy_filter) / 21.;
    Wz_filter = (1. * Wb(2,0) + 20. * Wz_filter) / 21.;
  }
   //else if (sepflg==0)  
   else if (s_s < 3) 
  {
    if (sepflg==0)  
    {
    Wx_filter = (1. * Wb(0,0) + 3. * Wx_filter) / 4.;
    Wy_filter = (1. * Wb(1,0) + 3. * Wy_filter) / 4.;
    Wz_filter = (1. * Wb(2,0) + 3. * Wz_filter) / 4.;
    }
    else
    {
    Wx_filter = (1. * Wb(0,0) + 0. * Wx_filter) / 1.;
    
    z1_new = 0.5864 * z1_old + 1. * Wb(2,0);
    Wz_filter = -0.17969 * z1_old + 1.22 * Wb(2,0); // 5.8
    z1_old = z1_new;

    y1_new = 0.5864 * y1_old + 1. * Wb(1,0);
    Wy_filter = -0.17969 * y1_old + 1.22 * Wb(1,0);
    y1_old = y1_new;
       //*/

      


    }
  }
  else 
  {
    Wx_filter = (1. *(Wb(0,0) + 0. * Wx_filter)) / 1.;
    //Wy_filter = (1. * Wb(1,0) + 2. * Wy_filter) / 3.;
    //Wz_filter = (1. * Wb(2,0) + 2. * Wz_filter) / 3.;
  }
   // ----------------------------------------------
 //  if ((s_s == 6)) // &&(q>40.||flg_q==1)&&sepflg==1)
   if ((s_s == 6))// || (s_s <3 && sepflg&& gamac_flg==0)) 
   {
  

    z1_new = 0.5864 * z1_old + 1. * Wb(2,0);
    Wz_filter = -0.17969 * z1_old + 1.22 * Wb(2,0); // 5.8
    z1_old = z1_new;

    y1_new = 0.5864 * y1_old + 1. * Wb(1,0);
    Wy_filter = -0.17969 * y1_old + 1.22 * Wb(1,0);
    y1_old = y1_new;
    //*/

    // Wy_filter = Wb(1,0);
    //  Wz_filter = Wb(2,0);

   }
  //---------------------------------------------
   az_b = sin(phim) * Ab(1,0)/9.81 + cos(phim) * Ab(2,0)/9.81;
   Azb_filter = (az_b + 3.*Azb_filter)/ 4.;
   ay_b = cos(phim) * Ab(1,0)/9.81 - sin(phim) * Ab(2,0)/9.81;
   Ayb_filter = (ay_b + 3.*Ayb_filter)/4. ;  // for 1.0 sec
  
   // ----------------------------------------------

   wzr = cos(phim) * Wz_filter + sin(phim) * Wy_filter;
   wyr = -sin(phim) * Wz_filter + cos(phim) * Wy_filter;
   // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   if (s_s == 3 || s_s == 5) {
      wzr = Wz_filter;
      wyr = Wy_filter;
   }
   // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  wy_compensate  = K_w_yaw * wyr;
  /* TAL_S6KWY -- extra terminal yaw RATE damping during the low-q pull
     (sim_fio.h).  Damping, not loop gain: the TAL_KPA6Y proportional floor
     was measured harmful (doubles terminal loads); this scales only the
     wyr feedback.  1.0 = stock, gate never fires. */
  if (sim_tal_flag && s_s == 6 && q < (TAL_S6BQ) && (TAL_S6KWY) != 1.0)
     wy_compensate *= (TAL_S6KWY);
  wy_compensate1  = K_w_psi * wyr;
  
  wz_compensate  = K_w_pitch * wzr;
  /* TAL_KWPF -- floor the stage-1/2 pitch rate damping; the schedule has a
     40% hole between q=20 and q=50.  See sim_fio.h. */
  double _kwp = K_w_pitch1;
  if ((TAL_KWPM) > 1. && sim_tal_flag && s_s <= 2 && q >= (TAL_KWPMQ0) && q < (TAL_KWPMQ)) _kwp *= (TAL_KWPM);
  if ((TAL_KWPF) > 0. && sim_tal_flag && s_s <= 2 && _kwp < (TAL_KWPF)) _kwp = (TAL_KWPF);
  wz_compensate1 = _kwp * wzr;
#if GNC_IS_SIM && DIAG_TELE
  O_vec[170]=_kwp;
#endif
  /* TAL_WOK -- DC-free extra pitch-rate damping (see sim_fio.h).
     MEASURED 2026-08-23. The design WORKS -- it delivers damping the plain
     gain could not buy: washout 0.05 gives -12% oscillation at 49/49, while
     the plain K_w_pitch1 x1.05 gives -12.5% and FAILS. But the usable band is
     narrow and bounded by a hard instability on T16:
       WOK 0.05 -> wz pk-pk 48.0 -> 42.4 (-12%), peak 24.55 -> 21.60, 49/49
       WOK 0.10 -> 37.6 (-22%), but T16n5 = 429 km  (FAIL)
       WOK 0.15 -> 33.4 (-30%), but T16 n4/n5/n6 = 329/438/418 km (46/49)
       WOK 0.50 -> 15.2 in-window, yet T10n4 DIVERGES at t=21.4, q=48 --
                   near MAX-Q, where the aerodynamic loop gain is highest;
                   wz 15.5 -> 124 -> 270 and alpha saturates at 20 deg.
     A PURE washout was tried first and was worse (its gain keeps rising with
     frequency, so it amplifies the actuator band): 0.5 drove peak wz to 67.5
     with the fins saturated. The low-pass (TAL_WOLP) makes it a BAND-PASS and
     fixed that, but not the max-q limit. Extending the gate from s_s==1 to
     s_s 1|2 (to remove the stage-boundary step in the added term) changed
     nothing -- the limit is max-q, not the boundary.
     SHIPPED at 0.05. It is a genuine but MODEST gain: -12% body oscillation,
     and it costs fin activity (achieved dele rate p99 9.7 -> 12.4 deg/s) and
     a little accuracy (mean miss 9.64 -> 9.83 m). 0.0 is a defensible choice.
     Built from wzr minus its low-pass, so it contributes NOTHING at steady
     state and cannot shift the boost trim the way scaling K_w_pitch1 does.
     tal_flag=0 is untouched -- the term is gated on sim_tal_flag. */
  if ((TAL_WOK) != 0. && (s_s == 1 || s_s == 2) && sim_tal_flag)
  {
     double dt_wo = (wo_t_prev < 0.) ? 0. : (t - wo_t_prev);
     if (dt_wo < 0. || dt_wo > 0.5) dt_wo = 0.;
     wo_t_prev = t;
     if (dt_wo > 0.)
     {
        double a_wo = dt_wo / ((TAL_WOTAU) + dt_wo);
        wo_wz_lp += a_wo * (wzr - wo_wz_lp);          /* washout: blocks DC   */
        double b_wo = dt_wo / ((TAL_WOLP) + dt_wo);
        wo_wz_bp += b_wo * ((wzr - wo_wz_lp) - wo_wz_bp); /* rolls off the HF */
     }
     wz_compensate1 += K_w_pitch1 * (TAL_WOK) * wo_wz_bp;
  }
  wz_compensate2 = K_w_pitch2 * wzr;
  double _pgkwr = K_w_roll;
  if ((TAL_PG_ROLL) && sim_tal_flag)
  {
  	if (((TAL_PG_ROLL) & 1) && _pgkwr < 0.) _pgkwr = 0.;
  	if (((TAL_PG_ROLL) & 2) && _pgkwr > (TAL_CAP_KWR)) _pgkwr = (TAL_CAP_KWR);
  }
#if GNC_IS_SIM && DIAG_TELE
  O_vec[149]=K_w_roll;
#endif
  wx_compensate  = _pgkwr * Wx_filter ;
  /* TAL_S6KWR -- extra terminal roll RATE damping during the low-q pull
     (sim_fio.h).  Memory: "a roll rate born as q->0 banks them into a
     pure-skid pull"; this damps wx before the bank develops.  1.0 = stock,
     gate never fires. */
  if (sim_tal_flag && s_s == 6 && q < (TAL_S6BQ) && (TAL_S6KWR) != 1.0)
     wx_compensate *= (TAL_S6KWR);


  if ((s_s == 3) || (s_s == 5)) 
  {
    if (Vm_t < 2.0) Vm_t = 2.0;
    // alfa_dot = (cos(tetam) * magg) / Vm_t + wzr;
  alfa_dot = wzr + mag_g * cos(tetam) / Vm_t;
    wz_compensate = K_w_pitch * alfa_dot;
  }
  else wz_compensate = K_w_pitch * wzr;

	if(sepflg && oi_flg==0)
   {
  oi_flg=1;
  out_integ_pitch=0.;
  out_integ_wz=0.;
  
  out_integ_yaw=0.;
  out_integ_wy=0.;
  out_integ_psi=0;
  
  in_integ_pitch = 0.0;

  out_integ_roll=0.;
   }

   // ------------------roll channel-------------------------------------
   du_roll = phim;

   /* TAL_ROLL180 -- see sim_fio.h.  Stock always drives the bank to ZERO, so a
      vehicle that leaves the coast at 172 deg must rotate THROUGH 90 deg,
      which is exactly where the guidance pitch demand resolves into pure
      rudder and beta runs into the 20 deg edge of the aero tables.  Rolling to
      the NEAREST of 0/+-180 keeps it out of that region.  Measured: the four
      flg==1 failures all traverse +-90 deg during stage 6; T13n1 (-157.8 deg)
      and T12n1 (34.8 deg) do not, and both hit.
      NEGATIVE RESULT 2026-08-23, left OFF.  The idea is sound and the lever is
      inert on healthy cases (bit-identical), but it does not stop the
      departure: maxAlf6 stays pinned at 135 deg in every setting, because the
      roll channel cannot move the bank in the time stage 6 allows -- delac is
      only 2-5 deg there and the rate is frozen.  Stage 6 only: T10n1 4974 ->
      3380, T10n4 7479 -> 7721, T11n1 6973 -> 7373.  All stages: 3343 / 7154 /
      7532.  Net zero. */
   if (sim_tal_flag && TAL_ROLL180 && (TAL_ROLL180 > 1 || s_s == 6))
   {
      if (du_roll >  1.5707963267948966) du_roll -= 3.141592653589793;
      if (du_roll < -1.5707963267948966) du_roll += 3.141592653589793;
   }
   double _pgkir = K_i_integ_roll;
   if ((TAL_PG_ROLL) && sim_tal_flag)
   {
   	if (((TAL_PG_ROLL) & 1) && _pgkir < 0.) _pgkir = 0.;
   	if (((TAL_PG_ROLL) & 2) && _pgkir > (TAL_CAP_KIR)) _pgkir = (TAL_CAP_KIR);
   }
#if GNC_IS_SIM && DIAG_TELE
   O_vec[151]=K_i_integ_roll;
#endif
   out_integ_roll += _pgkir * du_roll * Ts;
   du_roll = limit(du_roll, Roll_sat, -Roll_sat);
   /* TAL_ROLLI -- NEGATIVE RESULT 2026-08-22. Raising these clamps does not
      help. The dispersion DOES inject a constant roll torque: SUBSYS.CPP
      m273na_thrust() sets a TVC nozzle misalignment (eth1/eth2 = +/-8/60 deg)
      plus lateral thrust and CG offsets for flg==1 and flg==2, and sets NONE
      of them for flg==3 -- which is exactly why n0 holds 0.013 deg/s of roll
      while n1/n4 do not. But scaling these clamps by 3/10/30 leaves the
      residual rate UNCHANGED (0.279 -> 0.280 -> 0.286 -> 0.286), so the
      integrator is not saturating, while at 30x the healthy cases degrade
      (T11n0 13.6 -> 45.7 m, T12n0 5.5 -> 21.1 m) and T13n2 diverges to 819 km.
      Combined with the TAL_ROLLQ result above, the roll command is simply
      MINUSCULE in this regime: at q=0.59, K_w_roll=0.164 against p=0.328 deg/s
      gives about 0.02 deg of aileron. Neither more gain nor more integrator
      authority reaches the fins. */
   /* TAL_ROLLI_K -- early-boost roll-integrator clamp. The disturbance is a
      CONSTANT torque (mtb0 = thr*K, SUBSYS.CPP), so the integrator is the only
      term that can cancel it: the P term acts on roll ANGLE, which is ~0 this
      early, and the rate term is negligible at low q. Yet the clamp is
      0.01 rad = 0.573 deg while 4-9 deg of AILERON is available -- the
      integrator can reach only 6-14% of the authority the fin actually has. */
   if (q < 10. && s_s == 1 && pik_q == 0)
   {  double _ic = 0.01 * (sim_tal_flag ? (TAL_ROLLI_K) : 1.0);
      out_integ_roll = limit(out_integ_roll, _ic, -_ic);  }
   /* MERGE step 6 (2026-08-23): donor roll-integrator clamp, 0.02 -> 0.1. */
   else                                  out_integ_roll = limit(out_integ_roll, 0.1, -0.1);
 //  if (s_s == 2||( sepflg ))  out_integ_roll = limit(out_integ_roll, 0.01, -0.01);
   double _pgkpr = K_p_integ_roll;
   if ((TAL_PG_ROLL) && sim_tal_flag)
   {
   	if (((TAL_PG_ROLL) & 1) && _pgkpr < 0.) _pgkpr = 0.;
   	if (((TAL_PG_ROLL) & 2) && _pgkpr > (TAL_CAP_KPR)) _pgkpr = (TAL_CAP_KPR);
   }
#if GNC_IS_SIM && DIAG_TELE
   O_vec[150]=K_p_integ_roll;
#endif
   du_compensate1 = out_integ_roll + _pgkpr * du_roll; // */
   delac = (-du_compensate1 - wx_compensate);
   // ------------------yaw channel---------------------------------------
   
  if (psic_flg==0 ||(psic_flg==2 && s_s==1) || s_s>2) 
   {
   
    Acc_sat = q;

    if (s_s == 3 || s_s == 5) 
    {
      du_yaw = limit(du_yaw, 5. * d2r, -5. * d2r);
      out_integ_yaw = 0.;
   }
   if (s_s == 6) {
    du_yaw = limit(du_yaw, 2. * Acc_sat, -2. * Acc_sat);
            //out_integ_yaw += du_yaw * Ts ;
       }
   //if (s_s == 6)	du_yaw = limit(du_yaw, du_pitch_limit, -du_pitch_limit);

   double _pgacc = K_i_integ_acc;
   if ((TAL_PG_ACC) && sim_tal_flag)
   {
   	if (((TAL_PG_ACC) & 1) && _pgacc < 0.) _pgacc = 0.;
   	if (((TAL_PG_ACC) & 2) && _pgacc > (TAL_CAP_KIA)) _pgacc = (TAL_CAP_KIA);
   }
   out_integ_yaw += _pgacc * du_yaw * Ts;
   	out_integ_yaw = limit(out_integ_yaw, 0.0835, -0.0835);
   /* TAL_IYTAU0 -- flag-0 twin of the burnout yaw-trim release: the stored
      boost trim (wound chasing the saturated ayc against the misalignment
      force) torques the airframe INTO the post-burnout weathercock unwind
      and cancels the TAL_KPAF0 floor's correction.  See sim_fio.h. */
   if ((TAL_IYTAU0) > 0. && s_s == 2 && !sim_tal_flag)
   {
      double dt_iy0 = (iy0_t_prev < 0.) ? 0. : (t - iy0_t_prev);
      if (dt_iy0 < 0. || dt_iy0 > 0.5) dt_iy0 = 0.;
      iy0_t_prev = t;
      if (dt_iy0 > 0.) out_integ_yaw *= exp(-dt_iy0 / (TAL_IYTAU0));
   }
   /* TAL_IYK0 -- one-shot knock-down of the stored yaw trim at the s_s 1->2
      transition (flag-0): removes the invalid DC trim the instant the
      thrust-misalignment force vanishes, integrator live afterwards. */
   if ((TAL_IYK0) < 1. && s_s == 2 && !sim_tal_flag && !iy0_done)
   {
      out_integ_yaw *= (TAL_IYK0);
      iy0_done = 1;
   }
#if GNC_IS_SIM && DIAG_TELE
   O_vec[130]=out_integ_yaw; O_vec[131]=out_integ_yaw/0.0835; O_vec[135]=du_yaw;
#endif
   /* TAL_IYTAU -- release the yaw trim at burnout (see sim_fio.h). */
   if ((TAL_IYTAU) > 0. && s_s == 2 && sim_tal_flag)
   {
      static double t_prev_iy = -1.;
      double dt_iy = (t_prev_iy < 0.) ? 0. : (t - t_prev_iy);
      if (dt_iy < 0. || dt_iy > 0.5) dt_iy = 0.;
      t_prev_iy = t;
      if (dt_iy > 0.) out_integ_yaw *= exp(-dt_iy / (TAL_IYTAU));
   }
  //out_integ_yaw = limit(out_integ_yaw,0.3 / (K_i_integ_acc + 0.0001),-0.3 / (K_i_integ_acc + 0.0001));

		if ((pik_q == 1||q>45) && s_s <3&&sim_tal_flag)	out_integ_wy += wyr * Ts * K_p_integ_teta1 * 1.;//v15
//	else if ((pik_q == 0||(q>45)) && s_s ==1&&sim_tal_flag==0)		out_integ_wy += wyr * Ts * K_p_integ_teta1 *0.75032;//v15
	else if ((pik_q == 1||(q>70)) && s_s <3&&sim_tal_flag==0)		out_integ_wy += wyr * Ts * K_p_integ_teta1 *1.15;//v15

		else if (s_s == 6)
		{		
			if (guid_flg==1)
			{
				if (q>40 && flg_q==0) out_integ_wy += wyr * Ts * Ki_w_pitch*1.2;
				else out_integ_wy += wyr * Ts * Ki_w_pitch*1.15;
			}
			else
			{
					if (q>40 && flg_q==0) out_integ_wy += wyr * Ts * Ki_w_pitch*1.2;
					else out_integ_wy += wyr * Ts * Ki_w_pitch*1.;
			}
		}
		else	out_integ_wy = 0.;
		 if (sepflg && s_s <3) out_integ_wy=0;
		if (guid_flg==1) out_integ_wy = limit(out_integ_wy, q / 210., -q / 210.);
		else    out_integ_wy = limit(out_integ_wy, q / 40., -q / 40.);
		du_compensate1 = out_integ_yaw + K_p_integ_acc * du_yaw;
		delrc0 = (du_compensate1 - wy_compensate - 1*out_integ_wy);//*
		
   }
   else
   {
   
    if (q < 10 && pik_q == 0) du_yaw = limit(du_yaw, Psi_sat * 0.01, -Psi_sat * 0.01);
    out_integ_wy += wyr * Ts * K_p_integ_psi * 3.;
    out_integ_wy = limit(out_integ_wy, out_integ_wy_limit, -out_integ_wy_limit);
    du_yaw = limit(du_yaw, du_psi_limit, -du_psi_limit);
    in_integ_yaw = limit(du_yaw, in_integ_psi_limit, -in_integ_psi_limit);
    out_integ_psi += K_i_integ_psi * in_integ_yaw * Ts;
    out_integ_psi = limit(out_integ_psi, out_integ_psi_limit, -out_integ_psi_limit);
    du_compensate1 = out_integ_psi + K_p_integ_psi * du_yaw;
    delrc0 = du_compensate1 - wy_compensate1 - out_integ_wy;//*/
   }

   // --------------------pitch channel---------------------------------
   if (s_s == 1 )//|| s_s == 2) 
   {
      if (q < 10 && pik_q == 0) du_pitch = limit(du_pitch, Teta_sat * 0.1, -Teta_sat * 0.1);
  
       if (pik_q == 1)
	   {
	    if (guid_flg==1)out_integ_wz += wzr * Ts * K_p_integ_teta1 * 0.5;//1300p3
		else out_integ_wz += wzr * Ts * K_p_integ_teta1 * 0.15;//1300p3
	   }
     
    
    du_pitch = limit(du_pitch, 2.*Teta_sat, -2.*Teta_sat);//1300p3
      in_integ_pitch = limit(du_pitch, 5.*d2r, -5.*d2r);//1300p3
   /* if (sepflg ) 
    {
    out_integ_wz = 0.0;//1300p3
    in_integ_pitch = du_pitch;//1300p3
    }//*/
    
      { double _kit = K_i_integ_teta;
        if ((TAL_KITM) != 1.0 && sim_tal_flag && s_s <= 2) _kit *= (TAL_KITM);
        out_integ_pitch += _kit * in_integ_pitch * Ts; }
      if (q < 10 && pik_q == 0) out_integ_pitch = limit(out_integ_pitch, 0.05, -0.05);
      else            out_integ_pitch = limit(out_integ_pitch, 0.15, -0.15);
      du_compensate1 = out_integ_pitch + K_p_integ_teta1 * du_pitch;
      if ((TAL_PLEAD) > 1. && sim_tal_flag && q < (TAL_PLEADQ))
      {
         double _pl0 = du_compensate1;
         {  double _plw = 1.;
      if ((TAL_PLEADR1) > (TAL_PLEADR0))      /* transient-only ramp on |wzr| */
      {
         _plw = (fabs(wzr)*57.29578 - (TAL_PLEADR0)) / ((TAL_PLEADR1) - (TAL_PLEADR0));
         if (_plw < 0.) _plw = 0.;
         if (_plw > 1.) _plw = 1.;
      }
      double _ply = PitchLead(du_compensate1, t);
      du_compensate1 = du_compensate1 + _plw * (_ply - du_compensate1);
   }
      #if GNC_IS_SIM && DIAG_TELE
         O_vec[171] = du_compensate1 - _pl0;
      #endif
      }
      delec0 = du_compensate1 - wz_compensate1 - out_integ_wz;
   }
   // ..............................................
   else if (s_s == 2) 
   {
		out_integ_wz += wzr * Ts * K_p_integ_teta1*1.30;//1300p3
		out_integ_wz = limit(out_integ_wz, 2.918*q / 10., -2.918*q / 10.);

		//if (sepflg ) {out_integ_wz = 0.0;}//1300p3
		du_pitch = limit(du_pitch, 1.25*Teta_sat, -1.25*Teta_sat);
		in_integ_pitch = limit(du_pitch, Teta_sat, -Teta_sat);
		{ double _kit = K_i_integ_teta;
		  if ((TAL_KITM) != 1.0 && sim_tal_flag && s_s <= 2) _kit *= (TAL_KITM);
		  out_integ_pitch += _kit * in_integ_pitch * Ts; }
		if (guid_flg==1) out_integ_pitch = limit(out_integ_pitch, 0.64, -0.64);
		else  out_integ_pitch = limit(out_integ_pitch, 0.14, -0.14);
		du_compensate1 = out_integ_pitch + K_p_integ_teta1 * du_pitch;
		delec0 = du_compensate1 - wz_compensate1 - out_integ_wz; //*/
   }
   else if (s_s == 3 || s_s == 5 || s_s == 6) 
   {

      if (s_s == 3 || s_s == 5) 
    {
         du_pitch = limit(du_pitch, 5. * d2r, -5. * d2r);
         out_integ_pitch = 0.;
         out_integ_wz = 0.;
      }
      if (s_s == 6) {
        du_pitch = limit(du_pitch, du_pitch_limit, -du_pitch_limit);
              //  out_integ_pitch += du_pitch * Ts;


      }
      double _pgacc = K_i_integ_acc;
      if ((TAL_PG_ACC) && sim_tal_flag)
      {
      	if (((TAL_PG_ACC) & 1) && _pgacc < 0.) _pgacc = 0.;
      	if (((TAL_PG_ACC) & 2) && _pgacc > (TAL_CAP_KIA)) _pgacc = (TAL_CAP_KIA);
      }
      out_integ_pitch += _pgacc * du_pitch * Ts;
      out_integ_pitch = limit(out_integ_pitch, out_integ_limit, -out_integ_limit);
     
     //out_integ_pitch = limit(out_integ_pitch,0.5 / (K_i_integ_acc + 0.0001),-0.5 / (K_i_integ_acc + 0.0001));

      out_integ_wz += wzr * Ts * Ki_w_pitch;
      out_integ_wz = limit(out_integ_wz, out_integ_w_limit, -out_integ_w_limit);
      /* TAL_KPA6P -- terminal PITCH read of the same wrong-signed gain. */
      double _kpp = K_p_integ_acc;
      if ((TAL_KPA6P) >= 0. && sim_tal_flag)
      {
         double _pfl = (TAL_KPA6P) * 0.125 / (q + 1.e-6);
         if (_kpp < _pfl) _kpp = _pfl;
      }
      /* TAL_AYLD6 -- un-lag the Ay feedback in the terminal pitch loop.
         du_pitch carries -Ayb_filter[k-1], so SUBTRACT.  See sim_fio.h. */
      double _ayl = 0.;
      if ((TAL_AYLD6) > 0. && sim_tal_flag)
         _ayl = -(TAL_AYLD6) * (4./3.) * (ay_b - Ayb_filter);
      du_compensate1 = out_integ_pitch + _kpp * (du_pitch + _ayl);
#if GNC_IS_SIM && DIAG_TELE
      O_vec[153]=K_p_integ_teta1; O_vec[154]=K_w_pitch1; O_vec[155]=K_i_integ_teta;
      O_vec[156]=du_pitch; O_vec[157]=out_integ_pitch; O_vec[158]=_kpp; O_vec[159]=_ayl;
#endif
#if GNC_IS_SIM && DIAG_TELE
      O_vec[148]=_kpp;
#endif

	  if (guid_flg==1)  delec0 = du_compensate1 - wz_compensate - 1*out_integ_wz ;
      else    delec0 = du_compensate1 - wz_compensate - 0*out_integ_wz ;
   }

   SdreOperatingPoint operating_point = {};
   if (Ctrl_Mode != SDRE_MODE_PID) {
      SdreM4Inputs model_input = {};
      model_input.time_s = t;
      model_input.dynamic_pressure_pa = q_new * 1.0e4;
      model_input.velocity_mps = sqrt((Vm_t_2 > 0.0) ? Vm_t_2 : 0.0);
      model_input.mach = mach330;
      model_input.alpha_rad = alfa_g;
      model_input.beta_rad = beta_g;
      model_input.second_stage_ignition_time_s = t_ign2;
      model_input.stage = s_s;
      model_input.separated = sepflg;
      model_input.second_separation = sepflg2;
      model_input.first_stage_burned_out = burn_flg;
      model_input.second_stage_ignition_started = (t >= t_ign2);
      model_input.second_stage_burned_out = burn_flg2;
      model_input.tal_configuration = sim_tal_flag;
      SdreM4_BuildOperatingPoint(&model_input, &operating_point);
      Sdre_SetOperatingPoint(&operating_point);
   }

   Sdre_Controller(&delac, &delec0, &delrc0,
                   phim, Wx_filter, wzr, wyr,
                   du_pitch, alfa_g, beta_g, du_yaw, ayc,
                   (psic_flg==0 || (psic_flg==2 && s_s==1) || s_s>2),
                   t, Ts, t_ctrl, s_s, sepflg, pik_q);

   if (s_s == 4)
      delec0 = delrc0 = 0.;

   /* NEGATIVE RESULT, 2026-08-22 -- do not retry without new evidence.
      In the exo-atmospheric coast the roll channel is NOT zeroed the way
      pitch and yaw are on the line above, and the s_s==4 gain block
      (PILOT.CPP:547) never assigns the roll gains, so they keep stage 3's
      schedule whose 0.1/(q+0.01) term diverges as q -> 0. With delec0 and
      delrc0 zero the mixer degenerates to dac[i] = +/-Ac, so a saturated roll
      command drives all four fins to the +/-20 deg clip with no aerodynamic
      authority, and out_integ_roll winds up unopposed. Measured on the
      tal_flag=1 residual failures: stage-4 fin saturation 87-94% against 0%
      on a passing case, and roll reaching 180 deg -- the vehicle carries only
      ~0.5-0.6 deg/s of roll rate into a ~350 s coast, which free-integrates
      to 180-220 deg.
      Holding the channel through the coast --
          if (sim_tal_flag && s_s == 4) { delac = 0.; out_integ_roll = 0.; }
      -- was TRIED and made NO measurable difference: 42/49 before and after,
      with T10n4 7480 -> 7382 m and T13n2 41394 -> 41435 m. The coast roll is
      therefore a SYMPTOM, not the cause of these misses; the wasted actuator
      activity is real but does not drive the miss. Look elsewhere. */

   if (sepflg == 1) {
      delec0 = -delec0;
      delrc0 = -delrc0;
   }
   // ****************End of Loop***************************************
	if (guid_flg==0)
	{
	   lambda_ = atan(Vzm / Vxm);

	   if (GNC_STAGE2_ACTIVE)
	   {
		if(TVC_sf==0) t_stv0=t;
		if((t_ign2 <t) && burn_flg2==0) TVC();
		else {eth2_2=eth1_2=0.; }
	   }
	}
	
   // Transform the picth and yaw commands from body frame to execute frame.

   /* TAL_S6HOLD -- hold back the terminal manoeuvre while the vehicle is
      still banked, so the pull is not resolved into a pure skid.  |cos(phi)|
      because a 180 deg bank is as usable as 0 for a symmetric airframe.
      Floored so guidance never loses authority completely.  0 = stock. */
   if (sim_tal_flag && TAL_S6HOLD && s_s == 6)
   {
      double al_s6 = fabs(cos(phim));
      if (al_s6 < TAL_S6FLOOR) al_s6 = TAL_S6FLOOR;
      delec0 *= al_s6;
      delrc0 *= al_s6;
   }

   /* LATENT DEFECT (found 2026-08-22, left as-is): this roll-rate cutout is
      UNREACHABLE.  It demands |delac| > 14 deg, but delac is limited to at
      most 8 deg by aileron_limit below -- and to 4 deg once the incidence
      clamp trips -- so the guard never fires, not even at the 222 deg/s roll
      of the flg==2 arms it was evidently written for.
      Measured with the threshold made reachable (TAL_RCUT, sim_fio.h):
      at 2.0 deg T13n2 41394 -> 38105 m, peak roll 222 -> 177 deg/s, burnout
      velocity +3.5 m/s, healthy cases bit-identical; at 1.0 deg everything
      degrades (T13n2 45932, T11n2 11.2 -> 134.0 m, nearly a new failure).
      +3.5 m/s against the 63.8 m/s the case actually needs -- real but far
      too small to change the gate, so the guard is left dead rather than
      shipped half-live.  Fix it only as part of a full early-boost authority
      redesign, and re-gate both flags if you do. */
   if ((fabs(Wx_filter) > (90. * d2r))
       && (fabs(delac) > ((sim_tal_flag ? TAL_RCUT : 14.) * d2r)) && (s_s != 4.)) {
      delec = delrc = 0.0;
      out_integ_pitch = out_integ_yaw = out_integ_gama=0.0;
   }
   else if (s_s == 6. || s_s == 1. || s_s == 2.) {
      delec = delec0 * cos(phim) - delrc0 * sin(phim);
      delrc = delrc0 * cos(phim) + delec0 * sin(phim);
   }
   else {
      delec = delec0; // *cos(_phim) - delrc0*sin(_phim) ;
      delrc = delrc0; // *cos(_phim) + delec0*sin(_phim) ;
   }

   /* TAL_SLEW -- fin-command slew limit, matched to the 75 deg/s servo
      (SUBSYS.CPP speed_servo). dt is derived from elapsed t and is 0 on RK4
      sub-steps, where the limiter simply holds. */
   {
      static double t_prev_slew=-1., pe_slew=0., pr_slew=0., pa_slew=0.;
      double dt_slew = (t_prev_slew < 0.) ? 0. : (t - t_prev_slew);
      if (dt_slew < 0. || dt_slew > 0.5) dt_slew = 0.;
      t_prev_slew = t;
      if (t <= t_ctrl) { pe_slew = pr_slew = pa_slew = 0.; t_prev_slew = -1.; }
      if ((TAL_SLEW) > 0. && dt_slew > 0.)
      {
      double mx_slew = (TAL_SLEW) * d2r * dt_slew;
      delec = pe_slew + limit(delec - pe_slew, mx_slew, -mx_slew);
      delrc = pr_slew + limit(delrc - pr_slew, mx_slew, -mx_slew);
      delac = pa_slew + limit(delac - pa_slew, mx_slew, -mx_slew);
      /* advance the reference ONLY on a real time step -- on RK4 sub-steps
         dt is 0 and storing there would latch the UNLIMITED command and
         defeat the limiter entirely. */
      pe_slew = delec; pr_slew = delrc; pa_slew = delac;
      }
   }

}


/* Shared TVC implementation for simulation and firmware. */
void TVC()
{
      delec0 = 0.;
  
      double ax_b = GNC_ACC_X;
      if (GNC_ACC_X < 6.)
         ax_b = 6.;
      apc = limit(apc, ax_b * 0.6, -ax_b * 0.6);
      //teta_command = asin(apc / ax_b);
	  teta_command = du_pitch;
      apc=apc*sin(teta_command);
     //  teta_command=asin(apc/ax_b);
      ayc = limit(ayc, 0.36 * ax_b * cos(teta_command), -0.36 * ax_b * cos(teta_command));
      psi_command = psic;//-du_yaw;//-asin(ayc / (ax_b * cos(teta_command)));

      if (tvc_flg == 0) 
	  {
         del_tvc = (angle_tvc_sim - angle_tvc_jad) * 2.;
         tvc_flg = 1;
      }
      euler_d[0] =0.8*teta_command*r2d;// *sin(2*M_PI*.25*(t-t_s8));//
      euler_d[1] =psi_command*r2d;
      double Cf2d_1[3][3], Cf2d_2[3][3];
      e2m(euler_d[1]*d2r, euler_d[0]*d2r, euler_d[2]*d2r, Cf2d_1);

       double q_d[4], dq[4], qq[4], _q[4];
       e2q(euler_d[0]*d2r, euler_d[1]*d2r, euler_d[2]*d2r, q_d);
       GNC_Q2M(q_d, Cf2d_2);

      double mf2b[3][3], Cb2d[3][3];
      GNC_BIND_MB2F(mb2f_);
      for (int i = 0; i < 3; i++)
         for (int j = 0; j < 3; j++)
            mf2b[j][i] = mb2f_[i][j];

      for (int i = 0; i < 3; i++)
         for (int j = 0; j < 3; j++) {
            Cb2d[i][j] = 0.;
            for (int k = 0; k < 3; k++)
              //Cb2d[i][j] += Cf2d_2[i][k] * mb2f_[k][j];
               Cb2d[i][j] += Cf2d_1[i][k] * mb2f_[k][j];
         }

      double tab_d[3], tab[3], teta_tab;
      tab_d[0] = 1.;
      tab_d[1] = 0.;
      tab_d[2] = 0.;

      twm(2, Cb2d, tab_d, tab_b);
      cross(tab_b, tab_d, tab);

      teta_tab = acos(tab_d[0] * tab_b[0] + tab_d[1] * tab_b[1] +
          tab_d[2] * tab_b[2]);

      for (int i_tab = 0; i_tab < 3; i_tab++)
         a_tab[i_tab] = teta_tab * tab[i_tab] / sqrt
             (tab[0] * tab[0] + tab[1] * tab[1] + tab[2] * tab[2]);

      
       m2q(Cb2d,dq);

       double error_angle=(psic-psim)*2;//acos(dq[0])*2.;
       if (fabs(error_angle)>10*GNC_PI/180)
       error_angle=10.*GNC_PI/180.*fabs(error_angle)/error_angle;
       double eq[3];
       eq[0]=-1.*dq[1];eq[1]=-1.*dq[2];eq[2]=-1.*dq[3];
       if(fabs(error_angle)>1e-4)
       {
       for(int k=0;k<3;k++)
       eq[k]=eq[k]/sin(error_angle/2)*error_angle;
       }
       else
       {
       for(int k=0;k<3;k++)
       eq[k]=eq[k]*error_angle;
       }

      eth2_2=-(0.64*eq[1] + 0.64* Wy_filter);
    
	  eth1_2 =   (1*  du_pitch  - 0.64* Wz_filter);//CONTROL BY TETE COMMANED

     delrc0=0;
     eth2_2 = limit(eth2_2, 10 * d2r, -10 * d2r);
     eth1_2 = limit(eth1_2, 10 * d2r, -10 * d2r);
	 
      TVC_sf=1;
  
}
//-----------------------End stages Gains---------------------
/////////////////////////////////////////////////////////////////////////////
/* Shared Outputs implementation with local platform-only regions. */
void Outputs()
{
  //In rear view>>>  1 & 2 are in right side and 3 & 4 are in left side with clockwise order
  //Actuature + is clockwise 
  //Actuature - is anti-clockwise 
//------------------------------------------------
#if GNC_IS_SIM
  if ((m_flag == 0) || (m_flag == 1 && s_s > 1)) {
    delec = 0.;
    //delrc = delac = 0.;

  }
   if (m_flag == 0 || m_flag == 1) {
     delrc = delac = 0.;
        }
#endif
  //if(StopCtrlCmd && sepflg==0)//&& s_s<3) 
  //if(StopCtrlCmd && (s_s<3 || sepflg2)) 
  if((StopCtrlCmd && s_s<3) || sepflg2) 
  {
    delrc = delac = delec=0.;//ocv14
    eth1_2=eth2_2=0.;
    //return;
  }
  
  double lim=20;
  //if(s_s==6 && q>60)  lim = 20;

   double rudder_limit = lim * d2r - (fabs(delec) + fabs(delac));

  //   if (s_s == 2) rudder_limit = lim * d2r - 0.4*(fabs(delec) + fabs(delac));

   /* TAL_S6RF / TAL_S6EF -- re-split the fin budget during the low-q
      terminal pull (see sim_fio.h).  The rudder is clamped FIRST with the
      raw pinned elevator already subtracted, so in the pull it gets exactly
      its floor; raising that floor is the only way to give yaw authority
      there.  Shared simulation/firmware floors are 6 / 9 deg everywhere
      else; gate is observable state only. */
   double _rf6 = 6., _ef6 = GNC_OUTPUT_ELEVATOR_FLOOR;
   if (sim_tal_flag && s_s == 6 && q < (TAL_S6BQ))
   {  _rf6 = (TAL_S6RF);  _ef6 = (TAL_S6EF);  }

   if (rudder_limit < _rf6 * d2r)
      rudder_limit = _rf6 * d2r;
   delrc = limit(delrc, rudder_limit, -rudder_limit);
#if GNC_IS_SIM && DIAG_TELE
   O_vec[133]=delrc; O_vec[134]=rudder_limit;
#endif
   double delta_delrc = 6. * d2r - fabs(delrc);

   double elevator_limit = lim * d2r - (fabs(delac) + fabs(delrc));
   if (elevator_limit < _ef6 * d2r)
      elevator_limit = _ef6 * d2r;
   if ((TAL_ELOQ) > 1. && s_s == 1 && q < (TAL_ELOQQ)) delec *= (TAL_ELOQ);
   delec = limit(delec, elevator_limit, -elevator_limit);
#if GNC_IS_SIM && DIAG_TELE
   O_vec[160]=elevator_limit; O_vec[161]=delec; O_vec[162]=ay_b;
   O_vec[163]=K_p_integ_teta1; O_vec[164]=K_w_pitch1; O_vec[165]=K_i_integ_teta;
   O_vec[166]=Ki_w_pitch; O_vec[167]=K_w_pitch; O_vec[168]=delac; O_vec[169]=Ayb_filter;
#endif
   double delta_delec = GNC_OUTPUT_ELEVATOR_FLOOR * d2r - fabs(delec);

   double aileron_limit = lim * d2r - MAX(fabs(delec), fabs(delrc));
   if (aileron_limit >= 9. * d2r)
      aileron_limit = 9. * d2r + delta_delrc + delta_delec;
   if (aileron_limit < 9. * d2r)
      aileron_limit = 9. * d2r;

   if ((t > 1.) && (q < 10.) && (s_s == 1) && pik_q == 0) {
      /* TAL_AILIM/TAL_AILATCH -- see sim_fio.h.  Stock behaviour is
         preserved exactly at TAL_AILIM 4.0 / TAL_AILATCH 1 and for
         sim_tal_flag == 0.
         NEGATIVE RESULT 2026-08-22.  This clamp halves roll authority
         (8 -> 4 deg) once body incidence passes 7 deg in early boost, and
         latches.  It looked like the trap behind the flg==2 departure; it is
         not.  Sweeping the clamp 4/8/20 deg and disabling the latch leaves
         max sideslip INVARIANT at 18.24 deg and burnout velocity within
         3 m/s (2444.1 -> 2435.4), while peak roll gets WORSE (222 -> 300
         deg/s at 8 deg).  The latch is inert besides -- LATCH=0 is
         bit-identical to LATCH=1, because the q<10 window closes first.
         The incidence is set by the disturbance, not by the roll budget. */
      int latched_ail = (sim_tal_flag && !TAL_AILATCH) ? 0 : (flag_alfa == 1);
      if ((alfab_g > 7. * d2r ) || latched_ail) {
         aileron_limit = (sim_tal_flag ? TAL_AILIM : 4.) * d2r;
         flag_alfa = 1.;
      }
   }
   // */
   /* TAL_AREV -- aileron-reversal guard, see sim_fio.h. RAJEH_CO_ROLL_S1.dat
      is POSITIVE (i.e. the fin rolls the WRONG WAY) at alpha 12/16/20 deg and
      Mach 0.3/0.5/0.8, and negative everywhere else. Commanding a roll
      correction inside that region amplifies the excursion instead of
      arresting it. alfab_g and mach330 are both observable. */
   if ((TAL_AREV) && sim_tal_flag && s_s == 1 &&
       alfab_g > (TAL_AREV_ALF)*d2r && mach330 < (TAL_AREV_MACH))
   {
      if ((TAL_AREV) == 1) delac = 0.;
      else                 delac = -delac;
   }
   delac = limit(delac, aileron_limit, -aileron_limit); // */
#if GNC_IS_SIM && DIAG_TELE
   O_vec[136]=delec; O_vec[137]=delac;
#endif

  if (s_s == 4) {
    delrc = 0.0;
    delec = 0.0;
  }
  {  /* Corrosion-compensating allocation: invert the degraded mixing so the
        achieved (pitch, yaw, roll) match the pristine response. Off unless
        sim_fin_comp > 0, so the stock path is byte-identical. */
     double Ec = delec, Rc = delrc, Ac = delac;
     if (s_s == 6 && ( (sim_comp_track && sim_wear_on)
                    || (sim_fin_comp > 0.05 && sim_fin_comp < 1.0) ))
     {
        /* Tracking mode: the onboard model integrates the same measured-q
           wear, so the mixer knows k(t) instead of assuming a constant. */
        double kk2 = (sim_comp_track && sim_wear_on) ? sim_wear_k : sim_fin_comp;
        if (kk2 > 0.999) kk2 = 0.999;
        Ec = 2.0*delec / (1.0 + kk2);
        Rc = ((1.0+kk2)*delrc + (1.0-kk2)*delac) / (2.0*kk2);
        Ac = ((1.0-kk2)*delrc + (1.0+kk2)*delac) / (2.0*kk2);
     }
  dac[0] = (Ac - Rc - Ec)*r2d;
  dac[1] = (Ac + Rc - Ec)*r2d;
  dac[2] = (Ac + Rc + Ec)*r2d;
  dac[3] = (Ac - Rc + Ec)*r2d;
  }
  
  //1300C
  double finlim = 20.;
  if(s_s==6 && hgv_flg==0)  
  {
    //finlim = 20;
    if(q>60)  finlim = 20;
  }
	dac[0] =  limit(dac[0],finlim,-finlim);
	dac[1] =  limit(dac[1],finlim,-finlim);
	dac[2] =  limit(dac[2],finlim,-finlim);
	dac[3] =  limit(dac[3],finlim,-finlim);
	
//-------------------------------------
/*  if(t<100. && ((_100hz_counter%10)==0))
  {
  char str2[256];
  char str3[256];
  char str4[256];
  sprintf(str2," stage:- %d tetam:%2.2f psim:%2.2f phim:%2.2f ",s_s,tetam*r2d,psim*r2d,phim*r2d);//m273
  sprintf(str3," delec0:%2.2f delrc:%2.2f delac:%2.2f ",delec0*r2d,delrc*r2d,delac*r2d);//m273
  sprintf(str4," dac1:%2.2f dac2:%2.2f dac3:%2.2f  dac4:%2.2f ",dac[0],dac[1],dac[2],dac[3]);//m273

  outscreen(1,4,str2);
  outscreen(1,5,str3);
  outscreen(1,6,str4);
  }//*/
  /*
  for(int j=0;j<4;j++)
    {
    Dig[j]=2.0475*(de2di(dac[j])-7191.);                            //without actuator
    vOut[j]=Dig[j]*5./4095.;                            //without actuator
    OutVolt(j,vOut[j]);                             //without actuator
    Dig2[j]=de2di(dac[j]);                            //without actuator
    }
   SendFrame(Dig2,4);//with actuator with d2a 

  if(Exteraflg==1)return; //*/

#if !GNC_IS_SIM
  for (int j = 0; j < 4; j++)
  {
    Dig[j] = 2.0475 * (de2di(dac[j]) - 7191.);
    vOut[j] = Dig[j] * 5. / 4095.;
    OutVolt(j, vOut[j]);
    Dig2[j] = de2di(dac[j]);
  }
  SendFrame_Can(Dig2, 4);

  if (Exteraflg == 1) return;
#endif

  
return;

}

float interpol_MH(double m, double a, float mx[11][13])
{
   float h_t[11] = {0., 5.e3, 10.e3, 15.e3, 20.e3, 25.e3, 30.e3, 35.e3, 40.e3, 45.e3, 50.e3};
   float mach_t[13] = {1.5, 3., 4., 5., 6., 7., 8., 9., 10., 11., 12., 13., 14.};

   if (m <= 1.5)  m = 1.5;
   if (m >= 14.)  m = 14.;
   int ma = 11, mm = 13, i, j;
   float c1, c2, c, S = 0;
   S = (a >= 0) ? 0 : (a = -a);
   for (i = 1; i < ma && a > h_t[i]; i++);
   i = (--i == ma - 1) ? ma - 2 : i;
   for (j = 1; j < mm && m > mach_t[j]; j++);
   j = (--j == mm - 1) ? mm - 2 : j;
   c1 = mx[i][j] + (mx[i][j + 1] - mx[i][j]) * (m - mach_t[j]) /
       (mach_t[j + 1] - mach_t[j]);
   c2 = mx[i + 1][j] + (mx[i + 1][j + 1] - mx[i + 1][j]) * (m - mach_t[j]) /
       (mach_t[j + 1] - mach_t[j]);
   c = c1 + (c2 - c1) * (a - h_t[i]) / (h_t[i + 1] - h_t[i]);
   return (S) ? -c : c;

}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/* Platform-specific Destruct behavior. */
#if GNC_IS_SIM
void Destruct()
{
  
  double dx_tr, dy_tr, dz_tr;
  dx_tr = Xtrgt - Xm ;
  dy_tr = Hme-Ht ;
  dz_tr = Ztrgt - Zm ;

  if(destruct_flg==0)
  {
    //if (t>=1)       //destruct_flg = 1;
    if (s_s ==1)      destruct_flg = 1;
    else          destruct_flg=0;
  }
  else if(destruct_flg==1)
  {   
    //if (t>=10)  
    if(Hme >=5000)      destruct_flg = 2; 
    else          destruct_flg = 1; 
  }
  else if(destruct_flg==2)
  {
    //if (t>=20)  
    if(Xm >=20000)      destruct_flg = 3; 
    else          destruct_flg = 2;
  }       
  else if(destruct_flg>=3)
  {
    //if (t>=80)  
    if(Hme <=15000 && Vym<=0)
    {
      destruct_flg = 4; 
      CWarHead_Cmd=enemyzone;
//      WarHead(CWarHead_Cmd);
    }
    else          
    {
      destruct_flg = 3;
      CWarHead_Cmd=startflight;
//      WarHead(CWarHead_Cmd);
    }
  } 
   
  char str[200];
  sprintf(str," the output is :   %d",destruct_flg);
//  outscreen(1,22,str);

}
#else
void Destruct()
{
	
  double dx_tr, dy_tr, dz_tr;
	dx_tr = Xtrgt - Xm ;
	dy_tr = Hme-Ht ;
	dz_tr = Ztrgt - Zm ;

	if(destruct_flg==0)
	{
		//if (t>=1)				//destruct_flg = 1;
		if (s_s ==1)			destruct_flg = 1;
		else 					destruct_flg=0;
	}
	else if(destruct_flg==1)
	{		
		//if (t>=10)	
		if(Hme >=5000)			destruct_flg = 2; 
		else 					destruct_flg = 1; 
	}
	else if(destruct_flg==2)
	{
		//if (t>=20)	
		if(Xm >=20000)			destruct_flg = 3; 
		else 					destruct_flg = 2;
	}				
	else if(destruct_flg>=3)
	{
		//if (t>=80)	
		if(Hme <=15000 && Vym<=0)
		{
			destruct_flg = 4; 
			//CWarHead_Cmd=enemyzone;
			WarHead(enemyzone);
		}
		else 					
		{
			destruct_flg = 3;
			//CWarHead_Cmd=startflight;
			WarHead(startflight);
		}
	}	
	 
	char str[200];
	sprintf(str," the output is :   %d",destruct_flg);
	outscreen(1,22,str);

}
#endif//End of Destruct
/////////////////////////////////////////////////////////////////////////////
int sign(double sg)
{
  if(sg>0.) return 1;
  return -1;
}

double limit(double var , double max_val , double min_val)
{
  if(var>=max_val)       return max_val;
  else if(var<=min_val)  return min_val;
  return var;
}
////////////////////////////////////////////////////////////////////////
double max(double value1, double value2)
{
 return ( (value1 > value2) ? value1 : value2);
}

double min(double value1, double value2)
{
 return ( (value1 < value2) ? value1 : value2);
}
//////////////////
double look2_binlxpw(double _hm, double _Mach,double H_tab[],double Mach_tab[],float  table[],unsigned int NU_H_M[],unsigned int stride)
{

  double fractions[2],frac,yL_0d0,yL_0d1;
  unsigned int bpIndices[2],bpIdx,iLeft,iRght;

  if (_hm <= H_tab[0]) 
  {
    iLeft = 0;
    frac = (_hm - H_tab[0]) / (H_tab[1] - H_tab[0]);
  } 
  else if (_hm < H_tab[NU_H_M[0]]) 
  {
    bpIdx = NU_H_M[0] >> 1;
    iLeft = 0;
    iRght = NU_H_M[0];
    while (iRght - iLeft > 1) 
  {
      if (_hm < H_tab[bpIdx]) iRght = bpIdx;
      else iLeft = bpIdx;
      
      bpIdx = (iRght + iLeft) >> 1;
    }
    frac = (_hm - H_tab[iLeft]) / (H_tab[iLeft + 1] - H_tab[iLeft]);
  } 
  else 
  {
    iLeft = NU_H_M[0] - 1;
    frac = (_hm - H_tab[NU_H_M[0] - 1]) / (H_tab[NU_H_M[0]] - H_tab[NU_H_M[0]- 1]);
  }

  fractions[0] = frac;
  bpIndices[0] = iLeft;
  if (_Mach <= Mach_tab[0]) 
  {
    iLeft = 0;
    frac = (_Mach - Mach_tab[0]) / (Mach_tab[1] - Mach_tab[0]);
  } 
  else if (_Mach < Mach_tab[NU_H_M[1]]) 
  {
    bpIdx = NU_H_M[1] >> 1;/// Binary Search 
    iLeft = 0;
    iRght = NU_H_M[1];
    while (iRght - iLeft > 1) 
  {
      if (_Mach < Mach_tab[bpIdx]) iRght = bpIdx;
      else                 iLeft = bpIdx;
      bpIdx = (iRght + iLeft) >> 1;
    }
    frac = (_Mach - Mach_tab[iLeft]) / (Mach_tab[iLeft + 1] - Mach_tab[iLeft]);
  } else {
    iLeft = NU_H_M[1] - 1;
    frac = (_Mach - Mach_tab[NU_H_M[1] - 1]) / (Mach_tab[NU_H_M[1]] - Mach_tab[NU_H_M[1]- 1]);
  }
  bpIdx = iLeft * stride + bpIndices[0];
  yL_0d0 = table[bpIdx];
  yL_0d0 += (table[bpIdx + 1] - yL_0d0) * fractions[0];
  bpIdx += stride;
  yL_0d1 = table[bpIdx];
  return (((table[bpIdx + 1] - yL_0d1) * fractions[0] + yL_0d1) - yL_0d0) * frac + yL_0d0;
}

double gains_interpols(double x,int size,float  qMat[],float  gainsMat[])
{
  //int size = 140;
  //double *q=qMat;
  //double *g=gainsMat;// + idx*ng;
    
  if (x<=qMat[0]){
    return gainsMat[0];
  } else if (x>=qMat[size-1]){
    return gainsMat[size-1];
  } else {
    int i=1;
    while(qMat[i]<x){
      i++;
    }
    double x0 = qMat[i-1];
    double x1 = qMat[i];
    double g0 = gainsMat[i-1];
    double g1 = gainsMat[i];
    return g0 +(g1-g0) * (x - x0)/(x1 - x0);
  }
}
///////////////////////////////////////////////
//--- Grid axes ---
/*
static const double TBL_Kp[N_QQ][N_MACH] = {
    { 0.0067139139, 0.0048051592, 0.0040320908, 0.0030788659, 0.0023795651, 0.0009591616, 0.0535109746, 0.0535109746 },  // qq=1.0
    { 0.0067139139, 0.0045704301, 0.0039767554, 0.0027686782, 0.0023795651, 0.0010073964, 0.0289001011, 0.0289001011 },  // qq=2.0
    { 0.0065938386, 0.0041602728, 0.0030460602, 0.0027686782, 0.0018027773, 0.0007160555, 0.0178140663, 0.0178140663 },  // qq=3.5
    { 0.0048496093, 0.0038759537, 0.0030788659, 0.0025159652, 0.0015439601, 0.0007160555, 0.0129837407, 0.0129837407 },  // qq=5.0
    { 0.0037476707, 0.0036676918, 0.0024455635, 0.0018718188, 0.0013523127, 0.0007452237, 0.0094311327, 0.0094311327 },  // qq=7.0
    { 0.0033196762, 0.0027723444, 0.0022382752, 0.0018027773, 0.0017138094, 0.0005571109, 0.0054481472, 0.0054481472 },  // qq=10.0
    { 0.0023206997, 0.0025644514, 0.0017533939, 0.0017636244, 0.0010073964, 0.0003469496, 0.0002970357, 0.0036216340 },  // qq=15.0
    { 0.0017533939, 0.0012371152, 0.0012103928, 0.0014641524, 0.0007452237, 0.0008230526, 0.0005042135, 0.0026844488 },  // qq=20.0
    { 0.0013736200, 0.0007860111, 0.0006762465, 0.0005947019, 0.0008369663, 0.0002287861, 0.0001755443, 0.0010635269 },  // qq=30.0
    { 0.0005947019, 0.0003523284, 0.0008369663, 0.0002776061, 0.0002309544, 0.0001752571, 0.0001406311, 0.0005278241 },  // qq=40.0
    { 0.0002276034, 0.0002276034, 0.0003101936, 0.0001842411, 0.0002111390, 0.0001239557, 0.0000855185, 0.0001272916 },  // qq=55.0
    { 0.0002111390, 0.0002111390, 0.0001390963, 0.0001390963, 0.0005823903, 0.0000775880, 0.0000688042, 0.0000407856 },  // qq=70.0
    { 0.0001390963, 0.0001390963, 0.0001052028, 0.0001052028, 0.0000953149, 0.0000723007, 0.0000617899, 0.0000548421 }   // qq=80.0
};

static const double TBL_Ki[N_QQ][N_MACH] = {
    { 0.0026855656, 0.0019220637, 0.0016128363, 0.0015372762, 0.0009943052, 0.0004287356, 0.0214043899, 0.0214043899 },  // qq=1.0
    { 0.0026855656, 0.0018281721, 0.0015907022, 0.0013106701, 0.0009943052, 0.0004284693, 0.0115600404, 0.0115600404 },  // qq=2.0
    { 0.0026375354, 0.0016641091, 0.0015071074, 0.0013106701, 0.0008687067, 0.0003229402, 0.0071256265, 0.0071256265 },  // qq=3.5
    { 0.0019398437, 0.0016195736, 0.0015372762, 0.0010126565, 0.0007205515, 0.0003229402, 0.0051934963, 0.0051934963 },  // qq=5.0
    { 0.0015820320, 0.0014884684, 0.0012558391, 0.0009612113, 0.0005906991, 0.0002999467, 0.0037724531, 0.0037724531 },  // qq=7.0
    { 0.0014293895, 0.0013666280, 0.0008958695, 0.0008687067, 0.0006855237, 0.0002273258, 0.0021792589, 0.0021792589 },  // qq=10.0
    { 0.0009314654, 0.0010257806, 0.0007013576, 0.0007054498, 0.0004284693, 0.0001419529, 0.0001282080, 0.0014486536 },  // qq=15.0
    { 0.0007013576, 0.0005248635, 0.0005422506, 0.0005856610, 0.0002999467, 0.0003292210, 0.0002382108, 0.0010737795 },  // qq=20.0
    { 0.0005494480, 0.0003489571, 0.0002847472, 0.0002378808, 0.0003347865, 0.0000915145, 0.0000716298, 0.0004557061 },  // qq=30.0
    { 0.0002378808, 0.0001409314, 0.0003347865, 0.0001135813, 0.0000923818, 0.0000701029, 0.0000562524, 0.0002442862 },  // qq=40.0
    { 0.0000910414, 0.0000910414, 0.0001486019, 0.0000736964, 0.0000928772, 0.0000495823, 0.0000342074, 0.0000509166 },  // qq=55.0
    { 0.0000928772, 0.0000928772, 0.0000561405, 0.0000561405, 0.0003226358, 0.0000310352, 0.0000275217, 0.0000163142 },  // qq=70.0
    { 0.0000561405, 0.0000561405, 0.0000420811, 0.0000420811, 0.0000381260, 0.0000289203, 0.0000247160, 0.0000225587 }   // qq=80.0
};

static const double TBL_Kw[N_QQ][N_MACH] = {
    { 0.14924147, 0.16626662, 0.12831535, 0.15395455, 0.10618667, 0.09974753, 0.58337706, 0.58337706 },  // qq=1.0
    { 0.14924147, 0.16716301, 0.12698207, 0.13565552, 0.10618667, 0.09459578, 0.43287064, 0.43287064 },  // qq=2.0
    { 0.14538655, 0.16776731, 0.16101736, 0.13565552, 0.12195942, 0.09704364, 0.33933676, 0.33933676 },  // qq=3.5
    { 0.13971052, 0.16126992, 0.15395455, 0.11065000, 0.11954946, 0.09704364, 0.28897012, 0.28897012 },  // qq=5.0
    { 0.16895579, 0.13743979, 0.14570558, 0.13764740, 0.11517638, 0.08722631, 0.24523439, 0.24523439 },  // qq=7.0
    { 0.14616772, 0.14196921, 0.10760602, 0.12195942, 0.08530976, 0.08674957, 0.21835576, 0.21835576 },  // qq=10.0
    { 0.10850101, 0.10096729, 0.10082925, 0.09248284, 0.09459578, 0.07804279, 0.07392705, 0.17776162 },  // qq=15.0
    { 0.10082925, 0.11171381, 0.10722610, 0.08195345, 0.08722631, 0.07289449, 0.08012061, 0.15247154 },  // qq=20.0
    { 0.08042176, 0.09654909, 0.09052395, 0.08566057, 0.07310505, 0.05703446, 0.06006457, 0.11620000 },  // qq=30.0
    { 0.08566057, 0.06821319, 0.07310505, 0.07153536, 0.05718694, 0.05094781, 0.04607433, 0.09361598 },  // qq=40.0
    { 0.05700755, 0.05700755, 0.07662780, 0.05186295, 0.06557243, 0.04319800, 0.03591205, 0.04604144 },  // qq=55.0
    { 0.06557243, 0.06557243, 0.05340259, 0.05340259, 0.07985046, 0.03406002, 0.03239565, 0.02514798 },  // qq=70.0
    { 0.05340259, 0.05340259, 0.03987715, 0.03987715, 0.03778571, 0.03307973, 0.03091919, 0.03054182 }   // qq=80.0
};

static const double TBL_Kiw[N_QQ][N_MACH] = {
    { 0.01492415, 0.01662666, 0.01283153, 0.01539546, 0.01061867, 0.00997475, 0.05833771, 0.05833771 },  // qq=1.0
    { 0.01492415, 0.01671630, 0.01269821, 0.01356555, 0.01061867, 0.00945958, 0.04328706, 0.04328706 },  // qq=2.0
    { 0.01453865, 0.01677673, 0.01610174, 0.01356555, 0.01219594, 0.00970436, 0.03393368, 0.03393368 },  // qq=3.5
    { 0.01397105, 0.01612699, 0.01539546, 0.01106500, 0.01195495, 0.00970436, 0.02889701, 0.02889701 },  // qq=5.0
    { 0.01689558, 0.01374398, 0.01457056, 0.01376474, 0.01151764, 0.00872263, 0.02452344, 0.02452344 },  // qq=7.0
    { 0.01461677, 0.01419692, 0.01076060, 0.01219594, 0.00853098, 0.00867496, 0.02183558, 0.02183558 },  // qq=10.0
    { 0.01085010, 0.01009673, 0.01008292, 0.00924828, 0.00945958, 0.00780428, 0.00739271, 0.01777616 },  // qq=15.0
    { 0.01008292, 0.01117138, 0.01072261, 0.00819534, 0.00872263, 0.00728945, 0.00801206, 0.01524715 },  // qq=20.0
    { 0.00804218, 0.00965491, 0.00905239, 0.00856606, 0.00731051, 0.00570345, 0.00600646, 0.01162000 },  // qq=30.0
    { 0.00856606, 0.00682132, 0.00731051, 0.00715354, 0.00571869, 0.00509478, 0.00460743, 0.00936160 },  // qq=40.0
    { 0.00570076, 0.00570076, 0.00766278, 0.00518629, 0.00655724, 0.00431980, 0.00359121, 0.00460414 },  // qq=55.0
    { 0.00655724, 0.00655724, 0.00534026, 0.00534026, 0.00798505, 0.00340600, 0.00323957, 0.00251480 },  // qq=70.0
    { 0.00534026, 0.00534026, 0.00398771, 0.00398771, 0.00377857, 0.00330797, 0.00309192, 0.00305418 }   // qq=80.0
};

//--- Bilinear interpolation ---
static double interp2d(const double tbl[N_QQ][N_MACH], double qq, double mach1)
{
    // Clamp to grid bounds
    qq   = std::max(QQ_AXIS[0],   std::min(QQ_AXIS[N_QQ-1],     qq));
    mach1 = std::max(MACH_AXIS[0], std::min(MACH_AXIS[N_MACH-1], mach1));

    // Find qq bracket
    int iq = 0;
    for (int i = 0; i < N_QQ - 1; i++) {
        if (qq >= QQ_AXIS[i] && qq <= QQ_AXIS[i+1]) { iq = i; break; }
        if (i == N_QQ - 2) iq = i;
    }

    // Find Mach bracket
    int im = 0;
    for (int i = 0; i < N_MACH - 1; i++) {
        if (mach1 >= MACH_AXIS[i] && mach1 <= MACH_AXIS[i+1]) { im = i; break; }
        if (i == N_MACH - 2) im = i;
    }

    // Normalized coordinates [0,1]
    double dq = QQ_AXIS[iq+1] - QQ_AXIS[iq];
    double dm = MACH_AXIS[im+1] - MACH_AXIS[im];
    double tq = (dq > 1e-12) ? (qq - QQ_AXIS[iq]) / dq : 0.0;
    double tm = (dm > 1e-12) ? (mach1 - MACH_AXIS[im]) / dm : 0.0;
    tq = std::max(0.0, std::min(1.0, tq));
    tm = std::max(0.0, std::min(1.0, tm));

    // Bilinear interpolation
    double v00 = tbl[iq  ][im  ];
    double v10 = tbl[iq+1][im  ];
    double v01 = tbl[iq  ][im+1];
    double v11 = tbl[iq+1][im+1];

    return (1-tq)*(1-tm)*v00 + tq*(1-tm)*v10 + (1-tq)*tm*v01 + tq*tm*v11;
}

//--- Main API ---
static GainSet get_gains(double qq, double mach1)
{
    GainSet g;
    g.K_p_integ_accI = interp2d(TBL_Kp,  qq, mach1);
    g.K_i_integ_accI = interp2d(TBL_Ki,  qq, mach1);
    g.K_w_pitchI     = interp2d(TBL_Kw,  qq, mach1);
    g.Ki_w_pitchI    = interp2d(TBL_Kiw, qq, mach1);
    return g;
}

//*/
///////////////////////////////////////////////////////
