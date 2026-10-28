/* Shared M4-RAJEH / STM743-M397 guidance implementation. */
#include "gnc_platform.h"

double sim_dbg_ayc_raw = 0.0;
int M_stage(void)
{ 
	
	if (GNC_FORCE_STAGE4)                              return 4;
		   if (flg_stage)									return 6;
	if (t < (t_burn1+0) || (sepflg==0 && guid_flg==0 ))			return 1;

	else if ((q > 0.3 && Vym > 0. &&t<t_stg2) || ( burn_flg2==0&&t<t_stg2 && guid_flg==0 ))			return 2;//v25
//else	if (t < 45) 			return 2;
	else if (q > 0.001 && Vym > 0. && flg_stage == 0)					return 3;
	
	else if ((Vym < 0. && q>1) || flg_stage == 1) {flg_stage = 1;	return 6;}
	else if (Vym < 0. && q > 0.001)			return 5;
	else								    	return 4;//*/
}

/////////////////////////////////////////////////////////////////
//int sign(double sg);
void AlfaObtain() //1300 not used 
{    
	if(Vxy!=0.)
	alfa_xy = Vym / Vxy ;
	if((fabs(alfa_xy))>1.)
	alfa_xy=1.*sign(alfa_xy);

	alfa_xy = asin(alfa_xy) - tetam ;
	if(fabs(alfa_xy) >= 0.35)  alfa_xy = 0.35 * alfa_xy/fabs(alfa_xy) ;
}//End of AlfaObtain

/////////////////////////////////////////////////////////////////////////////
double M_teta_command(void)
{
 double teta1, Teta00,theta_pr=0,dt_r;
	/* MERGE step 9 -- INERT, local kept 2026-08-23. Donor uses kv=2.5
	   unconditionally (0.8 / 4.0 here). Measured BIT-IDENTICAL across the
	   12-case screen, because theta_p saturates: min theta_p is exactly
	   -5.0000 on T11n1, so the limit binds before kv can matter. */
	kv=0.8;
   if(hgv_flg)	kv=4.;

   if (t_tet[1] < .1)	return (Teta0);

   Teta00 = Teta0;
   if (t < t_tet[0])	return (Teta00);
   if (t<55)
   {
      int i;
    
	  if(flg_vc==0 && s_s==2)//ocv20_060425
	  {
	  			if (guid_flg==0 && Teta0>59.) dv_r=Vm_t-VburnOut+30.;
	  		else if (guid_flg==0 && Teta0>56.) dv_r=Vm_t-VburnOut+20.;
		else if (guid_flg==0 && Teta0>54.) dv_r=Vm_t-VburnOut+25.;
		else     if (guid_flg==1 && Teta0<60.)              dv_r=Vm_t-VburnOut+0.;
		else                   dv_r=Vm_t-VburnOut;

		if (Vm_t!=0)	theta_p=((-dv_r/Vm_t)*kv)*r2d;//+dteta_r*kv;//ocv20_060425
		
		if(hgv_flg==0 )
		{
			double coef = 8.*(0.1*Teta0 -5.);
			if (guid_flg==0)		coef =2.*(34.873*Teta0*Teta0 + -1403.8*Teta0 + -41.433) / (Teta0*Teta0*Teta0 + -133.96*Teta0*Teta0 + 4549.1*Teta0 + 277.88)*1.;
			else	coef =1.35*(34.873*Teta0*Teta0 + -1403.8*Teta0 + -41.433) / (Teta0*Teta0*Teta0 + -133.96*Teta0*Teta0 + 4549.1*Teta0 + 277.88)*1.;

			if (theta_p <0)//up
			{

				if (guid_flg==1&& Teta0<59&&Teta0>54.2)	theta_p*=0.518*coef;//0.818*coef;//1//2.18//1.18
				else if (guid_flg==1&& Teta0>59.)	theta_p*=0.5508*coef;//0.8508*coef;//1//2.18//1.18
				else if (guid_flg==0 && Teta0<56.)  theta_p*=0.6500814*coef;//1//2.4  
				else if (guid_flg==0 && Teta0<58.)  theta_p*=0.7600814*coef;//1//2.4 
				else if (guid_flg==0 && Teta0<60.5)  theta_p*=0.5400814*coef;//1//2.4  
				else if (guid_flg==0 && Teta0<65)  theta_p*=0.74814*coef;//1//2.4  //1.14814
				else
				{
					if (traject_up==1)	theta_p*=0.2*coef;//mm=1*coef;
					else   theta_p*=0.72*coef;//mm=1*coef;
					if(guid_flg==0 && Teta0>60)	theta_p *=0.6;//0.6; 
				}
			}
			else //if (theta_p >1.7){} //down
			{
				if (theta_p <0.1 && Teta0>59. && guid_flg==0 )  theta_p*=-3.05*coef;//13
				else 
				{
					if (guid_flg==1)	theta_p*=0.17*coef;//1
					else if (guid_flg==0 && Teta0<61) theta_p*=0.1*coef;//1
					else if (guid_flg==0 && Teta0<59.4) theta_p*=0.1*coef;//1
					if (guid_flg==0)	theta_p*=0.0*coef;//13
				}

			}
		//	else					theta_p*=0.0;
		}
		else
		{    

			if (guid_flg==1&& Teta0>54.2 && theta_p <-0.5)   theta_p*=2.239;//1.5;
			//else if (theta_p <-0.5 && mass_W==1)       theta_p*=0.51;//1.5; 
			else if (theta_p <-0.5 )       theta_p*=2.7;//1.5; 
		//	else if (theta_p>0.0 && mass_W==1)  theta_p*=1.8;//1.8;     
			else if (theta_p>0.0&&hgv_flg2==0)  theta_p*=0.35;//1.8;  
			else if (theta_p>0.0&&hgv_flg2&& Teta0 <65)  theta_p*=0.01;//1.8;     
			else if (theta_p>0.0)  theta_p*=1.03;//1.8;   
			else  theta_p*=1.4;//2.0;

       

		}

		theta_p = limit(theta_p,10.,-15.);
		if(Teta0<64 || hgv_flg) 	{
             
             if (hgv_flg2 && Teta0 <55) 		theta_p = limit(theta_p,5.,-5.);
             else if (hgv_flg2 ) 		theta_p = limit(theta_p,5.,-5.);
			      else if (Teta0 > 51  )     limit(theta_p,10.,-10.);
			        else  theta_p = limit(theta_p,0.,-10.);
		}
		/*Step5: short-range cold class overrides the native trim (which has
		  the wrong sign for an energy excess). Generation (nchoice 3/4,
		  hgv forced 0) never enters this branch.*/
		//if (hgv_flg && hgv_flg2==0 && nchoice==5 && Teta0 < 55. && Teta0 > 52. && dv_r < SC_DV_ON)
	//	{
		//	double ksc = SC_K_A + (Slant_Range - SC_SR_A) * (SC_K_B - SC_K_A) / (SC_SR_B - SC_SR_A);
	//		if (ksc > SC_K_A) ksc = SC_K_A;
	//		if (ksc < SC_K_B) ksc = SC_K_B;
	//		theta_p = ksc * dv_r;
	//		theta_p = limit(theta_p, 0., -SC_LIM);
//		}
		/* DIAGNOSTIC override -- last word on theta_p, after every native
		   scaling and clamp. Off unless the MEX field TpSet is supplied. */
		if (sim_tp_on && GNC_CASE_ACTIVE) theta_p = sim_tp_val;
		/* HB -- NEGATIVE RESULT 2026-08-22. Do NOT override the native trim
		   for hot cases. Reasoning that looked sound and was WRONG: the native
		   law theta_p = (-dv_r/Vm_t)*kv*r2d hands a HOT case (dv_r>0) a NEGATIVE
		   trim (flatten), the [0,-5] clamp forbids positive (loft) trim, and loft
		   sheds range (+6 deg measured at -31 km on this airframe) -- so a
		   dv_r-proportional loft was added above the clamp.
		   MEASURED: it is harmful, and the null case proves why. HB_K = 0.00 --
		   which merely forces theta_p to 0 for dv_r>30 -- is NOT a no-op: it took
		   T11n1 6973 -> 26882 m and T11n4 13163 -> 32448 m, and broke a passing
		   case (T12n0 5.5 -> 3293 m, dv_r 30.88 just over the gate). Adding loft
		   (K = 0.06/0.12/0.20) only partly recovers what removing the native trim
		   costs; every point is worse than leaving the law alone.
		   So the native NEGATIVE trim is LOAD-BEARING for the hot arms -- the
		   opposite of the premise. Whatever it does for them, it is not simply
		   "wrong sign for an energy excess" as it is for the SC short-range cold
		   class above. Understand what it actually buys before touching it. */
		/* RIG (reentry-ignition gate) -- REMOVED 2026-08-22, NON-PHYSICAL.
		   DO NOT RE-ENABLE. It appeared to fix the cold-motor arms (claimed
		   49/49) but the gain was an ENERGY-CONSERVATION VIOLATION, not thrust.
		   THERE IS NO SECOND MOTOR: SUBSYS.CPP:89 sets m_fuel2 = 0.0, so the
		   stage-2 motor carries zero propellant. Audit of T10n2 (RIG on vs off):
		     - sthr_2 (col 62) max 0.00 -- no second-motor thrust, ever
		     - mass identical at every sample (591.5255 -> 97.7452 kg), so no
		       propellant is consumed
		     - body-force sum ~0.28 N (0.003 m/s^2), yet the vehicle showed a
		       SUSTAINED +0.914 m/s^2 along-track acceleration (+89.3 N)
		     - specific mechanical energy rose +508,700 J/kg over the coast
		       (+1610 W/kg); the RIG-off run conserves energy to -0.00066 m/s^2
		   ROOT CAUSE -- separation-pusher latch-up. Commanding ignition in
		   stage 4 latches ign2_cmd; 4 s later the ignition-abort block
		   (t > t_ign2+4, Ab(0,0) < 0.5g -- permanently true in a vacuum coast
		   because there is no motor to sense) resets ign2_cmd/BeltCmd/PushCmd,
		   so the BeltCmd->PushCmd chain re-arms and f_belt (2500 N x 4 for
		   0.04 s = 400 N.s) fires AGAIN. Measured: 81 firings, re-latching every
		   4.076 s across the 330 s coast = 32,400 N.s = +331 m/s of free
		   delta-V. That, not a motor, was the "24 km range gain".
		   The true state without it is 45/49 full / 40/42 in scope; T10n2 and
		   T11n2 remain unsolved and are genuinely propulsion-limited. */

		/* LRG latch: choose the glide altitude from the measured deficit. */
		if (hgv_flg && hgv_flg2 && GNC_CASE_ACTIVE && dv_r < LRG_DV_ON)
		{
			/* LRG_HC applies to the Edit-4 vehicle only.  The 21.5 km retune was
			   forced by dropping DESIGN_MARGIN_T10/T11 to 1.00, and that change
			   regenerated the _TAL tables ONLY -- the stock vehicle still flies the
			   profiles the original 25 km value was fitted against, and lowering it
			   there cost tal_flag=0 T13n2 14547 -> 19297 m for no gain. */
			/* MERGE step 11c: two-tier LRG. The original LRG note recorded that
			   the DEEP group (dv_r -47..-89) wants ~25 km while the mild group
			   wants less; with the donor stage-1 gains T13n2 deepens to dv_r -77
			   and needs the taller glide, while T10n2 at -51 still wants 21.5. */
			if (sim_tal_flag) hc_eff = (dv_r < LRG_DEEP_DV) ? LRG_HC_DEEP : LRG_HC;
			else 
				hc_eff = 24.0e3;
		}
		else
			hc_eff = hc;
		homing_rng_eff = 1.15*(32e-9*hc_eff*hc_eff*hc_eff -112e-5*hc_eff*hc_eff
		                       + 14.8*hc_eff -27e+3) + 1*5e3;
		homing_rng = homing_rng_eff;
		flg_vc=1;
		t_vc=t;//+ 0.5;
		sim_dbg_theta_p = theta_p;	sim_dbg_dvr = dv_r;/*Step2 telemetry only*/
      }

#if GNC_IS_SIM
	 if(m_flag<2)	theta_p=0;//4sim
#endif
	
	 theta_pr =theta_p;
	 
	  double t_slop;
    if (ign2_stg!=2) t_slop=(sim_tal_flag ? (TAL_TSLOP) : (TAL_TSLOP0));
    else if(guid_flg==0 && Teta0>62 ) t_slop=5.5;
    else t_slop=10.5;
   
     if( (t - t_vc) <  t_slop)	theta_pr =1.*( theta_pr*(t-t_vc)/  t_slop);
	
	if (flg_vc)
		Teta00 += theta_pr;

      for (i = 0; t > t_tet[i]; i++);
      i -= 1;
      if (t == t_tet[i])	return (Teta00 + teta_c[i]);//+theta_pr;
	  
      double dx = t_tet[i + 1] - t_tet[i];
      double dy = teta_c[i + 1] - teta_c[i];
      double dt2 = t - t_tet[i];
      return (Teta00 + teta_c[i] + dt2 * dy / dx);
   }
   return 0;
}//End of M_teta_command

////////////////////////////////
#if GNC_IS_SIM
double M_teta_command11(void)
{
 if (t_tet[1] < .1)
      return (Teta0);

   double teta1, Teta00, dt2;
   /*
    if(t<10.)
    {
    teta1=(Teta_launch*r2d-Teta0)*sin(((10.-t)/10.)*(M_PI/2.))+Teta0;//-.075*t*t+.75*t;
    Teta00=teta1;
    }
    else// */
   Teta00 = Teta0;

   if (t < 1.)
      return (Teta00);

   if (t <= 68.) {
      int i;
      for (i = 0; t > t_tet[i]; i++);
      i -= 1;
      if (t == t_tet[i])
         return (Teta00 + teta_c[i]);
      double dx = t_tet[i + 1] - t_tet[i];
      double dy = teta_c[i + 1] - teta_c[i];
      double dt2 = t - t_tet[i];
      return (Teta00 + teta_c[i] + dt2 * dy / dx);
   }
   return 0;
}
#endif//End of M_teta_command

////////////////////////////////

////////////////////////////////
#if GNC_IS_SIM
void Gama_correction(void) {

   static double vm_t = vm;
   double vm_t_2,vm_dot;

   vm_t_2 = vmf[0] * vmf[0] + vmf[1] * vmf[1] + vmf[2] * vmf[2];
   vm_dot = (sqrt(vm_t_2) - vm_t) * 100.;
   vm_t = sqrt(vm_t_2);
   double x0, y0, vm0, vmxy;

   if (s_s > 1 && vm_dot < 0.) {

      int j;
      for (j = 0; t > t_pos[j]; j++);
      j -= 1;
      double dx1 = t_pos[j + 1] - t_pos[j];
      double dy1 = x_pos[j + 1] - x_pos[j];
      double dy2 = y_pos[j + 1] - y_pos[j];
      double dy3 = v_pos[j + 1] - v_pos[j];
      double dt1 = t - t_pos[j];

      x0 = x_pos[j] + dt1 * dy1 / dx1;
      y0 = y_pos[j] + dt1 * dy2 / dx1;
      vm0 = v_pos[j] + dt1 * dy3 / dx1;

      if (t == t_pos[j]) {
         x0 = x_pos[j];
         y0 = y_pos[j];
         vm0 = v_pos[j];
      }

      L_x = Lx_gui[0];
      L_y = Ly_gui[0];
      L_v = Lv_gui[0];
      L_gama = Lgama_gui[0];

      vmxy = sqrt(vmf[0] * vmf[0] + vmf[1] * vmf[1]);
      delx = rmf[0] - x0;
      dely = rmf[1] - y0;
      delv = vmxy - vm0;
      Gama_corec = -(L_v * delv + L_x * delx + L_y * dely) / L_gama;

      if (Gama_corec > .07)
         Gama_corec = .07;
      if (Gama_corec < -.07)
         Gama_corec = -.07;

      if (t < t_ramp_gama + 3. && t >= t_ramp_gama)
         Gama_corec = (Gama_corec / 3.) * (t - t_ramp_gama);

   }
   else {
      Gama_corec = 0.;
      t_ramp_gama = t;
   }

   // Gama_corec = 0.;

}
#endif



/////////////////////////////////////////////////////////////////////////////
double M_psi_command(void)
{
 double Psi00,dt_r,Psi0=0.*GNC_AIMING_ANGLE;
 
   double kvz=1.*0.15,dvz_r=0;

   if (t_psi[1] < .1)	return (Psi0);

   Psi00 = Psi0;
   if (t < t_psi[0])	return (Psi00);
   if (t<t_stg2)
   {
      int i;
	  
	  if (s_s==2 )//&& psip_flg2<300)
	  {
		if(psip_flg==0)	
		{
			t_psi0=t;
			psi_0=psim*r2d;
			
		}
		
		dvz_r=Vzm-vzm_c;
		psi_p=((-dvz_r/(Vzm))*kvz)*r2d;
#if GNC_IS_SIM
		if(m_flag<2)	psi_p=0;
#endif
		
		//if (vxzm_c!=0)	err_vxz = (Vxz - vxzm_c) / vxzm_c; //
		if (vzm_c!=0)	err_vxz = (Vzm - vzm_c) / vzm_c; //
		else 			err_vxz=0;
		if (fabs(err_vxz) < 0.002)	err_vxz = 0.;
		if (fabs(err_vxz) < 0.03)	
		{
			
			//psip_flg2++;
			//if(psip_flg2>=300)err_vxz = 0.;
		}else psip_flg2=0;
		
	  if (err_vxz >= 0)
         Aim_coms = 0.1 * (err_vxz * 100.);
      else
         Aim_coms = 0.1 * (err_vxz * 100.);
      // Aim_coms = -0.15 * (err_vxz * 100.);
#if GNC_IS_SIM
      if (m_flag <2)	Aim_coms = 0.;
#endif

	  
	 
	  // psi_p=1*(GNC_AIMING_ANGLE * r2d - 1. * Aim_coms);
	   psi_p= -1. * Aim_coms;

	   double t_sps;

	   if (guid_flg==0 && Teta0>59.) t_sps=20.;
	  else t_sps=10.;
	 
	   if((t - t_psi0) < t_sps)	psi_p *=(t-t_psi0)/t_sps;
		psi_p = limit(psi_p,15.,-15.);
		//psi_p *=ramp(t_psi0, t_psi0 + 1, t, 1)+psi_0*ramp(t_psi0, t_psi0 + 1, t, -1);

		//if(psip_flg==0)	
			//printf("\n t : %.2f \t vxzm_c = %.2f \t Aim_coms = %.2f \t err_vxz = %.3f  \t psi_p = %.5f  ",t,vzm_c,Aim_coms,err_vxz*100,psi_p);
		psip_flg=1;
		

	  }
	  
	  if(psip_flg)	Psi00 += psi_p;
	  //else 			t_psip=t;//*/

      for (i = 0; t > t_psi[i]; i++);
      i -= 1;
      if (t == t_psi[i])	return (Psi00 + psi_c[i]);//+theta_pr;
	  
      double dx = t_psi[i + 1] - t_psi[i];
      double dy = psi_c[i + 1] - psi_c[i];

      double dt2 = t - t_psi[i];
			  
      return (Psi00 + psi_c[i] + dt2 * dy / dx);//-err_vxz*100;
   }
   return 0;
}//End of M_psi_command
/////////////////////////////////////////////////////////////////////////////
//for  diving of hgv
void M_acc_pitch_command()
{
	double dx_tr, dy_tr, rxy2,Vnd;
	double qe_dot, Pnbias, Pg;

	dx_tr = rtaz(0,0) ;//Xtrgt - Xm ;
	dy_tr = rtaz(2,0);//Ytrgt - Ym ;

	if(fabs(dx_tr) <= 1.0)   dx_tr = 1.0;
	rxy2 = dx_tr*dx_tr + dy_tr*dy_tr ;
	if(rxy2 < 10000.)        rxy2 = 10000.;

	//if((dx_tr<0.0) || (dy_tr>=0.0))
	if(vmaz(0,0)<0.0)
	{
		ac_pitch = 0.0;
		return;
	}
	//qe_dot = (dy_tr*Vxm - dx_tr*Vym) / rxy2 ;
	qe_dot = -(dy_tr*vmaz(0,0) - dx_tr*vmaz(2,0)) / rxy2 ;
	Vnd=sqrt(vmaz(0,0)*vmaz(0,0)+vmaz(2,0)*vmaz(2,0));

	Pg =0.83;
	Pnbias =cos(tetam);

	ac_pitch =1.84*(qe_dot*Pg*Vnd  + Pnbias) ;

    acc_limit = q / 0.15;

   if(fabs(ac_pitch) > acc_limit)
		ac_pitch = acc_limit * ac_pitch/fabs(ac_pitch) ;
  // ac_pitch = limit(ac_pitch, 15., -15.);
   ac_pitch = limit(ac_pitch, 35., -35.);
  // ac_pitch = limit(ac_pitch, 15., -15.);

	return;

}//End of M_acc_pitch_command
//------------------------------------------------------------------------

//------------------------------------------------------------------------
void LOS_estimate2()
{
  double dxi,dyi;//,dzi,dVxi,dVyi,dVzi;

  //if(hgv_flg) Npn=2.5;
    if(guid_flg==0)
        { 
       if (flg_q == 1 ) Npn=4.5;
        else Npn=3.5;
            tau_pn1=15;
             }

	dxi = Xtrgt - Xm ;//rtaz(0,0);//Xtrgt - Xm ;//-rmf[0]+rtf[0];//
	dyi = Ytrgt - Ym ;//-rtaz(2,0);//Ytrgt - Ym ;//-rmf[1]+rtf[1];//

	sim_dbg_guard = 0;/*Step2 telemetry only*/

	/* Disengage only when the planar overflight test agrees with the range
	   actually OPENING (d(range)/dt >= 0  <=>  dxi*Vxm+dyi*Vym <= 0).
	   The bare planar test fires mid-dive while still closing (measured:
	   T16n2 at h=36 km, T15n5 at h=27.7 km with 27.7 km slant to fly). */
	if (((dxi < 0.0) || (dyi > 0.0)) && ((dxi*Vxm + dyi*Vym) <= 0.0) && (Hme < GUARD_ENDGAME_H))//if(vmaz(0,0)<0.0)
	{
		sim_dbg_guard = 1;/*Step2 telemetry only*/
		ac_pitch = 0.0;		return;
	}

	double rxy=sqrt(dxi*dxi+dyi*dyi);
	if(rxy< 500.)   rxy = 500.;
	//if(rxy< 10000.)   rxy = 10000.;//1300b_ymn
	if(fabs(dxi) < 1.0)   dxi = 1.0;
	//if(fabs(vmaz(0,0)) <= 0.01)   vmaz(0,0) = 0.01;

	double teta_l=atan2(dyi,dxi);
	double lambda=teta_l;
	double gama=atan2(Vym,Vxm);
	
	/* MERGE step 8 -- REJECTED 2026-08-23. Donor PN set (Npn 6.5 -> 2.5,
	   tau_pn1 15 -> 25, plus tau -> 5 above Teta0 59) erodes terminal margin
	   badly: screen capped-sum 286.6 -> 487.7 with the n1 arms at 65-69 m
	   against a 100 m gate (21-30 m locally). The two constants are a MATCHED
	   PAIR -- taking tau=25 while keeping Npn=6.5 is worse still: 3 failures
	   (T10n1 138.0, T13n1 1677.2). Local set kept. */
	if (guid_flg==1 )
	{
			  Npn=6.5;
			   tau_pn1=15;
	}
	if(guid_flg==1&&Teta0>53&&hgv_flg2&&sim_tal_flag==0) 
	{
	Npn=3.5;
tau_pn1=25;
}
	//if ( guid_flg==1 && Teta0>59) tau_pn1=25;

	if(kkk1==0)
	{
	 kkk1=1.;
	 t_old1=t;
	 Bref1=(1.-Npn)*impact_angle1*d2r-gama+Npn*lambda;
	}
  else
	{
		 E_bias=Bref1-b_int1;
		 b1=E_bias/tau_pn1;
	
		if ( guid_flg==0 )	 
		 {
			  acc_limit = q / 0.9800958 ;
		 if(q <22 && flg_q == 1)acc_limit =q / 1.09800958 ;
		 if ((q <50 && q > 22&&flg_q == 1))	  acc_limit =q /1.5;//3.;
		 }
		if ( guid_flg==1 ){acc_limit =1.5309*apc_limit;}
		/*Step8/R4: homing command limit sits ~4x below the measured achievable
		  envelope (a_max ~ 4q vs limit ~ q, slope method on col 20). Raise x2
		  for terminal agility (maneuvering-target requirement); still inside
		  the measured envelope. Fly-mode gated: generation byte-identical.*/
		/* TAL_ACCX -- NEGATIVE RESULT 2026-08-23.  The x2 here was justified by an
		   "a_max ~ 4q" estimate later measured to be 3-4x optimistic, so it looked
		   like the terminal over-command behind the flg==1 departures.  It is not:
		   tightening it makes everything WORSE (T11n1 6973 -> 12051 -> 17332 at
		   x1.0/x0.5, T11n4 13163 -> 17591 -> 23256), and at x0.5 the HEALTHY T11n0
		   departs outright (13.6 -> 802.7 m, maxAlf6 135).  The x2 is load-bearing;
		   the departures are insensitive to the cap.  Left at stock 2.0. */
		if (GNC_CASE_ACTIVE && hgv_flg) acc_limit *= (sim_tal_flag ? TAL_ACCX : 2.0);
		if( fabs(b1)>(acc_limit*mag_g/Vm_t))	b1=b1/fabs(b1)*acc_limit*mag_g/Vm_t;
	 
		b_int1=b_int1+b1*(t-t_old1);
//	 b_int1+=b1*0.01;
		t_old1=t;
	}
	
	double lambda_dot=(-Vm_t*sin(gama-lambda))/rxy;
	double gama_dot=Npn*lambda_dot;
	double ay_com=gama_dot*Vm_t+1.*b1*Vm_t;//am
	mat3x1 Gb(0),g_b(0);
	Gb = mib * GNC_GI_VEC;
  //g_b=C_b_n*g_n;
	// ac_pitch=(ay_com+g_b(2,0))/mag_g;//m351
	//ac_pitch=(ay_com+9.81*cos(tetam))/mag_g;//m302
	
   ac_pitch = (ay_com - 1. * (cos(phim) * Gb(1,0) - sin(phim) * Gb(2,0))) / mag_g;
//
/* MERGE step 2 (2026-08-23): this block is DISABLED here -- a real /* on its
   own line -- but ACTIVE in the donor PILOT/GUIDANCE pair, where the same line
   reads ///* (a line comment, so the /* never opens a block). Donor
   coefficients are 1.5 and 0.258 against 1.25 and 0.58 here. Enabled below
   with the donor values; this comment keeps the original for reference:
    if (flg_q == 0 )
	{
		if (q < 30.)					acc_limit *= 1.25;
		else if (q < 60. && q >= 30.)	acc_limit *= 0.58*(q / 60.);
    }
*/
    if (flg_q == 0&& hgv_flg2==1&&sim_tal_flag==0)
	{
		if (q < 30.)					acc_limit *= 1.5;
		else if (q < 60. && q >= 30.)	acc_limit *= 0.258*(q / 60.);
    }
//if ((q <10&&flg_q == 1))	  acc_limit *=0.8;//3.;
   ac_pitch = limit(ac_pitch, acc_limit, -acc_limit);
   ac_pitch = limit(ac_pitch, 35., -35.);
     // ac_pitch = limit(ac_pitch, 15., -15.);

	return;
}


/////////////////////////////////////////////////////////////////////////////
//m273x_TacNed_160823//m325xcv11
void M_acc_yaw_command2()
{
#ifndef GNC_S6_AYC_LIMIT_SCALE
#define GNC_S6_AYC_LIMIT_SCALE 2.0
#endif
	double dx_tr, dz_tr, rxz2,Vne;
	double qe_dot2;

	dx_tr = rtaz(0,0);//Xtrgt - Xm ;
	dz_tr = rtaz(1,0);//Ztrgt - Zm ;

	/*Seeker LOS-noise injection (lateral): a 1-sigma angular error [mrad]
	  displaces the measured cross-position by range*angle. OFF (==0) ->
	  PRNG untouched -> byte-identical. Fly-mode, stage-6 only.*/
	if (GNC_CASE_ACTIVE && hgv_flg && s_s==6 && sim_skr_noise != 0.)
	{
		double rr = sqrt(dx_tr*dx_tr + dz_tr*dz_tr);
		dz_tr += rr * (sim_skr_noise*1e-3) * sim_skr_sample(t);
	}

	if(fabs(dx_tr) <= 1.0)   dx_tr = 1.0;
	//  if(fabs(dz_tr) <= 10.0)   dz_tr = 10.0;

	rxz2 = dx_tr*dx_tr + dz_tr*dz_tr ;
	if(rxz2 < 10000.)        rxz2 = 10000.;

	//if(dx_tr<0.0
	if(vmaz(0,0)<0.0)// && ctrlNo!=5)
	{
		ayc = 0.0;
		return;
	}
  //  if(hgv_flg==0 )ayc_limit = q / 3.5;

	Vne=sqrt(vmaz(0,0)*vmaz(0,0)+vmaz(1,0)*vmaz(1,0));
	//qe_dot2 = (dz_tr*Vxm - dx_tr*Vzm) / rxz2 ;
	qe_dot2 = (dz_tr*vmaz(0,0) - dx_tr*vmaz(1,0)) / rxz2 ;

	ayc =0.10025*qe_dot2*Vne;

	if( s_s>2)
	{
		if (guid_flg==1) 		ayc =0.1091034*qe_dot2*Vne;
		else 	ayc =0.100034*qe_dot2*Vne;

	}//*/

	/*LATH — lateral homing (2026-08-18): the stock lateral PN is sub-critical
	  (effective N'~1; measured: gain x2 moved a 30 m/s crossing miss by 1 m,
	  consistent with PN theory below N'~2-3). Scale to a homing-grade N' in
	  stage 6 and open the caps with it (a_max~4q measured). Fly-mode gated;
	  rides the existing 3-s k_5 entry ramp — no new transition.*/
	if (GNC_CASE_ACTIVE && hgv_flg && s_s==6)
	{
		ayc *= LATH_SCALE;
		if(fabs(ayc) > GNC_S6_AYC_LIMIT_SCALE*ayc_limit)
			ayc = GNC_S6_AYC_LIMIT_SCALE*ayc_limit * ayc/fabs(ayc) ;
		ayc = limit(ayc, 14., -14.);
	}
	else
	{
	if(fabs(ayc) > ayc_limit)     // ayc_limit = q/4.;
		ayc = ayc_limit * ayc/fabs(ayc) ;
	ayc = limit(ayc, 7., -7.);
	}


	return;

}//End of M_acc_pitch_command
/////////////////////////////////////////////////////////////////////////////

void M_acc_yaw_command_0()
{
   const double Q_GAIN_START = 5.0;
   const double Q_GAIN_END = 55.0;
   const double K_YAW_MIN = 0.0008;
   const double K_YAW_MAX = 0.0017604;
   const double K_VZ_LOW_Q = 35.0;
   const double K_VZ_HIGH_Q = 7.5;
   const double GUIDANCE_GAIN = 2.0 * 0.29;
   double k_guidance_yaw, k_vz, ac_yaw_limit;

   if (q < Q_GAIN_START) k_guidance_yaw = K_YAW_MIN;
   else if (q < Q_GAIN_END)
      k_guidance_yaw = K_YAW_MIN +
         (K_YAW_MAX - K_YAW_MIN) * (q - Q_GAIN_START) /
         (Q_GAIN_END - Q_GAIN_START);
   else k_guidance_yaw = K_YAW_MAX;

   if (sim_tal_flag)
   {
      /* Smooth the old k_vz step 28 -> 7.5 around q=45. */
      double q_blend = (q - 35.0) / 20.0;
      if (q_blend < 0.0) q_blend = 0.0;
      if (q_blend > 1.0) q_blend = 1.0;
      q_blend = q_blend * q_blend * (3.0 - 2.0 * q_blend);
      k_vz = K_VZ_LOW_Q + (K_VZ_HIGH_Q - K_VZ_LOW_Q) * q_blend;
   }
   else
      k_vz = (q > 45.0) ? K_VZ_HIGH_Q : K_VZ_LOW_Q;


   if (t < t_z[0] || t_z[0] < 0.1)
   {
      zm_c = 0.;
      vzm_c = 0.;
   }

   else if (s_s<3.)////v15
   {
      int i;

      for (i = 0; t > t_z[i]; i++);
      i -= 1;
      if (t == t_z[i]) {
         zm_c = z_c[i];
         vzm_c = vz_c[i];
      }
      else {
         double dx = t_z[i + 1] - t_z[i];
         double dy = z_c[i + 1] - z_c[i];
         double dt = t - t_z[i];
         zm_c = z_c[i] + dt * dy / dx;
         double dx_ = t_z[i + 1] - t_z[i];
         double dy_ = vz_c[i + 1] - vz_c[i];
         vzm_c = vz_c[i] + dt * dy_ / dx_;
      }
   }
   z_err =1.0*(zm_c - Zm);
   vz_err = 2.1*k_vz * (vzm_c - Vzm);
   sim_dbg_ayc_raw = GUIDANCE_GAIN * k_guidance_yaw * (z_err + vz_err);

   /* Smooth command shaper: nearly linear below the knee and asymptotic at
      1.95 g. Unlike a hard limit it creates no flat top or slope break. */
   if (sim_tal_flag)
   {
      const double AYC_SHAPE = 1.70;
      const double AYC_SHAPE_ORDER = 8.0;
      const double ratio = fabs(sim_dbg_ayc_raw) / AYC_SHAPE;
      ayc = sim_dbg_ayc_raw /
         pow(1.0 + pow(ratio, AYC_SHAPE_ORDER), 1.0 / AYC_SHAPE_ORDER);
   }
   else
      ayc = sim_dbg_ayc_raw;

 // if (hgv_flg==1) ac_yaw_limit = q / 0.3;//0.8;//30.;//4;//30;//rev
  //else
    ac_yaw_limit = q / 0.8;
    if (ac_yaw_limit > 2.3) ac_yaw_limit = 2.3;
      //if (ac_yaw_limit > 3.5)	ac_yaw_limit = 3.5;
   /*Yaw-lever probes (2026-08-18, moving-target campaign) — ALL NEGATIVE, do
     not retry without new evidence: dead-path qe_dot2 cap x2 (0-2%), dead-path
     PN gain x2 (bit-identical: path inert in the tested regime), live gain x2
     and live cap 2.3->4.6 here (moving-target miss unchanged; ayc never
     reaches even 2.3 — max 0.8 measured at 30 m/s crossing). The cross-range
     residual is ARCHITECTURAL: this law holds cross-track to the reference
     frame; there is no lateral target-homing term. Fix = a lateral homing law
     in the homing phase (design task, own gate battery).*/

	ayc = limit(ayc, ac_yaw_limit, -ac_yaw_limit);
	ayc = limit(ayc, 2.3, -2.3);
	
}//End of M_acc_yaw_command
//-----------------------------------------------------

double control_altitude(double altcom)
{
	double AGRAV=mag_g;//9.80675445;		 //standard value of gravity acceleration - m/s^2 
	double SMALL=1e-7;			 	//small real number
	double ancomx,altrate;

	//localizing module-variables
	//input data
	double gainalt=0.05,gainaltrate=1.5*0.05;
	double grav=cos(tetam);
	double altc=Hme;
	double phibdcx=1*phim;
	//altitude rate feedback
	altrate=Vym;
	//acceleration command
	double eh=gainalt*(altcom-altc);
	if(phibdcx==0) phibdcx=SMALL;
	//ancomx=(1./cos(phibdcx))*(gainaltrate*(eh-altrate)+grav)/AGRAV;
	/* MERGE step 4 -- REJECTED 2026-08-23. Donor control_altitude gains
	   (1.11 -> 2.585957585958, 1.13 -> 2.53) are a WASH: screen capped-sum
	   283.4 -> 288.1, tal_flag=0 T13n2 14001.0 -> 13991.7 (0.07%), and
	   tal_flag=0 T13n5 5.2 -> 8.6. Kept local values. */
	if (q<10&& flg_q)ancomx=1.11*1.126*(gainaltrate*(eh-altrate)+grav)/AGRAV;
	else ancomx=1.13*1.180*(gainaltrate*(eh-altrate)+grav)/AGRAV;
        

       // if (q > 50.)
          {
           acc_limit =1.225309*apc_limit;

       if (flg_q == 0 )//&& hgv_flg==0) 
	   {
	   	if (q < 10.)					acc_limit *= 1.4725;

	else if (q < 30.)					acc_limit *= 2.725;
		else if (q < 60. && q >= 30.)	acc_limit *= 0.58*(q / 60.);
        }//*
          ancomx = limit(ancomx, acc_limit, -acc_limit);
  }
//*/
	ancomx = limit(ancomx, 17.5, -17.5)*1.1;//ocv20_060425
//	ancomx = limit(ancomx, 10., -10.);
	
	return ancomx;
}
//-----------------------------------------------------


//////////////321
double ramp(double t0,double tf,double t,double ramp)
//Generate a ramp signal from 't0' to 'tf' according to 't' and 'ramp'
//
//     __1__              |            __1__
//        \               |             /
//			   \             |           /
//     ramp<0 \           |         / ramp>0
//			       \         |       /
//			        --0---   |  --0--
{
	ramp=LIMIT(ramp,1);
	ramp=ramp*(t-t0)/(tf-t0)-floor(ramp-.5)/2.;
	return SAT(ramp,0,1);
}//End of ramp

/* Platform-specific NED_guide behavior. */
void NED_guide()
{
  double L2,l2,h2;
  if((GPS_ACTIVE!=0) && (ConvFlag==1))
  {
    L2=Ffi; l2=Flam; h2=Fh;
    vmn(0,0)=FvN; vmn(1,0)=FvE; vmn(2,0)=FvD;
    NED_Angles = Quat2Euler_n(q_b_n);
  }
  else
  {
    L2=Ifi; l2=Ilam; h2=Ih;
    vmn(0,0)=IvN; vmn(1,0)=IvE; vmn(2,0)=IvD;
    NED_Angles = Quat2Euler_n(q_b_n_ins);
  }

  phin=NED_Angles(0,0);
  tetan=NED_Angles(1,0);
  psin=NED_Angles(2,0);
  rmn(0,0)=L2; rmn(1,0)=l2; rmn(2,0)=h2;

  if(GNC_CASE_ACTIVE && hgv_flg && sim_tgt_spd != 0.)
  {
    if(sim_tgt_tprev < 0.) sim_tgt_tprev=t;
    double dtg=t-sim_tgt_tprev;
    if(dtg > 0.)
    {
      double hdg=(sim_tgt_hdg+sim_tgt_wva*sin(2.*GNC_PI*t/sim_tgt_wvt))*d2r;
      double Re_loc=6371000.+rtn(2,0);
      rtn(0,0)+=sim_tgt_spd*cos(hdg)*dtg/Re_loc;
      rtn(1,0)+=sim_tgt_spd*sin(hdg)*dtg/(Re_loc*cos(rtn(0,0)));
      mat3x1 rte_m=nav2cart(rtn);
      mat3x1 rt_l=C_e_l*(rte_m-R0_E);
      Xtrgt=rt_l(0,0);
      Ytrgt=rt_l(1,0);
      Ztrgt=rt_l(2,0);
      GNC_MIRROR_MOVING_TARGET();
    }
    sim_tgt_tprev=t;
  }

  rme=nav2cart(rmn);
  rte=nav2cart(rtn);
  rme_rte=rte-rme;
  azimuth(rmn(2,0),rmn(0,0),rmn(1,0),rtn(2,0),rtn(0,0),rtn(1,0),&Az_t,&dRange);
  MATRIXe2n(rmn(0,0),rmn(1,0));
  MATRIXAz2n(Az_t);
  rmn_rtn=me2n*rme_rte;
  rtaz=mAz2n*rmn_rtn;
  vmaz=mAz2n*vmn;

  if(GNC_SINGLE_TARGET_RETURN) return;

  if(GNC_MULTI_TARGET_STAGE6)
  {
    tgo=fabs(Slant_Range/Vm_t);
    int cond1=((Vym < 0. && tgt_flg == 0) && hgv_flg==0);
    int cond2=((hgv_flg && homing_phase==0) &&
               (tgt_flg==0 || (tgo>120. && tgt_flg && Vm_t<3.*340.)));
    if(cond1 || cond2)
    {
      int TrgtCntN=Check_targets();
      if(tgt_flg>0 && TrgtCntN!=TrgtCnt)
      {
        TrgtCnt=TrgtCntN;
        rtn(0,0)=Lattf[TrgtCnt]; rtn(1,0)=Longtf[TrgtCnt]; rtn(2,0)=Htf[TrgtCnt];
        rte=nav2cart(rtn);
        mat3x1 rt_l=C_e_l*(rte-R0_E);
        Xtrgt=rt_l(0,0);
        Ytrgt=rt_l(1,0);
        Ztrgt=rt_l(2,0);
      }
    }
  }
}
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//ocv20_060425
void Init_guid()
{
	//--------------------------------
	//if (MinMaxRng_flg==0)
	{
		if (GNC_RANGE<(MaxR_stg1+(hgv_rng*hgv_flg2)))
		{
			ign2_stg=6;//6;
			q6=1.;
			if(reentry_ign==0)	
			{
				ign2_stg=0;
				impact_angle1=-65;
			}
			guid_flg=1;

			Del_tm=22;
		//	if(gamac_flg)
		//	{
		//		Del_tm=d_j=20;//12.;	//20
		//		//gamac_flg=1;
		//		//q6=1.;
		//	}
			//impact_angle1 = -65;
		}
		else
		{
			ign2_stg=2;
			q6=1.;
			guid_flg=0;
			//if(Range>17e5)	impact_angle1 = -75;
		}
		if(hgv_flg)	q6=1;
		//printf("guid_flg = %d \t noise=%d\n",guid_flg,noise);
	}//*/
//--------------------------------
}
//////////////////////////////////////////////////////////////

/* Platform-specific Guidance behavior. */
void Guidance()
{
	//static float t_dive,t_yaw;
	double  k_5;

//	Init_guid();
	if(t<=t_ctrl )  //return;
	{
	//----------
		Init_guid();//ocv20_060425
	//-----------
		return;
	}
   s_s = M_stage();
  // s_s = stage;
	 AlfaObtain();//1300 not used 
	NED_guide();
	
	if((t > 3.) && (Ab(0,0)> (3.*9.81) ) && (start_flg == 0)) 
	{
      flgs_5 ++;
      if (flgs_5 == 100) 
	  {
        start_flg = 1;
      }
    }
	else  flgs_5 = 0;

	if((t > 18.0) && (Ab(0,0)< (0.*9.81) ) && (burn_flg == 0)) //ocv14
	{
      flgm_5 ++;
      if (flgm_5 == 5) 
	  {
        t_burn1 = t;
		//if (gamac_flg)	t_burn1=(t_burn + tberr);
        burn_flg = 1;
        
      }
    }
	else  flgm_5 = 0;
	
	if ((t > t_ign2 + 7.5) && (sepflg) && ( Ab(0,0) < 1*9.81 ) && (burn_flg2 == 0)) 
	{
	  flgb_5 ++;
      if (flgb_5 == 5) 
	  {
        burn_flg2 = 1;
		t_burn2 = t;
      }
    }
    else flgb_5 = 0;
	

//--------------------------------
	GNC_GUIDANCE_PLATFORM_TICK();

	if(s_s < 3)
	{
		
	 tetac = M_teta_command() * d2r; // +3.5*d2r;
	 psic = M_psi_command() * d2r; // +3.5*d2r;

	 if (t < 1.)			du_pitch = 0.0;
	 else if (t < 10.0)		du_pitch = ((tetac - tetam)*(t - 1.0)) /9.0;
	 else					du_pitch = tetac - tetam;

	// if( (t - t_vc) < 0.5 && flg_vc)	du_pitch =1.*( du_pitch*(t-t_vc)/ 0.5);

	  M_acc_yaw_command_0();
	  /* TAL_AYCR -- ramp the yaw command in over the start of stage 2. t_vc is
	     latched at stage-2 entry by the theta_p block in M_teta_command, so it
	     is exactly the entry time. Edit-4 only. */
	  if ((TAL_AYCR) > 0. && s_s == 2 && flg_vc && sim_tal_flag)
	  {
	     double r_ayc = (t - t_vc) / (TAL_AYCR);
	     if (r_ayc < 0.) r_ayc = 0.;  if (r_ayc > 1.) r_ayc = 1.;
	     ayc *= r_ayc;
	  }
	  //if(s_s>1)	M_acc_yaw_command2();
      du_yaw = -ayc + Azb_filter;
	  
	 if (psic_flg==1 ||(psic_flg==2 && s_s==2)) 
	 {
		 du_yaw = psic - psim;
	 }
	 
	  t_2=t;
	  
	}
	else if (s_s == 3 || s_s == 5) 
	{
	  du_pitch = -alfa_g ;
	  du_yaw = -beta_g ;
	  if( (t - t_2) <= 2 && s_s==3)	
	  {
	  	if (guid_flg==0)
	  	{
	  		du_pitch =1.*( du_pitch*(t-t_2)/ 2);
		  du_yaw =1.*( du_yaw*(t-t_2)/ 2);
	  	}
		  else{
              du_pitch =1.*( du_pitch*(t-t_2)/ 2);
		  du_yaw =1.*( du_yaw*(t-t_2)/ 2);


		  }
	  }
	}
	else if (s_s == 6) 
	{
	  /* MERGE step 3 (2026-08-23): donor HGV stage-6 state. These MUST be reset
	     at every stage-6 entry -- they are function statics and the MEX keeps
	     them across runs, so without the reset case N inherits case N-1. */
	  static int hgv_long_stage6_entry=0;
	  static int hgv_terminal_energy_set=0;
	  static int hgv_low_energy_terminal=0;
	  if (ss_6 == 0)
	  {
		  //printf("\n ss_6 = %d  \t noise=%d\n",ss_6,noise);
		 {
			double entry_x=rtaz(0,0), entry_y=rtaz(1,0);
			hgv_long_stage6_entry=(sqrt(entry_x*entry_x+entry_y*entry_y)>103.e3);
		 }
		 hgv_terminal_energy_set=0;
		 hgv_low_energy_terminal=0;
         t_dive = t;
		 azimuth(rmn(2,0),rmn(0,0),rmn(1,0),Hl,Ll,Lonl,&Az_6,&Rng_6);
         ss_6 = 1;
		 tetam_6=tetam*r2d;

      }
	  if (t < t_dive + 3.)	k_5 = (t - t_dive) / 3.;
	  else					k_5 = 1.;
      
	   M_acc_yaw_command2();
	  // if(stg6_guid ||hgv_flg)	
		  // M_acc_pitch_command();
	  // else
	   LOS_estimate2();//Imp_Angle2() ;
	   //LOS_estimate22();//Imp_Angle2() ;
	   
 	if( hgv_flg || hgv_flg2 )
	{
		  cond1=0,cond2=0,cond3=0,cond4=0;
		  azimuth(rmn(2,0),rmn(0,0),rmn(1,0),Hl,Ll,Lonl,&Az_0,&Rng_0);

		  /* MERGE step 5 -- REJECTED 2026-08-23: donor hgv_flg2==1 homing gates
		     (cond1 tgo 35->30, cond2 handover 0.62->0.65, tgo 40.25->40) measured
		     WORSE: screen capped-sum 283.4 -> 286.9, no failure moved. cond3 is
		     identical either way (1.5*340 = 1.7*300 = 510). Local values kept. */
		  cond1=1*((fabs(Rng_0-Rng_6)>20e3)  && tgo<=35.);
		  cond2=1*((Slant_Range<1.*0.62*homing_rng && tgo<=1.15*35.) );
	if	(!hgv_flg2)  cond3=((Vm_t<1.8*340) );//&& tgt_flg);//*/
	else            cond3=((Vm_t<1.5*340) );//&& tgt_flg);//*/

		  if( (cond1 || cond2 || cond3) && homing_phase==0 )
		  {
			homing_phase = 1;
			//printf("t : %.2f \t noise : %d \t cond1 : %d \t cond2 : %d \t cond3 : %d \t tgo : %.2f \n",t,noise,cond1,cond2,cond3,tgo);
		  }
		
		   //M_acc_yaw_command22();
		  /* MERGE step 3: latch the terminal energy state once, on first entry
		     inside 50 km. Selects the glide's target descent angle below. */
		  double hgv_terminal_range=sqrt(rtaz(0,0)*rtaz(0,0)+rtaz(1,0)*rtaz(1,0));
		  {
			if(!hgv_terminal_energy_set && hgv_terminal_range<=50.e3)
			{
			  hgv_low_energy_terminal=(Vm_t<675.);
			  hgv_terminal_energy_set=1;
			}
		  }
		  /* MERGE step 3b -- REJECTED 2026-08-23. The donor's long-entry gate
		     (homing blend held off until inside 50 km when stage 6 is entered
		     beyond 103 km) is INERT on tal_flag=1 (every screen value
		     identical) and mildly harmful on tal_flag=0 (T13n2 14012 -> 14782).
		     Original donor form kept here for reference:
		       if(homing_phase && (!hgv_long_stage6_entry ||
		                           hgv_terminal_range<=50.e3)) */
		  if(homing_phase)
		  {
			  double t_sm = 3.;
	//		 M_acc_pitch_command();
			
			if( (t - tt) <= t_sm )	
				ac_pitch =1.*( ac_pitch*(t-tt)/ t_sm)+(apc1*(t_sm-t+tt)/t_sm);
		  }
		  else	
		  {
			ac_pitch= control_altitude( hc_eff );/*LRG: cold-deficit cases glide higher*/
			/* EMG - energy-managed glide. Measured L/D in the cold glide is 0.98
			   at the 8-10 deg it naturally flies vs 1.71 near 14 deg, so there is
			   glide efficiency left unused. A true alpha-hold at max-L/D was
			   tested and was WORSE (the 1.71 samples are transient pull-ups, not
			   a sustainable operating point; holding high alpha adds as much drag
			   as lift in this steep reentry). Modestly scaling the glide command
			   measured best at 1.3 across all four deep-deficit arms; 1.6+ breaks
			   the T12n2 hit. Same observable gate as LRG. */
			if (GNC_CASE_ACTIVE && hgv_flg && dv_r < LRG_DV_ON) ac_pitch *= EMG_SCALE;
			apc1=ac_pitch;
			tt = t;
		  }

		  /* MERGE step 3 (2026-08-23) -- donor HGV stage-6 GLIDE CORRECTION.
		     Blends a flight-path-angle-tracking command in over the final 50 km
		     of horizontal range, driving the vehicle onto a beta-degree descent
		     (49 deg if it arrived slow, 59 deg otherwise).  Pure addition: it
		     only reshapes ac_pitch inside the last 50 km and leaves the legacy
		     terminal conditions untouched.  hc_eff (LRG) is KEPT here -- the
		     donor calls control_altitude(hc), but LRG is a measured local gain
		     and is deliberately not reverted. */
		  if( hgv_flg2==1 && (!GLIDE_LOWE_ONLY || hgv_low_energy_terminal) )
		  {
			double x=rtaz(0,0), y=rtaz(1,0), z=rtaz(2,0);
			double vx=vmaz(0,0), vz=vmaz(2,0);
			double hgv_horizontal_range=sqrt(x*x+y*y);
			double vpath=sqrt(vx*vx+vz*vz);
			if(hgv_horizontal_range<=50.e3 && x>2.e3 && vx>0. &&
			   vpath>1. && q>=0.3 && vz>=-0.1763269807*vx)
			{
			  double tau=x/vx;
			  if(tau>=8.)
			  {
				const double beta=(hgv_low_energy_terminal ? 49. : 59.)*d2r;
				double en=-x*sin(beta)+z*cos(beta);
				double vn=-vx*sin(beta)+vz*cos(beta);
				double an=6.*en/(tau*tau)-4.*vn/tau;
				double den=mag_g*(vx*cos(beta)+vz*sin(beta))/vpath;
				if(fabs(den)>0.25*mag_g)
				{
				  double glide_cmd=(mag_g*cos(beta)-an)/den;
				  double glide_limit=1.5+0.35*q;
				  if(glide_limit<1.5) glide_limit=1.5;
				  if(glide_limit>5.0) glide_limit=5.0;
				  glide_cmd=limit(glide_cmd,glide_limit,-glide_limit);
				  double blend_width=hgv_low_energy_terminal ? 1.e3 : 4.e3;
				  double blend=(50.e3-hgv_horizontal_range)/blend_width;
				  if(blend<0.) blend=0.;
				  if(blend>1.) blend=1.;
				  blend *= (GLIDE_AUTH);
				  ac_pitch=(1.-blend)*ac_pitch+blend*glide_cmd;
				}
			  }
			}
		  }
	  }
		
		/*Step6 NOTE: the apparent cross-pairing here is a compensating naming
		  convention (ay_b/az_b are themselves plane-swapped at their
		  derotation) — a "corrective" swap was tested and REJECTED: it wrecks
		  every arm incl. healthy (T13n0 33.4 m -> 67 km, huge cross-range).
		  The pitch-achieved telemetry is col 20 (Ayb_filter), NOT col 18.*/
		du_pitch = ac_pitch * k_5 - Ayb_filter; // 13
		du_yaw = -ayc * k_5 + Azb_filter;
	}
	return;
}

//////////////////////////////////////////////////////////////
void Sep_func()
{
//-------------- seperation & ignation conditions

	if(t<5. || start_flg==0)	return;
   if ( ((q < 0.3)||(t >= 80.0)||(Vym < 0.0)) && (StopCtrlCmd == 0 && sepflg==0)) 
	//if (t>(t_burn1+d_j) && (StopCtrlCmd_2 == 0 && sepflg==0) ) 
	//if (t>(t_burn1+Del_tm) && (StopCtrlCmd == 0 && sepflg==0) ) 
	{
		StopCtrlCmd=1;
		t_StopCtrlCmd=t;
		StopCtrlCmd_2=1;
		for (int j = 0; j < 5; j++)
		{
		SetDriverBoxCmd(SEP_Safe_On);
		SetDriverBoxCmd(PAYM_Safe_On);
		//SetDriverBoxCmd(igni_Safe_On);
		}
	}

	if ((t>(t_StopCtrlCmd+0.03) && StopCtrlCmd_2) && BeltCmd == 0 ) 
{
		for (int j = 0; j < 5; j++)
			SetDriverBoxCmd(SEP_Cmd_On);

		BeltCmd = 1;
		t_BeltCmd = t;
	}

	if ( (t>=(t_BeltCmd+0.05) ) && (BeltCmd && PushCmd == 0)  )
	{
		for (int j = 0; j < 5; j++)
			SetDriverBoxCmd(PAYM_Cmd_On);

		PushCmd = 1;
		t_PushCmd = t;
	}

	if (PushCmd && ign2_cmd==0 && (s_s==ign2_stg))
	{
		//if (WH_sep==0)	
		//{
			//for (int j = 0; j < 5; j++)
				//SetDriverBoxCmd(igni_Cmd_On);
			//}

		ign2_cmd = 1;
		t_ign2 = t;
		GNC_SIM_IGN2_DELAY();
		vb = 1;
		
	}
	if ( (t>(t_PushCmd+0.0) && PushCmd) && sepflg == 0 )
		{
		sepflg = 1;
		t_sep = t;
		
		StopCtrlCmd = 0;
	}
	if ( (t>(t_ign2+4.0) && ign2_cmd) && s_s==ign2_stg  && ign2_ok==0)
	{
		if(Ab(0,0) < (0.5*9.81))
		{
			ign2_cmd = BeltCmd = PushCmd = 0;
			ign2_cmd_off=BeltCmd_off=PushCmd_off=0;
			ign2_ok5=0;

			for (int j = 0; j < 5; j++)
			{
				SetDriverBoxCmd(SEP_Safe_On);
				SetDriverBoxCmd(PAYM_Safe_On);
				//SetDriverBoxCmd(igni_Safe_On);
			}
		}
		else
		{
			ign2_ok5++;
			if (ign2_ok5>=10)	ign2_ok=1;
		}
	}//*/
//-------------- WH sep 3

	if (WH_sep==1 && t>(t_sep+2.) && bad_sep3_cond==0 && sep3_off==0)
	{

		if(s_s>3 )
		{
			if (sep3_cmd==0)
		{
				if (fabs(vzm_c - Vzm) >20. )
					bad_sep3_cond++;
				else
				{
					SetDriverBoxCmd(igni_Safe_On);
					t_sep3=t;
					sep3_cmd=1;
					sepflg2 = 1;
					StopCtrlCmd=1;
				}
			}
			if (t>t_sep3 && sep3_cmd)
			{
				for (int j = 0; j < 5; j++)
					SetDriverBoxCmd(igni_Cmd_On);//v131
			}

		}
		if ((t>t_sep3+0.5 && sep3_cmd) || bad_sep3_cond)
		{
			SetDriverBoxCmd(igni_Safe_Off);
			SetDriverBoxCmd(igni_Cmd_Off);
			sep3_off=1;
		}
	}
	/*if (WH_sep>=1)
	{
		if (Vym < 0. && s_s>3 && Fh<200e3)
		{
			CWarHead_Cmd=changeprj;
			WarHead(CWarHead_Cmd);

		}
		else if (s_s>3)
		{
			CWarHead_Cmd=changeserial;
			WarHead(CWarHead_Cmd);
		}

	}//*/
//----------------- switch gpio off
	/*
	if(BeltCmd && t>=t_BeltCmd+0.1)
	{
		SetDriverBoxCmd(SEP_Safe_Off);
		SetDriverBoxCmd(SEP_Cmd_Off);
	}	//*/

	//if(s_s>2 || ((PushCmd && t>=t_PushCmd+0.2) || ign2_ok))
	if((PushCmd && t>=t_PushCmd+1.0 ) || ign2_ok)
	{
			
				SetDriverBoxCmd(SEP_Safe_Off);
				SetDriverBoxCmd(SEP_Cmd_Off);
				SetDriverBoxCmd(PAYM_Safe_Off);
				SetDriverBoxCmd(PAYM_Cmd_Off);

		if ((WH_sep==0 && ign2_ok==1))
			{
				SetDriverBoxCmd(igni_Safe_Off);
				SetDriverBoxCmd(igni_Cmd_Off);
			}
		
            BeltCmd_off++;
	}
//------------------------ warhead conditions
		//v14
		if(Vym<0 && Slant_Range<3e5 && s_s>2)
		{
			enemyzone_flg1=1;
		}
		if(enemyzone_flg1 && enemyzone_flg2 && (t >= enemyzone_t+8.))//(s_s==6 && Hme<25e3))
		{
			//CWarHead_Cmd=enemyzone;
			WarHead(enemyzone);
		}
		else if(ign2_cmd||t>40)
		{
			//CWarHead_Cmd=startflight;
			WarHead(startflight);
		}

		/* if(t>20)
		{
			CWarHead_Cmd=startflight;
			WarHead(CWarHead_Cmd);
		}
		if(t>40)
		{
			CWarHead_Cmd=enemyzone;
			WarHead(CWarHead_Cmd);
		}//*/
	if(enemyzone_flg1 && (Ab(0,0) <= (-1.5*9.81) ) && (enemyzone_flg2 == 0))
	{
      enemyzone_5 ++;
      if (enemyzone_5 == 5) 
	  {
        enemyzone_flg2 = 1;
        enemyzone_t=t;
      }
    }else  enemyzone_5 = 0;
 
}


int Check_targets()
{
	int  cnt2=0,cnt1=0,idx=TrgtCnt;
	//mat3x1 rt_l0,rte0,rtl_rml;
		
	if(((Vm_t>(5.*340) && tgt_flg==0 ) || homing_phase) && hgv_flg)	return TrgtCnt;
	else
	{
	for (int j = 0; j < PointsCnt+1; j++)// +1 of (TrgtCnt 0 index)//g
	{

		//int i=cnt2;
		mat3x1 rte0 = nav2cart(Lattf[j],Longtf[j], Htf[j]);

		mat3x1 rt_l0 = C_e_l * (rte0 - R0_E);
		double rt_l_Norm = rt_l0.Norm();
		
		if(rt_l_Norm >1e6  && rt_l_Norm<3e6)
		{
			//tgo = (Slant_Range/Vm_t);
			mat3x1 rtl_rml = rt_l0-r_l;
			
			double SlantR = rtl_rml.Norm();//R;
			double vmxz0 = sqrt(Vxm * Vxm + Vzm * Vzm);
			double rxz0 = sqrt(rtl_rml(0,0)*rtl_rml(0,0) + rtl_rml(2,0) *rtl_rml(2,0));
			double t2go_xz = (rxz0/vmxz0);

			if(hgv_flg)	ratio_m[cnt2] =1*fabs(50.-t2go_xz)+(fabs(rtl_rml(2,0)/SlantR))*50.;//50;	
			else		ratio_m[cnt2] =1*fabs(25.-t2go_xz)+(fabs(rtl_rml(2,0)/SlantR))*50.;//50;
			cnt2++;
		}

	}

		idx = MinId(cnt2,ratio_m);

		mat3x1 rte0= nav2cart(Lattf[idx],Longtf[idx], Htf[idx]);
		mat3x1 rt_l0 = C_e_l * (rte0 - R0_E);

		//azimuth(Htf[TrgtCnt], Lattf[TrgtCnt], Longtf[TrgtCnt], Htf[idx], Lattf[idx], Longtf[idx], &az, &Range2);

		tgt_flg++;

		if(TrgtCnt == idx || GNC_TARGET_SELECT_OVERRIDE || (rt_l0.Norm()<3e3))	return TrgtCnt;
		else													return idx;//TrgtCnt;
		//return TrgtCnt;

	}
}


int MinId(int no,double inp[])
{
	double val = inp[0];
	int idx = 0;

	for (int i=0; i<no; i++) 
	{
		if(inp[i]<val)
		{
			val = inp[i];
			idx = i;
		}
	}
	return idx;
}
