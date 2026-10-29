#include"stdafx.h"
#include "m273n.h"


//global constants to be used in the simulation
//physical constants
double const REARTH=6370987.308;		//mean earth radius - m
double const WEII3=7.292115e-5;			//angular rotation of earth - rad/s
double const AGRAV=9.80675445;			//standard value of gravity acceleration - m/s^2 
double const G=6.673e-11;				//universal gravitational constant - Nm^2/kg^2 
double const EARTH_MASS=5.973332e24;	//mass of the earth - kg 
double const GM=3.9860044e14;			//gravitational parameter=G*EARTH_MASS - m^3/s^2 
double const C20=-4.8416685e-4;			//second degree zonal gravitational coefficient - ND
double const FLATTENING=3.33528106e-3;  //flattening of the Earth (WGS84) - ND
double const SMAJOR_AXIS=6378137;        //semi-major axis of Earth's ellipsoid (WGS84) - m 
double const GW_CLONG=0;				//Greenwich celestial longitude at start of simulation - rad
double const RGAS=287.053;				//ideal gas constant - J/(K*kg)=N*m/(K*kg) 
double const KBOLTZ=1.38e-23;			//Boltzmann's constant - Ws/K  
//numerical constants
//double const PI=3.1415926536;			//circumference of unit diameter circle
double const EPS=1.e-10;				//machine precision error (type double)
double const SMALL=1e-7;				//small real number
int const    ILARGE=9999;				//large integer number
double const LARGE=1e10;				//large real number (type double)
//conversion factors
double const RAD=0.0174532925199432;	//conversion factor deg->rad
double const DEG=57.2957795130823;		//conversion factor rad->deg
double const FOOT=3.280834;				//conversion factor m->ft			
double const NMILES=5.399568e-4;		//conversion factor	m->nm	
//sizing of arrays
int const CHARN=31;						//character numbers in variable names
int const CHARL=200;					//character numbers in a line
//*** verify the following array sizes. If too small, dynamic memory allocation will fail! ***
int const NROUND6=300;					//size of 'round6' module-variable array
int const NHYPER=850;					//size of 'hyper' module-variable array
int const NROUND3=60;					//size of 'round3' module-variable array 
int const NSAT=20;						//size of 'satellite' module-variable array
int const NGROUND0=20;					//size of 'round3' module-variable array 
int const NRADAR=30;					//size of 'recce' module-variable array
int const NEVENT=20;					//max number of events
int const NVAR=50;						//max number of variables to be input at every event 
int const NMARKOV=20;					//max number of Markov noise variables
//#######################################################################################
extern double rmn_[],R0_t[],Re[],vmn_[],accb[];
int guidarc_flg=0;
extern double cnqb, cnqa, cad1, cad2, cad3, cad4;
extern double cnb, cmb, cmde, cmdr, cndr, cnde, cna, cma;
extern double K_w_pitch,K_i_integ_acc,K_p_integ_acc;

double dla,dmde,dmq,dma;
double dllp,dllda;
	double du_p=0;//hyper[516].real();
	double out_integ_p=0;//hyper[517].real();
//#######################################################################################
///////////////////////////////////////////////////////////////////////////////
//Returns the angle between two 3x1 vectors
//Example: theta=angle(VEC1,VEC2);
//010824 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double angle(mat3x1 VEC1,mat3x1 VEC2)
{
	double argument;
	double scalar=(VEC1 * VEC2);//dot(VEC1 , VEC2);
	double abs1=VEC1.Norm();
	double abs2=VEC2.Norm();

	double dum=abs1*abs2;
	if(abs1*abs2>EPS)
		argument=scalar/dum;
	else
		argument=1.;
	if(argument>1.) argument=1.;
	if(argument<-1.) argument=-1.;

	return acos(argument);
}
///////////////////////////////////////////////////////////////////////////////
//Returns the inertial displacement vector from longitude, latitude and altitude
// using the WGS 84 reference ellipsoid
// Reference: Britting,K.R."Inertial Navigation Systems Analysis"
// pp.45-49, Wiley, 1971
//
// Return output
//			       SBII(3x1) = Inertial vehicle position - m
// Parameter input
//			       lon = geodetic longitude - rad
//                 lat = geodetic latitude - rad
//                 alt = altitude above ellipsoid - m
//				   time = simulation time - sec 
//
//030411 Created from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
mat3x1 cad_in_geo84( double lon, double lat, double alt, double time)
{
	mat3x1 SBII(0);
	double SBID[3];

	//deflection of the normal, dd, and length of earth's radius to ellipse surface, R0
    double r0=SMAJOR_AXIS*(1.-FLATTENING*(1.-cos(2.*lat))/2.+5.*pow(FLATTENING,2)*(1.-cos(4.*lat))/16.); //eq 4-21
	double dd=FLATTENING*sin(2.*lat)*(1.-FLATTENING/2.-alt/r0); //eq 4-15

	//vehicle's displacement vector from earth's center SBID(3x1) in geodetic coord.
	double dbi=r0+alt;
	SBID[0]=-dbi*sin(dd);
	SBID[1]=0.;
	SBID[2]=-dbi*cos(dd);

	//celestial longitude of vehicle at simulation 'time' 
    double lon_cel=GW_CLONG+WEII3*time+lon;
	
	//vehicle's displacement vector in inertial coord. SBII(3x1)=TID(3x3)xSBID(3x3)
	double slat=sin(lat);
	double clat=cos(lat);
	double slon=sin(lon_cel);
	double clon=cos(lon_cel);
	double sbid1=SBID[0];
	double sbid2=SBID[1];
	double sbid3=SBID[2];
    double sbii1=-slat*clon*sbid1-clat*clon*sbid3;
    double sbii2=-slat*slon*sbid1-clat*slon*sbid3;
    double sbii3=clat*sbid1-slat*sbid3;
	SBII(0,0)=sbii1;
	SBII(1,0)=sbii2;
	SBII(2,0)=sbii3;

	return SBII;
}


///////////////////////////////////////////////////////////////////////////////
//Guidance on an arc through a waypoint (horizontal only)
//Waypoints are provided as: 'wp_lonx', 'wp_latx', 'wp_alt'(optional, default=0)
//applicable to:
//			mguide = 4 lateral arc-guidance 
//
//parameter input
//		wp_lonx=waypoint longitude - deg
//		wp_latx=waypoint latitude - deg
//		wp_alt=waypoint altitude - m (any value, e.g.:0)
//return output:
//	 phicomx = commanded bank angle - deg
//
//030616 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double guidance_arc(double wp_lonx,double wp_latx,double wp_alt)
{
	//local variables
	double argument(0);
	double phicomx(0);
	double rad_dynamic(0);

	//local module-variables
	mat3x1 SWBD;
	int wp_flag(0);
	double wp_grdrange(0);
	double rad_min(0);
	double rad_geometric(0);

	//localizing module-variables
	//input from other modules
	double time=t;//t;//round6[0].real();
	double grav=magg;//round6[63].real();
	double phibdx=phim;//round6[139].real();
	mat3x1 SBIIC=Arr2Mat(Re);//=hyper[304].vec();
	mat3x1 VBECD=Arr2Mat(vmn_);//=hyper[329].vec();
	double dvbec=vm;//hyper[330].real();
	mat3x3 TDCI=Rot_e_n( rmn_[0],  rmn_[1]);//=hyper[331].mat();
	//double psivdcx=hyper[333].real();
	mat3x1 FSPCB=Arr2Mat(accb);//=hyper[334].vec();
	double alphacx=alfa*d2r;//hyper[336].real();
	double philimx=60.;//hyper[511].real();

	//-----------------------------------------------------------------------------
	if(guidarc_flg)	time=0;
	//converting waypoint coordinates
	 mat3x1 SWII = cad_in_geo84(wp_lonx,wp_latx,wp_alt,time);
	 SBIIC = cad_in_geo84(rmn_[1],rmn_[0],rmn_[2],0);

	//displacement of waypoint wrt hyper in geographic coord
	 SWBD=TDCI*(SWII-SBIIC);

	//projection of displacement vector into horizontal plane, SH
	mat3x1 SH(0);
	SH(0,0)=SWBD(0,0);
	SH(1,0)=SWBD(1,0);
	SH(2,0)=0;

	//horizontal ground distance based on hyper geographic coordinates
	double dwbh=sqrt(SWBD(0,0)*SWBD(0,0)+SWBD(1,0)*SWBD(1,0));

	//calculating azimuth angle of waypoint LOS wrt velocity vector, psiwvx
	//projection of velocity vector into horizontal plane, VH
	mat3x1 VH(0);
	VH(0,0)=VBECD(0,0);
	VH(1,0)=VBECD(1,0);
	VH(2,0)=0;

	//vector normal to arc plane, UV
	mat3x1 UV=Vect2SkewSym(VH)*SH;
	//steering angle, psiwvx 
	double psiwvx=DEG*angle(VH,SH);
	//steering angle with proper sign
	mat3x1 ZZ(0);
	ZZ(2,0)=1;
	//psiwvx=psiwvx*sign(dot(UV,ZZ));
	psiwvx=psiwvx*sign((UV*ZZ));

	//getting acceleration component in the loadfactor plane
	double fspb3=FSPCB(2,0);

	//selecting guidance mode
	if(fabs(psiwvx)<90)
		//guiding on the arc through the waypoint
	{
		double num=-2*dvbec*dvbec*sin(psiwvx*RAD);
//		double denom=fspb3*dwbh;
		double denom=-grav*dwbh; //eliminates limit cycle, asssuming level flight
		if(denom!=0)
			argument=num/denom;
		if(fabs(asin(argument))<philimx*RAD)
			phicomx=DEG*asin(argument);
		else
			phicomx=philimx*sign(argument);
	}
	else
	{
		//making a minimum turn
		phicomx=philimx*sign(psiwvx);
	}
	//diagnostic: radii
	if((phibdx*fspb3)!=0)
		rad_dynamic=fabs(dvbec*dvbec/(fspb3*sin(phibdx*RAD)));
	if(psiwvx!=0) 
		rad_geometric=fabs(dwbh/(2*sin(psiwvx*RAD)));
		
	//setting way point flag (if within 2 times turning radius): closing (+1) or fleeting (-1)
	rad_min=dvbec*dvbec/(grav*tan(philimx*RAD));
	if(dwbh<2*rad_min)
		//wp_flag=sign(dot(VH,SH));
		wp_flag=sign((VH*SH));
	else
	{
		wp_flag=0;
		//return 0;
	}
	if(t>862.)
		t=t;
	//diagnostic: ground range to waypoint
	wp_grdrange=dwbh; 
	//-----------------------------------------------------------------------------
	////loading module-variables
	//hyper[415].gets(wp_grdrange); 
	//hyper[416].gets_vec(SWBD); 
	//hyper[417].gets(rad_min);
	//hyper[418].gets(rad_geometric);
	//hyper[419].gets(wp_flag);
	guidarc_flg=1;
	return phicomx * d2r;
}//*/
double guidance_arc2(double wp_lonx,double wp_latx,double wp_alt)
{
	//local variables
	double argument(0);
	double phicomx(0);
	double rad_dynamic(0);

	//local module-variables
	mat3x1 SWBD;
	int wp_flag(0);
	double wp_grdrange(0);
	double rad_min(0);
	double rad_geometric(0);

	//localizing module-variables
	//input from other modules
	double time=t;//t;//round6[0].real();
	double grav=magg;//round6[63].real();
	double phibdx=phim;//round6[139].real();
	mat3x1 SBIIC=Arr2Mat(Re);//=hyper[304].vec();
	mat3x1 VBECD=Arr2Mat(vmn_);//=hyper[329].vec();
	double dvbec=vm;//hyper[330].real();
	mat3x3 C_e_n=Rot_e_n(rmn_[0],  rmn_[1]);//TDCI=hyper[331].mat();
	//double psivdcx=hyper[333].real();
	mat3x1 FSPCB=Arr2Mat(accb);//=hyper[334].vec();
	//double alphacx=alfa*d2r;//hyper[336].real();
	double philimx=60.;//hyper[511].real();

	mat3x1 SWII(0);
	//-----------------------------------------------------------------------------
	if(guidarc_flg)	time=0;
	//converting waypoint coordinates
	 //SWII = cad_in_geo84(wp_lonx,wp_latx,wp_alt,time);
	 //SBIIC = cad_in_geo84(rmn_[1],rmn_[0],rmn_[2],0);

	 SWII  = nav2cart(wp_latx,wp_lonx,wp_alt);
	 //SBIIC = nav2cart(Arr2Mat(rmn_));
	 
	//displacement of waypoint wrt hyper in geographic coord
	 SWBD=(SWII-SBIIC);
	 SWBD=C_e_n*(SWII-SBIIC);

	//projection of displacement vector into horizontal plane, SH
	mat3x1 SH(0);
	SH(0,0)=SWBD(0,0);
	SH(1,0)=SWBD(1,0);
	SH(2,0)=0;

	//horizontal ground distance based on hyper geographic coordinates
	double dwbh=sqrt(SWBD(0,0)*SWBD(0,0)+SWBD(1,0)*SWBD(1,0));

	//calculating azimuth angle of waypoint LOS wrt velocity vector, psiwvx
	//projection of velocity vector into horizontal plane, VH
	mat3x1 VH=VBECD;
	VH(2,0)=0;

	//vector normal to arc plane, UV
	mat3x1 UV=Vect2SkewSym(VH)*SH;
	//steering angle, psiwvx 
	double psiwvx=DEG*angle(VH,SH);
	//steering angle with proper sign
	mat3x1 ZZ(0);
	ZZ(2,0)=1;
	psiwvx=psiwvx*sign(dot(UV,ZZ));

	//getting acceleration component in the loadfactor plane
	double fspb3=FSPCB(2,0);

	//selecting guidance mode
	if(fabs(psiwvx)<90)
		//guiding on the arc through the waypoint
	{
		double num=-2*dvbec*dvbec*sin(psiwvx*RAD);
//		double denom=fspb3*dwbh;
		double denom=-grav*dwbh; //eliminates limit cycle, asssuming level flight
		if(denom!=0)
			argument=num/denom;
		if(fabs(asin(argument))<philimx*RAD)
			phicomx=DEG*asin(argument);
		else
			phicomx=philimx*sign(argument);
	}
	else
	{
		//making a minimum turn
		phicomx=philimx*sign(psiwvx);
	}
	//diagnostic: radii
	if((phibdx*fspb3)!=0)
		rad_dynamic=fabs(dvbec*dvbec/(fspb3*sin(phibdx*RAD)));
	if(psiwvx!=0) 
		rad_geometric=fabs(dwbh/(2*sin(psiwvx*RAD)));
		
	//setting way point flag (if within 2 times turning radius): closing (+1) or fleeting (-1)
	rad_min=dvbec*dvbec/(grav*tan(philimx*RAD));
	if(dwbh<2*rad_min)
		wp_flag=sign(dot(VH,SH));
	else
	{
		wp_flag = 0;
		//return 0;
	}
	if(t>862.)
		t=t;
	//diagnostic: ground range to waypoint
	wp_grdrange=dwbh; 
	//-----------------------------------------------------------------------------
	////loading module-variables
	//hyper[415].gets(wp_grdrange); 
	//hyper[416].gets_vec(SWBD); 
	//hyper[417].gets(rad_min);
	//hyper[418].gets(rad_geometric);
	//hyper[419].gets(wp_flag);
	guidarc_flg=1;
	return phicomx * d2r;
}//*/


///////////////////////////////////////////////////////////////////////////////
// Heading hold control (mauty=4)
// Generates bank command for roll control
// Employs pole placement technique with the condition that the
//  real pole of the closed loop heading transfer function equals
//  the product of natural frequency and damping of the roll transfer function
//
// This subroutine performs the following functions:
// (1) Calculates the heading loop forward gain.
// (2) Outputs roll position command
//
// Return output:
//          phicomx=Roll position command - deg
// Parameter input:
//			psivdocx=Heading command - deg
//
//030522 Created by Peter H Zipfel 
///////////////////////////////////////////////////////////////////////////////

double control_heading(double psivdcomx)
{
	//local variable
	double phicomx(0);
	
	//local module-variables
	double gainpsi(0);

	//localizing module-variables
	//input data
	double wrcl=8.;//hyper[512].real();
	double zrcl=0.9;//hyper[513].real();
	double facthead=-0.95;//hyper[551].real();
	//input from other modules
	double grav=magg;//round6[63].real();
	double dvbec=vm;//hyper[330].real();
	double psivdcx=atan2(vmn_[1],vmn_[0]);//hyper[333].real();
	psivdcx = At - psim;
	//--------------------------------------------------------------------------

	//calculating heading gain
	gainpsi=(dvbec/grav)*zrcl*wrcl*(1.-zrcl*zrcl)*(1.+facthead);

	//roll command
	phicomx=gainpsi*(psivdcomx-psivdcx);

	//--------------------------------------------------------------------------

	return phicomx;
}//*/
/*
///////////////////////////////////////////////////////////////////////////////
// Altitude hold control (mautp=5)
// Generates accleration command for pitch acceleration autopilot,
//  correcting for bank angle
// MAUTP=5
//
// This subroutine performs the following functions:
// (1) Calculates the pitch acceleration command
//
// Return output
//          ancomx=Pich acceleration command - g's
// Parameter input
//			altcom=Altitude command - m
//
//030522 Created by Peter H Zipfel 
///////////////////////////////////////////////////////////////////////////////

double control_altitude(double altcom)
{
	//local variable
	double ancomx(0),altrate(0);

	//localizing module-variables
	//input data
	double gainalt=0.05,gainaltrate=1.5*0.05;//hyper[529].real();
	//double gainalt=1.*0.05,gainaltrate=1.025*0.05;//hyper[529].real();
	//input from other modules
	double grav=cos(tetam);//magg;//round6[63].real();
	double altc=hm;//hyper[328].real();
	//Matrix VBECD=hyper[329].vec();
	double phibdcx=0*phim;//hyper[338].real();
	//--------------------------------------------------------------------------
	//altitude rate feedback
	altrate=vmf[1];//-VBECD[2];
	//altrate=-1*vmn_[2];//-VBECD[2];

	//acceleration command
	double eh=gainalt*(altcom-altc);
	if(phibdcx==0) phibdcx=SMALL;
	ancomx=(1./cos(phibdcx))*(gainaltrate*(eh-altrate)+grav)/AGRAV;

	//--------------------------------------------------------------------------

	return ancomx;
}//*/
//-------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
//Terminal guidance using the advanced guidance law (AGL)
//Member function of class 'Hyper'
//
// Calculates acceleration commands based on relative position
//  and velocity of target wrt hyper vehicle in inertial coordinates.
//
// Parameter input
//			STBIK(3x1) = Displacment of target wrt hyper vehicle in inertial coord - m
//			VTBIK(3x1) = Velocity of target wrt hyper vehicle in inertial coord - m/s
//
// Return output
//			ACCOMX(3x1) = acceleration command in body coord - g's
// Parameter output
//			UTjjC(3x1) = Commanded unit thrust vector in body coor - ND  
//
//040608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

mat3x1 guidance_AGL( mat3x1 STBIK,mat3x1 VTBIK)
{
	//local variables
	double tgok(0);
	mat3x1 ACPURI(0), ACPNI(0),ACCOMX(0),UTBIK,UTjjC;

	//local module-variables
	double dtbc(0),psiobcx(0),thtobcx(0),dvtbc(0),tgoc(0);

	//double gnavpn=3,gnavps=3;//3.;
	//double gnavpn=2.05,gnavps=2.05;//3.;
	double gnavpn=2.5,gnavps=2.5;//3.;
	double gnmax=20.;//hyper[167].real();

	// ghd
	double Gb[3];
	twm(1, mi2b, Gi, Gb);
	mat3x3 TBIC,TIBC,TIFC;//hyper[315].mat();
	Arr2Mat3x3(mb2i,&TIBC);
	Arr2Mat3x3(mi2f,&TIFC);
	TBIC = ~TIBC;

	STBIK = ~TIFC*STBIK;
	VTBIK = ~TIFC*VTBIK;
	//-------------------------------------------------------------------------

	//range to go
	dtbc=STBIK.Norm();

	//unit los vector
	// also used as commanded unit thrust vector in body coord (keeping angle of attack at zero)
	UTBIK=univec3(STBIK);
	UTjjC=TBIC*UTBIK;

	//LOS angles wrt hyper body
	mat3x1 POLAR=pol_from_cart(UTjjC);
	psiobcx=POLAR(1,0)*DEG;
	thtobcx=POLAR(2,0)*DEG;

	//closing velocity
	dvtbc=dot(UTBIK,VTBIK);

	//time to go
	tgoc=fabs(dtbc/dvtbc);

	//acceleration command in inertial coordinates
	if(tgoc!=0){
		//pursuit term
		ACPURI=(gnavps/(tgoc*tgoc)) *STBIK;
		//proportional guidance term
		ACPNI=(gnavpn/tgoc)*VTBIK;
	}
	
	//acceleration desired in body coordinates
	ACCOMX = (1/AGRAV)*(TBIC*((ACPURI+ACPNI)-Arr2Mat(Gb)));

	//-------------------------------------------------------------------------
	double acc_limit = q / 3.;

   if (q < 30.)						acc_limit *= 0.5;
   else if (q < 60. && q >= 30.)	acc_limit *= q / 60.;

	for (int i = 0; i < 3; i++)
	{
		ACCOMX(i,0)=limit(ACCOMX(i,0),acc_limit,-acc_limit);
		ACCOMX(i,0)=limit(ACCOMX(i,0),gnmax,-gnmax);
	}
	
	return ACCOMX;
}

///////////////////////////////////////////////////////////////////////////////
//Roll controller
//Member function of class 'Hyper'
//Pole placement technique, specifying closed loop conj complex poles
// (1) Calculates gains
// (2) Calculates commanded roll control
//
//Return output
//		delacx = roll control command - deg
//Parameter input
//		phicomx = roll command - deg
//
//011212 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double control_roll(double phicomx)
{
	//local module-variables
	double gkp(0);
	double gkphi(0);

	//localizing module-variables
	//input data
	double wrcl=8.;//hyper[512].real();
	double zrcl=0.9;//hyper[513].real();
	//input from other modules
	double phibdcx=phim*r2d;//hyper[338].real(); 
	double ppcx=Wx_filter*r2d;//hyper[320].real();
	//double dllp=hyper[155].real();
	//double dllda=hyper[156].real();
	//-------------------------------------------------------------------------
	//calculating gains
	gkp=(2.*zrcl*wrcl+dllp)/dllda;
    gkphi=wrcl*wrcl/dllda;

	//roll position control
    double ephi=gkphi*(phicomx-phibdcx)*RAD;
    double dpc=ephi-gkp*ppcx*RAD;
    double delacx=dpc;//*DEG;
	//-------------------------------------------------------------------------

	return delacx;
}
///////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------
void aerodynamics_der()
{
	//local module-variables
	double stmarg(0);
	double  dlde(0);
	double dyb(0);
	double dydr(0);
	double dnb(0);
	double dnr(0);
	double dndr(0);
	double realp1(0);
	double realp2(0);
	double wnp(0);
	double zetp(0);
	double rpreal(0);
	double realy1(0);
	double realy2(0);
	double wny(0);
	double zety(0);
	double ryreal(0);



	//localizing module-variables
	//from initialization
	double refa=Aref;//hyper[104].real();
    double refb=D_body;//hyper[105].real();
    double refc=D_body;//hyper[106].real();
	//from other modules
	double time=t;//round6[0].real();
	double vmach=mach;//round6[56].real();
	double pdynmc=dynpres;//round6[57].real();
	double dvba=mag(3, vbwb);//round6[75].real();
    double vmass= mass;//hyper[15].real();
	mat3x1 IjjB;//=hyper[18].mat(); 
	//from aerodynamics module		
	double cla=cna;//hyper[123].real();
	double clde=cnde;//hyper[124].real();
	double cyb=cnb;//hyper[125].real();
	double cydr=cndr;//hyper[127].real();
	//double cllda=hyper[129].real();
	//double cllp=hyper[130].real();
	//double cma=hyper[133].real();
	//double cmde=hyper[134].real();
	double cmq=cmqa;//hyper[135].real();
	double clnb=cnb;//hyper[136].real();
	double clndr=cndr;//hyper[138].real();
	double clnr=0;//hyper[140].real();
	//-------------------------------------------------------------------------
	//MOI components
	double ibbb11=ixx;//IjjB.get_loc(0,0);
	double ibbb22=iyy;//IjjB.get_loc(1,1);
	double ibbb33=izz;//IjjB.get_loc(2,2);
	//Dimensional derivatives for pitch plane (converted to 1/rad where requried)
	double duml=(pdynmc*refa/vmass)/RAD;
	dla=duml*cla;
	dlde=duml*clde;
	double dumm=pdynmc*refa*refc/ibbb22;
    dma=dumm*cma/RAD;
    dmq=dumm*(refc/(2.*dvba))*cmq;
    dmde=dumm*cmde/RAD;

	//Dimensional derivatives in plane (converted to 1/rad where requried)
    double dumy=pdynmc*refa/vmass;
    dyb=dumy*cyb/RAD;
	dydr=dumy*cydr/RAD;
    double dumn=pdynmc*refa*refb/ibbb33;
    dnb=dumn*clnb/RAD;
    dnr=dumn*(refb/(2.*dvba))*clnr;
    dndr=dumn*clndr/RAD;

	//Dimensional derivatives in roll (converted to 1/rad where requried)
	double dumll=pdynmc*refa*refb/ibbb11;
    dllp=dumll*(refb/(2.*dvba))*cllp;
    dllda=dumll*cllda/RAD;

	//static margin (per chord length 'refc')
	if(cla) stmarg=-cma/cla;

	//diagnostics: pitch plane roots		
	double a11=dmq;
	double a12=dma/dla;
	double a21=dla;
	double a22=-dla/dvba;
	
	double arg=pow((a11+a22),2)-4.*(a11*a22-a12*a21);
	if(arg>=0.)
	{
	   wnp=0.;
	   zetp=0.;
	   double dum=a11+a22;
	   realp1=(dum+sqrt(arg))/2.;
	   realp2=(dum-sqrt(arg))/2.;
	   rpreal=(realp1+realp2)/2.;
	}
	else
	{
	   realp1=0.;
	   realp2=0.;
	   wnp=sqrt(a11*a22-a12*a21);
	   zetp=-(a11+a22)/(2.*wnp);
	   rpreal=-zetp*wnp;
	}
	
	//diagnostics: yaw plane roots		
	a11=dnr;
	a12=dnb/dyb;
	a21=-dyb;
	a22=dyb/dvba;
	
	arg=pow((a11+a22),2)-4.*(a11*a22-a12*a21);
	if(arg>=0.)
	{
	   wny=0.;
	   zety=0.;
	   double dum=a11+a22;
	   realy1=(dum+sqrt(arg))/2.;
	   realy2=(dum-sqrt(arg))/2.;
	   ryreal=(realy1+realy2)/2.;
	}
	else
	{
	   realy1=0.;
	   realy2=0.;
	   wny=sqrt(a11*a22-a12*a21);
	   zety=-(a11+a22)/(2.*wny);
	   ryreal=-zety*wny;
	}
	/*
	//-------------------------------------------------------------------------
	float _cmaw[9][9],_cnaw[9][9],_cmdew[9][9],_cndew[9][9];
	double czq,cmad,d=D_body,s=Aref,u=vm,m=mass,iy=iyy,teta=tetam,q=dynpres;
	double czdele,cmdele,cza;
	difdatw11(_cmaw,cmalfaw);
	difdatw11(_cnaw,cnalfaw);
	difdatw11(_cmdew,cmdeltaw);
	difdatw11(_cndew,cndeltaw);
	//difdatw1(_cldaw,clldelaw);

 czdele=1*interpolwd(mach,dele,_cndew)*r2d;
 cmdele=1*interpolwd(mach,dele,_cmdew)*r2d;//+czdele*xbc/d;
 cza=interpolw2(mach,alfa,_cnaw)*r2d;
 cma=interpolw2(mach,alfa,_cmaw)*r2d;//+cza*xbc/d;
 //cldela=fabs(interpolWt(mach,dela,_cldaw))*r2d;

 czq=fabs(cnqa)*r2d;
 cmq=cmqa*r2d;//+czq*xbc/d;
 //clp=clp*r2d;
 cx=fabs(ca);
 cmad=0.;
//**********
//	C1 = (-a2*a7+a3*a6)*kc;                  // ------- = ----------------------------
 dla =(q*s*(cza))/(m*u);
 //a2=q*s*cnde*r2d/(m*u);
 //a3=q*s*d*czq/(2.*m*u*u);
 //a4=0.*9.81*sin(teta)/u;//*cos(45.*d2r)/u;
 dma=q*s*d*cma/iy;
 dmde=q*s*d*cmdele/iy;
 dmq=q*s*d*d*cmq/(2*u*iy);
 //ar10 = cldela;
 //ar11 = ix/(s*q*d);
 //ar12 = -d*clp/(2*u);
 //ar13 = 0.;//*/

}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
////////////////////  Integration functions  //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//Integration of scalar state variable
//Modified Euler Midpoint method
//Example first order lag:
//			phid_new=(phic-phi)/tphi;
//			phi=integrate(phid_new,phid,phi,int_step);
//			phid=phid_new;
//010628 Created by Peter H Zipfel
//050202 Simplified and renamed to Modified Euler method, PZi 
///////////////////////////////////////////////////////////////////////////////
double integrate(const double &dydx_new,const double &dydx,const double &y,const double &int_step)
{
	return y+(dydx_new+dydx)*int_step/2;
}

///////////////////////////////////////////////////////////////////////////////
//Acceleration controller in normal (pitch) plane
//Member function of class 'Hyper'
//Employs pole placement technique (no matrix inversion required) 
//Feedback signals are: body rate (gyro) and acceleration (accel)
//
// (1) Calculates two feedback and one feed-forward gains
//     based on input of dominant closed loop conjugate complex
//     roots
// (2) Calculates the commanded pitch control deflection
//
//Return output
//		delecx = pitch control command - deg
//Parameter input
//		ancomx = normal loadfactor command - g
//		int_step = integration step size - s
//		
//021015 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double control_normal_accel(double ancomx,double int_step)
{
	//local module-variables
	//Matrix GAINFP(3,1);

	double wzr =cos(phim) * Wz_filter +  sin(phim) * Wy_filter;
	//localizing module-variables
	//input data
//!	int mfreeze=hyper[503].integer();
	double waclp=0.5;//hyper[504].real();
	double zaclp=0.7;//hyper[505].real();
	double paclp=5.;//hyper[506].real();
	double K_p_acc=0.005;//hyper[525].real();
	//input from other modules
	double time=t;//round6[0].real();
	double pdynmc=dynpres;//round6[57].real();
	//double dla=hyper[145].real();
	//double dma=hyper[147].real();
	//double dmq=hyper[148].real();
	//double dmde=hyper[149].real();
	double dvbec=vm;//hyper[330].real();
	double qqcx=wzr;//wz;//hyper[321].real();
	double FSPCB[3];//=Arr2Mat(accb);//hyper[334].vec();
	 for (int i = 0; i < 3; i++)
		 FSPCB[i]=accb[i];
	//state variables
	//double zzd=hyper[516].real();
	//double zz=hyper[517].real();
	//-------------------------------------------------------------------------
	 dmde=1.*(dmde);
	 dma=dma;
	 dmq=dmq;
	 dla=dla;
	//gain calculation
	double K_i_acc=waclp*waclp*paclp/(dla*dmde);
	double K_w_acc=(2.*zaclp*waclp+paclp+dmq-dla/dvbec)/dmde;
	double gainfb1=(waclp*waclp+2.*zaclp*waclp*paclp+dma+dmq*dla/dvbec-K_w_acc*dmde*dla/dvbec)/(dla*dmde)-K_p_acc;
	
	//K_w_acc=K_w_pitch;
	//K_i_acc=K_i_integ_acc;
	//K_p_acc=K_p_integ_acc;

	//	K_w_pitch=K_w_acc;
	//K_i_integ_acc=K_i_acc;
	//K_p_integ_acc=K_p_acc;

	//		K_w_acc=fabs(K_w_acc);
	//K_i_acc=fabs(K_i_acc);
	//K_p_acc=fabs(K_p_acc);


	if(K_i_acc<0 || K_w_acc<0 ||gainfb1<0)
		t=t;
	//pitch loop acceleration control, pitch control command
	double fspb3=-1*accb[1]/9.81;//
	//fspb3=-1*Ayb_filter*AGRAV;//1*FSPCB[1];
    double du_p_new=ancomx;//du_pitch;//1*ancomx+fspb3;
	 //double du_p_new=ancomx-fspb3;
	//du_p_new = limit(du_p_new, 2*q, -2*q);
	//out_integ_p=integrate(du_p_new,du_p,out_integ_p,int_step);
	
	out_integ_p += du_p_new *0.01;
	out_integ_p = limit(out_integ_p, .35, -.35);
	du_p=du_p_new;
	double dqc=0*-gainfb1*(-fspb3)-K_w_acc*qqcx+K_i_acc*out_integ_p+K_p_acc*du_p;
    double delecx=dqc;//*DEG;

	////diagnostic output
	//GAINFP.build_vec3(gainfb1,gainfb2,gainfb3);
	////--------------------------------------------------------------------------
	////loading module-variables
	////state variables
	//hyper[516].gets(zzd);
	//hyper[517].gets(zz);
	////diagnostics
	//hyper[524].gets_vec(GAINFP);


		   if(t>621.)
		   t=t;

	return delecx;
}