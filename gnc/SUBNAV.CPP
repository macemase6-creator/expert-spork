#include"stdafx.h"
#include"M273N.H"
void twm(int mod,double m[3][3],double v[3],double mv[3])

/*  PORPUSE :   To  multiply  3 x 3  matrix m by 3 x 1 vector v if
		mode equal 1 and multiply transpose of 3 x 3 matrix m
		by 3 x 1 vector v if mode equal 2.


		mode=1 =====>     mv =  m * v

					 T
		mode=2 =====>     mv =  m * v

				    T    -1
		Note :             m  = m
*/
{
int i,j;

switch(mod)
 {
   case 1:

    for(i=0;i<3;i++)
     {
       mv[i]=0.0;
	for(j=0;j<3;j++)
	 mv[i]=mv[i]+m[i][j]*v[j];
     }
   break;
   case 2:

    for(i=0;i<3;i++)
    {
     mv[i]=0.0;
	for(j=0;j<3;j++)
	 mv[i]=mv[i]+m[j][i]*v[j];
    }
   break;

 }
}

 void q2m(double qq[4],double mm[3][3])

/*   PORPUSE :    To  calculate the coordinate transformation  matrix from
		   coordinate system "Z1" to the coordinate system "Z2",[m]
		   in  terms of quaternions [q].
*/
	   {
	   double q00,q01,q02,q03,q11,q12,q13,q22,q23,q33;

	   q00=qq[0]*qq[0];
	   q11=qq[1]*qq[1];
	   q22=qq[2]*qq[2];
	   q33=qq[3]*qq[3];

	   q01=qq[0]*qq[1];
	   q02=qq[0]*qq[2];
	   q03=qq[0]*qq[3];
	   q12=qq[1]*qq[2];
	   q13=qq[1]*qq[3];
	   q23=qq[2]*qq[3];

	  mm[0][0]=q00+q11-q22-q33;
	  mm[0][1]=2.0*(q12+q03)  ;
	  mm[0][2]=2.0*(q13-q02)  ;
	  mm[1][0]=2.0*(q12-q03)  ;
	  mm[1][1]=q00-q11+q22-q33;
	  mm[1][2]=2.0*(q01+q23)  ;
	  mm[2][0]=2.0*(q13+q02)  ;
	  mm[2][1]=2.0*(q23-q01)  ;
	  mm[2][2]=q00-q11-q22+q33;

	   }

void q1q2(double q1[4],double q2[4],double q12[4])

/*      PURPOSE :   To multiplication of quaternions q1 and q2.
*/

	    {
	   q12[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
	   q12[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
	   q12[2]=q1[0]*q2[2]+q1[2]*q2[0]+q1[3]*q2[1]-q1[1]*q2[3];
	   q12[3]=q1[0]*q2[3]+q1[3]*q2[0]+q1[1]*q2[2]-q1[2]*q2[1];

	   }

void e2m(double psi,double teta,double phi,double m[3][3])
/*   PORPUSE  :     This function calculates the coordinate
		    transformation in terms of Euler's angles .

       INPUT  :     Euler angles  named psi,teta,phi .

      OUTPUT  :     Transformation matrix .
*/
{
  double ct,st,cs,ss,cp,sp;

  cs=cos(psi);
  ss=sin(psi);
  ct=cos(teta);
  st=sin(teta);
  cp=cos(phi);
  sp=sin(phi);

  m[0][0]=ct*cs;
  m[0][1]=st;
  m[0][2]=-ct*ss  ;
  m[1][0]=-cp*st*cs+sp*ss ;
  m[1][1]=cp*ct ;
  m[1][2]=cp*st*ss+sp*cs ;
  m[2][0]=sp*st*cs+cp*ss ;
  m[2][1]=-sp*ct ;
  m[2][2]=-sp*st*ss+cp*cs ;
 }


void e2q(double teta,double psi,double phi,double q[4])
/*        PORPUSE :    To calculate the four quaternion parameters
		       expressed in terms of the three EULER angles
		       teta,psi,phi .


	  IMPORTANT NOTE :    phi is rotation angle about X_axis.
			      psi is rotation angle about Y_axis.
			      teta is rotation angle about Z_axis.

			      q=q(psi)*q(teta)*q(phi)
*/

	  {
	  double cs2,ct2,cp2,ss2,st2,sp2;

	  cs2 = cos(psi*0.5) ;
	  ct2 = cos(teta*0.5);
	  cp2 = cos(phi*0.5) ;
	  ss2 = sin(psi*0.5) ;
	  st2 = sin(teta*0.5);
	  sp2 = sin(phi*0.5) ;

	  q[0] = cs2*ct2*cp2 - ss2*st2*sp2 ;
	  q[1] = cs2*ct2*sp2 + ss2*st2*cp2 ;
	  q[2] = ss2*ct2*cp2 + cs2*st2*sp2 ;
	  q[3] = cs2*st2*cp2 - ss2*ct2*sp2 ;
	  }

void qi2f(double qif[4])

/*   External constants : d1,d2,d3   */

{
    double cwt2,swt2;

    cwt2=cos(0.5*wie*t);
    swt2=sin(0.5*wie*t);
/*   Calculate the quaternion from the coordinate system Zi to the coordinate
     system Zf . */

    qif[0]=cwt2;
    qif[1]=d1*swt2;
    qif[2]=d2*swt2;
    qif[3]=d3*swt2;
}



void fmb2c(double mb2c[3][3])
 {
 mb2c[0][0] =1.0 ;
 mb2c[0][1] =mb2c[0][2] =mb2c[1][0] =mb2c[2][0] =  0.0 ;
 mb2c[1][1] = cos(.785398163);
 mb2c[1][2] =-sin(.785398163);
 mb2c[2][2] = cos(.785398163);
 mb2c[2][1]= sin(.785398163);
  }

void Gacc(double ri[],double Gi[])

/*  PORPUSE :   To computation of the gravitational acceleration
		with respect to the firing  point inertial  co -
		ordinate  system .
*/
{
double Re[3],R,R2;
double c1,c12,c2,c3,GR,Gw;
//register i  ;

 for(int i=0;i<3;i++)
	 Re[i]=rf0[i]+ri[i];

 R=mag(3,Re);
 s1=Re[0]/R;
 s2=Re[1]/R;
 s3=Re[2]/R;

/*  Compute the latitude  of missile .          */

 sinlatm=s1*d1+s2*d2+s3*d3;
 R2=R*R;
 c1=re/R;
 c12=c1*c1;
 c2=Mu/R2;
 c3=J*c2*c12;

 GR = -c2 - c3*(1.0-5.0*sinlatm*sinlatm);
 Gw = -2.0*c3*sinlatm;
 Gi[0]=GR*s1 + Gw*d1 ;
 Gi[1]=GR*s2 + Gw*d2 ;
 Gi[2]=GR*s3 + Gw*d3 ;
}
void azimuth(double h1,double phi1,double lon1,double h2
	       ,double phi2,double lon2,double *paz,double *prange)

/*  PURPOSE   To compute the azimuth  *paz  from  point A1 with latitude
	      'phi1' and longitude 'lon1'  to  point  A2  with  latitude
	      'phi2' and longitude  'lon1'  longitude  'lon2',  and  the
	      geodesic distance *prange  from  A1 to  A2.   The  formula
	      is taken from page 137 of the  Geodecy  book  by  Bomford.
	      All angles are in radians, and all distances are in meters.

    By Mohammad-Ali Massoumnia
    10-Bahman-1366
									  */
{
 double azim(double h1,double lt1,double ln1,double h2,double lt2,double ln2);
 double sphi1, cphi1, sphi2, cphi2, dlon, cdlon, sdlon;
 double g2, hh, g, h, dis, nu1, nu2, tanpsi2, psi2, az;
 double ssigma, sigma, sigma2, sigma3, sigma4, sigma5;
 double E2,EPS;

	 E2=(A*A-B*B)/(A*A);
	 EPS=E2/(1.-E2);
	 sphi1 = sin(phi1);
	 cphi1 = cos(phi1);
	 sphi2 = sin(phi2);
	 cphi2 = cos(phi2);
	 dlon = lon2-lon1;
	 cdlon = cos(dlon);
	 sdlon = sin(dlon);

	nu1 = A/sqrt((1.-E2*sphi1*sphi1));
	nu2 = A/sqrt((1.-E2*sphi2*sphi2));
	tanpsi2 = (1.-E2)*sphi2/cphi2+E2*nu1*sphi1/nu2/cphi2;
	psi2 = atan(tanpsi2);

	az=azim(h1,phi1,lon1,h2,phi2,lon2);

	if (az<0.) az+=2.*pi;
	if (az>(2.*pi)) az-=2.*pi;

	if(fabs(sin(az)) >= 0.1)
	  ssigma = sdlon*cos(psi2)/sin(az);
	else
	  ssigma = cos(psi2)*(cphi1*tanpsi2-sphi1*cdlon)/cos(az);

	sigma = asin(ssigma);
	sigma2 = sigma*sigma;
	sigma3 = sigma*sigma2;
	sigma4 = sigma*sigma3;
	sigma5 = sigma*sigma4;
	g2 = EPS*sphi1*sphi1;
	hh = EPS*cphi1*cphi1*cos(az)*cos(az);
	g = sqrt(g2);
	h = sqrt(hh);
	dis = nu1*sigma*(1.-sigma2*hh*(1.-hh)/6.+sigma3*g*h*(1.-2.*hh)/8.
	 +sigma4*(hh*(4.-7.*hh)-3.*g2*(1.-7.*hh))/120.-sigma5*g*h/48.);
    *paz = az;
    *prange = dis;

	return;
}

double azim(double h1,double lat1,double lon1,double h2,double lat2,double lon2)
{
 double r1[3],r2[3],a,b,slat1,slon1,clon1,dx,dy,az;
 void llhxyz(double lat,double lon,double h,double r[]);
	llhxyz(lat1,lon1,h1,r1);
	llhxyz(lat2,lon2,h2,r2);
	dx=r2[0]-r1[0];
	dy=r2[1]-r1[1];
        slat1=sin(lat1);
        slon1=sin(lon1);
        clon1=cos(lon1);

        a=-slat1*clon1*dx-slat1*slon1*dy+cos(lat1)*(r2[2]-r1[2]);
	b=-slon1*dx+clon1*dy;

	az=atan2(b,a);
	if(az < 0.0) az+=2.*pi;
	return(az);
}
void llhxyz(double lat,double lon,double h,double r[3])

/*  PURPOSE     To convert the spheroidal coordinates (geodetic latitude,
	      geodetic longitude and height above the geoid) to the
	      cartesian coordinates (x, y and z).

  INPUTS      lat,long,h
  OUTPUT      r
  EXTERNALS   mag        */
{
	double E2,slat,clat,nu;

	  E2=(A*A-B*B)/(A*A);
	  slat=sin(lat);
	  clat=cos(lat);
	  nu=A/sqrt(1.-E2*slat*slat);
	  r[0]=(nu+h)*clat*cos(lon);
	  r[1]=(nu+h)*clat*sin(lon);
	  r[2]=((1.-E2)*nu+h)*slat;
}
void MATRIXf2e()
{
 mf2e[0][0] =-cos(At)*sin(Latl)*cos(Longl)-sin(At)*sin(Longl) ;
 mf2e[1][0] =-cos(At)*sin(Latl)*sin(Longl)+sin(At)*cos(Longl);
 mf2e[2][0] =cos(At)*cos(Latl);
 mf2e[0][1] =cos(Latl)*cos(Longl);
 mf2e[1][1] =cos(Latl)*sin(Longl);
 mf2e[2][1] =sin(Latl);
 mf2e[0][2] =sin(At)*sin(Latl)*cos(Longl)-cos(At)*sin(Longl);
 mf2e[1][2] =sin(At)*sin(Latl)*sin(Longl)+cos(At)*cos(Longl);
 mf2e[2][2] =-sin(At)*cos(Latl);

}
void m2q(double C[3][3],double q[4])
{
	double s,v[4],max;
	int j,i;
	s = C[0][0] + C[1][1] + C[2][2] ;
	v[0] = 1 + s;
	v[1] = 1 + 2*C[0][0] - s ;
	v[2] = 1 + 2*C[1][1] - s ;
	v[3] = 1 + 2*C[2][2] - s ;
	max = 0;
	j=0;
	for(i=0; i<4; i++)
	{
		if(v[i] > max)
		{
			max = v[i];
			j = i;
		}
	}
	v[j] = sqrt((v[j]>0.0?v[j]:0.0));
	if(j == 0)
	{
		v[1] = (C[2][1] - C[1][2]) / v[0];
		v[2] = (C[0][2] - C[2][0]) / v[0];
		v[3] = (C[1][0] - C[0][1]) / v[0];
	}
	if(j == 1)
	{
		v[0] = (C[2][1] - C[1][2] ) / v[1];
		v[2] = (C[1][0] + C[0][1] ) / v[1];
		v[3] = (C[0][2] + C[2][0] ) / v[1];
	}
	if(j == 2)
	{
		v[0] = (C[0][2] - C[2][0] ) / v[2];
		v[1] = (C[1][0] + C[0][1] ) / v[2];
		v[3] = (C[2][1] + C[1][2] ) / v[2];
	}
	if(j == 3)
	{
		v[0] = (C[1][0] - C[0][1] ) / v[3];
		v[1] = (C[0][2] + C[2][0] ) / v[3];
		v[2] = (C[2][1] + C[1][2] ) / v[3];
	}
   for(i=0; i<4; i++)
	v[i] = 0.5*v[i];
	if(v[0] < 0)
   {
	for(i=0; i<4; i++) v[i] = -1* v[i];
   }
   for(i=1; i<4; i++) v[i] = -1* v[i];
	q[0] = v[0];
	q[1] = v[1];
	q[2] = v[2];
	q[3] = v[3];
}
//===========================================================================

void nav2cart(double r_n[3],double r_e[3])
{
	double fi,lambda,h;
	double a=6378137.0;
	double b=6356752.3;
	fi = r_n[0]; lambda = r_n[1]; h = r_n[2];

	double N = a*a/sqrt(a*a*cos(fi)*cos(fi)+b*b*sin(fi)*sin(fi));

	double f = (a-b)/a;
	double e = sqrt(2*f-f*f);

	double x = (N+h)*cos(fi)*cos(lambda);
	double y = (N+h)*cos(fi)*sin(lambda);
	double z = (N*(1-e*e)+h)*sin(fi);


	r_e[0]=x;
	r_e[1]=y;
	r_e[2]=z;

}

//===========================================================================

void nav2cart(double lat,double lon,double h,double r_e[3])
{
	//double fi,lambda,h;
	double a=6378137.0;
	double b=6356752.3;
	//fi = r_n[0]; lambda = r_n[1]; h = r_n[2];
	//double r_e[3];

	double N = a*a/sqrt(a*a*cos(lat)*cos(lat)+b*b*sin(lat)*sin(lat));

	double f = (a-b)/a;
	double e = sqrt(2*f-f*f);

	 r_e[0] = (N+h)*cos(lat)*cos(lon);
	 r_e[1] = (N+h)*cos(lat)*sin(lon);
	 r_e[2] = (N*(1-e*e)+h)*sin(lat);

}
//===========================================================================

void nav2local(double r_n[3],double r_l0[3],double r_l[3])
{
	
	double fi,lambda,h;
	double a=6378137.0;
	double b=6356752.3;
	double r_e[3];
	fi = r_n[0]; lambda = r_n[1]; h = r_n[2];
	if(fi==0.0 || lambda==0.0)
	{
		for (int i = 0; i < 3; i++)	r_l[i]=0.;
		return;
	}

	double N = a*a/sqrt(a*a*cos(fi)*cos(fi)+b*b*sin(fi)*sin(fi));

	double f = (a-b)/a;
	double e = sqrt(2*f-f*f);

	 r_e[0] = (N+h)*cos(fi)*cos(lambda);
	 r_e[1] = (N+h)*cos(fi)*sin(lambda);
	 r_e[2] = (N*(1-e*e)+h)*sin(fi);

	for (int i = 0; i < 3; i++)	
		r_e[i] = r_e[i] - r_l0[i];

	twm(2, mf2e, r_e, r_l);

}

//===========================================================================

void nav2local(double lat,double lon,double h,double r_l0[3],double r_l[3])
{
	
	//double fi,lambda,h;
	double a=6378137.0;
	double b=6356752.3;
	double r_e[3];
	//fi = r_n[0]; lambda = r_n[1]; h = r_n[2];

	if(lat==0.0 || lon==0.0)
	{
		for (int i = 0; i < 3; i++)	r_l[i]=0.;
		return;
	}

	double N = a*a/sqrt(a*a*cos(lat)*cos(lat)+b*b*sin(lat)*sin(lat));

	double f = (a-b)/a;
	double e = sqrt(2*f-f*f);

	 r_e[0] = (N+h)*cos(lat)*cos(lon);
	 r_e[1] = (N+h)*cos(lat)*sin(lon);
	 r_e[2] = (N*(1-e*e)+h)*sin(lat);

	for (int i = 0; i < 3; i++)	
		r_e[i] = r_e[i] - r_l0[i];

	twm(2, mf2e, r_e, r_l);

}

mat3x1 nav2local(mat3x1 r_n,mat3x1 rl0)
{
	
	mat3x1 rl,re;

	if(r_n(0,0)==0.0 || r_n(1,0)==0.0)
	{
		return 0;
	}

	re = nav2cart( r_n);
	re = re - rl0;

	rl = ~C_e_l * re;

}

void  imperr(double rl[3], double rm[3], double ht, double latt,double longt, double erre[3])
{
	/*
	PURPOSE     To compute the components of Impact Error in the 'E' frame .
	INPUTS      rl,rimp(coordinates of the launnch & impact points in the
	'C' frame. height (ht), the geodetic latitude and longitude
	of the target (latt,longt) .
	OUTPUTS     components of Impact Error in the 'E' frame .
	EXTERNALS   llhxyz,dot,mag,cross
	*/
	double ux[3], uy[3], uz[3], url[3], urt[3], magux, rt[3], errc[3], magrt, magrl;
	int i;

	// Find the target psition in "C" frame.
	//llh2xyz(latt, longt, ht, 0.0, rt);
	double r_n[3];
	r_n[0]=latt;	r_n[1]=longt;	r_n[2]=ht;	
	 nav2cart( r_n, rt);

	// Find the basis of 'E'frame in 'C' frame.
	magrt = mag(3, rt);
	magrl = mag(3, rl);
	for (i = 0; i < 3; i++)
	{
		urt[i] = rt[i] / magrt;
		url[i] = rl[i] / magrl;
	}
	cross(urt, url, uy);

	//   Find the unit vector (uz) perpendicular to the reference spheroid at the target point
	uz[0] = cos(latt) * cos(longt);
	uz[1] = cos(latt) * sin(longt);
	uz[2] = sin(latt);

	cross(uy, uz, ux);
	magux = mag(3, ux);
	for (i = 0; i < 3; i++)
		ux[i] = ux[i] / magux;

	cross(uz, ux, uy);

	//   Find the components of the error in 'C' frame.
	for (i = 0; i < 3; i++)
		errc[i] = rm[i] - rt[i];

	//   Find the components of the error in 'E' frame.
	erre[0] = dot(3, errc, ux);
	erre[1] = dot(3, errc, uy);
	erre[2] = dot(3, errc, uz);

}
mat3x1 imperr(mat3x1 rl, mat3x1 rmn, mat3x1 rtn)
{
	/*
	PURPOSE     To compute the components of Impact Error in the 'E' frame .
	INPUTS      rl,rimp(coordinates of the launnch & impact points in the
	'C' frame. height (ht), the geodetic latitude and longitude
	of the target (latt,longt) .
	OUTPUTS     components of Impact Error in the 'E' frame .
	EXTERNALS   llhxyz,dot,mag,cross
	*/
	int i;

	// Find the target psition in "C" frame.
	mat3x1 rm,rt,urt,url,ux,uy,uz,errc,erre;	
	rt=nav2cart(rtn);
	rm=nav2cart(rmn);

	// Find the basis of 'E'frame in 'C' frame.
	urt = (1./ rt.Norm()) * rt;
	url = (1. / rl.Norm()) * rl;
	uy = urt ^ url;

	//   Find the unit vector (uz) perpendicular to the reference spheroid at the target point
	uz(0,0) = cos(rtn(0,0)) * cos(rtn(1,0));
	uz(1,0) = cos(rtn(0,0)) * sin(rtn(1,0));
	uz(2,0) = sin(rtn(0,0));

	ux = uy ^ uz;
	ux =  (1. / ux.Norm()) *ux;
	uy = uz ^ ux;

	//   Find the components of the error in 'C' frame.
	errc = rm - rt;

	//   Find the components of the error in 'E' frame.
	erre(0,0) = (errc* ux);
	erre(1,0) = (errc* uy);
	erre(2,0) = (errc* uz);

	return erre;
}
mat3x1 imperr(mat3x1 rl, mat3x1 rmn, double h, double lat,double lon)
{
	/*
	PURPOSE     To compute the components of Impact Error in the 'E' frame .
	INPUTS      rl,rimp(coordinates of the launnch & impact points in the
	'C' frame. height (ht), the geodetic latitude and longitude
	of the target (latt,longt) .
	OUTPUTS     components of Impact Error in the 'E' frame .
	EXTERNALS   llhxyz,dot,mag,cross
	*/
	//double /*ux[3], uy[3], uz[3], url[3], urt[3],*/ magux,  magrt, magrl;
	int i;

	// Find the target psition in "C" frame.
	//llh2xyz(latt, longt, ht, 0.0, rt);
	mat3x1 rtn,rm,rt,urt,url,ux,uy,uz,errc,erre;

	rtn(0,0)=lat;	rtn(1,0)=lon;	rtn(2,0)=h;	
	rt=nav2cart(rtn);
	rm=nav2cart(rmn);

	// Find the basis of 'E'frame in 'C' frame.
	urt = (1./ rt.Norm()) * rt;
	url = (1. / rl.Norm()) * rl;
	uy = urt ^ url;

	//   Find the unit vector (uz) perpendicular to the reference spheroid at the target point
	uz(0,0) = cos(rtn(0,0)) * cos(rtn(1,0));
	uz(1,0) = cos(rtn(0,0)) * sin(rtn(1,0));
	uz(2,0) = sin(rtn(0,0));

	ux = uy ^ uz;
	ux =  (1. / ux.Norm()) *ux;
	uy = uz ^ ux;

	//   Find the components of the error in 'C' frame.
	errc = rm - rt;

	//   Find the components of the error in 'E' frame.
	erre(0,0) = (errc* ux);
	erre(1,0) = (errc* uy);
	erre(2,0) = (errc* uz);

	return erre;
}
///////
void Rot_e_n(double phi, double lam,double C_e_n[3][3])
{
	double slam=sin(lam), clam=cos(lam);
	double sfi=sin(phi),  cfi=cos(phi);

	C_e_n[0][0] = -sfi*clam;
	C_e_n[0][1] = -sfi*slam;
	C_e_n[0][2] =  cfi;
	C_e_n[1][0] = -slam;
	C_e_n[1][1] = clam;
	C_e_n[1][2] = 0;
	C_e_n[2][0] = -cfi*clam;
	C_e_n[2][1] = -cfi*slam;
	C_e_n[2][2] = -sfi;

}
////////////////////////////////
mat3x3 Rot_e_n(double phi, double lam)
{
	mat3x3 C_e_n;

	double slam=sin(lam), clam=cos(lam);
	double sfi=sin(phi),  cfi=cos(phi);

	C_e_n(0,0) = -sfi*clam;
	C_e_n(0,1) = -sfi*slam;
	C_e_n(0,2) =  cfi;
	C_e_n(1,0) = -slam;
	C_e_n(1,1) = clam;
	C_e_n(1,2) = 0;
	C_e_n(2,0) = -cfi*clam;
	C_e_n(2,1) = -cfi*slam;
	C_e_n(2,2) = -sfi;

	return C_e_n;
}
mat3x3 Rot_e_l(double psi0, double phi0, double lambda0)
{
	mat3x3 C_lp_l(0);
	C_lp_l.Unit();
	C_lp_l(0,0) = cos(psi0);
	C_lp_l(0,2) = sin(psi0);
	C_lp_l(2,0) = -sin(psi0);
	C_lp_l(2,2) = cos(psi0);

	mat3x3 C_n0_lp(0);
	C_n0_lp(0,0)=1;
	C_n0_lp(1,2)=-1;
	C_n0_lp(2,1)=1;

	mat3x3 C_e_n0;
	C_e_n0 = Rot_e_n(phi0, lambda0);

	return C_lp_l * C_n0_lp * C_e_n0;
}
mat3x3 Rot_b_n(mat3x1 Angles)
{
	double f = Angles(0,0), t = Angles(1,0), s = Angles(2,0);
	mat3x3 C_b_n;

	C_b_n(0,0) = cos(t)*cos(s);
	C_b_n(0,1) = sin(f)*sin(t)*cos(s)-cos(f)*sin(s);
	C_b_n(0,2) = cos(f)*sin(t)*cos(s)+sin(f)*sin(s);
	C_b_n(1,0) = cos(t)*sin(s);
	C_b_n(1,1) = sin(f)*sin(t)*sin(s)+cos(f)*cos(s);
	C_b_n(1,2) = cos(f)*sin(t)*sin(s)-sin(f)*cos(s);
	C_b_n(2,0) = -sin(t);
	C_b_n(2,1) = sin(f)*cos(t);
	C_b_n(2,2) = cos(f)*cos(t);

	return C_b_n;
}
//void nav2cart(double r_n[3],double r_e[3])
//{
//	double fi,lambda,h;
//	double a=6378137.0;
//	double b=6356752.3;
//	fi = r_n[0]; lambda = r_n[1]; h = r_n[2];
//
//	double N = a*a/sqrt(a*a*cos(fi)*cos(fi)+b*b*sin(fi)*sin(fi));
//
//	double f = (a-b)/a;
//	double e = sqrt(2*f-f*f);
//
//	double x = (N+h)*cos(fi)*cos(lambda);
//	double y = (N+h)*cos(fi)*sin(lambda);
//	double z = (N*(1-e*e)+h)*sin(fi);
//
//
//	r_e[0]=x;
//	r_e[1]=y;
//	r_e[2]=z;
//
//}
mat3x1 nav2cart(mat3x1 r_n)
{
	double fi,lambda,h;
	double a=EARTH_RADIUS_A;
	double b=EARTH_RADIUS_B;
	fi = r_n(0,0); lambda = r_n(1,0); h = r_n(2,0);

//	double c = a*a*cos(fi)*cos(fi)+b*b*sin(fi)*sin(fi);
	double N = a*a/sqrt(a*a*cos(fi)*cos(fi)+b*b*sin(fi)*sin(fi));

	double f = (a-b)/a;
	double e = sqrt(2*f-f*f);

	double x = (N+h)*cos(fi)*cos(lambda);
	double y = (N+h)*cos(fi)*sin(lambda);
	double z = (N*(1-e*e)+h)*sin(fi);

	mat3x1 r_e;
	r_e(0,0)=x;
	r_e(1,0)=y;
	r_e(2,0)=z;

	return r_e;
}
mat3x1 nav2cart(double fi , double lambda , double h)
{
	mat3x1 r_n;
	r_n(0,0)=fi;  
	r_n(1,0) = lambda;  
	r_n(2,0)=h;

	return nav2cart(r_n);
}
void Quat2Euler_n(double q[4],double *s,double *t,double *f)
{
  double q0,q1,q2,q3,bb;
  q0=q[0]; q1=q[1]; q2=q[2]; q3=q[3];
  bb=2*q1*q3-2*q0*q2;
  if(fabs(bb)>1)//sh
  bb=1.*sign(bb);

  *f = atan2((2*q2*q3+2*q0*q1),(q0*q0-q1*q1-q2*q2+q3*q3));
  *t = -asin(bb);
  *s = atan2((2*q0*q3+2*q1*q2),(q0*q0+q1*q1-q2*q2-q3*q3));

}



mat3x1 cart2nav(mat3x1 r_vec)
{
	double x,y,z;
	x = r_vec(0,0); y = r_vec(1,0); z = r_vec(2,0);

	double lambda = atan2(y,x);

	double f      = (EARTH_RADIUS_A-EARTH_RADIUS_B)/EARTH_RADIUS_A;
	double e      = sqrt(2*f-f*f);
	double s      = sqrt(x*x+y*y);

	double beta   = atan(z/s/(1-f));

	double mu=0, mu_old=1,h,N;

//	while (fabs(mu_old-mu)>1e-25)
	{
		mu_old = mu;
		mu     = atan((z+e*e*(1-f)/(1-e*e)*EARTH_RADIUS_A*pow(sin(beta),3))/(s-e*e*EARTH_RADIUS_A*pow(cos(beta),3)));

		N      = EARTH_RADIUS_A/sqrt(1-e*e*(pow(sin(mu),2)));
		h      = s*cos(mu)+(z+e*e*N*sin(mu))*sin(mu)-N;
		beta   = atan((1-f)*sin(mu)/cos(mu));

		mu     = atan((z+e*e*(1-f)/(1-e*e)*EARTH_RADIUS_A*pow(sin(beta),3))/(s-e*e*EARTH_RADIUS_A*pow(cos(beta),3)));

		N      = EARTH_RADIUS_A/sqrt(1-e*e*(pow(sin(mu),2)));
		h      = s*cos(mu)+(z+e*e*N*sin(mu))*sin(mu)-N;
		beta   = atan((1-f)*sin(mu)/cos(mu));

		mu     = atan((z+e*e*(1-f)/(1-e*e)*EARTH_RADIUS_A*pow(sin(beta),3))/(s-e*e*EARTH_RADIUS_A*pow(cos(beta),3)));
	}

	mat3x1 r_n;
	r_n(0,0) = mu;
	r_n(1,0) = lambda;
	r_n(2,0) = h;

	return r_n;
}
///////////////////////////////////////////////////////////////////////////////
//Returns polar from cartesian coordinates
// magnitude = POLAR[0] = |V|
// azimuth   = POLAR[1] = atan2(V2,V1)
// elevation = POLAR[2] = atan2(-V3,sqrt(V1^2+V2^2)
//Example: POLAR = VEC.pol_from_cart();
///////////////////////////////////////////////////////////////////////////////
mat3x1 pol_from_cart( mat3x1 cart) 
{
	double d=0;
	double azimuth=0;
	double elevation=0;
	double denom=0;
	mat3x1 POLAR(0);
	
	double v1=cart(0,0);
	double v2=cart(1,0);
	double v3=cart(2,0);

	d=sqrt(v1*v1+v2*v2+v3*v3);
	azimuth=atan2(v2,v1);

	denom=sqrt(v1*v1+v2*v2);
	if(denom>0.)
		elevation=atan2(-v3,denom);
	else{
		if(v3>0) elevation=-PI/2.;
		if(v3<0) elevation=PI/2.;
		if(v3==0) elevation=0.;
	}
	
	POLAR(0,0)=d;
	POLAR(1,0)=azimuth;
	POLAR(2,0)=elevation;

	return POLAR;
}

 void vreckon(double lat,double lon,double dist_m,double bearing_rad, double points[3])
{
	double a,f,b;
	double phi1,L1,alpha1,s,U1,sigma1,sinalpha,cossq_alpha,usq;
	double sigma,lastsigma,_2sigma,dsigma;
	double sqr_sigma_sin,cos_2sigma2;
	double numm,den,lat2,lon2,lmbda,C,L,A21;
	double ss,AA,BB;
	double accuracy=1.0E-12;
	a = A;//6378137.0;        // semi-major axis (km), WGS84
	f = 1./298.257223563; // flattening of the ellipsoid, WGS84
	b = (1-f)*a;          // semi-minor axis

	phi1 = (lat);
	L1   = (lon);
	alpha1 = bearing_rad;//*d2r;
	s = dist_m;///1000.;
	
	if(phi1==0||L1==0||s==0) return;//ADS18

	U1 = atan((1-f)*tan(phi1));

	sigma1 = atan2(tan(U1), cos(alpha1));

	sinalpha = cos(U1)*sin(alpha1);
	cossq_alpha = (1. - sinalpha*sinalpha);
	usq = cossq_alpha*(a*a-b*b)/(b*b);

	AA = 1 + usq/16384. * (4096. + usq*(-768 + usq*(320.-175.*usq)));
	BB = usq/1024.*(256. + usq*(-128. + usq*(74.-47.*usq)));

	sigma = s/(b*AA);
	lastsigma = 1.E40;
	sqr_sigma_sin = sin(sigma)*sin(sigma);
	cos_2sigma2 =cos(_2sigma)*cos(_2sigma);
	int count=0;
	do
	{
		count++;
		lastsigma = sigma;
		_2sigma = 2.*sigma1 + sigma;
		dsigma = (BB*sin(sigma)*(cos(_2sigma) + 0.25*BB *(cos(sigma)*(-1. + 2.*cos_2sigma2) - (1./6.)*BB*cos(_2sigma)* (-3. + 4. *sqr_sigma_sin)* (-3. + 4.*cos_2sigma2))));
		sigma = s/(b*AA) + dsigma;
		ss = fabs(sigma - lastsigma);
	} while (fabs(sigma - lastsigma) > accuracy && count<5);
	
	numm = sin(U1)*cos(sigma) + cos(U1)*sin(sigma)*cos(alpha1);
	den = (1.-f)*sqrt(sinalpha*sinalpha + (sin(U1)*sin(sigma) - cos(U1)*cos(sigma)*cos(alpha1))*(sin(U1)*sin(sigma) - cos(U1)*cos(sigma)*cos(alpha1)));

	lat2 = atan2(numm, den);

	numm = sin(sigma)*sin(alpha1);
	den = cos(U1)*cos(sigma) - sin(U1)*sin(sigma)*cos(alpha1);
	lmbda = atan2(numm, den);

	C = (f/16.)*cossq_alpha*(4. + f*(4. - 3.*cossq_alpha));

	L = (lmbda - (1. - C)*f*sinalpha * (sigma + C*sin(sigma)*(cos(_2sigma) + C*cos(sigma)* (-1. + 2.*cos_2sigma2))));
	lon2 = L + L1;

	numm = sinalpha;
	den = -1*sin(U1)*sin(sigma) + cos(U1)*cos(sigma)*cos(alpha1);
	A21 = atan2(numm, den);
	A21 = (A21 + 3.*pi); // (2.*pi)

	points[0] = lat2;
	points[1] = lon2;
	points[2] = 0;
	//int bearing2 = (A21%(360*d2r));
	//out[2] = (A21%(360*d2r));//mod(rad2deg(A21), 360);
}

 void vreckon(double lat1,double lon1,double dist_m,double bearing_rad, double *lat2,double *lon2)
{
	double a,f,b;
	double phi1,L1,alpha1,s,U1,sigma1,sinalpha,cossq_alpha,usq;
	double sigma,lastsigma,_2sigma,dsigma;
	double sqr_sigma_sin,cos_2sigma2;
	double numm,den,/*lat2,lon2,*/lmbda,C,L,A21;
	double ss,AA,BB;
	double accuracy=1.0E-12;
	a = A;//6378137.0;        // semi-major axis (km), WGS84
	f = 1./298.257223563; // flattening of the ellipsoid, WGS84
	b = (1-f)*a;          // semi-minor axis

	phi1 = (lat1);
	L1   = (lon1);
	alpha1 = bearing_rad;//*d2r;
	s = dist_m;///1000.;
	
	if(phi1==0||L1==0||s==0) return;//ADS18

	U1 = atan((1-f)*tan(phi1));

	sigma1 = atan2(tan(U1), cos(alpha1));

	sinalpha = cos(U1)*sin(alpha1);
	cossq_alpha = (1. - sinalpha*sinalpha);
	usq = cossq_alpha*(a*a-b*b)/(b*b);

	AA = 1 + usq/16384. * (4096. + usq*(-768 + usq*(320.-175.*usq)));
	BB = usq/1024.*(256. + usq*(-128. + usq*(74.-47.*usq)));

	sigma = s/(b*AA);
	lastsigma = 1.E40;
	sqr_sigma_sin = sin(sigma)*sin(sigma);
	cos_2sigma2 =cos(_2sigma)*cos(_2sigma);
	int count=0;
	do
	{
		count++;
		lastsigma = sigma;
		_2sigma = 2.*sigma1 + sigma;
		dsigma = (BB*sin(sigma)*(cos(_2sigma) + 0.25*BB *(cos(sigma)*(-1. + 2.*cos_2sigma2) - (1./6.)*BB*cos(_2sigma)* (-3. + 4. *sqr_sigma_sin)* (-3. + 4.*cos_2sigma2))));
		sigma = s/(b*AA) + dsigma;
		ss = fabs(sigma - lastsigma);
	} while (fabs(sigma - lastsigma) > accuracy && count<5);
	
	numm = sin(U1)*cos(sigma) + cos(U1)*sin(sigma)*cos(alpha1);
	den = (1.-f)*sqrt(sinalpha*sinalpha + (sin(U1)*sin(sigma) - cos(U1)*cos(sigma)*cos(alpha1))*(sin(U1)*sin(sigma) - cos(U1)*cos(sigma)*cos(alpha1)));

	*lat2 = atan2(numm, den);

	numm = sin(sigma)*sin(alpha1);
	den = cos(U1)*cos(sigma) - sin(U1)*sin(sigma)*cos(alpha1);
	lmbda = atan2(numm, den);

	C = (f/16.)*cossq_alpha*(4. + f*(4. - 3.*cossq_alpha));

	L = (lmbda - (1. - C)*f*sinalpha * (sigma + C*sin(sigma)*(cos(_2sigma) + C*cos(sigma)* (-1. + 2.*cos_2sigma2))));
	*lon2 = L + L1;

	numm = sinalpha;
	den = -1*sin(U1)*sin(sigma) + cos(U1)*cos(sigma)*cos(alpha1);
	A21 = atan2(numm, den);
	A21 = (A21 + 3.*pi); // (2.*pi)

	//points[0] = lat2;
	//points[1] = lon2;
	//points[2] = 0;
	//int bearing2 = (A21%(360*d2r));
	//out[2] = (A21%(360*d2r));//mod(rad2deg(A21), 360);
}

  
void traject(double lat1, double long1, double alt1, double Az, double R, double x_er, double z_er,double *lat22, double *long22)
{

double f, a, e, r1 ;
double x1, y1, z1, N;
mat3x3 ecef2enu,f2enu;
double x2, y2, z2;
double alfa1, R1, teta1 ;
double s,b,bv, uv ;
double s2, b2,bv2,uv2;
double lat2, long2;
double alfa2, R2, teta2;
double x22, y22, z22 ;

double r_e[3],Re[3],rn[3];
double aa=6378137.0;
double bb=6356752.3;

double ff = (aa-bb)/aa;
double ee = sqrt(2*f-f*f);


f=ff;//1/298.25722356;
a=aa;//6378137;
e=ee;//8.1819190842622e-2;

	a = A;//6378137.0;        // semi-major axis (km), WGS84
	f = 1./298.257223563; // flattening of the ellipsoid, WGS84
	b = (1-f)*a;          // semi-minor axis

r1=a*(1-f*sin(lat1)*sin(lat1));
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ecef frame %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N=a/sqrt(1-e*e*sin(lat1)*sin(lat1));
 //N = aa*aa/sqrt(aa*aa*cos(lat1)*cos(lat1)+b*b*sin(lat1)*sin(lat1));

x1=(N+alt1)*cos(lat1)*cos(long1);
y1=(N+alt1)*cos(lat1)*sin(long1);
z1=((1-e*e)*N+alt1)*sin(lat1);

 //nav2cart(lat1, long1, alt1, r_e);
 //x1=r_e[0];	 y1=r_e[1];	 z1=r_e[2];
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ecef2enu matrix %%%%%%%%%%%%%%%%%%%%%%%%%%%%
ecef2enu(0,0)=-sin(long1);
ecef2enu(0,1)=	cos(long1);
ecef2enu(0,2)=	 0. ;
ecef2enu(1,0)=	-sin(lat1)*cos(long1);
ecef2enu(1,1)=  -sin(lat1)*sin(long1);
ecef2enu(1,2)=  cos(lat1) ;
ecef2enu(2,0)=  cos(lat1)*cos(long1);
ecef2enu(2,1)=  cos(lat1)*sin(long1);
ecef2enu(2,2)=  sin(lat1);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% point 2 in firing frame %%%%%%%%%%%%%%%%%%%%

alfa1=R/r1;
R1=sqrt(2*r1*r1-2*r1*r1*cos(alfa1));
teta1=pi/2-acos(R1/(2*r1));

x2=R1*cos(teta1);
y2=-R1*sin(teta1);
z2=0;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% firing2enu matrix %%%%%%%%%%%%%%%%%%%%%%%%%%
f2enu(0,0) = sin(Az);
f2enu(0,1) = 0. ;
f2enu(0,2) = cos(Az);
f2enu(1,0) = cos(Az);
f2enu(1,1) = 0. ;
f2enu(1,2) = -sin(Az);
f2enu(2,0) = 0. ;
f2enu(2,1) = 1. ;
f2enu(2,2) = 0. ;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% point 2 in ecef frame %%%%%%%%%%%%%%%%%%%%%%
mat3x1 xyz1, Y1, xyz2,Y2, h, h2;
xyz1(0,0) = x2 ;
xyz1(0,1) = y2 ;
xyz1(0,2) = z2 ;

Y1 = (~ecef2enu) * f2enu * (xyz1);
h(0,0) = Y1(0,0)+x1;
h(0,1) = Y1(1,0)+y1;
h(0,2) = Y1(2,0)+z1;
Mat2Arr(h,Re);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% clculate lat & long for point 2 init %%%%%%%
s=sqrt(h(0,0)*h(0,0)+h(1,0)*h(1,0));
b=atan2(h(2,0),((1-f)*s));

for(int i=0;i<10;i++)
{
    lat2=atan2((h(2,0)+(e*e*(1-f)*a*sin(b)*sin(b)*sin(b))/(1-e*e)),(s-e*e*a*cos(b)*cos(b)*cos(b)));
    bv=atan((1-f)*tan(lat2));
    b=bv;
    if(i==1) uv=lat2;
    if((i!=1) && ((uv-lat2)<1e-10))
    break;
}
long2=atan2(h(1,0),h(0,0));

 //NED_function( Re, rn) ;
 //lat2=rn[0];	 long2=rn[1];	
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%% point with error in firing frame %%%%%%%%%%%%
alfa2=(R+x_er)/r1;
R2=sqrt(2*r1*r1-2*r1*r1*cos(alfa2));
teta2=pi/2-acos(R2/(2*r1));

xyz2(0,0) = R2*cos(teta2);
xyz2(1,0) = -R2*sin(teta2);
xyz2(2,0) = z_er;
//%%%%%%%%%%%%%%%%%%%% point with error in ecef frame %%%%%%%%%%%%%%%%%%%%%%%
Y2 = (~ecef2enu) * f2enu * xyz2;
h2(0,0)= Y2(0,0)+x1;
h2(1,0)= Y2(1,0)+y1;
h2(2,0)= Y2(2,0)+z1;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%% calculate lat & long init of point with error
s2=sqrt(h2(0,0)*h2(0,0)+h2(1,0)*h2(1,0));
b2=atan2(h2(2,0),((1-f)*s2));

for(int i=0; i<10 ;i++)
  {
  *lat22=atan2((h2(2,0)+(e*e*(1-f)*a*sin(b2)*sin(b2)*sin(b2))/(1-e*e)),(s2-e*e*a*cos(b2)*cos(b2)*cos(b2)));
  bv2=atan((1-f)*tan(*lat22));
  b2=bv2;

  if(i==1)
   uv2=*lat22;

  if((i!=1) && ((uv2-*lat22)<1e-10))
   break;
  }
*long22=atan2(h2(1,0),h2(0,0));

return;
}

void Ft_Convert(double Launch_lat,double Launch_lon, double L_virtual,double Aztv,double total_t,double *lat2,double *lon2)
{
  double lVar2,Ttotv,latVTarget,longVTarget;
  double rti0 [3];
  double crange,srange,caztv,saztv,clatl,slatl, local_12, range;
  double A = 6378137.0;        /* Earth Semi-Major axis (m) */
  
  Ttotv = total_t;//Ttotv
  range = (L_virtual * 1000.0) / A;
  slatl=sin(Launch_lat);
  clatl=cos(Launch_lat);
  saztv=sin(Aztv);
  caztv=cos(Aztv);
  srange=sin(range);
  crange=cos(range);


  latVTarget=asin(clatl * srange * caztv + slatl * crange);
  //latVTarget=asin(cos(lat) * sin(range) * cos(aztv) + sin(lat) * cos(range),0));
  lVar2=asin((srange * saztv / cos(latVTarget)));
  local_12 =  wie* Ttotv + lVar2;
  longVTarget = Launch_lon + local_12;

  *lat2 = latVTarget;
  *lon2 = longVTarget;
 // llh2xyz(latVTarget,longVTarget,Terminal_h,0.0,rti0);
 // twc(2,cg2i,rti0,rTvg);
 // R2 = mag(3,rTvg);
  return;
}

void MATRIXAz2n(double Az)
{
 mAz2n(0,0) =cos(Az) ; mAz2n(1,0) =-sin(Az); mAz2n(2,0) =0.;
 mAz2n(0,1) =sin(Az); mAz2n(1,1) =cos(Az); mAz2n(2,1) =0.;
 mAz2n(0,2) =0.; mAz2n(1,2)= 0.; mAz2n(2,2) =1. ;
}
void MATRIXe2n(double L,double l)
{
 me2n(0,0) =-sin(L)*cos(l) ;
 me2n(1,0) =-sin(l);
 me2n(2,0) =-cos(L)*cos(l);
 me2n(0,1) =-sin(L)*sin(l);
 me2n(1,1) = cos(l);
 me2n(2,1) = -cos(L)*sin(l);
 me2n(0,2) =cos(L);
 me2n(1,2)= 0;
 me2n(2,2) =-sin(L) ;

}
//------------------------------------

mat3x1 Quat2Euler_l(mat4x1 q)
{
	double q0,q1,q2,q3,aa;
	q0=q(0,0); q1=q(1,0); q2=q(2,0); q3=q(3,0);
	//	double f = atan2(-R(1,2),R(1,1));
	//	double t = asin(R(1,0));
	//	double s = atan2(-R(2,0),R(0,0));
	//in tabdil motabeghe titerton hast
	aa=2*(q1*q2+q0*q3);
	if(fabs(aa)>1)//sh
  aa=1.*sign(aa);

	double f = atan2(-2*(q2*q3-q0*q1),q0*q0-q1*q1+q2*q2-q3*q3);
//	double t = asin(2*(q1*q2+q0*q3));
	double t = asin(aa);
	double s = atan2(-2*(q1*q3-q0*q2),q0*q0+q1*q1-q2*q2-q3*q3);

	mat3x1 Angles;
	Angles(0,0) = f;
	Angles(1,0) = t;
	Angles(2,0) = s;

	return Angles;
}

mat3x1 Quat2Euler_n(mat4x1 q)
{
	double q0,q1,q2,q3,bb;
	q0=q(0,0); q1=q(1,0); q2=q(2,0); q3=q(3,0);
 bb=2*q1*q3-2*q0*q2;
 if(fabs(bb)>1)//sh
  bb=1.*sign(bb);

	double f = atan2((2*q2*q3+2*q0*q1),(q0*q0-q1*q1-q2*q2+q3*q3));
//	double t = -asin((2*q1*q3-2*q0*q2));
	double t = -asin(bb);
	double s = atan2((2*q0*q3+2*q1*q2),(q0*q0+q1*q1-q2*q2-q3*q3));

	mat3x1 Angles;
	Angles(0,0) = f;
	Angles(1,0) = t;
	Angles(2,0) = s;

	return Angles;
}



mat3x3 Quat2RotMat(mat4x1 q)
{
	double q0,q1,q2,q3;
	mat3x3 R;
	q0=q(0,0); q1=q(1,0); q2=q(2,0); q3=q(3,0);

	R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
	R(0,1) = 2*q1*q2-2*q0*q3;
	R(0,2) = 2*q0*q2+2*q1*q3;
	R(1,0) = 2*q0*q3+2*q1*q2;
	R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
	R(1,2) = 2*q2*q3-2*q0*q1;
	R(2,0) = 2*q1*q3-2*q0*q2;
	R(2,1) = 2*q2*q3+2*q0*q1;
	R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;

	return R;
}

mat4x1 RotMat2Quat(mat3x3 R)
{
	double tr = R(0,0) + R(1,1) + R(2,2);
	double S,qw,qx,qy,qz;

	if (tr > 0) {
		S = sqrt(tr+1.0) * 2; // S=4*qw
		qw = 0.25 * S;
		qx = (R(2,1) - R(1,2)) / S;
		qy = (R(0,2) - R(2,0)) / S;
		qz = (R(1,0) - R(0,1)) / S;
	}
	else {
	if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
		S = sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2; // S=4*qx
		qw = (R(2,1) - R(1,2)) / S;
		qx = 0.25 * S;
		qy = (R(0,1) + R(1,0)) / S;
		qz = (R(0,2) + R(2,0)) / S;
	}
	else {
	if (R(1,1) > R(2,2)) {
		S = sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2; // S=4*qy
		qw = (R(0,2) - R(2,0)) / S;
		qx = (R(0,1) + R(1,0)) / S;
		qy = 0.25 * S;
		qz = (R(1,2) + R(2,1)) / S;
	}
	else {
		S = sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2; // S=4*qz
		qw = (R(1,0) - R(0,1)) / S;
		qx = (R(0,2) + R(2,0)) / S;
		qy = (R(1,2) + R(2,1)) / S;
		qz = 0.25 * S;
	}}}
	mat4x1 q;

	if (qw > 0) {
		q(0,0) = qw;
		q(1,0) = qx;
		q(2,0) = qy;
		q(3,0) = qz;
	} else {
		q(0,0) = -qw;
		q(1,0) = -qx;
		q(2,0) = -qy;
		q(3,0) = -qz;
	}

	return q;
}


mat3x3 Rot_b_l(mat3x1 Angles)
{
	double f = Angles(0,0), t = Angles(1,0), s = Angles(2,0);
	mat3x3 C_b_l;

	C_b_l(0,0) =  cos(t)*cos(s);
	C_b_l(0,1) = -cos(f)*sin(t)*cos(s)+sin(f)*sin(s);
	C_b_l(0,2) =  sin(f)*sin(t)*cos(s)+cos(f)*sin(s);
	C_b_l(1,0) =  sin(t);
	C_b_l(1,1) =  cos(f)*cos(t);
	C_b_l(1,2) = -sin(f)*cos(t);
	C_b_l(2,0) = -cos(t)*sin(s);
	C_b_l(2,1) =  cos(f)*sin(t)*sin(s)+sin(f)*cos(s);
	C_b_l(2,2) = -sin(f)*sin(t)*sin(s)+cos(f)*cos(s);

	return C_b_l;
}

mat3x1 RotMat2Euler_l(mat3x3 R)
{

	if(fabs(R(1,0))>1)//sh
	R(1,0)=1.*sign(R(1,0));

	double f = atan2(-R(1,2),R(1,1));
	double t = asin(R(1,0));
	double s = atan2(-R(2,0),R(0,0));

	mat3x1 Angles;
	Angles(0,0) = f;
	Angles(1,0) = t;
	Angles(2,0) = s;

	return Angles;
}
