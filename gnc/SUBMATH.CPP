#include"stdafx.h"
#include"M273N.H"

int sign(float sg) 
{
	//if (sg > 0)	return 1;
	//return -1;
	return (sg > 0) - (sg < 0);
}

//double limit(double var , double max_val ,double min_val)
//{
//if(var >= max_val)
//   var = max_val;
//else if(var <= min_val)
//   var = min_val;
//return var;
//}


double mag(int num,double r[])

/*   PORPUSE  :  To computing the magnitude of  num x 1  vector .
*/
{
double sq,sum=0.0;
int i;
      for(i=0;i<num;i++)
      sum=sum+r[i]*r[i];
      sq=sqrt(sum);
      return(sq);
}

void multmat(double a[3][3],double b[3][3],double ab[3][3])
/*   PORPUSE  :  To multipling by the two matrices .
*/
{
int i,j,k;
for(i=0;i<3;i++)
 {
  for(j=0;j<3;j++)
  {
   ab[i][j]=0.0;
   for(k=0;k<3;k++)
	ab[i][j]=ab[i][j]+a[i][k]*b[k][j];
  }
 }
 }

void  cross(double a[3],double b[3],double acb[3])
{
 acb[0]=a[1]*b[2]-b[1]*a[2];
 acb[1]=a[2]*b[0]-b[2]*a[0];
 acb[2]=a[0]*b[1]-b[0]*a[1];
}

double dot(int n, double *a, double *b)
{
	/*
	DOUBLE PRECISION FUNCTION dot(n,a,b)
	c
	c PURPOSE      To compute the dot product of two  nx1 vectors 'a'
	c              and 'b'.
	c
	c USAGE        dotvec = dot(n,a,b)
	c              Besure to declare 'dot' as double precision in the
	c              calling program.
	c
	c INPUTS       n, a, b
	c
	c OUTPUTS      dot
	c
	c EXTERNALS    None
	c
	c
	c BY M.A. MASSOUMNIA
	c Shahrivar-30-1366
	c
	Translated to C
	Mehr 1375
	*/
	double sum;
	int i;

	sum=0.0;
	for(i=0;i<n;++i)
		sum = sum + a[i]*b[i];

	return sum;
}
double dot(mat3x1 a, mat3x1 b)
{

	double sum;
	int i;

	sum=0.0;
	for(int i=0;i<3;++i)
		sum +=(a(i,0)*b(i,0));

	return sum;
}
void amar(int n,double *xx,double *mean,double *sigma)
/*           To compute the mean and standard diviation of vector x .
*/
{
  double sum=0.0;
  int i;

  for(i=0;i<n;i++)
   sum+=xx[i];
  *mean=sum/n;
  sum=0.0;
  for(i=0;i<n;i++)
    sum=sum+(xx[i]-*mean)*(xx[i]-*mean);
  *sigma = sqrt(sum/(n-1));
}


void gaus(double mean,double sigma,double *x)
//         To generate the gaussian random numbers.

{
 int i;
 double r,g_num,sum=0.;

   for(i=0; i<12; i++)
    {
   // r=(double)random(RAND_MAX)/RAND_MAX;
    sum+=r;
    }
  g_num=sum-6.;
  *x = g_num*sigma+mean;

}

void  drkg4(int ndim,void (*fun)(void),int (*out)(void))
 {
 double aux[50],h,t0,tend,r1r,r2r;
 static double a[4]={.5,.2928932188134525,1.707106781186547,.1666666666666667};
 static double b[4]={2.0,1.0,1.0,2.0};
 static double c[4]={.5,.2928932188134525,1.707106781186547,.5};
 t = prmt[0]; tend = prmt[1]; h = prmt[2];
 prmt[3]= prmt[4]= 0;
 fun();
/*      ERROR TEST   */
	 if (tend < t){out();return;}

/*          PREPARATIONS FOR RUNGE-KUTTA METHOD     */
/*          PREPARATIONS OF FIRST RUNGE-KUTTA STEP  */
	 for(int i =0; i< ndim;i++)	 aux[i] = 0.0;

/*     START OF A RUNGE-KUTTA STEP */
do{
//if(t>500.) h=0.001;
			if ((t + h )> tend)  h = tend - t;

/*     RECORDING OF INITIAL VALUES OF THIS STEP */
	if(out())  return;
	else t0=t;

/*           START OF INNERMOST RUNGE-KUTTA LOOP */
for(int integ_step=0;integ_step<4;integ_step++)
	{
	  for(int i = 0;i< ndim;i++)
	  {
	  y[i]+=(r2r = a[integ_step] * ((r1r = h * dy[i]) - b[integ_step] * aux[i]));
	  aux[i] +=(3*r2r - c[integ_step] * r1r);
	  }
	  if      (integ_step==0)  t+= .5 * h;
	  else if (integ_step==2)  t=t0+h;
//	  t+= .25 * h;
	  fun();
	}
		 }while(t < tend);
 out();
return;
}
//double Interpol_gains(int dim,double q,float qq[],float gain[]) 
//{
//   int i, index;
//   int qn=dim;//,gn=36;
//   for (i = 0; i < qn + 1; i++) 
//   {
//      if (q <= qq[i]) 
//	  {
//         index = i - 1;
//         break;
//      }
//   }
//   if (q == qq[0])	index = 0;
//   if (q >= qq[qn-1])		return gain[qn - 1];
//   else						return (gain[index] + (gain[index + 1] - gain[index]) *(q - qq[index]) / (qq[index + 1] - qq[index]));
//}

void Vect2SkewSym(double W_vec[3],double skew[3][3])
{
	//mat3x3 W(0);
	skew[0][1] = -W_vec[2];
	skew[0][2] =  W_vec[1];
	skew[1][0] =  W_vec[2];
	skew[1][2] = -W_vec[0];
	skew[2][0] = -W_vec[1];
	skew[2][1] =  W_vec[0];

	//return W;
}

mat3x3 Vect2SkewSym(mat3x1 W_vec)
{
	mat3x3 W(0);
	W(0,1) = -W_vec(2,0);
	W(0,2) =  W_vec(1,0);
	W(1,0) =  W_vec(2,0);
	W(1,2) = -W_vec(0,0);
	W(2,0) = -W_vec(1,0);
	W(2,1) =  W_vec(0,0);

	return W;
}

mat3x1 Arr2Mat( double matrix[3])
{
mat3x1 mat;
  mat(0,0) = matrix[0];
	mat(1,0) = matrix[1];
	mat(2,0) =  matrix[2];
return mat;		
}

mat3x1 Arr2Mat(double a,double b,double c)
{
mat3x1 mat;
  mat(0,0) = a;
	mat(1,0) = b;
	mat(2,0) =  c;
return mat;		
}

void Mat2Arr( mat3x1 b,double c[3])
{
    c[0] = b(0,0);
	c[1] = b(1,0);
	c[2] = b(2,0);		
}

void Arr2Mat3x3(double Arr[3][3],mat3x3 *Mat )
{
	for (int i=0; i<3; i++) 
		for (int j=0; j<3; j++) 
			Mat->Val[i][j]    = Arr[i][j];
		
}
///////////////////////////////////////////////////////////////////////////////
//Returns unit vector from 3x1 vector
//Example: UVEC=VEC.univec3();
///////////////////////////////////////////////////////////////////////////////
mat3x1 univec3(mat3x1 vec)
{
	mat3x1 RESULT(0);
	//check for proper dimensions
	//if (num_col!=1||num_row!=3)
	//{cout<<" *** Error: not a 3x1 column vector in 'Matrix::univec()' *** \n";system("pause");exit(1);}

	double v1=vec(0,0);
	double v2=vec(1,0);
	double v3=vec(2,0);
	double d=sqrt(v1*v1+v2*v2+v3*v3);

	//if VEC is zero than the unit vector is also a zero vector
	if(d==0){		
		RESULT(0,0)=0;
		RESULT(1,0)=0;
		RESULT(2,0)=0;
	}
	else{

		RESULT(0,0)=v1/d;
		RESULT(1,0)=v2/d;
		RESULT(2,0)=v3/d;
	}	
	return RESULT;
}

void Min_Max( int mod,int no,double inp[],double val[],int idx[])
{
	double minv,Amin,Amax;
	 //no = sizeof(inp)/sizeof(inp[0]);
	val[1] = val[0] = inp[0];
	idx[0] = idx[1] = 0;
	if (mod==0)//Min
	{
		for (int i=0; i<no; i++) 
		{
			if(inp[i]<val[0])
			{
				val[0] = inp[i];
				idx[0] = i;
			}
		}
	}
	else if (mod==1)//Max
	{
		for (int i=0; i<no; i++) 
		{
			if(inp[i]>val[1])
			{
				val[1] = inp[i];
				idx[1] = i;
			}
		}
	}
	else if (mod==2)//MinMax
	{
		//val[1] = val[0] = inp[0];//,	B=inp[i+1];
		for (int i=0; i<no; i++) 
		{
			
			if(inp[i]<val[0])
			{
				val[0] = inp[i];
				idx[0] = i;
			}
			if(inp[i]>val[1])
			{
				val[1] = inp[i];
				idx[1] = i;
			}
		}

	}

}

void Array_Min(int no,double inp[],double val,int idx)
{
	val = inp[0];
	idx = 0;

	for (int i=0; i<no; i++) 
	{
		if(inp[i]<val)
		{
			val = inp[i];
			idx = i;
		}
	}
}

int MinId2(int no,double inp[])
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
double  erf(double x)
{

	 double a[5]={.254829592,-.284496736,1.421413741,-1.453152027,1.061405429};
	 double p=.3275911;

	int signn=sign(x);
	x=fabs(x);

	double t_t=1./(1.+p*x);
	double y = 1.-(((((a[4]*t_t+a[3])*t_t)+a[2])*t_t+a[1])*t_t+a[0])*t_t*exp(-x*x);

	return signn*y;
}

double  guss_prp(mat3x1 dxyz,mat3x1 sig)
{

	 double expon =-0.5*((pow(dxyz(0,0)/sig(0,0),2))+(pow(dxyz(1,0)/sig(1,0),2))+(pow(dxyz(2,0)/sig(2,0),2)));
	 double prob=exp(expon)/(pow(2*M_PI,1.5)*sig(0,0)*sig(1,0)*sig(2,0));

	return prob;
}

double distance(double a[3],double b[3])
{
	return sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
}


mat3x1 proj_tarj(mat3x1 pm0,mat3x1 vm0,mat3x1 am0,double t_)
 {
	 mat3x1 out = (pm0+vm0*t_+0.5*am0*t_*t_);
		return out;
 }

mat3x1 proj_tarj2(mat3x1 pt0,mat3x1 vt0,double t_)
 {
	  mat3x1 out = (pt0+vt0*t_);
		return out;
 }

double find_intercep_t(mat3x1 pm0,mat3x1 vm0,mat3x1 am0,mat3x1 pt0,mat3x1 vt0)
{
	double t_low=0.,t_high=1000.,t_mid=0;
	while(t_high-t_low>1e6)
	{
		t_mid=(t_low-t_high)/2.;
		mat3x1 p_m = proj_tarj(pm0,vm0,am0,t_mid);
		mat3x1 p_t = proj_tarj2(pt0,vt0,t_mid);//pt0;
		double dist = (p_m-p_t).Norm();
		if(dist<1e6) return t_mid;

		mat3x1 p_m_low = proj_tarj(pm0,vm0,am0,t_low);
		mat3x1 p_t_low = proj_tarj2(pt0,vt0,t_low);//pt0;

		if((p_m-p_t).Norm()>dist)	t_low = t_mid;
		else						t_high = t_mid;

	}
	return t_mid;
}
double calc_hit_prob(mat3x1 pm0,mat3x1 vm0,mat3x1 am0,mat3x1 pt0,mat3x1 vt0,double sigma_cep)
{
	double t_hit = find_intercep_t( pm0, vm0, am0, pt0,vt0);
	mat3x1 p_m_imp = proj_tarj(pm0,vm0,am0,t_hit);
	mat3x1 p_t_imp = proj_tarj2(pt0,vt0,t_hit);//pt0;

	double dist = (p_m_imp-p_t_imp).Norm(); 
	double p_hit = 0.5*(1.+erf(dist/(sqrt(2)*sigma_cep)));


	return p_hit;
}

double calc_hit_prob3(mat3x1 pm0,mat3x1 vm0,mat3x1 am0,mat3x1 pt0,mat3x1 vt0,mat3x1 sigma_xyz)
{
	mat3x1 dist,p_hit_xyz;
	double t_hit = find_intercep_t( pm0, vm0, am0, pt0,vt0);
	//t_hit = ((pm0-pt0).Norm())/vm0.Norm();
	mat3x1 p_m_imp = proj_tarj(pm0,vm0,am0,t_hit);
	mat3x1 p_t_imp = proj_tarj2(pt0,vt0,t_hit);//pt0;

		// p_m_imp = pm0;//proj_tarj(pm0,vm0,am0,t_hit);
	 //p_t_imp = pt0;//proj_tarj2(pt0,vt0,t_hit);//pt0;


	//double dist = (p_m_imp-p_t_imp).Norm(); 
	//double p_hit = 0.5*(1.+erf(dist/(sqrt(2)*sigma_sep)));

	 dist = (p_t_imp-p_m_imp); 
	 for (int i = 0; i < 3; i++)
	 {
		 double inp=fabs(dist(i,0))/(sqrt(2)*sigma_xyz(i,0));
		p_hit_xyz(i,0) = 0.5*(1.+erf(inp));
	 }

	 double p_hit=p_hit_xyz(0,0)*p_hit_xyz(1,0)*p_hit_xyz(2,0);

	return p_hit;
	//return  (p_hit(0,0)+p_hit(1,0)+p_hit(2,0))/3;
}