#ifndef _MAT_DEF_H_
#define _MAT_DEF_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* ترويسات تعتمد على المنصة.
 * المحاكاة (MSVC/mex) كانت تُضمِّن <conio.h> و <iostream>؛
 * الفيرموير (arm-none-eabi) يكتفي بـ <ostream>.
 * الحراسة تحفظ سلوك كل منصة كما هو بالضبط. */
#ifdef _WIN32
	#include <conio.h>
	#include <iostream>
#else
	#include <ostream>
#endif
	using namespace std;


class mat3x3
{
public:
	double Val[3][3];
	mat3x3() {;};
	mat3x3(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
	void Unit() {
			Val[0][0] = 1;
			Val[1][1] = 1;
			Val[2][2] = 1;
		};
	mat3x3 Inv(void);
	int pivot(int row);
};

class mat3x1
{
public:
	double Val[3][1];
	mat3x1(double a) {Val[0][0]=a;
			Val[1][0]=a;
			Val[2][0]=a;
		};
	mat3x1() {;};
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
	double Norm() {return sqrt(Val[0][0] * Val[0][0] + Val[1][0] * Val[1][0] + Val[2][0] * Val[2][0]);};
};

class mat6x6
{
public:
	double Val[6][6];
	mat6x6() {;};
	mat6x6(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
	void Unit() {
			Val[0][0] = 1; Val[3][3] = 1;
			Val[1][1] = 1; Val[4][4] = 1;
			Val[2][2] = 1; Val[5][5] = 1;
		};
	mat6x6 Inv();
	int pivot (int row);
};

class mat6x1
{
public:
	double Val[6][1];
	mat6x1(double a) {
			Val[0][0]=a; Val[3][0]=a;
			Val[1][0]=a; Val[4][0]=a;
			Val[2][0]=a; Val[5][0]=a;
		};
	mat6x1() {;};
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
};

class mat9x9
{
public:
	double Val[9][9];
	mat9x9() {;};
	mat9x9(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
	void Unit();
};

class mat9x3
{
public:
	double Val[9][3];
	mat9x3() {;};
	mat9x3(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
};
/*
class mat10x12
{
public:
	double Val[10][12];
	mat10x12(double a) {
		for (int i=0;i<10;++i)
			for (int j=0;j<11;++j)
				Val[i][j]=a;
		};
	mat10x12() {;};
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
};*/

class mat3x9
{
public:
	double Val[3][9];
	mat3x9() {;};
	mat3x9(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
};

class mat9x1
{
public:
	double Val[9][1];
	mat9x1() {;};
	mat9x1(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
};

class mat10x1
{
public:
	double Val[10][1];
	mat10x1() {;};
	mat10x1(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
};

class mat15x1
{
public:
	double Val[15][1];
	mat15x1() {;};
	mat15x1(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
};

class mat15x3
{
public:
	double Val[15][3];
	mat15x3() {;};
	mat15x3(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
};
class mat15x6
{
public:
	double Val[15][6];
	mat15x6() {;};
	mat15x6(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
};
class mat3x15
{
public:
	double Val[3][15];
	mat3x15() {;};
	mat3x15(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
};

class mat15x15
{
public:
	double Val[15][15];
	mat15x15() {;};
	mat15x15(double a);
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
};

class mat4x1
{
public:
	double Val[4][1];
	mat4x1(double a) {Val[0][0]=a; Val[1][0]=a;
			Val[2][0]=a; Val[3][0]=a;
		};
	mat4x1() {;};
	inline double operator() (const int x,const int y) const {return Val[x][y];};
	inline double& operator() (int x,int y) {return Val[x][y];};
	double Norm();
};


mat3x3  operator ~ (const mat3x3& b);
//mat10x6  operator ~ (const mat6x10& b);
mat9x9  operator ~ (const mat9x9& b);
mat15x15  operator ~ (const mat15x15& b);
//mat6x9   operator ~ (const mat9x6& b);
//mat6x6   operator ~ (const mat6x6& b);

mat3x3  operator + (mat3x3 m,mat3x3 n);
mat3x1  operator + (mat3x1 m,mat3x1 n);
//mat6x6   operator + (mat6x6 m,mat6x6 n);
mat9x9  operator + (mat9x9 m,mat9x9 n);
mat15x15 operator + (mat15x15 m,mat15x15 n);
//mat10x12 operator + (mat10x12 m,mat10x12 n);
mat9x1  operator + (const mat9x1 m, const mat9x1 n);
mat10x1 operator + (mat10x1 m,mat10x1 n);
mat15x1 operator + (mat15x1 m,mat15x1 n);
//mat9x1   operator + (mat9x1 m,mat9x1 n);
//mat6x1   operator - (mat6x1 m,mat6x1 n);
//mat3x3   operator - (mat3x3 m,mat3x3 n);
mat3x1  operator - (mat3x1 m,mat3x1 n);
mat9x9  operator - (mat9x9 m,mat9x9 n);


mat3x1   operator * (const double a, const mat3x1 &b);
mat4x1   operator * (const double a, const mat4x1 &b);
//mat10x10 operator * (const double a, const mat10x10 &b);
mat10x1  operator * (const double a, const mat10x1 &b);
mat15x1  operator * (const double a, const mat15x1 &b);
//mat10x12 operator * (const double a, const mat10x12 &b);

mat3x3   operator * (const mat3x3 &a, const mat3x3 &b);
mat3x1   operator * (const mat3x3 &a, const mat3x1 &b);
double   operator * (const mat3x1 &a, const mat3x1 &b);
mat3x1   operator ^ (const mat3x1 &a, const mat3x1 &b);
mat4x1   operator ^ (const mat4x1 &a, const mat4x1 &b);
//mat6x1   operator * (const mat6x9 &a, const mat9x1 &b);
//mat6x9   operator * (const mat6x9 &a, const mat10x10 &b);
//mat6x6   operator * (const mat6x9 &a, const mat9x6 &b);
mat6x6   operator * (const mat6x6 &a, const mat6x6 &b);
//mat9x6   operator * (const mat10x10 &a, const mat9x6 &b);
//mat9x6   operator * (const mat9x6 &a, const mat6x6 &b);
mat9x1 operator * (const mat9x3 &a, const mat3x1 &b);
mat15x1 operator * (const mat15x3 &a, const mat3x1 &b);
mat15x1 operator * (const mat15x6 &a, const mat6x1 &b);
mat9x1 operator * (const mat9x9 &a, const mat9x1 &b);
mat15x1 operator * (const mat15x15 &a, const mat15x1 &b);
mat9x3 operator * (const mat9x9 &a, const mat9x3 &b);
mat9x3 operator * (const mat9x3 &a, const mat3x3 &b);
mat15x3 operator * (const mat15x3 &a, const mat3x3 &b);
mat15x6 operator * (const mat15x6 &a, const mat6x6 &b);
mat9x9 operator * (const mat9x3 &a, const mat3x9 &b);
mat9x9 operator * (const mat9x9 &a, const mat9x9 &b);
mat15x15 operator * (const mat15x15 &a, const mat15x15 &b);
mat9x9 operator * (const double &a, const mat9x9 &b);
mat15x15 operator * (const double &a, const mat15x15 &b);

ostream& operator << (ostream& ostrm, mat3x3 a);
ostream& operator << (ostream& ostrm, mat6x6 a);
ostream& operator << (ostream& ostrm, mat3x1 a);
ostream& operator << (ostream& ostrm, mat4x1 a);

/***************** RotConv.cpp ***************/
mat3x3 Vect2SkewSym(mat3x1 W_vec);
mat3x3 Rot_b_n(mat3x1 Angles);
mat3x3 Rot_e_n(double lam);
mat3x3 Rot_e_n(double phi, double lam);
mat3x3 Quat2RotMat(mat4x1 q);
mat4x1 RotMat2Quat(mat3x3 R);
mat3x1 RotMat2Euler_l(mat3x3 R);
mat3x1 RotMat2Euler_n(mat3x3 R);
mat3x1 Quat2Euler_l(mat4x1 q);
mat3x1 Quat2Euler_n(mat4x1 q);
mat3x3 Rot_e_l(double psi0, double phi0, double lambda0);
mat3x3 Rot_b_l(mat3x1 Angles);
//v3zar
mat3x3 e2m(mat3x1 Angles);
//v3zar

/******************** NAVIG.H ***************************/
mat3x1 cart2nav(mat3x1 r_vec);
mat3x1 cart2nav(mat3x1 r_vec, double *M, double *N);
mat3x1 nav2cart(mat3x1 r_n);
mat3x1 nav2cart(mat3x1 r_n, double *M, double *N);
mat3x1 nav2cart(double fi , double lambda , double h);

//R2000v01
mat3x1 imperr(mat3x1 rl, mat3x1 rmn, double h, double lat,double lon);
mat3x1 imperr(mat3x1 rl, mat3x1 rmn, mat3x1 rtn);

void send2PC(mat3x1 const &vec, char msgID);
void send2PC(long int cnt_timer, int err, long t_gps, int mb_x, int mb_y, int mb_z, int Height);
void send2PC(int AX, int AY, int AZ, int WX, int WY, int WZ);
void q2m_(double qq[4],double mm[3][3]);
void e2q(double theta,double psi,double phi,double q[4]);
void Mat2Arr3x3(mat3x3 *Mat, double Arr[3][3]);
void e2m(double psi,double teta,double phi,double m[3][3]);
void m2q(double C[3][3],double q[4]);

/******************** تصريحات كانت في نسخة المحاكاة وحدها ***************
 * تُنفَّذ في SUBMATH.CPP و SUBNAV.CPP (M600)، وغير مُنفَّذة في الفيرموير.
 * غير ضار: التصريح وحده لا يُنتج خطأ ربط ما لم تُستدعَ الدالة.
 * ملاحظة: dot(mat3x1,mat3x1) و Arr2Mat مُصرَّح عنهما أصلاً في
 * M273N.H و mainh.h على التوالي — إعادة التصريح قانونية. */
mat3x3 MATRIXn2Az2(double Az_2);
mat3x1 univec3(mat3x1 vec);
mat3x1 pol_from_cart( mat3x1 cart) ;
double dot(mat3x1 a, mat3x1 b);
mat3x1 cart2nav22(mat3x1 r_vec);
void use_data(mat3x1 rtn, double bearing, double speed) ;
mat3x1 Arr2Mat( double matrix[3]);
void Mat2Arr( mat3x1 b,double c[3]);
void Arr2Mat3x3( double Arr[3][3],mat3x3 *Mat);

#endif