/* MAT_MUL.cpp -- نسخة قانونية موحَّدة بين مشروع المحاكاة وفيرموير الطيران.
 * لا تُعدِّل النسخ في M600_V8_1/ أو Core/Src/ -- هي ملفات تمرير.
 */
#include "MAT_DEF.h"

mat3x3 operator * (const mat3x3 &a, const mat3x3 &b)
{
	mat3x3 c(0);

	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			for (int k=0; k<3;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}
mat6x6 operator * (const mat6x6 &a, const mat6x6 &b)//m273
{
	mat6x6 c(0);

	for(int i=0;i<6;i++)
		for(int j=0;j<6;j++)
			for (int k=0; k<6;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}
mat3x1 operator * (const mat3x3 &a, const mat3x1 &b)
{
	mat3x1 c(0);

	for(int i=0;i<3;i++)
		for(int j=0;j<1;j++)
			for (int k=0; k<3;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

double operator * (const mat3x1 &a, const mat3x1 &b)
{
	return a(0,0)*b(0,0) + a(1,0)*b(1,0) + a(2,0)*b(2,0);
}

mat3x1   operator ^ (const mat3x1 &a, const mat3x1 &b)
{
	mat3x1 c;
	c(0,0) = a(1,0)*b(2,0) - b(1,0)*a(2,0);
	c(1,0) = a(2,0)*b(0,0) - b(2,0)*a(0,0);
	c(2,0) = a(0,0)*b(1,0) - b(0,0)*a(1,0);
	return c;
}

mat4x1 operator ^ (const mat4x1 &Q, const mat4x1 &q)
{
	mat4x1 qu;
	double Q0=Q(0,0), Q1=Q(1,0), Q2=Q(2,0), Q3=Q(3,0);
	double q0=q(0,0), q1=q(1,0), q2=q(2,0), q3=q(3,0);

	qu(0,0) = Q0*q0 - Q1*q1 - Q2*q2 - Q3*q3;
	qu(1,0) = Q0*q1 + Q1*q0 + Q2*q3 - Q3*q2;
	qu(2,0) = Q0*q2 + Q2*q0 + Q3*q1 - Q1*q3;
	qu(3,0) = Q0*q3 + Q3*q0 + Q1*q2 - Q2*q1;

	return qu;
}

mat9x3 operator * (const mat9x9 &a, const mat9x3 &b)
{
	mat9x3 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<3;j++)
			for (int k=0; k<9;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

mat9x3 operator * (const mat9x3 &a, const mat3x3 &b)
{
	mat9x3 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<3;j++)
			for (int k=0; k<3;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

mat15x1 operator * (const mat15x3 &a, const mat3x1 &b)
{
	mat15x1 c(0);

	for(int i=0;i<15;i++)
		for(int j=0;j<1;j++)
			for (int k=0; k<3;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}
mat15x1 operator * (const mat15x6 &a, const mat6x1 &b)
{
	mat15x1 c(0);

	for(int i=0;i<15;i++)
		for(int j=0;j<1;j++)
			for (int k=0; k<6;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}
mat15x1 operator * (const mat15x15 &a, const mat15x1 &b)
{
	mat15x1 c(0);

	for(int i=0;i<15;i++)
		for(int j=0;j<1;j++)
			for (int k=0; k<15;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

mat15x3 operator * (const mat15x3 &a, const mat3x3 &b)
{
	mat15x3 c(0);

	for(int i=0;i<15;i++)
		for(int j=0;j<3;j++)
			for (int k=0; k<3;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}
mat15x6 operator * (const mat15x6 &a, const mat6x6 &b)
{
	mat15x6 c(0);

	for(int i=0;i<15;i++)
		for(int j=0;j<6;j++)
			for (int k=0; k<6;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}
mat15x15 operator * (const mat15x15 &a, const mat15x15 &b)
{
	mat15x15 c(0);

	for(int i=0;i<15;i++)
		for(int j=0;j<15;j++)
			for (int k=0; k<15;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

/*
mat9x6 operator * (const mat9x6 &a, const mat6x6 &b)
{
	mat9x6 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<6;j++)
			for (int k=0; k<6;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

mat9x9 operator * (const mat9x3 &a, const mat3x9 &b)
{
	mat9x9 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<9;j++)
			for (int k=0; k<3;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

mat9x1 operator * (const mat9x3 &a, const mat3x1 &b)
{
	mat9x1 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<1;j++)
			for (int k=0; k<3;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}
/*
mat6x1 operator * (const mat6x9 &a, const mat9x1 &b)
{
	mat6x1 c(0);

	for(int i=0;i<6;i++)
		for(int j=0;j<1;j++)
			for (int k=0; k<9;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

mat9x1 operator * (const mat9x9 &a, const mat9x1 &b)
{
	mat9x1 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<1;j++)
			for (int k=0; k<9;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

mat9x9 operator * (const mat9x9 &a, const mat9x9 &b)
{
	mat9x9 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<9;j++)
			for (int k=0;k<9;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

mat6x9 operator * (const mat6x9 &a, const mat9x9 &b)
{
	mat6x9 c(0);

	for(int i=0;i<6;i++)
		for(int j=0;j<9;j++)
			for (int k=0; k<9;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}
  
mat6x6 operator * (const mat6x9 &a, const mat9x6 &b)
{
	mat6x6 c(0);

	for(int i=0;i<6;i++)
		for(int j=0;j<6;j++)
			for (int k=0; k<9;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

mat6x6 operator * (const mat6x6 &a, const mat6x6 &b)
{
	mat6x6 c(0);

	for(int i=0;i<6;i++)
		for(int j=0;j<6;j++)
			for (int k=0; k<6;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}
*/
mat3x1 operator * (const double a, const mat3x1 &b)
{
	mat3x1 c(0);

	for(int i=0;i<3;i++)
		c.Val[i][0] = a * b.Val[i][0];

	return c;
}

mat4x1 operator * (const double a, const mat4x1 &b)
{
	mat4x1 c(0);

	for(int i=0;i<4;i++)
		c.Val[i][0] = a * b.Val[i][0];

	return c;
}

mat10x1 operator * (const double a, const mat10x1 &b)
{
	mat10x1 c(0);

	for(int i=0;i<10;i++)
		c.Val[i][0] = a * b.Val[i][0];

	return c;
}

mat15x1 operator * (const double a, const mat15x1 &b)
{
	mat15x1 c(0);

	for(int i=0;i<15;i++)
		c.Val[i][0] = a * b.Val[i][0];

	return c;
}
/*
mat10x12 operator * (const double a, const mat10x12 &b)
{
	mat10x12 c(0);

	for(int i=0;i<10;i++)
		for(int j=0;j<12;j++)
			c.Val[i][j] = a * b.Val[i][j];

	return c;
}
*/
mat9x9 operator * (const double &a, const mat9x9 &b)
{
	mat9x9 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<9;j++)
			c.Val[i][j] = a * b.Val[i][j];

	return c;
}

mat15x15 operator * (const double &a, const mat15x15 &b)
{
	mat15x15 c(0);

	for(int i=0;i<15;i++)
		for(int j=0;j<15;j++)
			c.Val[i][j] = a * b.Val[i][j];

	return c;
}

/****** دوال كانت فعّالة في نسخة المحاكاة وحدها *********************
 * مُصرَّح عنها أصلاً في MAT_DEF.h (أسطر 251-261) لكنها لم تكن
 * مُعرَّفة في نسخة الفيرموير -- عدم تناسق قائم تُصلحه إضافتها هنا.
 * الفيرموير لا يستدعيها، ومع -ffunction-sections و --gc-sections
 * تُحذف من الثنائي فلا تزيد حجمه. */
mat9x9 operator * (const mat9x3 &a, const mat3x9 &b)
{
	mat9x9 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<9;j++)
			for (int k=0; k<3;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

mat9x1 operator * (const mat9x3 &a, const mat3x1 &b)
{
	mat9x1 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<1;j++)
			for (int k=0; k<3;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}
mat9x1 operator * (const mat9x9 &a, const mat9x1 &b)
{
	mat9x1 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<1;j++)
			for (int k=0; k<9;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}

mat9x9 operator * (const mat9x9 &a, const mat9x9 &b)
{
	mat9x9 c(0);

	for(int i=0;i<9;i++)
		for(int j=0;j<9;j++)
			for (int k=0;k<9;k++)
				c.Val[i][j] += a.Val[i][k] * b.Val[k][j];

	return c;
}
