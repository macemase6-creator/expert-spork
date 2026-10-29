/* MAT_SUM.cpp -- نسخة قانونية موحَّدة بين مشروع المحاكاة وفيرموير الطيران.
 * لا تُعدِّل النسخ في M600_V8_1/ أو Core/Src/ -- هي ملفات تمرير.
 */
#include "MAT_DEF.h"

mat3x1 operator + (mat3x1 m,mat3x1 n)
{
	mat3x1 r;
	for (int i=0;i<3;++i)
		r.Val[i][0]=m.Val[i][0] + n.Val[i][0];

	return r;
}

mat3x1 operator - (mat3x1 m,mat3x1 n)
{
	mat3x1 r;
	for (int i=0;i<3;++i)
		r.Val[i][0]=m.Val[i][0] - n.Val[i][0];

	return r;
}

mat3x3 operator + (mat3x3 m,mat3x3 n)
{
	mat3x3 r;
	for (int i=0;i<3;++i)
		for (int j=0;j<3;++j)
			r.Val[i][j]=m.Val[i][j] + n.Val[i][j];

	return r;
}

mat3x3 operator - (mat3x3 m,mat3x3 n)
{
	mat3x3 r;
	for (int i=0;i<3;++i)
		for (int j=0;j<3;++j)
			r.Val[i][j]=m.Val[i][j] - n.Val[i][j];

	return r;
}

/*mat6x1 operator - (mat6x1 m,mat6x1 n)
{
	mat6x1 r;
	for (int i=0;i<6;++i)
		r.Val[i][0]=m.Val[i][0] - n.Val[i][0];

	return r;
}
*/
mat6x6 operator + (mat6x6 m,mat6x6 n)
{
	mat6x6 r;
	for (int i=0;i<6;++i)
		for (int j=0;j<6;++j)
			r.Val[i][j]=m.Val[i][j] + n.Val[i][j];

	return r;
}

mat9x9 operator + (mat9x9 m,mat9x9 n)
{
	mat9x9 r;
	for (int i=0;i<9;++i)
		for (int j=0;j<9;++j)
			r.Val[i][j]=m.Val[i][j] + n.Val[i][j];

	return r;
}

mat15x15 operator + (mat15x15 m,mat15x15 n)
{
	mat15x15 r;
	for (int i=0;i<15;++i)
		for (int j=0;j<15;++j)
			r.Val[i][j]=m.Val[i][j] + n.Val[i][j];

	return r;
}
/*
mat10x12 operator + (mat10x12 m,mat10x12 n)
{
	mat10x12 r;
	for (int i=0;i<10;++i)
		for (int j=0;j<12;++j)
			r.Val[i][j]=m.Val[i][j] + n.Val[i][j];

	return r;
}
*/
mat9x9 operator - (mat9x9 m, mat9x9 n)
{
	mat9x9 r;
	for (int i=0;i<9;++i)
		for (int j=0;j<9;++j)
			r.Val[i][j]=m.Val[i][j] - n.Val[i][j];

	return r;
}

mat9x1 operator + (const mat9x1 m, const mat9x1 n)
{
	mat9x1 r;
	for (int i=0;i<9;++i)
		r.Val[i][0]=m.Val[i][0] + n.Val[i][0];

	return r;
}

mat10x1 operator + (mat10x1 m, mat10x1 n)
{
	mat10x1 r;
	for (int i=0;i<10;++i)
		r.Val[i][0]=m.Val[i][0] + n.Val[i][0];

	return r;
}

mat15x1 operator + (mat15x1 m, mat15x1 n)
{
	mat15x1 r;
	for (int i=0;i<15;++i)
		r.Val[i][0]=m.Val[i][0] + n.Val[i][0];

	return r;
}
