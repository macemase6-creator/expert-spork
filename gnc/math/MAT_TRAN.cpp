/* MAT_TRAN.cpp -- نسخة قانونية موحَّدة بين مشروع المحاكاة وفيرموير الطيران.
 * لا تُعدِّل النسخ في M600_V8_1/ أو Core/Src/ -- هي ملفات تمرير.
 */
#include "MAT_DEF.h"
#include "mat_platform.h"

mat3x3 operator ~ (const mat3x3& b)
{
	mat3x3 c;
	for(int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			c.Val[i][j] = b.Val[j][i];
	return c;
}

int mat3x3::pivot (int row)
{
	int k = int(row);
	int i;
	double amax,temp;

	amax = -1;
	for (i=row; i < 3; i++)
		if ( (temp = abs(Val[i][row])) > amax && temp != 0.0)
		{
			amax = temp;
			k = i;
		}
	if (Val[k][row] == 0.)
		return -1;
	if (k != int(row))
	{
		mat3x1 tmp;
		for (i=0;i<3;++i)
		{
			tmp.Val[i][0]=Val[row][i];
			Val[row][i]=Val[k][i];
			Val[k][i]=tmp.Val[i][0];
		}
		return k;
	}
	return 0;
}

mat3x3 mat3x3::Inv ()
{
	int i,j,k;
	double a1,a2;

	mat3x3 temp(0),back;
	for (i=0; i<3; ++i)
		for (j=0; j<3; ++j)
			back.Val[i][j] = Val[i][j];

	temp.Unit();
	for (k=0; k < 3; k++)
	{
		int indx = pivot(k);
		if (indx == -1)
		{
			MAT_ON_SINGULAR("Matrix is Singular : (3x3) \r\n");
		}

		if (indx != 0)
		{
			mat3x1 tmp;
			for (i=0;i<3;++i)
			{
				tmp.Val[i][0]=temp.Val[k][i];
				temp.Val[k][i]=temp.Val[indx][i];
				temp.Val[indx][i]=tmp.Val[i][0];
			}
		}
		a1 = Val[k][k];
		for (j=0; j < 3; j++)
		{
			Val[k][j] /= a1;
			temp.Val[k][j] /= a1;
		}
		for (i=0; i < 3; i++)
		if (i != k)
		{
			a2 = Val[i][k];
			for (j=0; j < 3; j++)
			{
				Val[i][j] -= a2 * Val[k][j];
				temp.Val[i][j] -= a2 * temp.Val[k][j];
			}
		}
	}
	for (i=0;i<3;++i)
		for(int jj=0;jj<3;++jj)
			Val[i][jj] = back.Val[i][jj];

	return temp;
}

mat6x6 mat6x6::Inv ()//m273
{
	int i,j,k;
	double a1,a2;

	mat6x6 temp(0),back;
	for (i=0; i<6; ++i)
		for (j=0; j<6; ++j)
			back.Val[i][j] = Val[i][j];

	temp.Unit();
	for (k=0; k < 6; k++)
	{
		int indx = pivot(k);
		if (indx == -1)
		{
			MAT_ON_SINGULAR("Matrix is Singular : (6x6) \r\n");
		}

		if (indx != 0)
		{
			mat6x1 tmp;
			for (i=0;i<6;++i)
			{
				tmp.Val[i][0]=temp.Val[k][i];
				temp.Val[k][i]=temp.Val[indx][i];
				temp.Val[indx][i]=tmp.Val[i][0];
			}
		}
		a1 = Val[k][k];
		for (j=0; j < 6; j++)
		{
			Val[k][j] /= a1;
			temp.Val[k][j] /= a1;
		}
		for (i=0; i < 6; i++)
		if (i != k)
		{
			a2 = Val[i][k];
			for (j=0; j < 6; j++)
			{
				Val[i][j] -= a2 * Val[k][j];
				temp.Val[i][j] -= a2 * temp.Val[k][j];
			}
		}
	}
	for (i=0;i<6;++i)
		for(int jj=0;jj<6;++jj)
			Val[i][jj] = back.Val[i][jj];

	return temp;
}//*/
int mat6x6::pivot (int row)//m273
{
	int k = int(row);
	int i;
	double amax,temp;

	amax = -1;
	for (i=row; i < 6; i++)
		if ( (temp = abs(Val[i][row])) > amax && temp != 0.0)
		{
			amax = temp;
			k = i;
		}
	if (Val[k][row] == 0.)
		return -1;
	if (k != int(row))
	{
		mat6x1 tmp;
		for (i=0;i<6;++i)
		{
			tmp.Val[i][0]=Val[row][i];
			Val[row][i]=Val[k][i];
			Val[k][i]=tmp.Val[i][0];
		}
		return k;
	}
	return 0;
}
mat6x6 operator ~ (const mat6x6& b)
{
	mat6x6 c;
	for(int i=0;i<6;i++)
		for (int j=0;j<6;j++)
			c.Val[i][j] = b.Val[j][i];
	return c;
}

mat9x9 operator ~ (const mat9x9& b)
{
	mat9x9 c;
	for(int i=0;i<9;i++)
		for (int j=0;j<9;j++)
			c.Val[i][j] = b.Val[j][i];
	return c;
}

mat15x15 operator ~ (const mat15x15 &b)
{
	mat15x15 c;
	for(int i=0;i<15;i++)
		for (int j=0;j<15;j++)
			c.Val[i][j] = b.Val[j][i];
	return c;
}

/*
mat10x6 operator ~ (const mat6x10& b)
{
	mat10x6 c;
	for(int i=0;i<10;i++)
		for (int j=0;j<6;j++)
			c.Val[i][j] = b.Val[j][i];
	return c;
}

mat6x10 operator ~ (const mat10x6& b)
{
	mat6x10 c;
	for(int i=0;i<6;i++)
		for (int j=0;j<10;j++)
			c.Val[i][j] = b.Val[j][i];
	return c;
}
*/
