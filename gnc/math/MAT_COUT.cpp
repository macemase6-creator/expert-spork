/* MAT_COUT.cpp -- نسخة قانونية موحَّدة بين مشروع المحاكاة وفيرموير الطيران.
 * لا تُعدِّل النسخ في M600_V8_1/ أو Core/Src/ -- هي ملفات تمرير.
 */
#include "MAT_DEF.h"

ostream& operator << (ostream& ostrm, mat3x3 a)
{
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			double x = a.Val[i][j];
			ostrm << x << '\t';
		}
		ostrm << endl;
	}
	return ostrm;
}

ostream& operator << (ostream& ostrm, mat6x6 a)
{
	for (int i=0; i<6; i++)
	{
		for (int j=0; j<6; j++)
		{
			double x = a.Val[i][j];
			ostrm << x << '\t';
		}
		ostrm << endl;
	}
	return ostrm;
}

ostream& operator << (ostream& ostrm, mat3x1 a)
{
	for (int i=0; i<3; i++)
	{
		double x = a.Val[i][0];
		ostrm << x << '\t';
	}
	ostrm << "              ";
	ostrm << endl;
	return ostrm;
}

ostream& operator << (ostream& ostrm, mat4x1 a)
{
	for (int i=0; i<4; i++)
	{
		double x = a.Val[i][0];
		ostrm << x << '\t';
	}
	ostrm << endl;
	return ostrm;
}
