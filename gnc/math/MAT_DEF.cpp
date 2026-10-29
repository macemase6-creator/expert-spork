/* MAT_DEF.cpp -- نسخة قانونية موحَّدة بين مشروع المحاكاة وفيرموير الطيران.
 * لا تُعدِّل النسخ في M600_V8_1/ أو Core/Src/ -- هي ملفات تمرير.
 */
#include "MAT_DEF.h"

mat3x3::mat3x3(double a) {
		for (int i=0;i<3;++i)
			for (int j=0;j<3;++j)
				Val[i][j]=a;
		};

mat6x6::mat6x6(double a) {
		for (int i=0;i<6;++i)
			for (int j=0;j<6;++j)
				Val[i][j]=a;
		};

mat9x9::mat9x9(double a) {
		for (int i=0;i<9;++i)
			for (int j=0;j<9;++j)
				Val[i][j]=a;
		};
void mat9x9::Unit() {
		for (int i=0;i<9;++i)
			Val[i][i] = 1;
		};

mat9x3::mat9x3(double a) {
		for (int i=0;i<9;++i)
			for (int j=0;j<3;++j)
				Val[i][j]=a;
		};

mat3x9::mat3x9(double a) {
		for (int i=0;i<3;++i)
			for (int j=0;j<9;++j)
				Val[i][j]=a;
		};

mat9x1::mat9x1(double a) {
		for (int i=0;i<9;++i)
			Val[i][0]=a;
		};

mat10x1::mat10x1(double a) {
		for (int i=0;i<10;++i)
			Val[i][0]=a;
		};

mat15x1::mat15x1(double a) {
		for (int i=0;i<15;++i)
			Val[i][0]=a;
		};

mat15x3::mat15x3(double a) {
		for (int i=0;i<15;++i)
			for (int j=0;j<3;++j)
				Val[i][j]=a;
		};
mat15x6::mat15x6(double a) {
		for (int i=0;i<15;++i)
			for (int j=0;j<6;++j)
				Val[i][j]=a;
		};

mat3x15::mat3x15(double a) {
		for (int i=0;i<3;++i)
			for (int j=0;j<15;++j)
				Val[i][j]=a;
		};

mat15x15::mat15x15(double a) {
		for (int i=0;i<15;++i)
			for (int j=0;j<15;++j)
				Val[i][j]=a;
		};

double mat4x1::Norm() {
		double n=0;
		for (int i=0;i<4;++i)
			n+=Val[i][0] * Val[i][0];
		return sqrt(n);
		};

