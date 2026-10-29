#ifndef MAT_PLATFORM_H
#define MAT_PLATFORM_H

#include <stdio.h>
#include <stdlib.h>

/* =====================================================================
 *  درز المنصّة لمكتبة المصفوفات المشتركة
 * =====================================================================
 *  عند مصفوفة شاذّة داخل Inv() كان لكل مشروع سلوك مختلف، وكلاهما
 *  صحيح في سياقه:
 *
 *    المحاكاة (M600)  : printf ثم exit(1)
 *        مصفوفة شاذّة هنا خطأ برمجي، والإيقاف الفوري يمنع انتشار
 *        أرقام فاسدة في نتائج المحاكاة بصمت.
 *
 *    الفيرموير (STM743): printf فقط -- exit(1) كانت معلَّقة بـ //edit
 *        exit() لا معنى لها على نظام مضمَّن بلا نظام تشغيل، واستدعاؤها
 *        أثناء الطيران يعني فقدان المركبة. الفيرموير لا يستدعي exit()
 *        في أي موضع من الشيفرة.
 *
 *  هذا الماكرو يحفظ السلوكين كما هما بالضبط، فيبقى الملف واحداً.
 *  ملاحظة: الفيرموير يطبع ثم يستمر بقيم فاسدة داخل مرشِّح كالمان --
 *  معالجة ذلك بند مستقل خارج نطاق توحيد المكتبة.
 * ===================================================================== */

/* المنصّة صريحة إن عُرِّفت مسبقاً في أعلام البناء، وإلا تُستنتج. */
#if !defined(MAT_PLATFORM_EMBEDDED) && !defined(MAT_PLATFORM_HOSTED)
	#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
		#define MAT_PLATFORM_HOSTED   1
	#else
		#define MAT_PLATFORM_EMBEDDED 1
	#endif
#endif

#if defined(MAT_PLATFORM_EMBEDDED)
	#define MAT_ON_SINGULAR(msg)   printf(msg)
#else
	#define MAT_ON_SINGULAR(msg)   do { printf(msg); exit(1); } while (0)
#endif

#endif /* MAT_PLATFORM_H */
