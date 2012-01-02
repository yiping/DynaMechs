/* This file contains code from two sources not covered by the GNU GPL that the
   DynaMechs library is distributed under:  LINPACK's ssvdc routine
   that was translated to C using f2c, and portions of the f2c header file that
   are need to compile the converted code.  The copyrights and licenses are
   found in comments in this file.
*/

#if defined(WIN32)
#include <windows.h>
#endif

#include <stdio.h>
#include <iomanip>
#include <iostream>

#ifdef __cplusplus
extern "C" {
#endif

using namespace std;
/* -------------------------------- f2c.h -------------------------------

   The following section was extracted from the f2c package (obtained from
   http://www.netlib.org/f2c), is not covered by the GNU GPL, and comes with
   the following permissions statement:

   Copyright 1990 - 1997 by AT&T, Lucent Technologies and Bellcore.

   Permission to use, copy, modify, and distribute this software
   and its documentation for any purpose and without fee is hereby
   granted, provided that the above copyright notice appear in all
   copies and that both that the copyright notice and this
   permission notice and warranty disclaimer appear in supporting
   documentation, and that the names of AT&T, Bell Laboratories,
   Lucent or Bellcore or any of their entities not be used in
   advertising or publicity pertaining to distribution of the
   software without specific, written prior permission.

   AT&T, Lucent and Bellcore disclaim all warranties with regard to
   this software, including all implied warranties of
   merchantability and fitness.  In no event shall AT&T, Lucent or
   Bellcore be liable for any special, indirect or consequential
   damages or any damages whatsoever resulting from loss of use,
   data or profits, whether in an action of contract, negligence or
   other tortious action, arising out of or in connection with the
   use or performance of this software.
*/

/* f2c.h  --  Standard Fortran to C header file */

/**  barf  [ba:rf]  2.  "He suggested using FORTRAN, and everybody barfed."

	- From The Shogakukan DICTIONARY OF NEW ENGLISH (Second edition) */

#ifndef F2C_INCLUDE
#define F2C_INCLUDE

typedef long int integer;
typedef unsigned long uinteger;
typedef char *address;
typedef short int shortint;
typedef float real;
typedef double doublereal;
typedef struct { real r, i; } complex;
typedef struct { doublereal r, i; } doublecomplex;
typedef long int logical;
typedef short int shortlogical;
typedef char logical1;
typedef char integer1;
#if 0	/* Adjust for integer*8. */
typedef long long longint;		/* system-dependent */
typedef unsigned long long ulongint;	/* system-dependent */
#define qbit_clear(a,b)	((a) & ~((ulongint)1 << (b)))
#define qbit_set(a,b)	((a) |  ((ulongint)1 << (b)))
#endif

#define TRUE_ (1)
#define FALSE_ (0)

/* Extern is for use with -E */
#ifndef Extern
#define Extern extern
#endif

/* I/O stuff */

#ifdef f2c_i2
/* for -i2 */
typedef short flag;
typedef short ftnlen;
typedef short ftnint;
#else
typedef long int flag;
typedef long int ftnlen;
typedef long int ftnint;
#endif

/*external read, write*/
typedef struct
{	flag cierr;
	ftnint ciunit;
	flag ciend;
	char *cifmt;
	ftnint cirec;
} cilist;

/*internal read, write*/
typedef struct
{	flag icierr;
	char *iciunit;
	flag iciend;
	char *icifmt;
	ftnint icirlen;
	ftnint icirnum;
} icilist;

/*open*/
typedef struct
{	flag oerr;
	ftnint ounit;
	char *ofnm;
	ftnlen ofnmlen;
	char *osta;
	char *oacc;
	char *ofm;
	ftnint orl;
	char *oblnk;
} olist;

/*close*/
typedef struct
{	flag cerr;
	ftnint cunit;
	char *csta;
} cllist;

/*rewind, backspace, endfile*/
typedef struct
{	flag aerr;
	ftnint aunit;
} alist;

/* inquire */
typedef struct
{	flag inerr;
	ftnint inunit;
	char *infile;
	ftnlen infilen;
	ftnint	*inex;	/*parameters in standard's order*/
	ftnint	*inopen;
	ftnint	*innum;
	ftnint	*innamed;
	char	*inname;
	ftnlen	innamlen;
	char	*inacc;
	ftnlen	inacclen;
	char	*inseq;
	ftnlen	inseqlen;
	char 	*indir;
	ftnlen	indirlen;
	char	*infmt;
	ftnlen	infmtlen;
	char	*inform;
	ftnint	informlen;
	char	*inunf;
	ftnlen	inunflen;
	ftnint	*inrecl;
	ftnint	*innrec;
	char	*inblank;
	ftnlen	inblanklen;
} inlist;

#define VOID void

union Multitype {	/* for multiple entry points */
	integer1 g;
	shortint h;
	integer i;
	/* longint j; */
	real r;
	doublereal d;
	complex c;
	doublecomplex z;
	};

typedef union Multitype Multitype;

/*typedef long int Long;*/	/* No longer used; formerly in Namelist */

struct Vardesc {	/* for Namelist */
	char *name;
	char *addr;
	ftnlen *dims;
	int  type;
	};
typedef struct Vardesc Vardesc;

struct Namelist {
	char *name;
	Vardesc **vars;
	int nvars;
	};
typedef struct Namelist Namelist;

#define abs(x) ((x) >= 0 ? (x) : -(x))
#define dabs(x) (doublereal)abs(x)
#define omin(a,b) ((a) <= (b) ? (a) : (b))
#define omax(a,b) ((a) >= (b) ? (a) : (b))
#define dmin(a,b) (doublereal)omin(a,b)
#define dmax(a,b) (doublereal)omax(a,b)
#define bit_test(a,b)	((a) >> (b) & 1)
#define bit_clear(a,b)	((a) & ~((uinteger)1 << (b)))
#define bit_set(a,b)	((a) |  ((uinteger)1 << (b)))

/* procedure parameter types for -A and -C++ */

#define F2C_proc_par_types 1
#ifdef __cplusplus
typedef int /* Unknown procedure type */ (*U_fp)(...);
typedef shortint (*J_fp)(...);
typedef integer (*I_fp)(...);
typedef real (*R_fp)(...);
typedef doublereal (*D_fp)(...), (*E_fp)(...);
typedef /* Complex */ VOID (*C_fp)(...);
typedef /* Double Complex */ VOID (*Z_fp)(...);
typedef logical (*L_fp)(...);
typedef shortlogical (*K_fp)(...);
typedef /* Character */ VOID (*H_fp)(...);
typedef /* Subroutine */ int (*S_fp)(...);
#else
typedef int /* Unknown procedure type */ (*U_fp)();
typedef shortint (*J_fp)();
typedef integer (*I_fp)();
typedef real (*R_fp)();
typedef doublereal (*D_fp)(), (*E_fp)();
typedef /* Complex */ VOID (*C_fp)();
typedef /* Double Complex */ VOID (*Z_fp)();
typedef logical (*L_fp)();
typedef shortlogical (*K_fp)();
typedef /* Character */ VOID (*H_fp)();
typedef /* Subroutine */ int (*S_fp)();
#endif
/* E_fp is for real functions when -R is not specified */
typedef VOID C_f;	/* complex function */
typedef VOID H_f;	/* character function */
typedef VOID Z_f;	/* double complex function */
typedef doublereal E_f;	/* real function with -R not specified */

/* undef any lower-case symbols that your C compiler predefines, e.g.: */

#ifndef Skip_f2c_Undefs
#undef cray
#undef gcos
#undef mc68010
#undef mc68020
#undef mips
#undef pdp11
#undef sgi
#undef sparc
#undef sun
#undef sun2
#undef sun3
#undef sun4
#undef u370
#undef u3b
#undef u3b2
#undef u3b5
#undef unix
#undef vax
#endif
#endif


/* ----------------------------------------------------------------------
   ssvdc - single precision SVD routine from LINPACK
   dsvdc - double precision SVD routine from LINPACK

   Obtained from http:://www.netlib.org/linpack (search for ssvdc and dsvdc)

   LINPACK is a collection of Fortran subroutines that analyze and
   solve linear equations and linear least-squares probles.  The
   package solves linear systems whose matrices are general, banded,
   symmetric indefinite, symmetric positive definite, triangular,
   and tridiagonal square.  In addition, the package computes
   the QR and singular value decompositions of rectangular matrices
   and applies them to least-squares problems.  LINPACK uses
   column-oriented algorithms to increase efficiency by preserving
   locality of reference.

   LINPACK was designed for supercomputers in use in the 1970s and
   early 1980s.  LINPACK has been largely superceded by LAPACK
   which has been designed to run efficiently on shared-memory, vector
   supercomputers.

   Developed by Jack Dongarra, Jim Bunch, Cleve Moler and Pete Stewart.
   1 Feb 84

   If you are interested in acquiring the entire LINPACK, it may
   make more sense to talk with NAG. NAG distribute the software
   on a mag tape for a nominal charge.
     NAG
     1400 Opus Place, Suite 200
     Downers Grove, IL  60515-5702
     708-971-2337, FAX 971-2706

----------------------------------------------------------------------------*/

/* ssvdc.f -- translated by f2c (version 19990311).
   You must link the resulting object file with the libraries:
	-lf2c -lm   (in that order)
*/

/* Duane Marhefka Note:  It is not necessary to link with f2c.  Only one
   function (r_sign) in the f2c library was required, so I appended that
   function from the libf77.c file to the end of this file.

   This file was created with the following commands:
   > f2cx -C++ -R ssvdc.f
   > move ssvdc.c ssvdc.cpp

   And then the r_sign function was appended from libf77.c.
 */

/* Scott McMillan Notes:

   This wouldn't build on SGI because of a redifinition of
   the sqrt function, so I "fixed" that.  But this code will not converge on
   SGI systems....I still don't know why.

   On Windoze systems the min and max macros were redefinitions to I changed
   them to omin and omax.
*/

/* Table of constant values */

static integer c__1 = 1;
static real c_b44 = (float)-1.;
static real c_b122 = (float)1.;

/* Subroutine */ int ssvdc_(real *x, integer *ldx, integer *n, integer *p,
	real *s, real *e, real *u, integer *ldu, real *v, integer *ldv, real *
	work, integer *job, integer *info)
{
#if 0
  cout << "\nBi_star (in routine);" << endl;
  for (int rr = 0; rr < *n; rr++)
    {
      for (int cc = 0; cc < *n; cc++)
	cout << setprecision(4) << setw(12) << x[rr*(*n) + cc];
      cout << endl;
    }
#endif

    /* System generated locals */
    integer x_dim1, x_offset, u_dim1, u_offset, v_dim1, v_offset, i__1, i__2,
	    i__3;
    real r__1, r__2, r__3, r__4, r__5, r__6, r__7;

    /* Builtin functions */
    real r_sign(real *, real *);
    double sqrt(doublereal);

    /* Local variables */
    static integer kase, jobu, iter;
    extern real sdot_(integer *, real *, integer *, real *, integer *);
    static real test;
    extern /* Subroutine */ int srot_(integer *, real *, integer *, real *,
	    integer *, real *, real *);
    static integer nctp1;
    extern real snrm2_(integer *, real *, integer *);
    static real b, c__;
    static integer nrtp1;
    static real f, g;
    static integer i__, j, k, l, m;
    static real t, scale;
    extern /* Subroutine */ int sscal_(integer *, real *, real *, integer *);
    static real shift;
    static integer maxit;
    extern /* Subroutine */ int sswap_(integer *, real *, integer *, real *,
	    integer *);
    static logical wantu, wantv;
    extern /* Subroutine */ int srotg_(real *, real *, real *, real *),
	    saxpy_(integer *, real *, real *, integer *, real *, integer *);
    static real t1, ztest, el;
    static integer kk;
    static real cs;
    static integer ll, mm, ls;
    static real sl;
    static integer lu;
    static real sm, sn;
    static integer lm1, mm1, lp1, mp1, nct, ncu, lls, nrt;
    static real emm1, smm1;



/*     ssvdc is a subroutine to reduce a real nxp matrix x by */
/*     orthogonal transformations u and v to diagonal form.  the */
/*     diagonal elements s(i) are the singular values of x.  the */
/*     columns of u are the corresponding left singular vectors, */
/*     and the columns of v the right singular vectors. */

/*     on entry */

/*         x         real(ldx,p), where ldx.ge.n. */
/*                   x contains the matrix whose singular value */
/*                   decomposition is to be computed.  x is */
/*                   destroyed by ssvdc. */

/*         ldx       integer. */
/*                   ldx is the leading dimension of the array x. */

/*         n         integer. */
/*                   n is the number of rows of the matrix x. */

/*         p         integer. */
/*                   p is the number of columns of the matrix x. */

/*         ldu       integer. */
/*                   ldu is the leading dimension of the array u. */
/*                   (see below). */

/*         ldv       integer. */
/*                   ldv is the leading dimension of the array v. */
/*                   (see below). */

/*         work      real(n). */
/*                   work is a scratch array. */

/*         job       integer. */
/*                   job controls the computation of the singular */
/*                   vectors.  it has the decimal expansion ab */
/*                   with the following meaning */

/*                        a.eq.0    do not compute the left singular */
/*                                  vectors. */
/*                        a.eq.1    return the n left singular vectors */
/*                                  in u. */
/*                        a.ge.2    return the first min(n,p) singular */
/*                                  vectors in u. */
/*                        b.eq.0    do not compute the right singular */
/*                                  vectors. */
/*                        b.eq.1    return the right singular vectors */
/*                                  in v. */

/*     on return */

/*         s         real(mm), where mm=min(n+1,p). */
/*                   the first min(n,p) entries of s contain the */
/*                   singular values of x arranged in descending */
/*                   order of magnitude. */

/*         e         real(p). */
/*                   e ordinarily contains zeros.  however see the */
/*                   discussion of info for exceptions. */

/*         u         real(ldu,k), where ldu.ge.n.  if joba.eq.1 then */
/*                                   k.eq.n, if joba.ge.2 then */
/*                                   k.eq.min(n,p). */
/*                   u contains the matrix of left singular vectors. */
/*                   u is not referenced if joba.eq.0.  if n.le.p */
/*                   or if joba.eq.2, then u may be identified with x */
/*                   in the subroutine call. */

/*         v         real(ldv,p), where ldv.ge.p. */
/*                   v contains the matrix of right singular vectors. */
/*                   v is not referenced if job.eq.0.  if p.le.n, */
/*                   then v may be identified with x in the */
/*                   subroutine call. */

/*         info      integer. */
/*                   the singular values (and their corresponding */
/*                   singular vectors) s(info+1),s(info+2),...,s(m) */
/*                   are correct (here m=min(n,p)).  thus if */
/*                   info.eq.0, all the singular values and their */
/*                   vectors are correct.  in any event, the matrix */
/*                   b = trans(u)*x*v is the bidiagonal matrix */
/*                   with the elements of s on its diagonal and the */
/*                   elements of e on its super-diagonal (trans(u) */
/*                   is the transpose of u).  thus the singular */
/*                   values of x and b are the same. */

/*     linpack. this version dated 03/19/79 . */
/*              correction to shift calculation made 2/85. */
/*     g.w. stewart, university of maryland, argonne national lab. */

/*     ***** uses the following functions and subprograms. */

/*     external srot */
/*     blas saxpy,sdot,sscal,sswap,snrm2,srotg */
/*     fortran abs,amax1,max0,min0,mod,sqrt */

/*     internal variables */



/*     set the maximum number of iterations. */

    /* Parameter adjustments */
    x_dim1 = *ldx;
    x_offset = 1 + x_dim1 * 1;
    x -= x_offset;
    --s;
    --e;
    u_dim1 = *ldu;
    u_offset = 1 + u_dim1 * 1;
    u -= u_offset;
    v_dim1 = *ldv;
    v_offset = 1 + v_dim1 * 1;
    v -= v_offset;
    --work;

    /* Function Body */
    maxit = 30;

/*     determine what is to be computed. */

    wantu = FALSE_;
    wantv = FALSE_;
    jobu = *job % 100 / 10;
    ncu = *n;
    if (jobu > 1) {
	ncu = omin(*n,*p);
    }
    if (jobu != 0) {
	wantu = TRUE_;
    }
    if (*job % 10 != 0) {
	wantv = TRUE_;
    }

/*     reduce x to bidiagonal form, storing the diagonal elements */
/*     in s and the super-diagonal elements in e. */

    *info = 0;
/* Computing MIN */
    i__1 = *n - 1;
    nct = omin(i__1,*p);
/* Computing MAX */
/* Computing MIN */
    i__3 = *p - 2;
    i__1 = 0, i__2 = omin(i__3,*n);
    nrt = omax(i__1,i__2);
    lu = omax(nct,nrt);
    if (lu < 1) {
	goto L170;
    }
    i__1 = lu;
    for (l = 1; l <= i__1; ++l) {
	lp1 = l + 1;
	if (l > nct) {
	    goto L20;
	}

/*           compute the transformation for the l-th column and */
/*           place the l-th diagonal in s(l). */

	i__2 = *n - l + 1;
	s[l] = snrm2_(&i__2, &x[l + l * x_dim1], &c__1);
	if (s[l] == (float)0.) {
	    goto L10;
	}
	if (x[l + l * x_dim1] != (float)0.) {
	    s[l] = r_sign(&s[l], &x[l + l * x_dim1]);
	}
	i__2 = *n - l + 1;
	r__1 = (float)1. / s[l];
	sscal_(&i__2, &r__1, &x[l + l * x_dim1], &c__1);
	x[l + l * x_dim1] += (float)1.;
L10:
	s[l] = -s[l];
L20:
	if (*p < lp1) {
	    goto L50;
	}
	i__2 = *p;
	for (j = lp1; j <= i__2; ++j) {
	    if (l > nct) {
		goto L30;
	    }
	    if (s[l] == (float)0.) {
		goto L30;
	    }

/*              apply the transformation. */

	    i__3 = *n - l + 1;
	    t = -sdot_(&i__3, &x[l + l * x_dim1], &c__1, &x[l + j * x_dim1], &
		    c__1) / x[l + l * x_dim1];
	    i__3 = *n - l + 1;
	    saxpy_(&i__3, &t, &x[l + l * x_dim1], &c__1, &x[l + j * x_dim1], &
		    c__1);
L30:

/*           place the l-th row of x into  e for the */
/*           subsequent calculation of the row transformation. */

	    e[j] = x[l + j * x_dim1];
/* L40: */
	}
L50:
	if (! wantu || l > nct) {
	    goto L70;
	}

/*           place the transformation in u for subsequent back */
/*           multiplication. */

	i__2 = *n;
	for (i__ = l; i__ <= i__2; ++i__) {
	    u[i__ + l * u_dim1] = x[i__ + l * x_dim1];
/* L60: */
	}
L70:
	if (l > nrt) {
	    goto L150;
	}

/*           compute the l-th row transformation and place the */
/*           l-th super-diagonal in e(l). */

	i__2 = *p - l;
	e[l] = snrm2_(&i__2, &e[lp1], &c__1);
	if (e[l] == (float)0.) {
	    goto L80;
	}
	if (e[lp1] != (float)0.) {
	    e[l] = r_sign(&e[l], &e[lp1]);
	}
	i__2 = *p - l;
	r__1 = (float)1. / e[l];
	sscal_(&i__2, &r__1, &e[lp1], &c__1);
	e[lp1] += (float)1.;
L80:
	e[l] = -e[l];
	if (lp1 > *n || e[l] == (float)0.) {
	    goto L120;
	}

/*              apply the transformation. */

	i__2 = *n;
	for (i__ = lp1; i__ <= i__2; ++i__) {
	    work[i__] = (float)0.;
/* L90: */
	}
	i__2 = *p;
	for (j = lp1; j <= i__2; ++j) {
	    i__3 = *n - l;
	    saxpy_(&i__3, &e[j], &x[lp1 + j * x_dim1], &c__1, &work[lp1], &
		    c__1);
/* L100: */
	}
	i__2 = *p;
	for (j = lp1; j <= i__2; ++j) {
	    i__3 = *n - l;
	    r__1 = -e[j] / e[lp1];
	    saxpy_(&i__3, &r__1, &work[lp1], &c__1, &x[lp1 + j * x_dim1], &
		    c__1);
/* L110: */
	}
L120:
	if (! wantv) {
	    goto L140;
	}

/*              place the transformation in v for subsequent */
/*              back multiplication. */

	i__2 = *p;
	for (i__ = lp1; i__ <= i__2; ++i__) {
	    v[i__ + l * v_dim1] = e[i__];
/* L130: */
	}
L140:
L150:
/* L160: */
	;
    }
L170:

/*     set up the final bidiagonal matrix or order m. */

/* Computing MIN */
    i__1 = *p, i__2 = *n + 1;
    m = omin(i__1,i__2);
    nctp1 = nct + 1;
    nrtp1 = nrt + 1;
    if (nct < *p) {
	s[nctp1] = x[nctp1 + nctp1 * x_dim1];
    }
    if (*n < m) {
	s[m] = (float)0.;
    }
    if (nrtp1 < m) {
	e[nrtp1] = x[nrtp1 + m * x_dim1];
    }
    e[m] = (float)0.;

/*     if required, generate u. */

    if (! wantu) {
	goto L300;
    }
    if (ncu < nctp1) {
	goto L200;
    }
    i__1 = ncu;
    for (j = nctp1; j <= i__1; ++j) {
	i__2 = *n;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    u[i__ + j * u_dim1] = (float)0.;
/* L180: */
	}
	u[j + j * u_dim1] = (float)1.;
/* L190: */
    }
L200:
    if (nct < 1) {
	goto L290;
    }
    i__1 = nct;
    for (ll = 1; ll <= i__1; ++ll) {
	l = nct - ll + 1;
	if (s[l] == (float)0.) {
	    goto L250;
	}
	lp1 = l + 1;
	if (ncu < lp1) {
	    goto L220;
	}
	i__2 = ncu;
	for (j = lp1; j <= i__2; ++j) {
	    i__3 = *n - l + 1;
	    t = -sdot_(&i__3, &u[l + l * u_dim1], &c__1, &u[l + j * u_dim1], &
		    c__1) / u[l + l * u_dim1];
	    i__3 = *n - l + 1;
	    saxpy_(&i__3, &t, &u[l + l * u_dim1], &c__1, &u[l + j * u_dim1], &
		    c__1);
/* L210: */
	}
L220:
	i__2 = *n - l + 1;
	sscal_(&i__2, &c_b44, &u[l + l * u_dim1], &c__1);
	u[l + l * u_dim1] += (float)1.;
	lm1 = l - 1;
	if (lm1 < 1) {
	    goto L240;
	}
	i__2 = lm1;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    u[i__ + l * u_dim1] = (float)0.;
/* L230: */
	}
L240:
	goto L270;
L250:
	i__2 = *n;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    u[i__ + l * u_dim1] = (float)0.;
/* L260: */
	}
	u[l + l * u_dim1] = (float)1.;
L270:
/* L280: */
	;
    }
L290:
L300:

/*     if it is required, generate v. */

    if (! wantv) {
	goto L350;
    }
    i__1 = *p;
    for (ll = 1; ll <= i__1; ++ll) {
	l = *p - ll + 1;
	lp1 = l + 1;
	if (l > nrt) {
	    goto L320;
	}
	if (e[l] == (float)0.) {
	    goto L320;
	}
	i__2 = *p;
	for (j = lp1; j <= i__2; ++j) {
	    i__3 = *p - l;
	    t = -sdot_(&i__3, &v[lp1 + l * v_dim1], &c__1, &v[lp1 + j *
		    v_dim1], &c__1) / v[lp1 + l * v_dim1];
	    i__3 = *p - l;
	    saxpy_(&i__3, &t, &v[lp1 + l * v_dim1], &c__1, &v[lp1 + j *
		    v_dim1], &c__1);
/* L310: */
	}
L320:
	i__2 = *p;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    v[i__ + l * v_dim1] = (float)0.;
/* L330: */
	}
	v[l + l * v_dim1] = (float)1.;
/* L340: */
    }
L350:

/*     main iteration loop for the singular values. */

    mm = m;
    iter = 0;
L360:

/*        quit if all the singular values have been found. */

/*     ...exit */
    if (m == 0) {
	goto L620;
    }

/*        if too many iterations have been performed, set */
/*        flag and return. */

    if (iter < maxit) {
      //      cout << "Iteration = " << iter << endl;
	goto L370;
    }
    *info = m;
/*     ......exit */
    goto L620;
L370:

/*        this section of the program inspects for */
/*        negligible elements in the s and e arrays.  on */
/*        completion the variables kase and l are set as follows. */

/*           kase = 1     if s(m) and e(l-1) are negligible and l.lt.m */
/*           kase = 2     if s(l) is negligible and l.lt.m */
/*           kase = 3     if e(l-1) is negligible, l.lt.m, and */
/*                        s(l), ..., s(m) are not negligible (qr step). */
/*           kase = 4     if e(m-1) is negligible (convergence). */

    i__1 = m;
    for (ll = 1; ll <= i__1; ++ll) {
	l = m - ll;
/*        ...exit */
	if (l == 0) {
	    goto L400;
	}
	test = (r__1 = s[l], abs(r__1)) + (r__2 = s[l + 1], abs(r__2));
	ztest = test + (r__1 = e[l], abs(r__1));
	if (ztest != test) {
	    goto L380;
	}
	e[l] = (float)0.;
/*        ......exit */
	goto L400;
L380:
/* L390: */
	;
    }
L400:
    if (l != m - 1) {
	goto L410;
    }
    kase = 4;
    goto L480;
L410:
    lp1 = l + 1;
    mp1 = m + 1;
    i__1 = mp1;
    for (lls = lp1; lls <= i__1; ++lls) {
	ls = m - lls + lp1;
/*           ...exit */
	if (ls == l) {
	    goto L440;
	}
	test = (float)0.;
	if (ls != m) {
	    test += (r__1 = e[ls], abs(r__1));
	}
	if (ls != l + 1) {
	    test += (r__1 = e[ls - 1], abs(r__1));
	}
	ztest = test + (r__1 = s[ls], abs(r__1));
	if (ztest != test) {
	    goto L420;
	}
	s[ls] = (float)0.;
/*           ......exit */
	goto L440;
L420:
/* L430: */
	;
    }
L440:
    if (ls != l) {
	goto L450;
    }
    kase = 3;
    goto L470;
L450:
    if (ls != m) {
	goto L460;
    }
    kase = 1;
    goto L470;
L460:
    kase = 2;
    l = ls;
L470:
L480:
    ++l;

/*        perform the task indicated by kase. */

    switch (kase) {
	case 1:  goto L490;
	case 2:  goto L520;
	case 3:  goto L540;
	case 4:  goto L570;
    }

/*        deflate negligible s(m). */

L490:
    mm1 = m - 1;
    f = e[m - 1];
    e[m - 1] = (float)0.;
    i__1 = mm1;
    for (kk = l; kk <= i__1; ++kk) {
	k = mm1 - kk + l;
	t1 = s[k];
	srotg_(&t1, &f, &cs, &sn);
	s[k] = t1;
	if (k == l) {
	    goto L500;
	}
	f = -sn * e[k - 1];
	e[k - 1] = cs * e[k - 1];
L500:
	if (wantv) {
	    srot_(p, &v[k * v_dim1 + 1], &c__1, &v[m * v_dim1 + 1], &c__1, &
		    cs, &sn);
	}
/* L510: */
    }
    goto L610;

/*        split at negligible s(l). */

L520:
    f = e[l - 1];
    e[l - 1] = (float)0.;
    i__1 = m;
    for (k = l; k <= i__1; ++k) {
	t1 = s[k];
	srotg_(&t1, &f, &cs, &sn);
	s[k] = t1;
	f = -sn * e[k];
	e[k] = cs * e[k];
	if (wantu) {
	    srot_(n, &u[k * u_dim1 + 1], &c__1, &u[(l - 1) * u_dim1 + 1], &
		    c__1, &cs, &sn);
	}
/* L530: */
    }
    goto L610;

/*        perform one qr step. */

L540:

/*           calculate the shift. */

/* Computing MAX */
    r__6 = (r__1 = s[m], abs(r__1)), r__7 = (r__2 = s[m - 1], abs(r__2)),
	    r__6 = omax(r__6,r__7), r__7 = (r__3 = e[m - 1], abs(r__3)), r__6 =
	    omax(r__6,r__7), r__7 = (r__4 = s[l], abs(r__4)), r__6 = omax(r__6,
	    r__7), r__7 = (r__5 = e[l], abs(r__5));
    scale = omax(r__6,r__7);
    sm = s[m] / scale;
    smm1 = s[m - 1] / scale;
    emm1 = e[m - 1] / scale;
    sl = s[l] / scale;
    el = e[l] / scale;
/* Computing 2nd power */
    r__1 = emm1;
    b = ((smm1 + sm) * (smm1 - sm) + r__1 * r__1) / (float)2.;
/* Computing 2nd power */
    r__1 = sm * emm1;
    c__ = r__1 * r__1;
    shift = (float)0.;
    if (b == (float)0. && c__ == (float)0.) {
	goto L550;
    }
/* Computing 2nd power */
    r__1 = b;
    if ( (r__1 * r__1 + c__) < 0 )
      cout << "ERROR: sqrt(NEG)" << endl;
    shift = (float)sqrt(r__1 * r__1 + c__);
    if (b < (float)0.) {
	shift = -shift;
    }
    shift = c__ / (b + shift);
L550:
    f = (sl + sm) * (sl - sm) + shift;
    g = sl * el;

/*           chase zeros. */

    mm1 = m - 1;
    i__1 = mm1;
    for (k = l; k <= i__1; ++k) {
	srotg_(&f, &g, &cs, &sn);
	if (k != l) {
	    e[k - 1] = f;
	}
	f = cs * s[k] + sn * e[k];
	e[k] = cs * e[k] - sn * s[k];
	g = sn * s[k + 1];
	s[k + 1] = cs * s[k + 1];
	if (wantv) {
	    srot_(p, &v[k * v_dim1 + 1], &c__1, &v[(k + 1) * v_dim1 + 1], &
		    c__1, &cs, &sn);
	}
	srotg_(&f, &g, &cs, &sn);
	s[k] = f;
	f = cs * e[k] + sn * s[k + 1];
	s[k + 1] = -sn * e[k] + cs * s[k + 1];
	g = sn * e[k + 1];
	e[k + 1] = cs * e[k + 1];
	if (wantu && k < *n) {
	    srot_(n, &u[k * u_dim1 + 1], &c__1, &u[(k + 1) * u_dim1 + 1], &
		    c__1, &cs, &sn);
	}
/* L560: */
    }
    e[m - 1] = f;
    ++iter;
    goto L610;

/*        convergence. */

L570:

/*           make the singular value  positive. */

    if (s[l] >= (float)0.) {
	goto L580;
    }
    s[l] = -s[l];
    if (wantv) {
	sscal_(p, &c_b44, &v[l * v_dim1 + 1], &c__1);
    }
L580:

/*           order the singular value. */

L590:
    if (l == mm) {
	goto L600;
    }
/*           ...exit */
    if (s[l] >= s[l + 1]) {
	goto L600;
    }
    t = s[l];
    s[l] = s[l + 1];
    s[l + 1] = t;
    if (wantv && l < *p) {
	sswap_(p, &v[l * v_dim1 + 1], &c__1, &v[(l + 1) * v_dim1 + 1], &c__1);
    }
    if (wantu && l < *n) {
	sswap_(n, &u[l * u_dim1 + 1], &c__1, &u[(l + 1) * u_dim1 + 1], &c__1);
    }
    ++l;
    goto L590;
L600:
    iter = 0;
    --m;
L610:
    goto L360;
L620:
    return 0;
} /* ssvdc_ */

/* Subroutine */ int saxpy_(integer *n, real *sa, real *sx, integer *incx,
	real *sy, integer *incy)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__, m, ix, iy, mp1;


/*     constant times a vector plus a vector. */
/*     uses unrolled loop for increments equal to one. */
/*     jack dongarra, linpack, 3/11/78. */
/*     modified 12/3/93, array(1) declarations changed to array(*) */


    /* Parameter adjustments */
    --sy;
    --sx;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    if (*sa == (float)0.) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*        code for unequal increments or equal increments */
/*          not equal to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	sy[iy] += *sa * sx[ix];
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*        code for both increments equal to 1 */


/*        clean-up loop */

L20:
    m = *n % 4;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	sy[i__] += *sa * sx[i__];
/* L30: */
    }
    if (*n < 4) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i__ = mp1; i__ <= i__1; i__ += 4) {
	sy[i__] += *sa * sx[i__];
	sy[i__ + 1] += *sa * sx[i__ + 1];
	sy[i__ + 2] += *sa * sx[i__ + 2];
	sy[i__ + 3] += *sa * sx[i__ + 3];
/* L50: */
    }
    return 0;
} /* saxpy_ */

real sdot_(integer *n, real *sx, integer *incx, real *sy, integer *incy)
{
    /* System generated locals */
    integer i__1;
    real ret_val;

    /* Local variables */
    static integer i__, m;
    static real stemp;
    static integer ix, iy, mp1;


/*     forms the dot product of two vectors. */
/*     uses unrolled loops for increments equal to one. */
/*     jack dongarra, linpack, 3/11/78. */
/*     modified 12/3/93, array(1) declarations changed to array(*) */


    /* Parameter adjustments */
    --sy;
    --sx;

    /* Function Body */
    stemp = (float)0.;
    ret_val = (float)0.;
    if (*n <= 0) {
	return ret_val;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*        code for unequal increments or equal increments */
/*          not equal to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	stemp += sx[ix] * sy[iy];
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    ret_val = stemp;
    return ret_val;

/*        code for both increments equal to 1 */


/*        clean-up loop */

L20:
    m = *n % 5;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	stemp += sx[i__] * sy[i__];
/* L30: */
    }
    if (*n < 5) {
	goto L60;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i__ = mp1; i__ <= i__1; i__ += 5) {
	stemp = stemp + sx[i__] * sy[i__] + sx[i__ + 1] * sy[i__ + 1] + sx[
		i__ + 2] * sy[i__ + 2] + sx[i__ + 3] * sy[i__ + 3] + sx[i__ +
		4] * sy[i__ + 4];
/* L50: */
    }
L60:
    ret_val = stemp;
    return ret_val;
} /* sdot_ */

real snrm2_(integer *n, real *x, integer *incx)
{
    /* System generated locals */
    integer i__1, i__2;
    real ret_val, r__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static real norm, scale, absxi;
    static integer ix;
    static real ssq;

/*     .. Scalar Arguments .. */
/*     .. Array Arguments .. */
/*     .. */

/*  SNRM2 returns the euclidean norm of a vector via the function */
/*  name, so that */

/*     SNRM2 := sqrt( x'*x ) */



/*  -- This version written on 25-October-1982. */
/*     Modified on 14-October-1993 to inline the call to SLASSQ. */
/*     Sven Hammarling, Nag Ltd. */


/*     .. Parameters .. */
/*     .. Local Scalars .. */
/*     .. Intrinsic Functions .. */
/*     .. */
/*     .. Executable Statements .. */
    /* Parameter adjustments */
    --x;

    /* Function Body */
    if (*n < 1 || *incx < 1) {
	norm = (float)0.;
    } else if (*n == 1) {
	norm = abs(x[1]);
    } else {
	scale = (float)0.;
	ssq = (float)1.;
/*        The following loop is equivalent to this call to the LAPACK */
/*        auxiliary routine: */
/*        CALL SLASSQ( N, X, INCX, SCALE, SSQ ) */

	i__1 = (*n - 1) * *incx + 1;
	i__2 = *incx;
	for (ix = 1; i__2 < 0 ? ix >= i__1 : ix <= i__1; ix += i__2) {
	    if (x[ix] != (float)0.) {
		absxi = (r__1 = x[ix], abs(r__1));
		if (scale < absxi) {
/* Computing 2nd power */
		    r__1 = scale / absxi;
		    ssq = ssq * (r__1 * r__1) + (float)1.;
		    scale = absxi;
		} else {
/* Computing 2nd power */
		    r__1 = absxi / scale;
		    ssq += r__1 * r__1;
		}
	    }
/* L10: */
	}
	if ( ssq < 0 )
	  cout << "ERROR: sqrt(NEG)" << endl;
	norm = float(scale * sqrt(ssq));
    }

    ret_val = norm;
    return ret_val;

/*     End of SNRM2. */

} /* snrm2_ */

/* Subroutine */ int srot_(integer *n, real *sx, integer *incx, real *sy,
	integer *incy, real *c__, real *s)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;
    static real stemp;
    static integer ix, iy;


/*     applies a plane rotation. */
/*     jack dongarra, linpack, 3/11/78. */
/*     modified 12/3/93, array(1) declarations changed to array(*) */


    /* Parameter adjustments */
    --sy;
    --sx;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*       code for unequal increments or equal increments not equal */
/*         to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	stemp = *c__ * sx[ix] + *s * sy[iy];
	sy[iy] = *c__ * sy[iy] - *s * sx[ix];
	sx[ix] = stemp;
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*       code for both increments equal to 1 */

L20:
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	stemp = *c__ * sx[i__] + *s * sy[i__];
	sy[i__] = *c__ * sy[i__] - *s * sx[i__];
	sx[i__] = stemp;
/* L30: */
    }
    return 0;
} /* srot_ */

/* Subroutine */ int srotg_(real *sa, real *sb, real *c__, real *s)
{
    /* System generated locals */
    real r__1, r__2;

    /* Builtin functions */
    double sqrt(doublereal);
    real r_sign(real *, real *);

    /* Local variables */
    static real r__, scale, z__, roe;


/*     construct givens plane rotation. */
/*     jack dongarra, linpack, 3/11/78. */


    roe = *sb;
    if (abs(*sa) > abs(*sb)) {
	roe = *sa;
    }
    scale = abs(*sa) + abs(*sb);
    if (scale != (float)0.) {
	goto L10;
    }
    *c__ = (float)1.;
    *s = (float)0.;
    r__ = (float)0.;
    z__ = (float)0.;
    goto L20;
L10:
/* Computing 2nd power */
    r__1 = *sa / scale;
/* Computing 2nd power */
    r__2 = *sb / scale;
    if ( (r__1 * r__1 + r__2 * r__2) < 0 )
      cout << "ERROR: sqrt(NEG)" << endl;
    r__ = float(scale * sqrt(r__1 * r__1 + r__2 * r__2));
    r__ = r_sign(&c_b122, &roe) * r__;
    *c__ = *sa / r__;
    *s = *sb / r__;
    z__ = (float)1.;
    if (abs(*sa) > abs(*sb)) {
	z__ = *s;
    }
    if (abs(*sb) >= abs(*sa) && *c__ != (float)0.) {
	z__ = (float)1. / *c__;
    }
L20:
    *sa = r__;
    *sb = z__;
    return 0;
} /* srotg_ */

/* Subroutine */ int sscal_(integer *n, real *sa, real *sx, integer *incx)
{
    /* System generated locals */
    integer i__1, i__2;

    /* Local variables */
    static integer i__, m, nincx, mp1;


/*     scales a vector by a constant. */
/*     uses unrolled loops for increment equal to 1. */
/*     jack dongarra, linpack, 3/11/78. */
/*     modified 3/93 to return if incx .le. 0. */
/*     modified 12/3/93, array(1) declarations changed to array(*) */


    /* Parameter adjustments */
    --sx;

    /* Function Body */
    if (*n <= 0 || *incx <= 0) {
	return 0;
    }
    if (*incx == 1) {
	goto L20;
    }

/*        code for increment not equal to 1 */

    nincx = *n * *incx;
    i__1 = nincx;
    i__2 = *incx;
    for (i__ = 1; i__2 < 0 ? i__ >= i__1 : i__ <= i__1; i__ += i__2) {
	sx[i__] = *sa * sx[i__];
/* L10: */
    }
    return 0;

/*        code for increment equal to 1 */


/*        clean-up loop */

L20:
    m = *n % 5;
    if (m == 0) {
	goto L40;
    }
    i__2 = m;
    for (i__ = 1; i__ <= i__2; ++i__) {
	sx[i__] = *sa * sx[i__];
/* L30: */
    }
    if (*n < 5) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__2 = *n;
    for (i__ = mp1; i__ <= i__2; i__ += 5) {
	sx[i__] = *sa * sx[i__];
	sx[i__ + 1] = *sa * sx[i__ + 1];
	sx[i__ + 2] = *sa * sx[i__ + 2];
	sx[i__ + 3] = *sa * sx[i__ + 3];
	sx[i__ + 4] = *sa * sx[i__ + 4];
/* L50: */
    }
    return 0;
} /* sscal_ */

/* Subroutine */ int sswap_(integer *n, real *sx, integer *incx, real *sy,
	integer *incy)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__, m;
    static real stemp;
    static integer ix, iy, mp1;


/*     interchanges two vectors. */
/*     uses unrolled loops for increments equal to 1. */
/*     jack dongarra, linpack, 3/11/78. */
/*     modified 12/3/93, array(1) declarations changed to array(*) */


    /* Parameter adjustments */
    --sy;
    --sx;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*       code for unequal increments or equal increments not equal */
/*         to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	stemp = sx[ix];
	sx[ix] = sy[iy];
	sy[iy] = stemp;
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*       code for both increments equal to 1 */


/*       clean-up loop */

L20:
    m = *n % 3;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	stemp = sx[i__];
	sx[i__] = sy[i__];
	sy[i__] = stemp;
/* L30: */
    }
    if (*n < 3) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i__ = mp1; i__ <= i__1; i__ += 3) {
	stemp = sx[i__];
	sx[i__] = sy[i__];
	sy[i__] = stemp;
	stemp = sx[i__ + 1];
	sx[i__ + 1] = sy[i__ + 1];
	sy[i__ + 1] = stemp;
	stemp = sx[i__ + 2];
	sx[i__ + 2] = sy[i__ + 2];
	sy[i__ + 2] = stemp;
/* L50: */
    }
    return 0;
} /* sswap_ */

#ifdef KR_headers
real r_sign(a,b) real *a, *b;
#else
real r_sign(real *a, real *b)
#endif
{
  real x;
  x = (*a >= 0 ? *a : - *a);
  return( *b >= 0 ? x : -x);
}

/* ------------------------------------------------------------------------ */

/* dsvdc.f -- translated by f2c (version 19990311).
   You must link the resulting object file with the libraries:
	-lf2c -lm   (in that order)
*/

/* Duane Marhefka Note:  It is not necessary to link with f2c.  Only one
   function (d_sign) in the f2c library was required, so I appended that
   function from the libf77.c file to the end of this file.

   This file was created with the following commands:
   > f2cx -C++ -R dsvdc.f
   > move dsvdc.c dsvdc.cpp

   And then the d_sign function was appended from libf77.c.
 */

/* Table of constant values */

static doublereal c_b23 = 1.;
static integer c__1d = 1;
static doublereal c_b79 = -1.;

/* Subroutine */ int daxpy_(integer *n, doublereal *da, doublereal *dx,
	integer *incx, doublereal *dy, integer *incy)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__, m, ix, iy, mp1;


/*     constant times a vector plus a vector. */
/*     uses unrolled loops for increments equal to one. */
/*     jack dongarra, linpack, 3/11/78. */
/*     modified 12/3/93, array(1) declarations changed to array(*) */


    /* Parameter adjustments */
    --dy;
    --dx;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    if (*da == 0.) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*        code for unequal increments or equal increments */
/*          not equal to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dy[iy] += *da * dx[ix];
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*        code for both increments equal to 1 */


/*        clean-up loop */

L20:
    m = *n % 4;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dy[i__] += *da * dx[i__];
/* L30: */
    }
    if (*n < 4) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i__ = mp1; i__ <= i__1; i__ += 4) {
	dy[i__] += *da * dx[i__];
	dy[i__ + 1] += *da * dx[i__ + 1];
	dy[i__ + 2] += *da * dx[i__ + 2];
	dy[i__ + 3] += *da * dx[i__ + 3];
/* L50: */
    }
    return 0;
} /* daxpy_ */

doublereal ddot_(integer *n, doublereal *dx, integer *incx, doublereal *dy,
	integer *incy)
{
    /* System generated locals */
    integer i__1;
    doublereal ret_val;

    /* Local variables */
    static integer i__, m;
    static doublereal dtemp;
    static integer ix, iy, mp1;


/*     forms the dot product of two vectors. */
/*     uses unrolled loops for increments equal to one. */
/*     jack dongarra, linpack, 3/11/78. */
/*     modified 12/3/93, array(1) declarations changed to array(*) */


    /* Parameter adjustments */
    --dy;
    --dx;

    /* Function Body */
    ret_val = 0.;
    dtemp = 0.;
    if (*n <= 0) {
	return ret_val;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*        code for unequal increments or equal increments */
/*          not equal to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dtemp += dx[ix] * dy[iy];
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    ret_val = dtemp;
    return ret_val;

/*        code for both increments equal to 1 */


/*        clean-up loop */

L20:
    m = *n % 5;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dtemp += dx[i__] * dy[i__];
/* L30: */
    }
    if (*n < 5) {
	goto L60;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i__ = mp1; i__ <= i__1; i__ += 5) {
	dtemp = dtemp + dx[i__] * dy[i__] + dx[i__ + 1] * dy[i__ + 1] + dx[
		i__ + 2] * dy[i__ + 2] + dx[i__ + 3] * dy[i__ + 3] + dx[i__ +
		4] * dy[i__ + 4];
/* L50: */
    }
L60:
    ret_val = dtemp;
    return ret_val;
} /* ddot_ */

doublereal dnrm2_(integer *n, doublereal *x, integer *incx)
{
    /* System generated locals */
    integer i__1, i__2;
    doublereal ret_val, d__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static doublereal norm, scale, absxi;
    static integer ix;
    static doublereal ssq;

/*     .. Scalar Arguments .. */
/*     .. Array Arguments .. */
/*     .. */

/*  DNRM2 returns the euclidean norm of a vector via the function */
/*  name, so that */

/*     DNRM2 := sqrt( x'*x ) */



/*  -- This version written on 25-October-1982. */
/*     Modified on 14-October-1993 to inline the call to DLASSQ. */
/*     Sven Hammarling, Nag Ltd. */


/*     .. Parameters .. */
/*     .. Local Scalars .. */
/*     .. Intrinsic Functions .. */
/*     .. */
/*     .. Executable Statements .. */
    /* Parameter adjustments */
    --x;

    /* Function Body */
    if (*n < 1 || *incx < 1) {
	norm = 0.;
    } else if (*n == 1) {
	norm = abs(x[1]);
    } else {
	scale = 0.;
	ssq = 1.;
/*        The following loop is equivalent to this call to the LAPACK */
/*        auxiliary routine: */
/*        CALL DLASSQ( N, X, INCX, SCALE, SSQ ) */

	i__1 = (*n - 1) * *incx + 1;
	i__2 = *incx;
	for (ix = 1; i__2 < 0 ? ix >= i__1 : ix <= i__1; ix += i__2) {
	    if (x[ix] != 0.) {
		absxi = (d__1 = x[ix], abs(d__1));
		if (scale < absxi) {
/* Computing 2nd power */
		    d__1 = scale / absxi;
		    ssq = ssq * (d__1 * d__1) + 1.;
		    scale = absxi;
		} else {
/* Computing 2nd power */
		    d__1 = absxi / scale;
		    ssq += d__1 * d__1;
		}
	    }
/* L10: */
	}
	norm = scale * sqrt(ssq);
    }

    ret_val = norm;
    return ret_val;

/*     End of DNRM2. */

} /* dnrm2_ */

/* Subroutine */ int drot_(integer *n, doublereal *dx, integer *incx,
	doublereal *dy, integer *incy, doublereal *c__, doublereal *s)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;
    static doublereal dtemp;
    static integer ix, iy;


/*     applies a plane rotation. */
/*     jack dongarra, linpack, 3/11/78. */
/*     modified 12/3/93, array(1) declarations changed to array(*) */


    /* Parameter adjustments */
    --dy;
    --dx;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*       code for unequal increments or equal increments not equal */
/*         to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dtemp = *c__ * dx[ix] + *s * dy[iy];
	dy[iy] = *c__ * dy[iy] - *s * dx[ix];
	dx[ix] = dtemp;
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*       code for both increments equal to 1 */

L20:
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dtemp = *c__ * dx[i__] + *s * dy[i__];
	dy[i__] = *c__ * dy[i__] - *s * dx[i__];
	dx[i__] = dtemp;
/* L30: */
    }
    return 0;
} /* drot_ */

/* Subroutine */ int drotg_(doublereal *da, doublereal *db, doublereal *c__,
	doublereal *s)
{
    /* System generated locals */
    doublereal d__1, d__2;

    /* Builtin functions */
    double sqrt(doublereal), d_sign(doublereal *, doublereal *);

    /* Local variables */
    static doublereal r__, scale, z__, roe;


/*     construct givens plane rotation. */
/*     jack dongarra, linpack, 3/11/78. */


    roe = *db;
    if (abs(*da) > abs(*db)) {
	roe = *da;
    }
    scale = abs(*da) + abs(*db);
    if (scale != 0.) {
	goto L10;
    }
    *c__ = 1.;
    *s = 0.;
    r__ = 0.;
    z__ = 0.;
    goto L20;
L10:
/* Computing 2nd power */
    d__1 = *da / scale;
/* Computing 2nd power */
    d__2 = *db / scale;
    r__ = scale * sqrt(d__1 * d__1 + d__2 * d__2);
    r__ = d_sign(&c_b23, &roe) * r__;
    *c__ = *da / r__;
    *s = *db / r__;
    z__ = 1.;
    if (abs(*da) > abs(*db)) {
	z__ = *s;
    }
    if (abs(*db) >= abs(*da) && *c__ != 0.) {
	z__ = 1. / *c__;
    }
L20:
    *da = r__;
    *db = z__;
    return 0;
} /* drotg_ */

/* Subroutine */ int dscal_(integer *n, doublereal *da, doublereal *dx,
	integer *incx)
{
    /* System generated locals */
    integer i__1, i__2;

    /* Local variables */
    static integer i__, m, nincx, mp1;


/*     scales a vector by a constant. */
/*     uses unrolled loops for increment equal to one. */
/*     jack dongarra, linpack, 3/11/78. */
/*     modified 3/93 to return if incx .le. 0. */
/*     modified 12/3/93, array(1) declarations changed to array(*) */


    /* Parameter adjustments */
    --dx;

    /* Function Body */
    if (*n <= 0 || *incx <= 0) {
	return 0;
    }
    if (*incx == 1) {
	goto L20;
    }

/*        code for increment not equal to 1 */

    nincx = *n * *incx;
    i__1 = nincx;
    i__2 = *incx;
    for (i__ = 1; i__2 < 0 ? i__ >= i__1 : i__ <= i__1; i__ += i__2) {
	dx[i__] = *da * dx[i__];
/* L10: */
    }
    return 0;

/*        code for increment equal to 1 */


/*        clean-up loop */

L20:
    m = *n % 5;
    if (m == 0) {
	goto L40;
    }
    i__2 = m;
    for (i__ = 1; i__ <= i__2; ++i__) {
	dx[i__] = *da * dx[i__];
/* L30: */
    }
    if (*n < 5) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__2 = *n;
    for (i__ = mp1; i__ <= i__2; i__ += 5) {
	dx[i__] = *da * dx[i__];
	dx[i__ + 1] = *da * dx[i__ + 1];
	dx[i__ + 2] = *da * dx[i__ + 2];
	dx[i__ + 3] = *da * dx[i__ + 3];
	dx[i__ + 4] = *da * dx[i__ + 4];
/* L50: */
    }
    return 0;
} /* dscal_ */

/* Subroutine */ int dswap_(integer *n, doublereal *dx, integer *incx,
	doublereal *dy, integer *incy)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__, m;
    static doublereal dtemp;
    static integer ix, iy, mp1;


/*     interchanges two vectors. */
/*     uses unrolled loops for increments equal one. */
/*     jack dongarra, linpack, 3/11/78. */
/*     modified 12/3/93, array(1) declarations changed to array(*) */


    /* Parameter adjustments */
    --dy;
    --dx;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*       code for unequal increments or equal increments not equal */
/*         to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dtemp = dx[ix];
	dx[ix] = dy[iy];
	dy[iy] = dtemp;
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*       code for both increments equal to 1 */


/*       clean-up loop */

L20:
    m = *n % 3;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dtemp = dx[i__];
	dx[i__] = dy[i__];
	dy[i__] = dtemp;
/* L30: */
    }
    if (*n < 3) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i__ = mp1; i__ <= i__1; i__ += 3) {
	dtemp = dx[i__];
	dx[i__] = dy[i__];
	dy[i__] = dtemp;
	dtemp = dx[i__ + 1];
	dx[i__ + 1] = dy[i__ + 1];
	dy[i__ + 1] = dtemp;
	dtemp = dx[i__ + 2];
	dx[i__ + 2] = dy[i__ + 2];
	dy[i__ + 2] = dtemp;
/* L50: */
    }
    return 0;
} /* dswap_ */

/* Subroutine */ int dsvdc_(doublereal *x, integer *ldx, integer *n, integer *
	p, doublereal *s, doublereal *e, doublereal *u, integer *ldu,
	doublereal *v, integer *ldv, doublereal *work, integer *job, integer *
	info)
{
    /* System generated locals */
    integer x_dim1, x_offset, u_dim1, u_offset, v_dim1, v_offset, i__1, i__2,
	    i__3;
    doublereal d__1, d__2, d__3, d__4, d__5, d__6, d__7;

    /* Builtin functions */
    double d_sign(doublereal *, doublereal *), sqrt(doublereal);

    /* Local variables */
    static integer kase;
    extern doublereal ddot_(integer *, doublereal *, integer *, doublereal *,
	    integer *);
    static integer jobu, iter;
    extern /* Subroutine */ int drot_(integer *, doublereal *, integer *,
	    doublereal *, integer *, doublereal *, doublereal *);
    static doublereal test;
    extern doublereal dnrm2_(integer *, doublereal *, integer *);
    static integer nctp1;
    static doublereal b, c__;
    static integer nrtp1;
    static doublereal f, g;
    static integer i__, j, k, l, m;
    static doublereal t, scale;
    extern /* Subroutine */ int dscal_(integer *, doublereal *, doublereal *,
	    integer *);
    static doublereal shift;
    extern /* Subroutine */ int dswap_(integer *, doublereal *, integer *,
	    doublereal *, integer *), drotg_(doublereal *, doublereal *,
	    doublereal *, doublereal *);
    static integer maxit;
    extern /* Subroutine */ int daxpy_(integer *, doublereal *, doublereal *,
	    integer *, doublereal *, integer *);
    static logical wantu, wantv;
    static doublereal t1, ztest, el;
    static integer kk;
    static doublereal cs;
    static integer ll, mm, ls;
    static doublereal sl;
    static integer lu;
    static doublereal sm, sn;
    static integer lm1, mm1, lp1, mp1, nct, ncu, lls, nrt;
    static doublereal emm1, smm1;



/*     dsvdc is a subroutine to reduce a double precision nxp matrix x */
/*     by orthogonal transformations u and v to diagonal form.  the */
/*     diagonal elements s(i) are the singular values of x.  the */
/*     columns of u are the corresponding left singular vectors, */
/*     and the columns of v the right singular vectors. */

/*     on entry */

/*         x         double precision(ldx,p), where ldx.ge.n. */
/*                   x contains the matrix whose singular value */
/*                   decomposition is to be computed.  x is */
/*                   destroyed by dsvdc. */

/*         ldx       integer. */
/*                   ldx is the leading dimension of the array x. */

/*         n         integer. */
/*                   n is the number of rows of the matrix x. */

/*         p         integer. */
/*                   p is the number of columns of the matrix x. */

/*         ldu       integer. */
/*                   ldu is the leading dimension of the array u. */
/*                   (see below). */

/*         ldv       integer. */
/*                   ldv is the leading dimension of the array v. */
/*                   (see below). */

/*         work      double precision(n). */
/*                   work is a scratch array. */

/*         job       integer. */
/*                   job controls the computation of the singular */
/*                   vectors.  it has the decimal expansion ab */
/*                   with the following meaning */

/*                        a.eq.0    do not compute the left singular */
/*                                  vectors. */
/*                        a.eq.1    return the n left singular vectors */
/*                                  in u. */
/*                        a.ge.2    return the first omin(n,p) singular */
/*                                  vectors in u. */
/*                        b.eq.0    do not compute the right singular */
/*                                  vectors. */
/*                        b.eq.1    return the right singular vectors */
/*                                  in v. */

/*     on return */

/*         s         double precision(mm), where mm=omin(n+1,p). */
/*                   the first omin(n,p) entries of s contain the */
/*                   singular values of x arranged in descending */
/*                   order of magnitude. */

/*         e         double precision(p), */
/*                   e ordinarily contains zeros.  however see the */
/*                   discussion of info for exceptions. */

/*         u         double precision(ldu,k), where ldu.ge.n.  if */
/*                                   joba.eq.1 then k.eq.n, if joba.ge.2 */
/*                                   then k.eq.omin(n,p). */
/*                   u contains the matrix of left singular vectors. */
/*                   u is not referenced if joba.eq.0.  if n.le.p */
/*                   or if joba.eq.2, then u may be identified with x */
/*                   in the subroutine call. */

/*         v         double precision(ldv,p), where ldv.ge.p. */
/*                   v contains the matrix of right singular vectors. */
/*                   v is not referenced if job.eq.0.  if p.le.n, */
/*                   then v may be identified with x in the */
/*                   subroutine call. */

/*         info      integer. */
/*                   the singular values (and their corresponding */
/*                   singular vectors) s(info+1),s(info+2),...,s(m) */
/*                   are correct (here m=omin(n,p)).  thus if */
/*                   info.eq.0, all the singular values and their */
/*                   vectors are correct.  in any event, the matrix */
/*                   b = trans(u)*x*v is the bidiagonal matrix */
/*                   with the elements of s on its diagonal and the */
/*                   elements of e on its super-diagonal (trans(u) */
/*                   is the transpose of u).  thus the singular */
/*                   values of x and b are the same. */

/*     linpack. this version dated 08/14/78 . */
/*              correction made to shift 2/84. */
/*     g.w. stewart, university of maryland, argonne national lab. */

/*     dsvdc uses the following functions and subprograms. */

/*     external drot */
/*     blas daxpy,ddot,dscal,dswap,dnrm2,drotg */
/*     fortran dabs,dmax1,max0,min0,mod,dsqrt */

/*     internal variables */



/*     set the maximum number of iterations. */

    /* Parameter adjustments */
    x_dim1 = *ldx;
    x_offset = 1 + x_dim1 * 1;
    x -= x_offset;
    --s;
    --e;
    u_dim1 = *ldu;
    u_offset = 1 + u_dim1 * 1;
    u -= u_offset;
    v_dim1 = *ldv;
    v_offset = 1 + v_dim1 * 1;
    v -= v_offset;
    --work;

    /* Function Body */
    maxit = 30;

/*     determine what is to be computed. */

    wantu = FALSE_;
    wantv = FALSE_;
    jobu = *job % 100 / 10;
    ncu = *n;
    if (jobu > 1) {
	ncu = omin(*n,*p);
    }
    if (jobu != 0) {
	wantu = TRUE_;
    }
    if (*job % 10 != 0) {
	wantv = TRUE_;
    }

/*     reduce x to bidiagonal form, storing the diagonal elements */
/*     in s and the super-diagonal elements in e. */

    *info = 0;
/* Computing MIN */
    i__1 = *n - 1;
    nct = omin(i__1,*p);
/* Computing MAX */
/* Computing MIN */
    i__3 = *p - 2;
    i__1 = 0, i__2 = omin(i__3,*n);
    nrt = omax(i__1,i__2);
    lu = omax(nct,nrt);
    if (lu < 1) {
	goto L170;
    }
    i__1 = lu;
    for (l = 1; l <= i__1; ++l) {
	lp1 = l + 1;
	if (l > nct) {
	    goto L20;
	}

/*           compute the transformation for the l-th column and */
/*           place the l-th diagonal in s(l). */

	i__2 = *n - l + 1;
	s[l] = dnrm2_(&i__2, &x[l + l * x_dim1], &c__1d);
	if (s[l] == 0.) {
	    goto L10;
	}
	if (x[l + l * x_dim1] != 0.) {
	    s[l] = d_sign(&s[l], &x[l + l * x_dim1]);
	}
	i__2 = *n - l + 1;
	d__1 = 1. / s[l];
	dscal_(&i__2, &d__1, &x[l + l * x_dim1], &c__1d);
	x[l + l * x_dim1] += 1.;
L10:
	s[l] = -s[l];
L20:
	if (*p < lp1) {
	    goto L50;
	}
	i__2 = *p;
	for (j = lp1; j <= i__2; ++j) {
	    if (l > nct) {
		goto L30;
	    }
	    if (s[l] == 0.) {
		goto L30;
	    }

/*              apply the transformation. */

	    i__3 = *n - l + 1;
	    t = -ddot_(&i__3, &x[l + l * x_dim1], &c__1d, &x[l + j * x_dim1], &
		    c__1d) / x[l + l * x_dim1];
	    i__3 = *n - l + 1;
	    daxpy_(&i__3, &t, &x[l + l * x_dim1], &c__1d, &x[l + j * x_dim1], &
		    c__1d);
L30:

/*           place the l-th row of x into  e for the */
/*           subsequent calculation of the row transformation. */

	    e[j] = x[l + j * x_dim1];
/* L40: */
	}
L50:
	if (! wantu || l > nct) {
	    goto L70;
	}

/*           place the transformation in u for subsequent back */
/*           multiplication. */

	i__2 = *n;
	for (i__ = l; i__ <= i__2; ++i__) {
	    u[i__ + l * u_dim1] = x[i__ + l * x_dim1];
/* L60: */
	}
L70:
	if (l > nrt) {
	    goto L150;
	}

/*           compute the l-th row transformation and place the */
/*           l-th super-diagonal in e(l). */

	i__2 = *p - l;
	e[l] = dnrm2_(&i__2, &e[lp1], &c__1d);
	if (e[l] == 0.) {
	    goto L80;
	}
	if (e[lp1] != 0.) {
	    e[l] = d_sign(&e[l], &e[lp1]);
	}
	i__2 = *p - l;
	d__1 = 1. / e[l];
	dscal_(&i__2, &d__1, &e[lp1], &c__1d);
	e[lp1] += 1.;
L80:
	e[l] = -e[l];
	if (lp1 > *n || e[l] == 0.) {
	    goto L120;
	}

/*              apply the transformation. */

	i__2 = *n;
	for (i__ = lp1; i__ <= i__2; ++i__) {
	    work[i__] = 0.;
/* L90: */
	}
	i__2 = *p;
	for (j = lp1; j <= i__2; ++j) {
	    i__3 = *n - l;
	    daxpy_(&i__3, &e[j], &x[lp1 + j * x_dim1], &c__1d, &work[lp1], &
		    c__1d);
/* L100: */
	}
	i__2 = *p;
	for (j = lp1; j <= i__2; ++j) {
	    i__3 = *n - l;
	    d__1 = -e[j] / e[lp1];
	    daxpy_(&i__3, &d__1, &work[lp1], &c__1d, &x[lp1 + j * x_dim1], &
		    c__1d);
/* L110: */
	}
L120:
	if (! wantv) {
	    goto L140;
	}

/*              place the transformation in v for subsequent */
/*              back multiplication. */

	i__2 = *p;
	for (i__ = lp1; i__ <= i__2; ++i__) {
	    v[i__ + l * v_dim1] = e[i__];
/* L130: */
	}
L140:
L150:
/* L160: */
	;
    }
L170:

/*     set up the final bidiagonal matrix or order m. */

/* Computing MIN */
    i__1 = *p, i__2 = *n + 1;
    m = omin(i__1,i__2);
    nctp1 = nct + 1;
    nrtp1 = nrt + 1;
    if (nct < *p) {
	s[nctp1] = x[nctp1 + nctp1 * x_dim1];
    }
    if (*n < m) {
	s[m] = 0.;
    }
    if (nrtp1 < m) {
	e[nrtp1] = x[nrtp1 + m * x_dim1];
    }
    e[m] = 0.;

/*     if required, generate u. */

    if (! wantu) {
	goto L300;
    }
    if (ncu < nctp1) {
	goto L200;
    }
    i__1 = ncu;
    for (j = nctp1; j <= i__1; ++j) {
	i__2 = *n;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    u[i__ + j * u_dim1] = 0.;
/* L180: */
	}
	u[j + j * u_dim1] = 1.;
/* L190: */
    }
L200:
    if (nct < 1) {
	goto L290;
    }
    i__1 = nct;
    for (ll = 1; ll <= i__1; ++ll) {
	l = nct - ll + 1;
	if (s[l] == 0.) {
	    goto L250;
	}
	lp1 = l + 1;
	if (ncu < lp1) {
	    goto L220;
	}
	i__2 = ncu;
	for (j = lp1; j <= i__2; ++j) {
	    i__3 = *n - l + 1;
	    t = -ddot_(&i__3, &u[l + l * u_dim1], &c__1d, &u[l + j * u_dim1], &
		    c__1d) / u[l + l * u_dim1];
	    i__3 = *n - l + 1;
	    daxpy_(&i__3, &t, &u[l + l * u_dim1], &c__1d, &u[l + j * u_dim1], &
		    c__1d);
/* L210: */
	}
L220:
	i__2 = *n - l + 1;
	dscal_(&i__2, &c_b79, &u[l + l * u_dim1], &c__1d);
	u[l + l * u_dim1] += 1.;
	lm1 = l - 1;
	if (lm1 < 1) {
	    goto L240;
	}
	i__2 = lm1;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    u[i__ + l * u_dim1] = 0.;
/* L230: */
	}
L240:
	goto L270;
L250:
	i__2 = *n;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    u[i__ + l * u_dim1] = 0.;
/* L260: */
	}
	u[l + l * u_dim1] = 1.;
L270:
/* L280: */
	;
    }
L290:
L300:

/*     if it is required, generate v. */

    if (! wantv) {
	goto L350;
    }
    i__1 = *p;
    for (ll = 1; ll <= i__1; ++ll) {
	l = *p - ll + 1;
	lp1 = l + 1;
	if (l > nrt) {
	    goto L320;
	}
	if (e[l] == 0.) {
	    goto L320;
	}
	i__2 = *p;
	for (j = lp1; j <= i__2; ++j) {
	    i__3 = *p - l;
	    t = -ddot_(&i__3, &v[lp1 + l * v_dim1], &c__1d, &v[lp1 + j *
		    v_dim1], &c__1d) / v[lp1 + l * v_dim1];
	    i__3 = *p - l;
	    daxpy_(&i__3, &t, &v[lp1 + l * v_dim1], &c__1d, &v[lp1 + j *
		    v_dim1], &c__1d);
/* L310: */
	}
L320:
	i__2 = *p;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    v[i__ + l * v_dim1] = 0.;
/* L330: */
	}
	v[l + l * v_dim1] = 1.;
/* L340: */
    }
L350:

/*     main iteration loop for the singular values. */

    mm = m;
    iter = 0;
L360:

/*        quit if all the singular values have been found. */

/*     ...exit */
    if (m == 0) {
	goto L620;
    }

/*        if too many iterations have been performed, set */
/*        flag and return. */

    if (iter < maxit) {
	goto L370;
    }
    *info = m;
/*     ......exit */
    goto L620;
L370:

/*        this section of the program inspects for */
/*        negligible elements in the s and e arrays.  on */
/*        completion the variables kase and l are set as follows. */

/*           kase = 1     if s(m) and e(l-1) are negligible and l.lt.m */
/*           kase = 2     if s(l) is negligible and l.lt.m */
/*           kase = 3     if e(l-1) is negligible, l.lt.m, and */
/*                        s(l), ..., s(m) are not negligible (qr step). */
/*           kase = 4     if e(m-1) is negligible (convergence). */

    i__1 = m;
    for (ll = 1; ll <= i__1; ++ll) {
	l = m - ll;
/*        ...exit */
	if (l == 0) {
	    goto L400;
	}
	test = (d__1 = s[l], abs(d__1)) + (d__2 = s[l + 1], abs(d__2));
	ztest = test + (d__1 = e[l], abs(d__1));
	if (ztest != test) {
	    goto L380;
	}
	e[l] = 0.;
/*        ......exit */
	goto L400;
L380:
/* L390: */
	;
    }
L400:
    if (l != m - 1) {
	goto L410;
    }
    kase = 4;
    goto L480;
L410:
    lp1 = l + 1;
    mp1 = m + 1;
    i__1 = mp1;
    for (lls = lp1; lls <= i__1; ++lls) {
	ls = m - lls + lp1;
/*           ...exit */
	if (ls == l) {
	    goto L440;
	}
	test = 0.;
	if (ls != m) {
	    test += (d__1 = e[ls], abs(d__1));
	}
	if (ls != l + 1) {
	    test += (d__1 = e[ls - 1], abs(d__1));
	}
	ztest = test + (d__1 = s[ls], abs(d__1));
	if (ztest != test) {
	    goto L420;
	}
	s[ls] = 0.;
/*           ......exit */
	goto L440;
L420:
/* L430: */
	;
    }
L440:
    if (ls != l) {
	goto L450;
    }
    kase = 3;
    goto L470;
L450:
    if (ls != m) {
	goto L460;
    }
    kase = 1;
    goto L470;
L460:
    kase = 2;
    l = ls;
L470:
L480:
    ++l;

/*        perform the task indicated by kase. */

    switch (kase) {
	case 1:  goto L490;
	case 2:  goto L520;
	case 3:  goto L540;
	case 4:  goto L570;
    }

/*        deflate negligible s(m). */

L490:
    mm1 = m - 1;
    f = e[m - 1];
    e[m - 1] = 0.;
    i__1 = mm1;
    for (kk = l; kk <= i__1; ++kk) {
	k = mm1 - kk + l;
	t1 = s[k];
	drotg_(&t1, &f, &cs, &sn);
	s[k] = t1;
	if (k == l) {
	    goto L500;
	}
	f = -sn * e[k - 1];
	e[k - 1] = cs * e[k - 1];
L500:
	if (wantv) {
	    drot_(p, &v[k * v_dim1 + 1], &c__1d, &v[m * v_dim1 + 1], &c__1d, &
		    cs, &sn);
	}
/* L510: */
    }
    goto L610;

/*        split at negligible s(l). */

L520:
    f = e[l - 1];
    e[l - 1] = 0.;
    i__1 = m;
    for (k = l; k <= i__1; ++k) {
	t1 = s[k];
	drotg_(&t1, &f, &cs, &sn);
	s[k] = t1;
	f = -sn * e[k];
	e[k] = cs * e[k];
	if (wantu) {
	    drot_(n, &u[k * u_dim1 + 1], &c__1d, &u[(l - 1) * u_dim1 + 1], &
		    c__1d, &cs, &sn);
	}
/* L530: */
    }
    goto L610;

/*        perform one qr step. */

L540:

/*           calculate the shift. */

/* Computing MAX */
    d__6 = (d__1 = s[m], abs(d__1)), d__7 = (d__2 = s[m - 1], abs(d__2)),
	    d__6 = omax(d__6,d__7), d__7 = (d__3 = e[m - 1], abs(d__3)), d__6 =
	    omax(d__6,d__7), d__7 = (d__4 = s[l], abs(d__4)), d__6 = omax(d__6,
	    d__7), d__7 = (d__5 = e[l], abs(d__5));
    scale = omax(d__6,d__7);
    sm = s[m] / scale;
    smm1 = s[m - 1] / scale;
    emm1 = e[m - 1] / scale;
    sl = s[l] / scale;
    el = e[l] / scale;
/* Computing 2nd power */
    d__1 = emm1;
    b = ((smm1 + sm) * (smm1 - sm) + d__1 * d__1) / 2.;
/* Computing 2nd power */
    d__1 = sm * emm1;
    c__ = d__1 * d__1;
    shift = 0.;
    if (b == 0. && c__ == 0.) {
	goto L550;
    }
/* Computing 2nd power */
    d__1 = b;
    shift = sqrt(d__1 * d__1 + c__);
    if (b < 0.) {
	shift = -shift;
    }
    shift = c__ / (b + shift);
L550:
    f = (sl + sm) * (sl - sm) + shift;
    g = sl * el;

/*           chase zeros. */

    mm1 = m - 1;
    i__1 = mm1;
    for (k = l; k <= i__1; ++k) {
	drotg_(&f, &g, &cs, &sn);
	if (k != l) {
	    e[k - 1] = f;
	}
	f = cs * s[k] + sn * e[k];
	e[k] = cs * e[k] - sn * s[k];
	g = sn * s[k + 1];
	s[k + 1] = cs * s[k + 1];
	if (wantv) {
	    drot_(p, &v[k * v_dim1 + 1], &c__1d, &v[(k + 1) * v_dim1 + 1], &
		    c__1d, &cs, &sn);
	}
	drotg_(&f, &g, &cs, &sn);
	s[k] = f;
	f = cs * e[k] + sn * s[k + 1];
	s[k + 1] = -sn * e[k] + cs * s[k + 1];
	g = sn * e[k + 1];
	e[k + 1] = cs * e[k + 1];
	if (wantu && k < *n) {
	    drot_(n, &u[k * u_dim1 + 1], &c__1d, &u[(k + 1) * u_dim1 + 1], &
		    c__1d, &cs, &sn);
	}
/* L560: */
    }
    e[m - 1] = f;
    ++iter;
    goto L610;

/*        convergence. */

L570:

/*           make the singular value  positive. */

    if (s[l] >= 0.) {
	goto L580;
    }
    s[l] = -s[l];
    if (wantv) {
	dscal_(p, &c_b79, &v[l * v_dim1 + 1], &c__1d);
    }
L580:

/*           order the singular value. */

L590:
    if (l == mm) {
	goto L600;
    }
/*           ...exit */
    if (s[l] >= s[l + 1]) {
	goto L600;
    }
    t = s[l];
    s[l] = s[l + 1];
    s[l + 1] = t;
    if (wantv && l < *p) {
       dswap_(p, &v[l * v_dim1 + 1], &c__1d, &v[(l + 1) * v_dim1 + 1], &c__1d);
    }
    if (wantu && l < *n) {
       dswap_(n, &u[l * u_dim1 + 1], &c__1d, &u[(l + 1) * u_dim1 + 1], &c__1d);
    }
    ++l;
    goto L590;
L600:
    iter = 0;
    --m;
L610:
    goto L360;
L620:
    return 0;
} /* dsvdc_ */


#ifdef KR_headers
double d_sign(a,b) doublereal *a, *b;
#else
double d_sign(doublereal *a, doublereal *b)
#endif
{
  double x;
  x = (*a >= 0 ? *a : - *a);
  return( *b >= 0 ? x : -x);
}

#ifdef __cplusplus
	}
#endif
