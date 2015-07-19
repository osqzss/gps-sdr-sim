#define _CRT_SECURE_NO_DEPRECATE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <omp.h>
#ifdef _WIN32
#include "getopt.h"
#else
#include <unistd.h>
#endif

#ifndef bool
typedef int bool;
#define true 1
#define false 0
#endif

#define MAX_CHAR (100)

#define MAX_SAT (32)
#define MAX_CHAN (16)

#define USER_MOTION_SIZE (3000) // max 300 sec at 10Hz

#define N_SBF (51) // 6 seconds per subframe, 6 sec * 51 = 306 sec (max)
#define N_DWRD (N_SBF*10) // 10 word per subframe

#define SECONDS_IN_WEEK 604800.0
#define SECONDS_IN_HALF_WEEK 302400.0
#define SECONDS_IN_DAY 86400.0
#define SECONDS_IN_HOUR 3600.0
#define SECONDS_IN_MINUTE 60.0

#define POW2_M5  0.03125
#define POW2_M19 1.907348632812500e-6
#define POW2_M29 1.862645149230957e-9
#define POW2_M31 4.656612873077393e-10
#define POW2_M33 1.164153218269348e-10
#define POW2_M43 1.136868377216160e-13
#define POW2_M55 2.775557561562891e-17

// Conventional values employed in GPS ephemeris model (ICD-GPS-200)
#define GM_EARTH 3.986005e14
#define OMEGA_EARTH 7.2921151467e-5
#define PI 3.1415926535898

#define R2D 57.2957795131

#define SPEED_OF_LIGHT 2.99792458e8
#define LAMBDA_L1 0.190293672798365

#define CARR_FREQ (1575.42e6)
#define CODE_FREQ (1.023e6)
#define CARR_TO_CODE (1.0/1540.0)

// Sampling data format
#define SC08 (8)
#define SC16 (16)

#define ADC_GAIN (250) // for bladeRF txvga1 = -25dB with 50dB external attenuation

#define _SINE_LUT
#ifdef _SINE_LUT
int sinTable512[] = {
	   2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
	  50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
	  97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
	 140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
	 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
	 209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
	 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
	 245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250,
	 250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
	 245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
	 230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
	 207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
	 176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
	 138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
	  94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
	  47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
	  -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
	 -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
	 -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
	-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
	-178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
	-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
	-232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
	-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
	-250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
	-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
	-230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
	-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
	-176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
	-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
	 -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
	 -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2
};

int cosTable512[] = {
	 250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
	 245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
	 230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
	 207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
	 176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
	 138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
	  94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
	  47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
	  -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
	 -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
	 -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
	-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
	-178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
	-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
	-232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
	-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
	-250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
	-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
	-230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
	-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
	-176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
	-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
	 -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
	 -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2,
	   2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
	  50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
	  97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
	 140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
	 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
	 209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
	 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
	 245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250
};
#endif

typedef struct
{
	int week;
	double sec;
} gpstime_t;

typedef struct
{
	int y;
	int m;
	int d;
	int hh;
	int mm;
	double sec;
} datetime_t;

typedef struct
{
	int vflg;
	gpstime_t toc;
	gpstime_t toe;
	int iodc;
	int iode;
	double deltan;
	double cuc;
	double cus;
	double cic;
	double cis;
	double crc;
	double crs;
	double ecc;
	double sqrta;
	double m0;
	double omg0;
	double inc0;
	double aop;
	double omgdot;
	double idot;
	double af0;
	double af1;
	double af2;
	double tgd;
} ephem_t;

typedef struct
{
	gpstime_t g;
	double range;
	double rate;
} range_t;

typedef struct
{
	int prn;
	int ca[1023];
	double f_carr,f_code;
	double carr_phase,code_phase;
	gpstime_t g0;
	unsigned long dwrd[N_DWRD];
	int iword;
	int ibit;
	int icode;
	int dataBit;
	int codeCA;
	short *iq_buff;
} channel_t;

void codegen(int *ca, int prn)
{
	int delay[] = {
		  5,   6,   7,   8,  17,  18, 139, 140, 141, 251,
		252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
		473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
		861, 862};
	
	int g1[1023],g2[1023],r1[10],r2[10],c1,c2;
	int i,j;

	if (prn<1 || prn>32)
		return;

	for (i=0;i<10;i++)
		r1[i] = r2[i] = -1;

	for (i=0; i<1023; i++) 
	{
		g1[i] = r1[9];
		g2[i] = r2[9];
		c1 = r1[2]*r1[9];
		c2 = r2[1]*r2[2]*r2[5]*r2[7]*r2[8]*r2[9];

		for (j=9; j>0; j--) 
		{
			r1[j] = r1[j-1];
			r2[j] = r2[j-1];
		}
		r1[0] = c1;
		r2[0] = c2;
	}

	for (i=0,j=1023-delay[prn-1]; i<1023; i++,j++)
		ca[i] = (1-g1[i]*g2[j%1023])/2;
	
	return;
}

void date2gps(datetime_t *t, gpstime_t *g)
{
	int doy[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
	int ye;
	int de;
	int lpdays;

	ye = t->y - 1980;

	// Compute the number of leap days since Jan 5/Jan 6, 1980.
	lpdays = ye/4 + 1;
	if ((ye%4)==0 && t->m<=2)
		lpdays--;

	// Compute the number of days elapsed since Jan 5/Jan 6, 1980.
	de = ye*365 + doy[t->m-1] + t->d + lpdays - 6;

	// Convert time to GPS weeks and seconds.
	g->week = de / 7;
	g->sec = (double)(de%7)*SECONDS_IN_DAY + t->hh*SECONDS_IN_HOUR 
		+ t->mm*SECONDS_IN_MINUTE + t->sec;

	return;
}

void xyz2llh(double *xyz, double *llh)
{
	double a,eps,e,e2;
	double x,y,z;
	double rho2,dz,zdz,nh,slat,n,dz_new;

	a = 6378137.0;
	e = 0.0818191908426;

	eps = 1.0e-3;
	e2 = e*e;

	x = xyz[0];
	y = xyz[1];
	z = xyz[2];

	rho2 = x*x + y*y;
	dz = e2*z;

	while (1)
	{
		zdz = z + dz;
		nh = sqrt(rho2 + zdz*zdz);
		slat = zdz / nh;
		n = a / sqrt(1.0-e2*slat*slat);
		dz_new = n*e2*slat;

		if (fabs(dz-dz_new) < eps)
			break;

		dz = dz_new;
	}

	llh[0] = atan2(zdz, sqrt(rho2));
	llh[1] = atan2(y, x);
	llh[2] = nh - n;

	return;
}

void llh2xyz(double *llh, double *xyz)
{
	double n;
	double a;
	double e;
	double e2;
	double clat;
	double slat;
	double clon;
	double slon;
	double d,nph;
	double tmp;

	a = 6378137.0;
	e = 0.0818191908426;
	e2 = e*e;

	clat = cos(llh[0]);
	slat = sin(llh[0]);
	clon = cos(llh[1]);
	slon = sin(llh[1]);
	d = e*slat;

	n = a/sqrt(1.0-d*d);
	nph = n + llh[2];

	tmp = nph*clat;
	xyz[0] = tmp*clon;
	xyz[1] = tmp*slon;
	xyz[2] = ((1.0-e2)*n + llh[2])*slat;

	return;
}

void ltcmat(double *llh, double t[3][3])
{
	double slat, clat;
	double slon, clon;

	slat = sin(llh[0]);
	clat = cos(llh[0]);
	slon = sin(llh[1]);
	clon = cos(llh[1]);

	t[0][0] = -slat*clon;
	t[0][1] = -slat*slon;
	t[0][2] = clat;
	t[1][0] = -slon;
	t[1][1] = clon;
	t[1][2] = 0.0;
	t[2][0] = clat*clon;
	t[2][1] = clat*slon;
	t[2][2] = slat;

	return;
}

void ecef2neu(double *xyz, double t[3][3], double *neu)
{
	neu[0] = t[0][0]*xyz[0] + t[0][1]*xyz[1] + t[0][2]*xyz[2];
	neu[1] = t[1][0]*xyz[0] + t[1][1]*xyz[1] + t[1][2]*xyz[2];
	neu[2] = t[2][0]*xyz[0] + t[2][1]*xyz[1] + t[2][2]*xyz[2];

	return;
}

void neu2azel(double *azel, double *neu)
{
	double ne;

	azel[0] = atan2(neu[1],neu[0]);
	if (azel[0]<0.0)
		azel[0] += (2.0*PI);

	ne = sqrt(neu[0]*neu[0] + neu[1]*neu[1]);
	azel[1] = atan2(neu[2], ne);

	return;
}

void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk)
{
	// Computing Satellite Velocity using the Broadcast Ephemeris
	// http://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm

	double tk;
	double a;
	double mk;
	double mkdot;
	double ek;
	double ekold;
	double ekdot;
	double cek,sek;
	double pk;
	double pkdot;
	double c2pk,s2pk;
	double uk;
	double ukdot;
	double cuk,suk;
	double ok;
	double okdot;
	double sok,cok;
	double ik;
	double ikdot;
	double sik,cik;
	double rk;
	double rkdot;
	double xpk,ypk;
	double xpkdot,ypkdot;

	double relativistic, OneMinusecosE, sqrtOneMinuse2, tmp;

	tk = g.sec - eph.toe.sec;

	if(tk>SECONDS_IN_HALF_WEEK)
		tk -= SECONDS_IN_WEEK;
	else if(tk<-SECONDS_IN_HALF_WEEK)
		tk += SECONDS_IN_WEEK;

	a = eph.sqrta*eph.sqrta;

	mkdot = sqrt(GM_EARTH/(a*a*a)) + eph.deltan;
	mk = eph.m0 + mkdot*tk;

	ek = mk;
	ekold = ek + 1.0;
  
	while(fabs(ek-ekold)>1.0E-14)
	{
		ekold = ek;
		OneMinusecosE = 1.0-eph.ecc*cos(ekold);
		ek = ek + (mk-ekold+eph.ecc*sin(ekold))/OneMinusecosE;
	}

	sek = sin(ek);
	cek = cos(ek);

	ekdot = mkdot/OneMinusecosE;

	relativistic = -4.442807633E-10*eph.ecc*eph.sqrta*sek;

	sqrtOneMinuse2 = sqrt(1.0 - eph.ecc*eph.ecc);

	pk = atan2(sqrtOneMinuse2*sek,cek-eph.ecc) + eph.aop;
	pkdot = sqrtOneMinuse2*ekdot/OneMinusecosE;

	s2pk = sin(2.0*pk);
	c2pk = cos(2.0*pk);

	uk = pk + eph.cus*s2pk + eph.cuc*c2pk;
	suk = sin(uk);
	cuk = cos(uk);
	ukdot = pkdot*(1.0 + 2.0*(eph.cus*c2pk - eph.cuc*s2pk));

	rk = a*OneMinusecosE + eph.crc*c2pk + eph.crs*s2pk;
	rkdot = a*eph.ecc*sek*ekdot + 2.0*pkdot*(eph.crs*c2pk - eph.crc*s2pk);

	ik = eph.inc0 + eph.idot*tk + eph.cic*c2pk + eph.cis*s2pk;
	sik = sin(ik);
	cik = cos(ik);
	ikdot = eph.idot + 2.0*pkdot*(eph.cis*c2pk - eph.cic*s2pk);

	xpk = rk*cuk;
	ypk = rk*suk;
	xpkdot = rkdot*cuk - ypk*ukdot;
	ypkdot = rkdot*suk + xpk*ukdot;

	okdot = eph.omgdot - OMEGA_EARTH;
	ok = eph.omg0 + tk*okdot - OMEGA_EARTH*eph.toe.sec;
	sok = sin(ok);
	cok = cos(ok);

	pos[0] = xpk*cok - ypk*cik*sok;
	pos[1] = xpk*sok + ypk*cik*cok;
	pos[2] = ypk*sik;

	tmp = ypkdot*cik - ypk*sik*ikdot;

	vel[0] = -okdot*pos[1] + xpkdot*cok - tmp*sok;
	vel[1] = okdot*pos[0] + xpkdot*sok + tmp*cok;
	vel[2] = ypk*cik*ikdot + ypkdot*sik;

	clk[0] = eph.af0 + tk*(eph.af1 + tk*eph.af2) + relativistic - eph.tgd;  
	clk[1] = eph.af1 + 2.0*tk*eph.af2; 

	return;
}

void eph2sbf(ephem_t eph, unsigned long sbf[5][10])
{
	unsigned long wn;
	unsigned long toe;
	unsigned long toc;
	unsigned long iode;
	unsigned long iodc;
	long deltan;
	long cuc;
	long cus;
	long cic;
	long cis;
	long crc;
	long crs;
	unsigned long ecc;
	unsigned long sqrta;
	long m0;
	long omg0;
	long inc0;
	long aop;
	long omgdot;
	long idot;
	long af0;
	long af1;
	long af2;
	long tgd;

	unsigned long ura = 2UL;
	unsigned long dataId = 1UL;
	unsigned long sbf4_page25_svId = 63UL;
	unsigned long sbf5_page25_svId = 51UL;

	unsigned long wna;
	unsigned long toa;

	wn = (unsigned long)(eph.toe.week%1024);
	toe = (unsigned long)(eph.toe.sec/16.0);
	toc = (unsigned long)(eph.toc.sec/16.0);
	iode = (unsigned long)(eph.iode);
	iodc = (unsigned long)(eph.iodc);
	deltan = (long)(eph.deltan/POW2_M43/PI);
	cuc = (long)(eph.cuc/POW2_M29);
	cus = (long)(eph.cus/POW2_M29);
	cic = (long)(eph.cic/POW2_M29);
	cis = (long)(eph.cis/POW2_M29);
	crc = (long)(eph.crc/POW2_M5);
	crs = (long)(eph.crs/POW2_M5);
	ecc = (unsigned long)(eph.ecc/POW2_M33);
	sqrta = (unsigned long)(eph.sqrta/POW2_M19);
	m0 = (long)(eph.m0/POW2_M31/PI);
	omg0 = (long)(eph.omg0/POW2_M31/PI);
	inc0 = (long)(eph.inc0/POW2_M31/PI);
	aop = (long)(eph.aop/POW2_M31/PI);
	omgdot = (long)(eph.omgdot/POW2_M43/PI);
	idot = (long)(eph.idot/POW2_M43/PI);
	af0 = (long)(eph.af0/POW2_M31);
	af1 = (long)(eph.af1/POW2_M43);
	af2 = (long)(eph.af2/POW2_M55);
	tgd = (long)(eph.tgd/POW2_M31);

	wna = (unsigned long)(eph.toe.week%256);
	toa = (unsigned long)(eph.toe.sec/4096.0);

	// Subframe 1
	sbf[0][0] = 0x8B0000UL<<6;
	sbf[0][1] = 0x1UL<<8;
	sbf[0][2] = ((wn&0x3FFUL)<<20) | (ura<<14) | (((iodc>>8)&0x3UL)<<6);
	sbf[0][3] = 0UL;
	sbf[0][4] = 0UL;
	sbf[0][5] = 0UL;
	sbf[0][6] = (tgd&0xFFUL)<<6;
	sbf[0][7] = ((iodc&0xFFUL)<<22) | ((toc&0xFFFFUL)<<6);
	sbf[0][8] = ((af2&0xFFUL)<<22) | ((af1&0xFFFFUL)<<6);
	sbf[0][9] = (af0&0x3FFFFFUL)<<8;

	// Subframe 2
	sbf[1][0] = 0x8B0000UL<<6;
	sbf[1][1] = 0x2UL<<8;
	sbf[1][2] = ((iode&0xFFUL)<<22) | ((crs&0xFFFFUL)<<6);
	sbf[1][3] = ((deltan&0xFFFFUL)<<14) | (((m0>>24)&0xFFUL)<<6);
	sbf[1][4] = (m0&0xFFFFFFUL)<<6;
	sbf[1][5] = ((cuc&0xFFFFUL)<<14) | (((ecc>>24)&0xFFUL)<<6);
	sbf[1][6] = (ecc&0xFFFFFFUL)<<6;
	sbf[1][7] = ((cus&0xFFFFUL)<<14) | (((sqrta>>24)&0xFFUL)<<6);
	sbf[1][8] = (sqrta&0xFFFFFFUL)<<6;
	sbf[1][9] = (toe&0xFFFFUL)<<14;

	// Subframe 3
	sbf[2][0] = 0x8B0000UL<<6;
	sbf[2][1] = 0x3UL<<8;
	sbf[2][2] = ((cic&0xFFFFUL)<<14) | (((omg0>>24)&0xFFUL)<<6);
	sbf[2][3] = (omg0&0xFFFFFFUL)<<6;
	sbf[2][4] = ((cis&0xFFFFUL)<<14) | (((inc0>>24)&0xFFUL)<<6);
	sbf[2][5] = (inc0&0xFFFFFFUL)<<6;
	sbf[2][6] = ((crc&0xFFFFUL)<<14) | (((aop>>24)&0xFFUL)<<6);
	sbf[2][7] = (aop&0xFFFFFFUL)<<6;
	sbf[2][8] = (omgdot&0xFFFFFFUL)<<6;
	sbf[2][9] = ((iode&0xFFUL)<<22) | ((idot&0x3FFFUL)<<8);

	// Subframe 4, page 25
	sbf[3][0] = 0x8B0000UL<<6;
	sbf[3][1] = 0x4UL<<8;
	sbf[3][2] = (dataId<<28) | (sbf4_page25_svId<<22);
	sbf[3][3] = 0UL;
	sbf[3][4] = 0UL;
	sbf[3][5] = 0UL;
	sbf[3][6] = 0UL;
	sbf[3][7] = 0UL;
	sbf[3][8] = 0UL;
	sbf[3][9] = 0UL;

	// Subframe 5, page 25
	sbf[4][0] = 0x8B0000UL<<6;
	sbf[4][1] = 0x5UL<<8;
	sbf[4][2] = (dataId<<28) | (sbf5_page25_svId<<22) | ((toa&0xFFUL)<<14) | ((wna&0xFFUL)<<6);
	sbf[4][3] = 0UL;
	sbf[4][4] = 0UL;
	sbf[4][5] = 0UL;
	sbf[4][6] = 0UL;
	sbf[4][7] = 0UL;
	sbf[4][8] = 0UL;
	sbf[4][9] = 0UL;

	return;
}

unsigned long countBits(unsigned long v)
{
	unsigned long c;
	const int S[] = {1, 2, 4, 8, 16};
	const unsigned long B[] = {
		0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF};

	c = v;
	c = ((c >> S[0]) & B[0]) + (c & B[0]);
	c = ((c >> S[1]) & B[1]) + (c & B[1]);
	c = ((c >> S[2]) & B[2]) + (c & B[2]);
	c = ((c >> S[3]) & B[3]) + (c & B[3]);
	c = ((c >> S[4]) & B[4]) + (c & B[4]);

	return(c);
}

unsigned long computeChecksum(unsigned long source, int nib)
{
	/*
	Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
	Bits 29 to  6 = Source data bits, d1, d2, ..., d24
	Bits  5 to  0 = Empty parity bits
	*/ 

	/*
	Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
	Bits 29 to  6 = Data bits transmitted by the SV, D1, D2, ..., D24
	Bits  5 to  0 = Computed parity bits, D25, D26, ..., D30
	*/ 

	/*
	                  1            2           3
	bit    12 3456 7890 1234 5678 9012 3456 7890
	---    -------------------------------------
	D25    11 1011 0001 1111 0011 0100 1000 0000
	D26    01 1101 1000 1111 1001 1010 0100 0000
	D27    10 1110 1100 0111 1100 1101 0000 0000
	D28    01 0111 0110 0011 1110 0110 1000 0000
	D29    10 1011 1011 0001 1111 0011 0100 0000
	D30    00 1011 0111 1010 1000 1001 1100 0000
	*/

	unsigned long bmask[6] = { 
		0x3B1F3480UL, 0x1D8F9A40UL, 0x2EC7CD00UL,
		0x1763E680UL, 0x2BB1F340UL, 0x0B7A89C0UL };

	unsigned long D;
	unsigned long d = source & 0x3FFFFFC0UL;
	unsigned long D29 = (source>>31)&0x1UL;
	unsigned long D30 = (source>>30)&0x1UL;

	if (nib) // Non-information bearing bits for word 2 and 10
	{
		/*
		Solve bits 23 and 24 to presearve parity check
		with zeros in bits 29 and 30.
		*/

		if ((D30 + countBits(bmask[4] & d)) % 2)
			d ^= (0x1UL<<6);
		if ((D29 + countBits(bmask[5] & d)) % 2)
			d ^= (0x1UL<<7);
	}

	D = d;
	if (D30)
		D ^= 0x3FFFFFC0UL;

	D |= ((D29 + countBits(bmask[0] & d)) % 2) << 5;
	D |= ((D30 + countBits(bmask[1] & d)) % 2) << 4;
	D |= ((D29 + countBits(bmask[2] & d)) % 2) << 3;
	D |= ((D30 + countBits(bmask[3] & d)) % 2) << 2;
	D |= ((D30 + countBits(bmask[4] & d)) % 2) << 1;
	D |= ((D29 + countBits(bmask[5] & d)) % 2);
	
	D &= 0x3FFFFFFFUL;
	//D |= (source & 0xC0000000UL); // Add D29* and D30* from source data bits

	return(D);
}

int checkExpDesignator(char *str, int len)
{
	int i,n=0;

	for (i=0; i<len; i++)
	{
		if (str[i]=='D')
		{
			n++;
			str[i] = 'E';
		}
	}
	
	return(n);
}

int readRinexNav(ephem_t eph[], char *fname)
{
	FILE *fp;
	int nsat;
	int sv;
	char str[MAX_CHAR];
	char tmp[20];

	datetime_t t;
	gpstime_t g;
	gpstime_t g0;
	double dt;

	if (NULL==(fp=fopen(fname, "rt")))
		return(-1);

	for (sv=0; sv<MAX_SAT; sv++)
		eph[sv].vflg = 0; // Clear valid flag

	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		if (0==strncmp(str+60, "END OF HEADER", 13))
			break;
	}

	nsat = 0;
	g0.week = -1;

	while (1)
	{
		// PRN / EPOCH / SV CLK
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 2);
		tmp[2] = 0;
		t.y = atoi(tmp) + 2000;

		strncpy(tmp, str+6, 2);
		tmp[2] = 0;
		t.m = atoi(tmp);

		strncpy(tmp, str+9, 2);
		tmp[2] = 0;
		t.d = atoi(tmp);

		strncpy(tmp, str+12, 2);
		tmp[2] = 0;
		t.hh = atoi(tmp);

		strncpy(tmp, str+15, 2);
		tmp[2] = 0;
		t.mm = atoi(tmp);

		strncpy(tmp, str+18, 4);
		tmp[2] = 0;
		t.sec = atof(tmp);

		date2gps(&t, &g);
		
		if (g0.week==-1)
			g0 = g;

		dt = g.sec - g0.sec;

		if ((g.week==g0.week) && (dt>-SECONDS_IN_HOUR) && (dt<=SECONDS_IN_HOUR))
		{
			strncpy(tmp, str, 2);
			tmp[2] = 0;
			sv = atoi(tmp)-1;

			if (eph[sv].vflg==0)
			{
				eph[sv].toc = g;

				strncpy(tmp, str+22, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19); // tmp[15]='E';
				eph[sv].af0 = atof(tmp);

				strncpy(tmp, str+41, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].af1 = atof(tmp);

				strncpy(tmp, str+60, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].af2 = atof(tmp);

				// BROADCAST ORBIT - 1
				if (NULL==fgets(str, MAX_CHAR, fp))
					break;

				strncpy(tmp, str+3, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].iode = (int)atof(tmp);

				strncpy(tmp, str+22, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].crs = atof(tmp);

				strncpy(tmp, str+41, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].deltan = atof(tmp);

				strncpy(tmp, str+60, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].m0 = atof(tmp);

				// BROADCAST ORBIT - 2
				if (NULL==fgets(str, MAX_CHAR, fp))
					break;

				strncpy(tmp, str+3, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].cuc = atof(tmp);

				strncpy(tmp, str+22, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].ecc = atof(tmp);

				strncpy(tmp, str+41, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].cus = atof(tmp);

				strncpy(tmp, str+60, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].sqrta = atof(tmp);

				// BROADCAST ORBIT - 3
				if (NULL==fgets(str, MAX_CHAR, fp))
					break;

				strncpy(tmp, str+3, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].toe.sec = atof(tmp);

				strncpy(tmp, str+22, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].cic = atof(tmp);

				strncpy(tmp, str+41, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].omg0 = atof(tmp);

				strncpy(tmp, str+60, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].cis = atof(tmp);

				// BROADCAST ORBIT - 4
				if (NULL==fgets(str, MAX_CHAR, fp))
					break;

				strncpy(tmp, str+3, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].inc0 = atof(tmp);

				strncpy(tmp, str+22, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].crc = atof(tmp);

				strncpy(tmp, str+41, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].aop = atof(tmp);

				strncpy(tmp, str+60, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].omgdot = atof(tmp);

				// BROADCAST ORBIT - 5
				if (NULL==fgets(str, MAX_CHAR, fp))
					break;

				strncpy(tmp, str+3, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].idot = atof(tmp);

				strncpy(tmp, str+41, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].toe.week = (int)atof(tmp);

				// BROADCAST ORBIT - 6
				if (NULL==fgets(str, MAX_CHAR, fp))
					break;

				strncpy(tmp, str+41, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].tgd = atof(tmp);

				strncpy(tmp, str+60, 19);
				tmp[19] = 0;
				checkExpDesignator(tmp, 19);
				eph[sv].iodc = (int)atof(tmp);

				// BROADCAST ORBIT - 7
				if (NULL==fgets(str, MAX_CHAR, fp))
					break;

				eph[sv].vflg = 1;
				
				nsat++;
			}
		}
		else
			break;
	}

	fclose(fp);

	return(nsat);
}

void subVect(double *y, double *x1, double *x2)
{
	y[0] = x1[0]-x2[0];
	y[1] = x1[1]-x2[1];
	y[2] = x1[2]-x2[2];

	return;
}

double normVect(double *x)
{
	return(sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]));
}

double dotProd(double *x1, double *x2)
{
	return(x1[0]*x2[0]+x1[1]*x2[1]+x1[2]*x2[2]);
}

void computeRange(range_t *rho, ephem_t eph, gpstime_t g, double xyz[])
{
	double pos[3],vel[3],clk[2];
	double los[3];
	double tau;
	double range,rate;
	double tmp;
	
	// SV position at time of the pseudorange observation.
	satpos(eph, g, pos, vel, clk);

	// Receiver to satellite vector and light-time.
	subVect(los, pos, xyz);
	tau = normVect(los)/SPEED_OF_LIGHT;

	// Extrapolate the satellite position backwards to the transmission time.
	pos[0] -= vel[0]*tau;
	pos[1] -= vel[1]*tau;
	pos[2] -= vel[2]*tau;

	// Earth rotation correction. The change in velocity can be neglected.
	tmp = pos[0]*OMEGA_EARTH*tau;
	pos[0] += pos[1]*OMEGA_EARTH*tau;
	pos[1] -= tmp;

	// New observer to satellite vector and satellite range.
	subVect(los, pos, xyz);
	range = normVect(los);

	// Pseudorange.
	rho->range = range - SPEED_OF_LIGHT*clk[0];

	// Relative velocity of SV and receiver.
	rate = dotProd(vel, los)/range;

	// Pseudorange rate.
	rho->rate = rate; // - SPEED_OF_LIGHT*clk[1];

	// Time of application
	rho->g = g;

	return;
}

void computeCodePhase(channel_t *chan, range_t rho0, range_t rho1, double dt)
{
	double ms;
	int ims;
	double rhorate;
	
	// Pseudorange rate.
	rhorate = (rho1.range - rho0.range)/dt;

	// Carrier and code frequency.
	chan->f_carr = -rhorate/LAMBDA_L1;
	chan->f_code = CODE_FREQ + chan->f_carr*CARR_TO_CODE;

	// Initial code phase and data bit counters.
	ms = (((rho0.g.sec-chan->g0.sec)+6.0) - rho0.range/SPEED_OF_LIGHT)*1000.0;

	ims = (int)ms;
	chan->code_phase = (ms-(double)ims)*1023.0; // in chip

	chan->iword = ims/600; // 1 word = 30 bits = 600 ms
	ims -= chan->iword*600;
			
	chan->ibit = ims/20; // 1 bit = 20 code = 20 ms
	ims -= chan->ibit*20;

	chan->icode = ims; // 1 code = 1 ms

	chan->codeCA = chan->ca[(int)chan->code_phase]*2-1;
	chan->dataBit = (int)((chan->dwrd[chan->iword]>>(29-chan->ibit)) & 0x1UL)*2-1;

	return;
}

int readUserMotion(double xyz[USER_MOTION_SIZE][3], char *filename)
{
	FILE *fp;
	int numd;
	double t,x,y,z;

	if (NULL==(fp=fopen(filename,"rt")))
		return(-1);

	for (numd=0; numd<USER_MOTION_SIZE; numd++)
	{
		if (EOF==fscanf(fp, "%lf,%lf,%lf,%lf", &t, &x, &y, &z)) // Read CSV file
			break;

		xyz[numd][0] = x;
		xyz[numd][1] = y;
		xyz[numd][2] = z;
	}

	fclose(fp);

	return (numd);
}

void usage(void)
{
	printf("Usage: gps-sdr-sim [options]\n"
		"Options:\n"
		"  -e <gps_nav>     RINEX navigation file for GPS ephemerides (required)\n"
		"  -u <user_motion> User motion file (dynamic mode)\n"
		"  -l <location>    Lat,Lon,Hgt (static mode) e.g. 30.286502,120.032669,100\n"
		"  -o <output>      I/Q sampling data file (default: gpssim.bin)\n"
		"  -s <frequency>   Sampling frequency [Hz] (default: 2600000)\n"
		"  -b <iq_bits>     I/Q data format [8/16] (default: 8)\n");

	return;
}

int main(int argc, char *argv[])
{
	clock_t tstart,tend;

	FILE *fp;

	int sv;
	int neph;
	ephem_t eph[MAX_SAT];
	gpstime_t g0;
	
	double llh[3];
	double pos[3],vel[3],clk[2];
	double tmat[3][3];
	double los[3];
	double neu[3];
	double azel[2];
	
	int i;
	int nsat;
	channel_t chan[MAX_CHAN];
	double elvmask = 0.0/R2D;

	int isbf,iwrd;
	unsigned long tow;
	unsigned long sbf[5][10];
	unsigned long sbfwrd;
	unsigned long prevwrd;
	int nib;

#ifdef _SINE_LUT
	int ip,qp;
	int iTable;
#else
	double ip,qp;
#endif
	void *iq_buff = NULL;

	gpstime_t grx;
	range_t rho0[MAX_SAT];

	double delt;
	int isamp;

	int iumd;
	int numd;
	char umfile[MAX_CHAR];
	bool staticLocationMode = false;
	double xyz[USER_MOTION_SIZE][3];

	char navfile[MAX_CHAR];
	char outfile[MAX_CHAR];

	double samp_freq;
	int iq_buff_size;
	int data_format;

	int result;

	////////////////////////////////////////////////////////////
	// Read options
	////////////////////////////////////////////////////////////

	// Default options
	navfile[0] = 0;
	umfile[0] = 0;
	strcpy(outfile, "gpssim.bin");
	samp_freq = 2.6e6;
	data_format = SC08;

	if (argc<3)
	{
		usage();
		exit(1);
	}

	while ((result=getopt(argc,argv,"e:u:l:o:s:b:"))!=-1)
	{
		switch (result)
		{
		case 'e':
			strcpy(navfile, optarg);
			break;
		case 'u':
			strcpy(umfile, optarg);
			break;
		case 'l':
			// Static geodetic coordinates input mode
			// Added by scateu@gmail.com
			staticLocationMode = true;
			sscanf(optarg,"%lf,%lf,%lf",&llh[0],&llh[1],&llh[2]);
			llh[0] = llh[0] / R2D; // convert to RAD
			llh[1] = llh[1] / R2D; // convert to RAD
			break;
		case 'o':
			strcpy(outfile, optarg);
			break;
		case 's':
			samp_freq = atof(optarg);
			if (samp_freq<1.0e6)
			{
				printf("Invalid sampling frequency.\n");
				exit(1);
			}
			break;
		case 'b':
			data_format = atoi(optarg);
			if (data_format!=SC08 && data_format!=SC16)
			{
				printf("Invalid data format.\n");
				exit(1);
			}
			break;
		case ':':
		case '?':
			usage();
			exit(1);
		default:
			break;
		}
	}

	if (navfile[0]==0)
	{
		printf("GPS ephemeris file is not specified.\n");
		exit(1);
	}

	if (umfile[0]==0 && !staticLocationMode)
	{
		printf("User motion file is not specified.\n");
		printf("Or you may use -l to specify llh coordinate directly.\n");
		exit(1);
	}

	// Buffer size	
	samp_freq = floor(samp_freq/10.0);
	iq_buff_size = (int)samp_freq; // samples per 0.1sec
	samp_freq *= 10.0;

	delt = 1.0/samp_freq;

	////////////////////////////////////////////////////////////
	// Receiver position
	////////////////////////////////////////////////////////////

	if (!staticLocationMode)
	{
		// Read user motion file
		numd = readUserMotion(xyz, umfile);
		if (numd==-1)
		{
			printf("Failed to open user motion file.\n");
			exit(1);
		}
		else if (numd==0)
		{
			printf("Failed to read user motion data.\n");
			exit(1);
		}

		printf("User motion data = %d\n", numd);

		// Initial location in Geodetic coordinate system
		xyz2llh(xyz[0], llh);
	} 
	else 
	{ 
		// Static geodetic coordinates input mode: "-l"
		// Added by scateu@gmail.com 
		printf("Using static location mode.\n");
		llh2xyz(llh,xyz[0]); // Convert llh to xyz

		numd = USER_MOTION_SIZE;
		
		for (iumd=1; iumd<numd; iumd++)
		{
			xyz[iumd][0] = xyz[0][0];
			xyz[iumd][1] = xyz[0][1];
			xyz[iumd][2] = xyz[0][2];
		}
	}

	printf("xyz = %11.1f, %11.1f, %11.1f\n", xyz[0][0], xyz[0][1], xyz[0][2]);
	printf("llh = %11.6f, %11.6f, %11.1f\n", llh[0]*R2D, llh[1]*R2D, llh[2]);

	////////////////////////////////////////////////////////////
	// Read ephemeris
	////////////////////////////////////////////////////////////

	neph = readRinexNav(eph, navfile);

	if (neph==-1)
	{
		printf("Failed to open ephemeris file.\n");
		exit(1);
	}

	g0.week = -1;

	if (neph>0)
	{
		for (sv=0; sv<MAX_SAT; sv++)
		{
			if (g0.week<0 && eph[sv].vflg==1)
			{
				g0 = eph[sv].toe; // Set simulation start time
				break;
			}
		}
	}

	g0.sec = (double)(((unsigned long)g0.sec)/30UL) * 30.0; // align with the full frame length = 30 sec

	printf("Start Time = %4d:%.1f\n", g0.week, g0.sec);

	////////////////////////////////////////////////////////////
	// Check visible satellites
	////////////////////////////////////////////////////////////

	for (i=0; i<MAX_CHAN; i++)
		chan[i].prn = 0;

	ltcmat(llh, tmat);

	nsat = 0;

	for (sv=0; sv<MAX_SAT; sv++)
	{
		if (eph[sv].vflg==1)
		{
			satpos(eph[sv], g0, pos, vel, clk);
			subVect(los, pos, xyz[0]);
			ecef2neu(los, tmat, neu);
			neu2azel(azel, neu);

			if (azel[1]>elvmask)
			{
				chan[nsat].prn = sv+1;
				nsat++;

				printf("%02d %6.1f %5.1f\n", sv+1, azel[0]*R2D, azel[1]*R2D);
			}
		}
	}

	printf("Number of channels = %d\n", nsat);

	////////////////////////////////////////////////////////////
	// Baseband signal buffer and output file
	////////////////////////////////////////////////////////////

	// Allocate I/Q buffer
	if (data_format==SC08)
		iq_buff = (signed char *)calloc(2*iq_buff_size, 1);
	else
		iq_buff = (short *)calloc(2*iq_buff_size, 2);

	if (iq_buff==NULL)
	{
		printf("Faild to allocate IQ buffer.\n");
		exit(1);
	}

	// Open output file
	if (NULL==(fp=fopen(outfile,"wb")))
	{
		printf("Failed to open output file.\n");
		exit(1);
	}

	////////////////////////////////////////////////////////////
	// Initialize channels
	////////////////////////////////////////////////////////////

	// Initial reception time
	grx = g0;

	for (i=0; i<nsat; i++)
	{
		// C/A code generation
		codegen(chan[i].ca, chan[i].prn);

		// Allocate I/Q buffer
		chan[i].iq_buff = (short *)calloc(2*iq_buff_size, 2);

		if (chan[i].iq_buff==NULL)
		{
			printf("Faild to allocate IQ buffer.\n");
			exit(1);
		}
	}

	// Initialize carrier phase
	for (i=0; i<nsat; i++)
	{
		range_t tmp;
		double ref[3]={0.0};
		double phase_offset,phase_offset_time;
		double phase_ini,phase_ini_time;

		sv = chan[i].prn-1;

		computeRange(&tmp, eph[sv], grx, ref);
		phase_offset_time = grx.sec - tmp.range/SPEED_OF_LIGHT;
		phase_offset = tmp.range/LAMBDA_L1;
		phase_offset -= floor(phase_offset);

		computeRange(&tmp, eph[sv], grx, xyz[0]);
		phase_ini_time = grx.sec - tmp.range/SPEED_OF_LIGHT;
		phase_ini = phase_offset + (phase_ini_time - phase_offset_time)*SPEED_OF_LIGHT/LAMBDA_L1;
		phase_ini -= floor(phase_ini);

		chan[i].carr_phase = phase_ini;
	}

	////////////////////////////////////////////////////////////
	// Generate subframes and data bits
	////////////////////////////////////////////////////////////

	for (i=0; i<nsat; i++)
	{
		sv = chan[i].prn-1;

		eph2sbf(eph[sv], sbf);

		chan[i].g0 = g0; // Data bit reference time

		tow = ((unsigned long)g0.sec)/6UL;

		prevwrd = 0UL;

		for (isbf=0; isbf<N_SBF; isbf++)
		{
			for (iwrd=0; iwrd<10; iwrd++)
			{
				sbfwrd = sbf[(isbf+4)%5][iwrd]; // Start from subframe 5

				// Add TOW-count message into HOW
				if (iwrd==1)
					sbfwrd |= ((tow&0x1FFFFUL)<<13);

				// Compute checksum
				sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
				
				nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10

				chan[i].dwrd[isbf*10+iwrd] = computeChecksum(sbfwrd, nib);

				prevwrd = chan[i].dwrd[isbf*10+iwrd];
			}

			tow++; // Next subframe
		}
	}

	////////////////////////////////////////////////////////////
	// Generate baseband signals
	////////////////////////////////////////////////////////////

	tstart = clock();

	printf("Generating baseband signals...\n");

	printf("\rTime = %4.1f", grx.sec-g0.sec);
	fflush(stdout);

	//
	// Generate I/Q samples for every user motion data
	//

	// Initial pseudorange
	for (i=0; i<nsat; i++)
	{
		sv = chan[i].prn-1;
		computeRange(&rho0[sv], eph[sv], grx, xyz[0]);
	}

	// Update receiver time
	grx.sec += 0.1;
	
	for (iumd=1; iumd<numd; iumd++)
	{
		//#pragma omp parallel for private(isamp) // !!!FIXME!!! The current code runs faster without OpenMP support
		// Properties -> Configuration Properties -> C/C++ -> Language -> Open MP Support -> Yes (/openmp)
		for (i=0; i<nsat; i++)
		{
			// Refresh code phase and data bit counters
			int sv = chan[i].prn-1;
			range_t rho;

			// Current pseudorange
			computeRange(&rho, eph[sv], grx, xyz[iumd]);

			// Update code phase and data bit counters
			computeCodePhase(&chan[i], rho0[sv], rho, 0.1);
			
			// Save current pseudorange
			rho0[sv] = rho;

			for (isamp=0; isamp<iq_buff_size; isamp++)
			{
#ifdef _SINE_LUT
				iTable = (int)floor(chan[i].carr_phase*512.0);

				ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable];
				qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable];

				// Store I/Q samples into buffer
				chan[i].iq_buff[isamp*2]   = (short)ip;
				chan[i].iq_buff[isamp*2+1] = (short)qp;
#else
				ip = chan[i].dataBit * chan[i].codeCA * cos(2.0*PI*chan[i].carr_phase);
				qp = chan[i].dataBit * chan[i].codeCA * sin(2.0*PI*chan[i].carr_phase);

				// Store I/Q samples into buffer
				chan[i].iq_buff[isamp*2]   = (short)(ADC_GAIN*ip);
				chan[i].iq_buff[isamp*2+1] = (short)(ADC_GAIN*qp);
#endif
				// Update code phase
				chan[i].code_phase += chan[i].f_code * delt;

				if (chan[i].code_phase>=1023.0)
				{
					chan[i].code_phase -= 1023.0;

					chan[i].icode++;
					
					if (chan[i].icode>=20) // 20 C/A codes = 1 navigation data bit
					{
						chan[i].icode = 0;
						chan[i].ibit++;
						
						if (chan[i].ibit>=30) // 30 navigation data bits = 1 word
						{
							chan[i].ibit = 0;
							chan[i].iword++;
						}

						// Set new navigation data bit
						chan[i].dataBit = (int)((chan[i].dwrd[chan[i].iword]>>(29-chan[i].ibit)) & 0x1UL)*2-1;
					}
				}

				// Set currnt code chip
				chan[i].codeCA = chan[i].ca[(int)chan[i].code_phase]*2-1;

				// Update carrier phase
				chan[i].carr_phase += chan[i].f_carr * delt;

				if (chan[i].carr_phase>=1.0)
					chan[i].carr_phase -= 1.0;
				else if (chan[i].carr_phase<0.0)
					chan[i].carr_phase += 1.0;
			}
		} // End of omp parallel for

		if (data_format==SC08)
		{
			for (isamp=0; isamp<2*iq_buff_size; isamp++)
			{
				signed char sample = 0;
				for (i=0; i<nsat; i++)
					sample += (signed char)(chan[i].iq_buff[isamp]>>4); // 12-bit bladeRF -> 8-bit HackRF
				((signed char*)iq_buff)[isamp] = sample;
			}
			fwrite(iq_buff, 1, 2*iq_buff_size, fp);
		} 
		else 
		{
			for (isamp=0; isamp<2*iq_buff_size; isamp++)
			{
				short sample = 0;
				for (i=0; i<nsat; i++)
					sample += chan[i].iq_buff[isamp];
				((short*)iq_buff)[isamp] = sample;
			}
			fwrite(iq_buff, 2, 2*iq_buff_size, fp);
		}

		// Update receiver time
		grx.sec += 0.1;

		// Update time counter
		printf("\rTime = %4.1f", grx.sec-g0.sec);
		fflush(stdout);
	}

	tend = clock();

	printf("\nDone!\n");

	// Free I/Q buffer
	free(iq_buff);
	for (i=0; i<nsat; i++)
		free(chan[i].iq_buff);

	// Close file
	fclose(fp);

	// Process time
	printf("Process time = %.3f[sec]\n", (double)(tend-tstart)/CLOCKS_PER_SEC);

	return(0);
}
