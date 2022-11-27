/*------------------------------------------------------------------------------
* allystar.c : allystar receiver dependent functions
*
*          Copyright (C) 
*          Copyright (C) 
*
* reference :
*
*-----------------------------------------------------------------------------*/
#include "rtklib2.h"

#define ALLSYNC1    0xF1        /* message sync code 1 */
#define ALLSYNC2    0xD9        /* message sync code 2 */
#define ALLCFG      0x06        /* message cfg-??? */
#define ALLNAV      0x01
#define ALLMON      0x0A
#define ALLMON_VER  0x0A04
#define ALLMON_INFO 0x0A05

#define ID_MSG_RTCM 0xF8
#define ID_MSG_RTCM1005 0xF805
#define ID_MSG_RTCM1006 0xF806
#define ID_MSG_RTCM1033 0xF821
#define ID_MSG_RTCM1077 0xF84D
#define ID_MSG_RTCM1087 0xF857
#define ID_MSG_RTCM1097 0xF861
#define ID_MSG_RTCM1117 0xF875
#define ID_MSG_RTCM1127 0xF87F

#define ID_MSG_RTCM1074 0xF84A
#define ID_MSG_RTCM1084 0xF854
#define ID_MSG_RTCM1094 0xF858
#define ID_MSG_RTCM1114 0xF872
#define ID_MSG_RTCM1124 0xF87C

#define ID_MSG_RTCM1019 0xF813
#define ID_MSG_RTCM1020 0xF814
#define ID_MSG_RTCM1044 0xF82C
#define ID_MSG_RTCM1045 0xF82D
#define ID_MSG_RTCM1046 0xF82E
#define ID_MSG_RTCM1042 0xF82A

#define FU1         1           /* ubx message field types */
#define FU2         2
#define FU4         3
#define FS1         4
#define FS2         5
#define FS4         6
#define FR4         7

//U1 S1 U2 S2 U4 S4 R4
/* set fields (little-endian) ------------------------------------------------*/
static void setU1(unsigned char *p, unsigned char  u) {*p=u;}
static void setS1(unsigned char *p,   signed char  s) {*p=s;}

static void setU2(unsigned char *p, unsigned short u) {memcpy(p,&u,2);}
static void setS2(unsigned char *p, short          u) {memcpy(p,&u,2);}
static void setU4(unsigned char *p, unsigned int   u) {memcpy(p,&u,4);}
static void setS4(unsigned char *p,          int   i) {memcpy(p,&i,4);}
static void setR4(unsigned char *p, float          r) {memcpy(p,&r,4);}

/* checksum ------------------------------------------------------------------*/
static int checksum(unsigned char *buff, int len)
{
    unsigned char cka=0,ckb=0;
    int i;
    
    for (i=2;i<len-2;i++) {
        cka+=buff[i]; ckb+=cka;
    }
    return cka==buff[len-2]&&ckb==buff[len-1];
}

/* set checksum ------------------------------------------------------------------*/
static void setcs(unsigned char *buff, int len)
{
    unsigned char cka=0,ckb=0;
    int i;
    
    for (i=2;i<len-2;i++) {
        cka+=buff[i]; ckb+=cka;
    }
    buff[len-2]=cka;
    buff[len-1]=ckb;
}

/* TODO : add survey-in mode */

/* TODO : add cfg-rate */

extern void gen_cfg_fixedecef_hex(double *xyz,unsigned char *buff)
{
	unsigned char *q=buff;

	//cfg-fixedecef F1 D9 06 14 0C 00 00 00 00 00 00 00 00 00 00 00 00 00 26 34
    //F1 D9 06 14 0C 00 2F 5C F7 EE D5 15 C7 1B E2 B2 84 13 8D 85
    //-2857788.974 4660320.857 3274636.501
    //
	
	*q++=ALLSYNC1;
	*q++=ALLSYNC2;
	*q++=ALLCFG;
	*q++=0x14; //cfg_fixedecef
	*q++=0x0C; //payload length,2 bytes
	*q++=0x00;

    int i4;

	for(int i=0;i<3;i++){
	    i4=(int)(xyz[i]*100);
	    tracet(3,"float xyz(cm):%f\n",i4);
	    setS4(q,i4);
	    q+=4;
	}

	setcs(buff,20);

    traceb(3,buff,20);

    return;
}