/*------------------------------------------------------------------------------
* allystar.c : allystar receiver dependent functions
*
*          Copyright (C) 
*          Copyright (C) 
*
* reference :
*
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define ALLSYNC1    0xF1        /* message sync code 1 */
#define ALLSYNC2    0xD9        /* message sync code 2 */
#define ALLCFG      0x06        /* message cfg-??? */
#define ALLCFGMSG   0x0601
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

#define ID_MSG_NMEA 0xF0
/* NMEA data */
#define ID_MSG_GGA 0xF000
#define ID_MSG_GSA 0xF002
#define ID_MSG_GSV 0xF004
#define ID_MSG_RMC 0xF005
#define ID_MSG_GST 0xF008
#define ID_MSG_ZDA 0xF007
#define ID_MSG_TXT 0xF020

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

#define FU1         1           /* 1,unsigned char, 0 to 255 */
#define FU2         2           /* 2,unsigned short, 0 to 65,535 */
#define FU4         3           /* 4,unsigned int, 0 to 4,294,967,295 */
#define FS1         4           /* 1,signed char, -128 to 127 FI1*/
#define FS2         5           /* 2,signed short, -32,768 to 32,767 FI2*/
#define FS4         6           /* 4,signed int FI4*/
#define FR4         7           /* 4,float */
/* T-5-2003-012-GNSS Protocol Specification-V2.3.5.pdf */
/* https://learn.microsoft.com/en-us/cpp/cpp/data-type-ranges?view=msvc-170 */


#define ROUND(x)    (int)floor((x)+0.5)
/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define S1(p) (*((int8_t  *)(p)))
static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}
static int32_t  S4(uint8_t *p) {int32_t  u; memcpy(&u,p,4); return u;}
static float    R4(uint8_t *p) {float    r; memcpy(&r,p,4); return r;}

/* set fields (little-endian) ------------------------------------------------*/
static void setU1(uint8_t *p, uint8_t   u) {*p=u;}
static void setS1(uint8_t *p, int8_t    i) {*p=(uint8_t)i;}

static void setU2(uint8_t *p, uint16_t  u) {memcpy(p,&u,2);}
static void setS2(uint8_t *p, int16_t   i) {memcpy(p,&i,2);}
static void setU4(uint8_t *p, uint32_t  u) {memcpy(p,&u,4);}
static void setS4(uint8_t *p, int32_t   i) {memcpy(p,&i,4);}
static void setR4(uint8_t *p, float     r) {memcpy(p,&r,4);}

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

/* convert string to integer -------------------------------------------------*/
static int stoi(const char *s)
{
    uint32_t n;
    if (sscanf(s,"0x%X",&n)==1) return (int)n; /* hex (0xXXXX) */
    return atoi(s);
}

/* TODO : add survey-in mode */

/* TODO : add cfg-rate */

extern void gen_cfg_fixedecef_hex(double *xyz,uint8_t *buff)
{
	uint8_t *q=buff;
    int i4;
    int i=0,n;

	/* 
    cfg-fixedecef F1 D9 06 14 0C 00 00 00 00 00 00 00 00 00 00 00 00 00 26 34
    F1 D9 06 14 0C 00 2F 5C F7 EE D5 15 C7 1B E2 B2 84 13 8D 85
    -2857788.974 4660320.857 3274636.501
    */
	
	*q++=ALLSYNC1;
	*q++=ALLSYNC2;
	*q++=ALLCFG;
	*q++=0x14; /* cfg_fixedecef */
	q+=2; /* payload length,2 bytes */


	for(i=0;i<3;i++){
	    i4=(int)(xyz[i]*100);
	    tracet(3,"float xyz(cm):%f\n",i4);
	    setS4(q,i4);
	    q+=4;
	}

    n=(int)(q-buff)+2;
    setU2(buff+4,(unsigned short)(n-8)); /* set length */
	setcs(buff,n);

    traceb(3,buff,n);

    return;
}

const char *freq_no[]= {
            "GPS L1",
            "GLONASS G1" ,
            "BEIDOU B1" ,
            "GALILEO E1" ,
            "QZSS L1" ,
            "SBAS L1" ,
            "IRNSS L5" ,
            "GPS L1C" ,
            "GPS L2C" ,
            "GPS L5" ,
            "GLONASS G2" ,
            "BEIDOU B1C" ,
            "BEIDOU B2" ,
            "BEIDOU B2A" ,
            "BEIDOU B3I" ,
            "BEIDOU B5" ,
            "GALILEO E5A" ,
            "GALILEO E5B" ,
            "GALILEO E6" ,
            "QZSS L2C" ,
            "QZSS L5" ,
            "QZSS L1C" ,
            "QZSS L6" 
};

const uint32_t gnss_id[]= {
            0x00000001, /* GPS L1 */
            0x00000002, /* GLONASS G1 */
            0x00000004, /* BEIDOU B1 */
            0x00000010, /* GALILEO E1 */
            0x00000020, /* QZSS L1 */
            0x00000040, /* SBAS L1 */
            0x00000080, /* IRNSS L5 */
            0x00000100, /* GPS L1C */
            0x00000400, /* GPS L2C */
            0x00000200, /* GPS L5 */
            0x00002000, /* GLONASS G2 */
            0x00004000, /* BEIDOU B1C */
            0x00040000, /* BEIDOU B2 */
            0x00008000, /* BEIDOU B2A */
            0x00010000, /* BEIDOU B3I */
            0x00020000, /* BEIDOU B5 */
            0x00100000, /* GALILEO E5A */
            0x00200000, /* GALILEO E5B */
            0x00400000, /* GALILEO E6 */
            0x08000000, /* QZSS L2C */
            0x04000000, /* QZSS L5 */
            0x02000000, /* QZSS L1C */
            0x01000000  /* QZSS L6 */
};

/* generate ublox binary message -----------------------------------------------
* generate ublox binary message from message string
* args   : char  *msg   IO     message string 
*            "CFG-MSG   msgid msgsubid period,0xF8 0x61 1,1097"
*            "CFG-NMEA  nmea rate"
*            "CFG-RTCM  4/7 rate"
*            "CFG-NUMSV min max"
*            "CFG-FIXEDECEF x y z"
*            "CFG-SURVEY mindura acclimit"
*            "CFG-BDGEO id flag"
*            "CFG-CARRSMOOTH window_value"
*/
extern int gen_ally(const char *msg, uint8_t *buff)
{
    const char *cmd[]={
        "MSG","CFG","ELEV","NAVSAT","NUMSV","SURVEY","FIXEDLLA",
        "FIXEDECEF","BDGEO","CARRSMOOTH","SIMPLERST","NMEAVER","RATE"
    };
    const uint8_t id[]={
        0x01,0x09,0x0B,0x0C,0x11,0x12,0x13,0x14,0x16,0x17,0x40,0x43,0x44
    };

    const uint8_t nmeaid[]={0x00, 0x02, 0x04, 0x05, 0x07, 0x08, 0x20};

    const int prm[][32]={
        {FU1,FU1,FU1},                  /* MSG */
        {FU4,FU4},                      /* CFG,0 7 */
        {FR4,FR4},                      /* ELEV */
        {FU4},                          /* NAVSAT */
        {FU1,FU1},                      /* NUMSV,3 20 */
        {FU4,FU4},                      /* SURVEY,30s 3000mm */
        {FS4,FS4,FS4},                  /* FIXEDLLA */
        {FS4,FS4,FS4},                  /* FIXEDECEF */
        {FU1,FU1},                      /* BDGEO */
        {FS1},                          /* CARRSMOOTH */
        {FU1},                          /* SIMPLERST */
        {FU1},                          /* NMEAVER */
        {FU1,FU1,FU2,FS4,FU4,FU4}       /* RATE,PWRCTL2,0 0 1 1 200 0 */
    };

    uint8_t *q=buff;

    char mbuff[1024],*args[32],*p;
    int i,j,n,narg=0;
    
    trace(4,"gen_ally: msg=%s\n",msg);
    
    strcpy(mbuff,msg);
    for (p=strtok(mbuff," ");p&&narg<32;p=strtok(NULL," ")) {
        args[narg++]=p;
    }
    if (narg<1||strncmp(args[0],"CFG-",4)) return 0;

    *q++=ALLSYNC1;
    *q++=ALLSYNC2;
    *q++=ALLCFG;

    for (i=0;*cmd[i];i++) {
            if (!strcmp(args[0]+4,cmd[i])) break;
        }
        if (!*cmd[i]) return 0;
        
        *q++=id[i];
        q+=2;

        for (j=1;prm[i][j-1]||j<narg;j++) {
            switch (prm[i][j-1]) {
                case FU1 : setU1(q,j<narg?(uint8_t )stoi(args[j]):0); q+=1; break;
                case FU2 : setU2(q,j<narg?(uint16_t)stoi(args[j]):0); q+=2; break;
                case FU4 : setU4(q,j<narg?(uint32_t)stoi(args[j]):0); q+=4; break;
                case FS1 : setS1(q,j<narg?(int8_t  )stoi(args[j]):0); q+=1; break;
                case FS2 : setS2(q,j<narg?(int16_t )stoi(args[j]):0); q+=2; break;
                case FS4 : setS4(q,j<narg?(int32_t )stoi(args[j]):0); q+=4; break;
                case FR4 : setR4(q,j<narg?(float         )atof(args[j]):0); q+=4; break;
                default  : setU1(q,j<narg?(uint8_t )stoi(args[j]):0); q+=1; break;
            }
        }

        n=(int)(q-buff)+2;
        setU2(buff+4,(unsigned short)(n-8));
        setcs(buff,n);
    
    trace(5,"gen_ally: buff=\n"); traceb(5,buff,n);
    return n;
}