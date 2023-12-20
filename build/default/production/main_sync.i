
# 1 "main_sync.c"

# 4 "/opt/microchip/xc8/v2.10/pic/include/__size_t.h"
typedef unsigned size_t;

# 7 "/opt/microchip/xc8/v2.10/pic/include/c90/stdarg.h"
typedef void * va_list[1];

#pragma intrinsic(__va_start)
extern void * __va_start(void);

#pragma intrinsic(__va_arg)
extern void * __va_arg(void *, ...);

# 43 "/opt/microchip/xc8/v2.10/pic/include/c90/stdio.h"
struct __prbuf
{
char * ptr;
void (* func)(char);
};

# 29 "/opt/microchip/xc8/v2.10/pic/include/c90/errno.h"
extern int errno;

# 12 "/opt/microchip/xc8/v2.10/pic/include/c90/conio.h"
extern void init_uart(void);

extern char getch(void);
extern char getche(void);
extern void putch(char);
extern void ungetch(char);

extern __bit kbhit(void);

# 23
extern char * cgets(char *);
extern void cputs(const char *);

# 88 "/opt/microchip/xc8/v2.10/pic/include/c90/stdio.h"
extern int cprintf(char *, ...);
#pragma printf_check(cprintf)



extern int _doprnt(struct __prbuf *, const register char *, register va_list);


# 180
#pragma printf_check(vprintf) const
#pragma printf_check(vsprintf) const

extern char * gets(char *);
extern int puts(const char *);
extern int scanf(const char *, ...) __attribute__((unsupported("scanf() is not supported by this compiler")));
extern int sscanf(const char *, const char *, ...) __attribute__((unsupported("sscanf() is not supported by this compiler")));
extern int vprintf(const char *, va_list) __attribute__((unsupported("vprintf() is not supported by this compiler")));
extern int vsprintf(char *, const char *, va_list) __attribute__((unsupported("vsprintf() is not supported by this compiler")));
extern int vscanf(const char *, va_list ap) __attribute__((unsupported("vscanf() is not supported by this compiler")));
extern int vsscanf(const char *, const char *, va_list) __attribute__((unsupported("vsscanf() is not supported by this compiler")));

#pragma printf_check(printf) const
#pragma printf_check(sprintf) const
extern int sprintf(char *, const char *, ...);
extern int printf(const char *, ...);

# 7 "/opt/microchip/xc8/v2.10/pic/include/c90/stdlib.h"
typedef unsigned short wchar_t;

# 15
typedef struct {
int rem;
int quot;
} div_t;
typedef struct {
unsigned rem;
unsigned quot;
} udiv_t;
typedef struct {
long quot;
long rem;
} ldiv_t;
typedef struct {
unsigned long quot;
unsigned long rem;
} uldiv_t;

# 65
extern double atof(const char *);
extern double strtod(const char *, const char **);
extern int atoi(const char *);
extern unsigned xtoi(const char *);
extern long atol(const char *);

# 73
extern long strtol(const char *, char **, int);

extern int rand(void);
extern void srand(unsigned int);
extern void * calloc(size_t, size_t);
extern div_t div(int numer, int denom);
extern udiv_t udiv(unsigned numer, unsigned denom);
extern ldiv_t ldiv(long numer, long denom);
extern uldiv_t uldiv(unsigned long numer,unsigned long denom);

# 85
extern unsigned long _lrotl(unsigned long value, unsigned int shift);
extern unsigned long _lrotr(unsigned long value, unsigned int shift);
extern unsigned int _rotl(unsigned int value, unsigned int shift);
extern unsigned int _rotr(unsigned int value, unsigned int shift);




extern void * malloc(size_t);
extern void free(void *);
extern void * realloc(void *, size_t);


# 13 "/opt/microchip/xc8/v2.10/pic/include/c90/xc8debug.h"
#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);

# 104 "/opt/microchip/xc8/v2.10/pic/include/c90/stdlib.h"
extern int atexit(void (*)(void));
extern char * getenv(const char *);
extern char ** environ;
extern int system(char *);
extern void qsort(void *, size_t, size_t, int (*)(const void *, const void *));
extern void * bsearch(const void *, void *, size_t, size_t, int(*)(const void *, const void *));
extern int abs(int);
extern long labs(long);

extern char * itoa(char * buf, int val, int base);
extern char * utoa(char * buf, unsigned val, int base);




extern char * ltoa(char * buf, long val, int base);
extern char * ultoa(char * buf, unsigned long val, int base);

extern char * ftoa(float f, int * status);

# 30 "/opt/microchip/xc8/v2.10/pic/include/c90/math.h"
extern double fabs(double);
extern double floor(double);
extern double ceil(double);
extern double modf(double, double *);
extern double sqrt(double);
extern double atof(const char *);
extern double sin(double) ;
extern double cos(double) ;
extern double tan(double) ;
extern double asin(double) ;
extern double acos(double) ;
extern double atan(double);
extern double atan2(double, double) ;
extern double log(double);
extern double log10(double);
extern double pow(double, double) ;
extern double exp(double) ;
extern double sinh(double) ;
extern double cosh(double) ;
extern double tanh(double);
extern double eval_poly(double, const double *, int);
extern double frexp(double, int *);
extern double ldexp(double, int);
extern double fmod(double, double);
extern double trunc(double);
extern double round(double);

# 18 "/opt/microchip/xc8/v2.10/pic/include/xc.h"
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);

# 52 "/opt/microchip/xc8/v2.10/pic/include/pic16f887.h"
extern volatile unsigned char INDF __at(0x000);

asm("INDF equ 00h");




extern volatile unsigned char TMR0 __at(0x001);

asm("TMR0 equ 01h");




extern volatile unsigned char PCL __at(0x002);

asm("PCL equ 02h");




extern volatile unsigned char STATUS __at(0x003);

asm("STATUS equ 03h");


typedef union {
struct {
unsigned C :1;
unsigned DC :1;
unsigned Z :1;
unsigned nPD :1;
unsigned nTO :1;
unsigned RP :2;
unsigned IRP :1;
};
struct {
unsigned :5;
unsigned RP0 :1;
unsigned RP1 :1;
};
struct {
unsigned CARRY :1;
unsigned :1;
unsigned ZERO :1;
};
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __at(0x003);

# 159
extern volatile unsigned char FSR __at(0x004);

asm("FSR equ 04h");




extern volatile unsigned char PORTA __at(0x005);

asm("PORTA equ 05h");


typedef union {
struct {
unsigned RA0 :1;
unsigned RA1 :1;
unsigned RA2 :1;
unsigned RA3 :1;
unsigned RA4 :1;
unsigned RA5 :1;
unsigned RA6 :1;
unsigned RA7 :1;
};
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __at(0x005);

# 228
extern volatile unsigned char PORTB __at(0x006);

asm("PORTB equ 06h");


typedef union {
struct {
unsigned RB0 :1;
unsigned RB1 :1;
unsigned RB2 :1;
unsigned RB3 :1;
unsigned RB4 :1;
unsigned RB5 :1;
unsigned RB6 :1;
unsigned RB7 :1;
};
} PORTBbits_t;
extern volatile PORTBbits_t PORTBbits __at(0x006);

# 290
extern volatile unsigned char PORTC __at(0x007);

asm("PORTC equ 07h");


typedef union {
struct {
unsigned RC0 :1;
unsigned RC1 :1;
unsigned RC2 :1;
unsigned RC3 :1;
unsigned RC4 :1;
unsigned RC5 :1;
unsigned RC6 :1;
unsigned RC7 :1;
};
} PORTCbits_t;
extern volatile PORTCbits_t PORTCbits __at(0x007);

# 352
extern volatile unsigned char PORTD __at(0x008);

asm("PORTD equ 08h");


typedef union {
struct {
unsigned RD0 :1;
unsigned RD1 :1;
unsigned RD2 :1;
unsigned RD3 :1;
unsigned RD4 :1;
unsigned RD5 :1;
unsigned RD6 :1;
unsigned RD7 :1;
};
} PORTDbits_t;
extern volatile PORTDbits_t PORTDbits __at(0x008);

# 414
extern volatile unsigned char PORTE __at(0x009);

asm("PORTE equ 09h");


typedef union {
struct {
unsigned RE0 :1;
unsigned RE1 :1;
unsigned RE2 :1;
unsigned RE3 :1;
};
} PORTEbits_t;
extern volatile PORTEbits_t PORTEbits __at(0x009);

# 452
extern volatile unsigned char PCLATH __at(0x00A);

asm("PCLATH equ 0Ah");




extern volatile unsigned char INTCON __at(0x00B);

asm("INTCON equ 0Bh");


typedef union {
struct {
unsigned RBIF :1;
unsigned INTF :1;
unsigned T0IF :1;
unsigned RBIE :1;
unsigned INTE :1;
unsigned T0IE :1;
unsigned PEIE :1;
unsigned GIE :1;
};
struct {
unsigned :2;
unsigned TMR0IF :1;
unsigned :2;
unsigned TMR0IE :1;
};
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __at(0x00B);

# 537
extern volatile unsigned char PIR1 __at(0x00C);

asm("PIR1 equ 0Ch");


typedef union {
struct {
unsigned TMR1IF :1;
unsigned TMR2IF :1;
unsigned CCP1IF :1;
unsigned SSPIF :1;
unsigned TXIF :1;
unsigned RCIF :1;
unsigned ADIF :1;
};
} PIR1bits_t;
extern volatile PIR1bits_t PIR1bits __at(0x00C);

# 593
extern volatile unsigned char PIR2 __at(0x00D);

asm("PIR2 equ 0Dh");


typedef union {
struct {
unsigned CCP2IF :1;
unsigned :1;
unsigned ULPWUIF :1;
unsigned BCLIF :1;
unsigned EEIF :1;
unsigned C1IF :1;
unsigned C2IF :1;
unsigned OSFIF :1;
};
} PIR2bits_t;
extern volatile PIR2bits_t PIR2bits __at(0x00D);

# 650
extern volatile unsigned short TMR1 __at(0x00E);

asm("TMR1 equ 0Eh");




extern volatile unsigned char TMR1L __at(0x00E);

asm("TMR1L equ 0Eh");




extern volatile unsigned char TMR1H __at(0x00F);

asm("TMR1H equ 0Fh");




extern volatile unsigned char T1CON __at(0x010);

asm("T1CON equ 010h");


typedef union {
struct {
unsigned TMR1ON :1;
unsigned TMR1CS :1;
unsigned nT1SYNC :1;
unsigned T1OSCEN :1;
unsigned T1CKPS :2;
unsigned TMR1GE :1;
unsigned T1GINV :1;
};
struct {
unsigned :2;
unsigned T1INSYNC :1;
unsigned :1;
unsigned T1CKPS0 :1;
unsigned T1CKPS1 :1;
unsigned :1;
unsigned T1GIV :1;
};
struct {
unsigned :2;
unsigned T1SYNC :1;
};
} T1CONbits_t;
extern volatile T1CONbits_t T1CONbits __at(0x010);

# 765
extern volatile unsigned char TMR2 __at(0x011);

asm("TMR2 equ 011h");




extern volatile unsigned char T2CON __at(0x012);

asm("T2CON equ 012h");


typedef union {
struct {
unsigned T2CKPS :2;
unsigned TMR2ON :1;
unsigned TOUTPS :4;
};
struct {
unsigned T2CKPS0 :1;
unsigned T2CKPS1 :1;
unsigned :1;
unsigned TOUTPS0 :1;
unsigned TOUTPS1 :1;
unsigned TOUTPS2 :1;
unsigned TOUTPS3 :1;
};
} T2CONbits_t;
extern volatile T2CONbits_t T2CONbits __at(0x012);

# 843
extern volatile unsigned char SSPBUF __at(0x013);

asm("SSPBUF equ 013h");




extern volatile unsigned char SSPCON __at(0x014);

asm("SSPCON equ 014h");


typedef union {
struct {
unsigned SSPM :4;
unsigned CKP :1;
unsigned SSPEN :1;
unsigned SSPOV :1;
unsigned WCOL :1;
};
struct {
unsigned SSPM0 :1;
unsigned SSPM1 :1;
unsigned SSPM2 :1;
unsigned SSPM3 :1;
};
} SSPCONbits_t;
extern volatile SSPCONbits_t SSPCONbits __at(0x014);

# 920
extern volatile unsigned short CCPR1 __at(0x015);

asm("CCPR1 equ 015h");




extern volatile unsigned char CCPR1L __at(0x015);

asm("CCPR1L equ 015h");




extern volatile unsigned char CCPR1H __at(0x016);

asm("CCPR1H equ 016h");




extern volatile unsigned char CCP1CON __at(0x017);

asm("CCP1CON equ 017h");


typedef union {
struct {
unsigned CCP1M :4;
unsigned DC1B :2;
unsigned P1M :2;
};
struct {
unsigned CCP1M0 :1;
unsigned CCP1M1 :1;
unsigned CCP1M2 :1;
unsigned CCP1M3 :1;
unsigned DC1B0 :1;
unsigned DC1B1 :1;
unsigned P1M0 :1;
unsigned P1M1 :1;
};
struct {
unsigned :4;
unsigned CCP1Y :1;
unsigned CCP1X :1;
};
} CCP1CONbits_t;
extern volatile CCP1CONbits_t CCP1CONbits __at(0x017);

# 1038
extern volatile unsigned char RCSTA __at(0x018);

asm("RCSTA equ 018h");


typedef union {
struct {
unsigned RX9D :1;
unsigned OERR :1;
unsigned FERR :1;
unsigned ADDEN :1;
unsigned CREN :1;
unsigned SREN :1;
unsigned RX9 :1;
unsigned SPEN :1;
};
struct {
unsigned RCD8 :1;
unsigned :5;
unsigned RC9 :1;
};
struct {
unsigned :6;
unsigned nRC8 :1;
};
struct {
unsigned :6;
unsigned RC8_9 :1;
};
} RCSTAbits_t;
extern volatile RCSTAbits_t RCSTAbits __at(0x018);

# 1133
extern volatile unsigned char TXREG __at(0x019);

asm("TXREG equ 019h");




extern volatile unsigned char RCREG __at(0x01A);

asm("RCREG equ 01Ah");




extern volatile unsigned short CCPR2 __at(0x01B);

asm("CCPR2 equ 01Bh");




extern volatile unsigned char CCPR2L __at(0x01B);

asm("CCPR2L equ 01Bh");




extern volatile unsigned char CCPR2H __at(0x01C);

asm("CCPR2H equ 01Ch");




extern volatile unsigned char CCP2CON __at(0x01D);

asm("CCP2CON equ 01Dh");


typedef union {
struct {
unsigned CCP2M :4;
unsigned DC2B0 :1;
unsigned DC2B1 :1;
};
struct {
unsigned CCP2M0 :1;
unsigned CCP2M1 :1;
unsigned CCP2M2 :1;
unsigned CCP2M3 :1;
unsigned CCP2Y :1;
unsigned CCP2X :1;
};
} CCP2CONbits_t;
extern volatile CCP2CONbits_t CCP2CONbits __at(0x01D);

# 1238
extern volatile unsigned char ADRESH __at(0x01E);

asm("ADRESH equ 01Eh");




extern volatile unsigned char ADCON0 __at(0x01F);

asm("ADCON0 equ 01Fh");


typedef union {
struct {
unsigned ADON :1;
unsigned GO_nDONE :1;
unsigned CHS :4;
unsigned ADCS :2;
};
struct {
unsigned :1;
unsigned GO :1;
unsigned CHS0 :1;
unsigned CHS1 :1;
unsigned CHS2 :1;
unsigned CHS3 :1;
unsigned ADCS0 :1;
unsigned ADCS1 :1;
};
struct {
unsigned :1;
unsigned nDONE :1;
};
struct {
unsigned :1;
unsigned GO_DONE :1;
};
} ADCON0bits_t;
extern volatile ADCON0bits_t ADCON0bits __at(0x01F);

# 1346
extern volatile unsigned char OPTION_REG __at(0x081);

asm("OPTION_REG equ 081h");


typedef union {
struct {
unsigned PS :3;
unsigned PSA :1;
unsigned T0SE :1;
unsigned T0CS :1;
unsigned INTEDG :1;
unsigned nRBPU :1;
};
struct {
unsigned PS0 :1;
unsigned PS1 :1;
unsigned PS2 :1;
};
} OPTION_REGbits_t;
extern volatile OPTION_REGbits_t OPTION_REGbits __at(0x081);

# 1416
extern volatile unsigned char TRISA __at(0x085);

asm("TRISA equ 085h");


typedef union {
struct {
unsigned TRISA0 :1;
unsigned TRISA1 :1;
unsigned TRISA2 :1;
unsigned TRISA3 :1;
unsigned TRISA4 :1;
unsigned TRISA5 :1;
unsigned TRISA6 :1;
unsigned TRISA7 :1;
};
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __at(0x085);

# 1478
extern volatile unsigned char TRISB __at(0x086);

asm("TRISB equ 086h");


typedef union {
struct {
unsigned TRISB0 :1;
unsigned TRISB1 :1;
unsigned TRISB2 :1;
unsigned TRISB3 :1;
unsigned TRISB4 :1;
unsigned TRISB5 :1;
unsigned TRISB6 :1;
unsigned TRISB7 :1;
};
} TRISBbits_t;
extern volatile TRISBbits_t TRISBbits __at(0x086);

# 1540
extern volatile unsigned char TRISC __at(0x087);

asm("TRISC equ 087h");


typedef union {
struct {
unsigned TRISC0 :1;
unsigned TRISC1 :1;
unsigned TRISC2 :1;
unsigned TRISC3 :1;
unsigned TRISC4 :1;
unsigned TRISC5 :1;
unsigned TRISC6 :1;
unsigned TRISC7 :1;
};
} TRISCbits_t;
extern volatile TRISCbits_t TRISCbits __at(0x087);

# 1602
extern volatile unsigned char TRISD __at(0x088);

asm("TRISD equ 088h");


typedef union {
struct {
unsigned TRISD0 :1;
unsigned TRISD1 :1;
unsigned TRISD2 :1;
unsigned TRISD3 :1;
unsigned TRISD4 :1;
unsigned TRISD5 :1;
unsigned TRISD6 :1;
unsigned TRISD7 :1;
};
} TRISDbits_t;
extern volatile TRISDbits_t TRISDbits __at(0x088);

# 1664
extern volatile unsigned char TRISE __at(0x089);

asm("TRISE equ 089h");


typedef union {
struct {
unsigned TRISE0 :1;
unsigned TRISE1 :1;
unsigned TRISE2 :1;
unsigned TRISE3 :1;
};
} TRISEbits_t;
extern volatile TRISEbits_t TRISEbits __at(0x089);

# 1702
extern volatile unsigned char PIE1 __at(0x08C);

asm("PIE1 equ 08Ch");


typedef union {
struct {
unsigned TMR1IE :1;
unsigned TMR2IE :1;
unsigned CCP1IE :1;
unsigned SSPIE :1;
unsigned TXIE :1;
unsigned RCIE :1;
unsigned ADIE :1;
};
} PIE1bits_t;
extern volatile PIE1bits_t PIE1bits __at(0x08C);

# 1758
extern volatile unsigned char PIE2 __at(0x08D);

asm("PIE2 equ 08Dh");


typedef union {
struct {
unsigned CCP2IE :1;
unsigned :1;
unsigned ULPWUIE :1;
unsigned BCLIE :1;
unsigned EEIE :1;
unsigned C1IE :1;
unsigned C2IE :1;
unsigned OSFIE :1;
};
} PIE2bits_t;
extern volatile PIE2bits_t PIE2bits __at(0x08D);

# 1815
extern volatile unsigned char PCON __at(0x08E);

asm("PCON equ 08Eh");


typedef union {
struct {
unsigned nBOR :1;
unsigned nPOR :1;
unsigned :2;
unsigned SBOREN :1;
unsigned ULPWUE :1;
};
struct {
unsigned nBO :1;
};
} PCONbits_t;
extern volatile PCONbits_t PCONbits __at(0x08E);

# 1862
extern volatile unsigned char OSCCON __at(0x08F);

asm("OSCCON equ 08Fh");


typedef union {
struct {
unsigned SCS :1;
unsigned LTS :1;
unsigned HTS :1;
unsigned OSTS :1;
unsigned IRCF :3;
};
struct {
unsigned :4;
unsigned IRCF0 :1;
unsigned IRCF1 :1;
unsigned IRCF2 :1;
};
} OSCCONbits_t;
extern volatile OSCCONbits_t OSCCONbits __at(0x08F);

# 1927
extern volatile unsigned char OSCTUNE __at(0x090);

asm("OSCTUNE equ 090h");


typedef union {
struct {
unsigned TUN :5;
};
struct {
unsigned TUN0 :1;
unsigned TUN1 :1;
unsigned TUN2 :1;
unsigned TUN3 :1;
unsigned TUN4 :1;
};
} OSCTUNEbits_t;
extern volatile OSCTUNEbits_t OSCTUNEbits __at(0x090);

# 1979
extern volatile unsigned char SSPCON2 __at(0x091);

asm("SSPCON2 equ 091h");


typedef union {
struct {
unsigned SEN :1;
unsigned RSEN :1;
unsigned PEN :1;
unsigned RCEN :1;
unsigned ACKEN :1;
unsigned ACKDT :1;
unsigned ACKSTAT :1;
unsigned GCEN :1;
};
} SSPCON2bits_t;
extern volatile SSPCON2bits_t SSPCON2bits __at(0x091);

# 2041
extern volatile unsigned char PR2 __at(0x092);

asm("PR2 equ 092h");




extern volatile unsigned char SSPADD __at(0x093);

asm("SSPADD equ 093h");




extern volatile unsigned char SSPMSK __at(0x093);

asm("SSPMSK equ 093h");


extern volatile unsigned char MSK __at(0x093);

asm("MSK equ 093h");


typedef union {
struct {
unsigned MSK0 :1;
unsigned MSK1 :1;
unsigned MSK2 :1;
unsigned MSK3 :1;
unsigned MSK4 :1;
unsigned MSK5 :1;
unsigned MSK6 :1;
unsigned MSK7 :1;
};
} SSPMSKbits_t;
extern volatile SSPMSKbits_t SSPMSKbits __at(0x093);

# 2120
typedef union {
struct {
unsigned MSK0 :1;
unsigned MSK1 :1;
unsigned MSK2 :1;
unsigned MSK3 :1;
unsigned MSK4 :1;
unsigned MSK5 :1;
unsigned MSK6 :1;
unsigned MSK7 :1;
};
} MSKbits_t;
extern volatile MSKbits_t MSKbits __at(0x093);

# 2177
extern volatile unsigned char SSPSTAT __at(0x094);

asm("SSPSTAT equ 094h");


typedef union {
struct {
unsigned BF :1;
unsigned UA :1;
unsigned R_nW :1;
unsigned S :1;
unsigned P :1;
unsigned D_nA :1;
unsigned CKE :1;
unsigned SMP :1;
};
struct {
unsigned :2;
unsigned R :1;
unsigned :2;
unsigned D :1;
};
struct {
unsigned :2;
unsigned I2C_READ :1;
unsigned I2C_START :1;
unsigned I2C_STOP :1;
unsigned I2C_DATA :1;
};
struct {
unsigned :2;
unsigned nW :1;
unsigned :2;
unsigned nA :1;
};
struct {
unsigned :2;
unsigned nWRITE :1;
unsigned :2;
unsigned nADDRESS :1;
};
struct {
unsigned :2;
unsigned R_W :1;
unsigned :2;
unsigned D_A :1;
};
struct {
unsigned :2;
unsigned READ_WRITE :1;
unsigned :2;
unsigned DATA_ADDRESS :1;
};
} SSPSTATbits_t;
extern volatile SSPSTATbits_t SSPSTATbits __at(0x094);

# 2346
extern volatile unsigned char WPUB __at(0x095);

asm("WPUB equ 095h");


typedef union {
struct {
unsigned WPUB :8;
};
struct {
unsigned WPUB0 :1;
unsigned WPUB1 :1;
unsigned WPUB2 :1;
unsigned WPUB3 :1;
unsigned WPUB4 :1;
unsigned WPUB5 :1;
unsigned WPUB6 :1;
unsigned WPUB7 :1;
};
} WPUBbits_t;
extern volatile WPUBbits_t WPUBbits __at(0x095);

# 2416
extern volatile unsigned char IOCB __at(0x096);

asm("IOCB equ 096h");


typedef union {
struct {
unsigned IOCB :8;
};
struct {
unsigned IOCB0 :1;
unsigned IOCB1 :1;
unsigned IOCB2 :1;
unsigned IOCB3 :1;
unsigned IOCB4 :1;
unsigned IOCB5 :1;
unsigned IOCB6 :1;
unsigned IOCB7 :1;
};
} IOCBbits_t;
extern volatile IOCBbits_t IOCBbits __at(0x096);

# 2486
extern volatile unsigned char VRCON __at(0x097);

asm("VRCON equ 097h");


typedef union {
struct {
unsigned VR :4;
unsigned VRSS :1;
unsigned VRR :1;
unsigned VROE :1;
unsigned VREN :1;
};
struct {
unsigned VR0 :1;
unsigned VR1 :1;
unsigned VR2 :1;
unsigned VR3 :1;
};
} VRCONbits_t;
extern volatile VRCONbits_t VRCONbits __at(0x097);

# 2556
extern volatile unsigned char TXSTA __at(0x098);

asm("TXSTA equ 098h");


typedef union {
struct {
unsigned TX9D :1;
unsigned TRMT :1;
unsigned BRGH :1;
unsigned SENDB :1;
unsigned SYNC :1;
unsigned TXEN :1;
unsigned TX9 :1;
unsigned CSRC :1;
};
struct {
unsigned TXD8 :1;
unsigned :5;
unsigned nTX8 :1;
};
struct {
unsigned :6;
unsigned TX8_9 :1;
};
} TXSTAbits_t;
extern volatile TXSTAbits_t TXSTAbits __at(0x098);

# 2642
extern volatile unsigned char SPBRG __at(0x099);

asm("SPBRG equ 099h");


typedef union {
struct {
unsigned BRG0 :1;
unsigned BRG1 :1;
unsigned BRG2 :1;
unsigned BRG3 :1;
unsigned BRG4 :1;
unsigned BRG5 :1;
unsigned BRG6 :1;
unsigned BRG7 :1;
};
} SPBRGbits_t;
extern volatile SPBRGbits_t SPBRGbits __at(0x099);

# 2704
extern volatile unsigned char SPBRGH __at(0x09A);

asm("SPBRGH equ 09Ah");


typedef union {
struct {
unsigned SPBRGH :8;
};
struct {
unsigned BRG8 :1;
unsigned BRG9 :1;
unsigned BRG10 :1;
unsigned BRG11 :1;
unsigned BRG12 :1;
unsigned BRG13 :1;
unsigned BRG14 :1;
unsigned BRG15 :1;
};
} SPBRGHbits_t;
extern volatile SPBRGHbits_t SPBRGHbits __at(0x09A);

# 2774
extern volatile unsigned char PWM1CON __at(0x09B);

asm("PWM1CON equ 09Bh");


typedef union {
struct {
unsigned PDC :7;
unsigned PRSEN :1;
};
struct {
unsigned PDC0 :1;
unsigned PDC1 :1;
unsigned PDC2 :1;
unsigned PDC3 :1;
unsigned PDC4 :1;
unsigned PDC5 :1;
unsigned PDC6 :1;
};
} PWM1CONbits_t;
extern volatile PWM1CONbits_t PWM1CONbits __at(0x09B);

# 2844
extern volatile unsigned char ECCPAS __at(0x09C);

asm("ECCPAS equ 09Ch");


typedef union {
struct {
unsigned PSSBD :2;
unsigned PSSAC :2;
unsigned ECCPAS :3;
unsigned ECCPASE :1;
};
struct {
unsigned PSSBD0 :1;
unsigned PSSBD1 :1;
unsigned PSSAC0 :1;
unsigned PSSAC1 :1;
unsigned ECCPAS0 :1;
unsigned ECCPAS1 :1;
unsigned ECCPAS2 :1;
};
} ECCPASbits_t;
extern volatile ECCPASbits_t ECCPASbits __at(0x09C);

# 2926
extern volatile unsigned char PSTRCON __at(0x09D);

asm("PSTRCON equ 09Dh");


typedef union {
struct {
unsigned STRA :1;
unsigned STRB :1;
unsigned STRC :1;
unsigned STRD :1;
unsigned STRSYNC :1;
};
} PSTRCONbits_t;
extern volatile PSTRCONbits_t PSTRCONbits __at(0x09D);

# 2970
extern volatile unsigned char ADRESL __at(0x09E);

asm("ADRESL equ 09Eh");




extern volatile unsigned char ADCON1 __at(0x09F);

asm("ADCON1 equ 09Fh");


typedef union {
struct {
unsigned :4;
unsigned VCFG0 :1;
unsigned VCFG1 :1;
unsigned :1;
unsigned ADFM :1;
};
} ADCON1bits_t;
extern volatile ADCON1bits_t ADCON1bits __at(0x09F);

# 3011
extern volatile unsigned char WDTCON __at(0x105);

asm("WDTCON equ 0105h");


typedef union {
struct {
unsigned SWDTEN :1;
unsigned WDTPS :4;
};
struct {
unsigned :1;
unsigned WDTPS0 :1;
unsigned WDTPS1 :1;
unsigned WDTPS2 :1;
unsigned WDTPS3 :1;
};
} WDTCONbits_t;
extern volatile WDTCONbits_t WDTCONbits __at(0x105);

# 3064
extern volatile unsigned char CM1CON0 __at(0x107);

asm("CM1CON0 equ 0107h");


typedef union {
struct {
unsigned C1CH :2;
unsigned C1R :1;
unsigned :1;
unsigned C1POL :1;
unsigned C1OE :1;
unsigned C1OUT :1;
unsigned C1ON :1;
};
struct {
unsigned C1CH0 :1;
unsigned C1CH1 :1;
};
} CM1CON0bits_t;
extern volatile CM1CON0bits_t CM1CON0bits __at(0x107);

# 3129
extern volatile unsigned char CM2CON0 __at(0x108);

asm("CM2CON0 equ 0108h");


typedef union {
struct {
unsigned C2CH :2;
unsigned C2R :1;
unsigned :1;
unsigned C2POL :1;
unsigned C2OE :1;
unsigned C2OUT :1;
unsigned C2ON :1;
};
struct {
unsigned C2CH0 :1;
unsigned C2CH1 :1;
};
} CM2CON0bits_t;
extern volatile CM2CON0bits_t CM2CON0bits __at(0x108);

# 3194
extern volatile unsigned char CM2CON1 __at(0x109);

asm("CM2CON1 equ 0109h");


typedef union {
struct {
unsigned C2SYNC :1;
unsigned T1GSS :1;
unsigned :2;
unsigned C2RSEL :1;
unsigned C1RSEL :1;
unsigned MC2OUT :1;
unsigned MC1OUT :1;
};
} CM2CON1bits_t;
extern volatile CM2CON1bits_t CM2CON1bits __at(0x109);

# 3245
extern volatile unsigned char EEDATA __at(0x10C);

asm("EEDATA equ 010Ch");


extern volatile unsigned char EEDAT __at(0x10C);

asm("EEDAT equ 010Ch");




extern volatile unsigned char EEADR __at(0x10D);

asm("EEADR equ 010Dh");




extern volatile unsigned char EEDATH __at(0x10E);

asm("EEDATH equ 010Eh");




extern volatile unsigned char EEADRH __at(0x10F);

asm("EEADRH equ 010Fh");




extern volatile unsigned char SRCON __at(0x185);

asm("SRCON equ 0185h");


typedef union {
struct {
unsigned FVREN :1;
unsigned :1;
unsigned PULSR :1;
unsigned PULSS :1;
unsigned C2REN :1;
unsigned C1SEN :1;
unsigned SR0 :1;
unsigned SR1 :1;
};
} SRCONbits_t;
extern volatile SRCONbits_t SRCONbits __at(0x185);

# 3335
extern volatile unsigned char BAUDCTL __at(0x187);

asm("BAUDCTL equ 0187h");


typedef union {
struct {
unsigned ABDEN :1;
unsigned WUE :1;
unsigned :1;
unsigned BRG16 :1;
unsigned SCKP :1;
unsigned :1;
unsigned RCIDL :1;
unsigned ABDOVF :1;
};
} BAUDCTLbits_t;
extern volatile BAUDCTLbits_t BAUDCTLbits __at(0x187);

# 3387
extern volatile unsigned char ANSEL __at(0x188);

asm("ANSEL equ 0188h");


typedef union {
struct {
unsigned ANS0 :1;
unsigned ANS1 :1;
unsigned ANS2 :1;
unsigned ANS3 :1;
unsigned ANS4 :1;
unsigned ANS5 :1;
unsigned ANS6 :1;
unsigned ANS7 :1;
};
} ANSELbits_t;
extern volatile ANSELbits_t ANSELbits __at(0x188);

# 3449
extern volatile unsigned char ANSELH __at(0x189);

asm("ANSELH equ 0189h");


typedef union {
struct {
unsigned ANS8 :1;
unsigned ANS9 :1;
unsigned ANS10 :1;
unsigned ANS11 :1;
unsigned ANS12 :1;
unsigned ANS13 :1;
};
} ANSELHbits_t;
extern volatile ANSELHbits_t ANSELHbits __at(0x189);

# 3499
extern volatile unsigned char EECON1 __at(0x18C);

asm("EECON1 equ 018Ch");


typedef union {
struct {
unsigned RD :1;
unsigned WR :1;
unsigned WREN :1;
unsigned WRERR :1;
unsigned :3;
unsigned EEPGD :1;
};
} EECON1bits_t;
extern volatile EECON1bits_t EECON1bits __at(0x18C);

# 3544
extern volatile unsigned char EECON2 __at(0x18D);

asm("EECON2 equ 018Dh");

# 3557
extern volatile __bit ABDEN __at(0xC38);


extern volatile __bit ABDOVF __at(0xC3F);


extern volatile __bit ACKDT __at(0x48D);


extern volatile __bit ACKEN __at(0x48C);


extern volatile __bit ACKSTAT __at(0x48E);


extern volatile __bit ADCS0 __at(0xFE);


extern volatile __bit ADCS1 __at(0xFF);


extern volatile __bit ADDEN __at(0xC3);


extern volatile __bit ADFM __at(0x4FF);


extern volatile __bit ADIE __at(0x466);


extern volatile __bit ADIF __at(0x66);


extern volatile __bit ADON __at(0xF8);


extern volatile __bit ANS0 __at(0xC40);


extern volatile __bit ANS1 __at(0xC41);


extern volatile __bit ANS10 __at(0xC4A);


extern volatile __bit ANS11 __at(0xC4B);


extern volatile __bit ANS12 __at(0xC4C);


extern volatile __bit ANS13 __at(0xC4D);


extern volatile __bit ANS2 __at(0xC42);


extern volatile __bit ANS3 __at(0xC43);


extern volatile __bit ANS4 __at(0xC44);


extern volatile __bit ANS5 __at(0xC45);


extern volatile __bit ANS6 __at(0xC46);


extern volatile __bit ANS7 __at(0xC47);


extern volatile __bit ANS8 __at(0xC48);


extern volatile __bit ANS9 __at(0xC49);


extern volatile __bit BCLIE __at(0x46B);


extern volatile __bit BCLIF __at(0x6B);


extern volatile __bit BF __at(0x4A0);


extern volatile __bit BRG0 __at(0x4C8);


extern volatile __bit BRG1 __at(0x4C9);


extern volatile __bit BRG10 __at(0x4D2);


extern volatile __bit BRG11 __at(0x4D3);


extern volatile __bit BRG12 __at(0x4D4);


extern volatile __bit BRG13 __at(0x4D5);


extern volatile __bit BRG14 __at(0x4D6);


extern volatile __bit BRG15 __at(0x4D7);


extern volatile __bit BRG16 __at(0xC3B);


extern volatile __bit BRG2 __at(0x4CA);


extern volatile __bit BRG3 __at(0x4CB);


extern volatile __bit BRG4 __at(0x4CC);


extern volatile __bit BRG5 __at(0x4CD);


extern volatile __bit BRG6 __at(0x4CE);


extern volatile __bit BRG7 __at(0x4CF);


extern volatile __bit BRG8 __at(0x4D0);


extern volatile __bit BRG9 __at(0x4D1);


extern volatile __bit BRGH __at(0x4C2);


extern volatile __bit C1CH0 __at(0x838);


extern volatile __bit C1CH1 __at(0x839);


extern volatile __bit C1IE __at(0x46D);


extern volatile __bit C1IF __at(0x6D);


extern volatile __bit C1OE __at(0x83D);


extern volatile __bit C1ON __at(0x83F);


extern volatile __bit C1OUT __at(0x83E);


extern volatile __bit C1POL __at(0x83C);


extern volatile __bit C1R __at(0x83A);


extern volatile __bit C1RSEL __at(0x84D);


extern volatile __bit C1SEN __at(0xC2D);


extern volatile __bit C2CH0 __at(0x840);


extern volatile __bit C2CH1 __at(0x841);


extern volatile __bit C2IE __at(0x46E);


extern volatile __bit C2IF __at(0x6E);


extern volatile __bit C2OE __at(0x845);


extern volatile __bit C2ON __at(0x847);


extern volatile __bit C2OUT __at(0x846);


extern volatile __bit C2POL __at(0x844);


extern volatile __bit C2R __at(0x842);


extern volatile __bit C2REN __at(0xC2C);


extern volatile __bit C2RSEL __at(0x84C);


extern volatile __bit C2SYNC __at(0x848);


extern volatile __bit CARRY __at(0x18);


extern volatile __bit CCP1IE __at(0x462);


extern volatile __bit CCP1IF __at(0x62);


extern volatile __bit CCP1M0 __at(0xB8);


extern volatile __bit CCP1M1 __at(0xB9);


extern volatile __bit CCP1M2 __at(0xBA);


extern volatile __bit CCP1M3 __at(0xBB);


extern volatile __bit CCP1X __at(0xBD);


extern volatile __bit CCP1Y __at(0xBC);


extern volatile __bit CCP2IE __at(0x468);


extern volatile __bit CCP2IF __at(0x68);


extern volatile __bit CCP2M0 __at(0xE8);


extern volatile __bit CCP2M1 __at(0xE9);


extern volatile __bit CCP2M2 __at(0xEA);


extern volatile __bit CCP2M3 __at(0xEB);


extern volatile __bit CCP2X __at(0xED);


extern volatile __bit CCP2Y __at(0xEC);


extern volatile __bit CHS0 __at(0xFA);


extern volatile __bit CHS1 __at(0xFB);


extern volatile __bit CHS2 __at(0xFC);


extern volatile __bit CHS3 __at(0xFD);


extern volatile __bit CKE __at(0x4A6);


extern volatile __bit CKP __at(0xA4);


extern volatile __bit CREN __at(0xC4);


extern volatile __bit CSRC __at(0x4C7);


extern volatile __bit DATA_ADDRESS __at(0x4A5);


extern volatile __bit DC __at(0x19);


extern volatile __bit DC1B0 __at(0xBC);


extern volatile __bit DC1B1 __at(0xBD);


extern volatile __bit DC2B0 __at(0xEC);


extern volatile __bit DC2B1 __at(0xED);


extern volatile __bit D_A __at(0x4A5);


extern volatile __bit D_nA __at(0x4A5);


extern volatile __bit ECCPAS0 __at(0x4E4);


extern volatile __bit ECCPAS1 __at(0x4E5);


extern volatile __bit ECCPAS2 __at(0x4E6);


extern volatile __bit ECCPASE __at(0x4E7);


extern volatile __bit EEIE __at(0x46C);


extern volatile __bit EEIF __at(0x6C);


extern volatile __bit EEPGD __at(0xC67);


extern volatile __bit FERR __at(0xC2);


extern volatile __bit FVREN __at(0xC28);


extern volatile __bit GCEN __at(0x48F);


extern volatile __bit GIE __at(0x5F);


extern volatile __bit GO __at(0xF9);


extern volatile __bit GO_DONE __at(0xF9);


extern volatile __bit GO_nDONE __at(0xF9);


extern volatile __bit HTS __at(0x47A);


extern volatile __bit I2C_DATA __at(0x4A5);


extern volatile __bit I2C_READ __at(0x4A2);


extern volatile __bit I2C_START __at(0x4A3);


extern volatile __bit I2C_STOP __at(0x4A4);


extern volatile __bit INTE __at(0x5C);


extern volatile __bit INTEDG __at(0x40E);


extern volatile __bit INTF __at(0x59);


extern volatile __bit IOCB0 __at(0x4B0);


extern volatile __bit IOCB1 __at(0x4B1);


extern volatile __bit IOCB2 __at(0x4B2);


extern volatile __bit IOCB3 __at(0x4B3);


extern volatile __bit IOCB4 __at(0x4B4);


extern volatile __bit IOCB5 __at(0x4B5);


extern volatile __bit IOCB6 __at(0x4B6);


extern volatile __bit IOCB7 __at(0x4B7);


extern volatile __bit IRCF0 __at(0x47C);


extern volatile __bit IRCF1 __at(0x47D);


extern volatile __bit IRCF2 __at(0x47E);


extern volatile __bit IRP __at(0x1F);


extern volatile __bit LTS __at(0x479);


extern volatile __bit MC1OUT __at(0x84F);


extern volatile __bit MC2OUT __at(0x84E);


extern volatile __bit MSK0 __at(0x498);


extern volatile __bit MSK1 __at(0x499);


extern volatile __bit MSK2 __at(0x49A);


extern volatile __bit MSK3 __at(0x49B);


extern volatile __bit MSK4 __at(0x49C);


extern volatile __bit MSK5 __at(0x49D);


extern volatile __bit MSK6 __at(0x49E);


extern volatile __bit MSK7 __at(0x49F);


extern volatile __bit OERR __at(0xC1);


extern volatile __bit OSFIE __at(0x46F);


extern volatile __bit OSFIF __at(0x6F);


extern volatile __bit OSTS __at(0x47B);


extern volatile __bit P1M0 __at(0xBE);


extern volatile __bit P1M1 __at(0xBF);


extern volatile __bit PDC0 __at(0x4D8);


extern volatile __bit PDC1 __at(0x4D9);


extern volatile __bit PDC2 __at(0x4DA);


extern volatile __bit PDC3 __at(0x4DB);


extern volatile __bit PDC4 __at(0x4DC);


extern volatile __bit PDC5 __at(0x4DD);


extern volatile __bit PDC6 __at(0x4DE);


extern volatile __bit PEIE __at(0x5E);


extern volatile __bit PEN __at(0x48A);


extern volatile __bit PRSEN __at(0x4DF);


extern volatile __bit PS0 __at(0x408);


extern volatile __bit PS1 __at(0x409);


extern volatile __bit PS2 __at(0x40A);


extern volatile __bit PSA __at(0x40B);


extern volatile __bit PSSAC0 __at(0x4E2);


extern volatile __bit PSSAC1 __at(0x4E3);


extern volatile __bit PSSBD0 __at(0x4E0);


extern volatile __bit PSSBD1 __at(0x4E1);


extern volatile __bit PULSR __at(0xC2A);


extern volatile __bit PULSS __at(0xC2B);


extern volatile __bit RA0 __at(0x28);


extern volatile __bit RA1 __at(0x29);


extern volatile __bit RA2 __at(0x2A);


extern volatile __bit RA3 __at(0x2B);


extern volatile __bit RA4 __at(0x2C);


extern volatile __bit RA5 __at(0x2D);


extern volatile __bit RA6 __at(0x2E);


extern volatile __bit RA7 __at(0x2F);


extern volatile __bit RB0 __at(0x30);


extern volatile __bit RB1 __at(0x31);


extern volatile __bit RB2 __at(0x32);


extern volatile __bit RB3 __at(0x33);


extern volatile __bit RB4 __at(0x34);


extern volatile __bit RB5 __at(0x35);


extern volatile __bit RB6 __at(0x36);


extern volatile __bit RB7 __at(0x37);


extern volatile __bit RBIE __at(0x5B);


extern volatile __bit RBIF __at(0x58);


extern volatile __bit RC0 __at(0x38);


extern volatile __bit RC1 __at(0x39);


extern volatile __bit RC2 __at(0x3A);


extern volatile __bit RC3 __at(0x3B);


extern volatile __bit RC4 __at(0x3C);


extern volatile __bit RC5 __at(0x3D);


extern volatile __bit RC6 __at(0x3E);


extern volatile __bit RC7 __at(0x3F);


extern volatile __bit RC8_9 __at(0xC6);


extern volatile __bit RC9 __at(0xC6);


extern volatile __bit RCD8 __at(0xC0);


extern volatile __bit RCEN __at(0x48B);


extern volatile __bit RCIDL __at(0xC3E);


extern volatile __bit RCIE __at(0x465);


extern volatile __bit RCIF __at(0x65);


extern volatile __bit RD __at(0xC60);


extern volatile __bit RD0 __at(0x40);


extern volatile __bit RD1 __at(0x41);


extern volatile __bit RD2 __at(0x42);


extern volatile __bit RD3 __at(0x43);


extern volatile __bit RD4 __at(0x44);


extern volatile __bit RD5 __at(0x45);


extern volatile __bit RD6 __at(0x46);


extern volatile __bit RD7 __at(0x47);


extern volatile __bit RE0 __at(0x48);


extern volatile __bit RE1 __at(0x49);


extern volatile __bit RE2 __at(0x4A);


extern volatile __bit RE3 __at(0x4B);


extern volatile __bit READ_WRITE __at(0x4A2);


extern volatile __bit RP0 __at(0x1D);


extern volatile __bit RP1 __at(0x1E);


extern volatile __bit RSEN __at(0x489);


extern volatile __bit RX9 __at(0xC6);


extern volatile __bit RX9D __at(0xC0);


extern volatile __bit R_W __at(0x4A2);


extern volatile __bit R_nW __at(0x4A2);


extern volatile __bit SBOREN __at(0x474);


extern volatile __bit SCKP __at(0xC3C);


extern volatile __bit SCS __at(0x478);


extern volatile __bit SEN __at(0x488);


extern volatile __bit SENDB __at(0x4C3);


extern volatile __bit SMP __at(0x4A7);


extern volatile __bit SPEN __at(0xC7);


extern volatile __bit SR0 __at(0xC2E);


extern volatile __bit SR1 __at(0xC2F);


extern volatile __bit SREN __at(0xC5);


extern volatile __bit SSPEN __at(0xA5);


extern volatile __bit SSPIE __at(0x463);


extern volatile __bit SSPIF __at(0x63);


extern volatile __bit SSPM0 __at(0xA0);


extern volatile __bit SSPM1 __at(0xA1);


extern volatile __bit SSPM2 __at(0xA2);


extern volatile __bit SSPM3 __at(0xA3);


extern volatile __bit SSPOV __at(0xA6);


extern volatile __bit STRA __at(0x4E8);


extern volatile __bit STRB __at(0x4E9);


extern volatile __bit STRC __at(0x4EA);


extern volatile __bit STRD __at(0x4EB);


extern volatile __bit STRSYNC __at(0x4EC);


extern volatile __bit SWDTEN __at(0x828);


extern volatile __bit SYNC __at(0x4C4);


extern volatile __bit T0CS __at(0x40D);


extern volatile __bit T0IE __at(0x5D);


extern volatile __bit T0IF __at(0x5A);


extern volatile __bit T0SE __at(0x40C);


extern volatile __bit T1CKPS0 __at(0x84);


extern volatile __bit T1CKPS1 __at(0x85);


extern volatile __bit T1GINV __at(0x87);


extern volatile __bit T1GIV __at(0x87);


extern volatile __bit T1GSS __at(0x849);


extern volatile __bit T1INSYNC __at(0x82);


extern volatile __bit T1OSCEN __at(0x83);


extern volatile __bit T1SYNC __at(0x82);


extern volatile __bit T2CKPS0 __at(0x90);


extern volatile __bit T2CKPS1 __at(0x91);


extern volatile __bit TMR0IE __at(0x5D);


extern volatile __bit TMR0IF __at(0x5A);


extern volatile __bit TMR1CS __at(0x81);


extern volatile __bit TMR1GE __at(0x86);


extern volatile __bit TMR1IE __at(0x460);


extern volatile __bit TMR1IF __at(0x60);


extern volatile __bit TMR1ON __at(0x80);


extern volatile __bit TMR2IE __at(0x461);


extern volatile __bit TMR2IF __at(0x61);


extern volatile __bit TMR2ON __at(0x92);


extern volatile __bit TOUTPS0 __at(0x93);


extern volatile __bit TOUTPS1 __at(0x94);


extern volatile __bit TOUTPS2 __at(0x95);


extern volatile __bit TOUTPS3 __at(0x96);


extern volatile __bit TRISA0 __at(0x428);


extern volatile __bit TRISA1 __at(0x429);


extern volatile __bit TRISA2 __at(0x42A);


extern volatile __bit TRISA3 __at(0x42B);


extern volatile __bit TRISA4 __at(0x42C);


extern volatile __bit TRISA5 __at(0x42D);


extern volatile __bit TRISA6 __at(0x42E);


extern volatile __bit TRISA7 __at(0x42F);


extern volatile __bit TRISB0 __at(0x430);


extern volatile __bit TRISB1 __at(0x431);


extern volatile __bit TRISB2 __at(0x432);


extern volatile __bit TRISB3 __at(0x433);


extern volatile __bit TRISB4 __at(0x434);


extern volatile __bit TRISB5 __at(0x435);


extern volatile __bit TRISB6 __at(0x436);


extern volatile __bit TRISB7 __at(0x437);


extern volatile __bit TRISC0 __at(0x438);


extern volatile __bit TRISC1 __at(0x439);


extern volatile __bit TRISC2 __at(0x43A);


extern volatile __bit TRISC3 __at(0x43B);


extern volatile __bit TRISC4 __at(0x43C);


extern volatile __bit TRISC5 __at(0x43D);


extern volatile __bit TRISC6 __at(0x43E);


extern volatile __bit TRISC7 __at(0x43F);


extern volatile __bit TRISD0 __at(0x440);


extern volatile __bit TRISD1 __at(0x441);


extern volatile __bit TRISD2 __at(0x442);


extern volatile __bit TRISD3 __at(0x443);


extern volatile __bit TRISD4 __at(0x444);


extern volatile __bit TRISD5 __at(0x445);


extern volatile __bit TRISD6 __at(0x446);


extern volatile __bit TRISD7 __at(0x447);


extern volatile __bit TRISE0 __at(0x448);


extern volatile __bit TRISE1 __at(0x449);


extern volatile __bit TRISE2 __at(0x44A);


extern volatile __bit TRISE3 __at(0x44B);


extern volatile __bit TRMT __at(0x4C1);


extern volatile __bit TUN0 __at(0x480);


extern volatile __bit TUN1 __at(0x481);


extern volatile __bit TUN2 __at(0x482);


extern volatile __bit TUN3 __at(0x483);


extern volatile __bit TUN4 __at(0x484);


extern volatile __bit TX8_9 __at(0x4C6);


extern volatile __bit TX9 __at(0x4C6);


extern volatile __bit TX9D __at(0x4C0);


extern volatile __bit TXD8 __at(0x4C0);


extern volatile __bit TXEN __at(0x4C5);


extern volatile __bit TXIE __at(0x464);


extern volatile __bit TXIF __at(0x64);


extern volatile __bit UA __at(0x4A1);


extern volatile __bit ULPWUE __at(0x475);


extern volatile __bit ULPWUIE __at(0x46A);


extern volatile __bit ULPWUIF __at(0x6A);


extern volatile __bit VCFG0 __at(0x4FC);


extern volatile __bit VCFG1 __at(0x4FD);


extern volatile __bit VR0 __at(0x4B8);


extern volatile __bit VR1 __at(0x4B9);


extern volatile __bit VR2 __at(0x4BA);


extern volatile __bit VR3 __at(0x4BB);


extern volatile __bit VREN __at(0x4BF);


extern volatile __bit VROE __at(0x4BE);


extern volatile __bit VRR __at(0x4BD);


extern volatile __bit VRSS __at(0x4BC);


extern volatile __bit WCOL __at(0xA7);


extern volatile __bit WDTPS0 __at(0x829);


extern volatile __bit WDTPS1 __at(0x82A);


extern volatile __bit WDTPS2 __at(0x82B);


extern volatile __bit WDTPS3 __at(0x82C);


extern volatile __bit WPUB0 __at(0x4A8);


extern volatile __bit WPUB1 __at(0x4A9);


extern volatile __bit WPUB2 __at(0x4AA);


extern volatile __bit WPUB3 __at(0x4AB);


extern volatile __bit WPUB4 __at(0x4AC);


extern volatile __bit WPUB5 __at(0x4AD);


extern volatile __bit WPUB6 __at(0x4AE);


extern volatile __bit WPUB7 __at(0x4AF);


extern volatile __bit WR __at(0xC61);


extern volatile __bit WREN __at(0xC62);


extern volatile __bit WRERR __at(0xC63);


extern volatile __bit WUE __at(0xC39);


extern volatile __bit ZERO __at(0x1A);


extern volatile __bit nA __at(0x4A5);


extern volatile __bit nADDRESS __at(0x4A5);


extern volatile __bit nBO __at(0x470);


extern volatile __bit nBOR __at(0x470);


extern volatile __bit nDONE __at(0xF9);


extern volatile __bit nPD __at(0x1B);


extern volatile __bit nPOR __at(0x471);


extern volatile __bit nRBPU __at(0x40F);


extern volatile __bit nRC8 __at(0xC6);


extern volatile __bit nT1SYNC __at(0x82);


extern volatile __bit nTO __at(0x1C);


extern volatile __bit nTX8 __at(0x4C6);


extern volatile __bit nW __at(0x4A2);


extern volatile __bit nWRITE __at(0x4A2);


# 30 "/opt/microchip/xc8/v2.10/pic/include/pic.h"
#pragma intrinsic(__nop)
extern void __nop(void);

# 78
__attribute__((__unsupported__("The " "FLASH_READ" " macro function is no longer supported. Please use the MPLAB X MCC."))) unsigned char __flash_read(unsigned short addr);

__attribute__((__unsupported__("The " "FLASH_WRITE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_write(unsigned short addr, unsigned short data);

__attribute__((__unsupported__("The " "FLASH_ERASE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_erase(unsigned short addr);

# 114 "/opt/microchip/xc8/v2.10/pic/include/eeprom_routines.h"
extern void eeprom_write(unsigned char addr, unsigned char value);
extern unsigned char eeprom_read(unsigned char addr);


# 91 "/opt/microchip/xc8/v2.10/pic/include/pic.h"
#pragma intrinsic(_delay)
extern __nonreentrant void _delay(unsigned long);
#pragma intrinsic(_delaywdt)
extern __nonreentrant void _delaywdt(unsigned long);

# 137
extern __bank0 unsigned char __resetbits;
extern __bank0 __bit __powerdown;
extern __bank0 __bit __timeout;

# 13 "/opt/microchip/xc8/v2.10/pic/include/c90/stdint.h"
typedef signed char int8_t;

# 20
typedef signed int int16_t;

# 28
typedef __int24 int24_t;

# 36
typedef signed long int int32_t;

# 52
typedef unsigned char uint8_t;

# 58
typedef unsigned int uint16_t;

# 65
typedef __uint24 uint24_t;

# 72
typedef unsigned long int uint32_t;

# 88
typedef signed char int_least8_t;

# 96
typedef signed int int_least16_t;

# 109
typedef __int24 int_least24_t;

# 118
typedef signed long int int_least32_t;

# 136
typedef unsigned char uint_least8_t;

# 143
typedef unsigned int uint_least16_t;

# 154
typedef __uint24 uint_least24_t;

# 162
typedef unsigned long int uint_least32_t;

# 181
typedef signed char int_fast8_t;

# 188
typedef signed int int_fast16_t;

# 200
typedef __int24 int_fast24_t;

# 208
typedef signed long int int_fast32_t;

# 224
typedef unsigned char uint_fast8_t;

# 230
typedef unsigned int uint_fast16_t;

# 240
typedef __uint24 uint_fast24_t;

# 247
typedef unsigned long int uint_fast32_t;

# 268
typedef int32_t intmax_t;

# 282
typedef uint32_t uintmax_t;

# 289
typedef int16_t intptr_t;




typedef uint16_t uintptr_t;

# 15 "/opt/microchip/xc8/v2.10/pic/include/c90/stdbool.h"
typedef unsigned char bool;

# 15
typedef unsigned char bool;

# 23 "libbsmp/bsmp.h"
struct bsmp_version
{
uint8_t major;
uint8_t minor;
uint8_t revision;
char str[20];
};



enum group_id
{
GROUP_ALL_ID,
GROUP_READ_ID,
GROUP_WRITE_ID,

GROUP_STANDARD_COUNT,
};



enum bsmp_bin_op
{
BIN_OP_AND,
BIN_OP_OR,
BIN_OP_XOR,
BIN_OP_SET,
BIN_OP_CLEAR,
BIN_OP_TOGGLE,

BIN_OP_COUNT,
};

typedef void (*bin_op_function) (uint8_t *data, uint8_t *mask, uint8_t size);
extern bin_op_function bin_op[256];



enum bsmp_err
{
BSMP_SUCCESS,
BSMP_ERR_PARAM_INVALID,
BSMP_ERR_PARAM_OUT_OF_RANGE,

BSMP_ERR_OUT_OF_MEMORY,
BSMP_ERR_DUPLICATE,

BSMP_ERR_COMM,

BSMP_ERR_NOT_INITIALIZED,
BSMP_ERR_MAX
};

# 95
struct bsmp_var_info
{
uint8_t id;
bool writable;
uint8_t size;
};

struct bsmp_var
{
struct bsmp_var_info info;
bool (*value_ok) (struct bsmp_var *, uint8_t *);
uint8_t *data;
void *user;

};

struct bsmp_var_info_list
{
uint32_t count;
struct bsmp_var_info list[128];
};

struct bsmp_var_info_ptr_list
{
uint32_t count;
struct bsmp_var_info *list[128];
};

struct bsmp_var_ptr_list
{
uint32_t count;
struct bsmp_var *list[128];
};



struct bsmp_group
{
uint8_t id;
bool writable;
uint16_t size;


struct bsmp_var_info_ptr_list vars;
};

struct bsmp_group_list
{
uint32_t count;
struct bsmp_group list[8];
};



struct bsmp_curve_info
{
uint8_t id;
bool writable;
uint32_t nblocks;
uint16_t block_size;
uint8_t checksum[16];
};

struct bsmp_curve;

typedef void (*bsmp_curve_read_t) (struct bsmp_curve *curve, uint16_t block,
uint8_t *data, uint16_t *len);
typedef void (*bsmp_curve_write_t) (struct bsmp_curve *curve, uint16_t block,
uint8_t *data, uint16_t len);
struct bsmp_curve
{

struct bsmp_curve_info info;


void (*read_block)(struct bsmp_curve *curve, uint16_t block, uint8_t *data,
uint16_t *len);

void (*write_block)(struct bsmp_curve *curve, uint16_t block, uint8_t *data,
uint16_t len);



void *user;
};

struct bsmp_curve_info_list
{
uint32_t count;
struct bsmp_curve_info list[128];
};

struct bsmp_curve_ptr_list
{
uint32_t count;
struct bsmp_curve *list[128];
};



struct bsmp_func_info
{
uint8_t id;
uint8_t input_size;
uint8_t output_size;
};

typedef uint8_t (*bsmp_func_t) (uint8_t *input, uint8_t *output);
struct bsmp_func
{
struct bsmp_func_info info;
bsmp_func_t func_p;
};


struct bsmp_func_info_list
{
uint32_t count;
struct bsmp_func_info list[128];
};

struct bsmp_func_ptr_list
{
uint32_t count;
struct bsmp_func *list[128];
};

# 229
const char * bsmp_error_str (enum bsmp_err error);

# 9 "libbsmp/server.h"
typedef struct bsmp_server bsmp_server_t;



enum bsmp_operation
{
BSMP_OP_READ,
BSMP_OP_WRITE,
};

typedef void (*bsmp_hook_t) (enum bsmp_operation op, struct bsmp_var **list);




struct bsmp_raw_packet
{
uint8_t *data;
uint16_t len;
};

# 38
bsmp_server_t *bsmp_server_new (void);

# 50
enum bsmp_err bsmp_server_destroy (bsmp_server_t *server);

# 72
enum bsmp_err bsmp_register_variable (bsmp_server_t *server,
struct bsmp_var *var);

# 103
enum bsmp_err bsmp_register_curve (bsmp_server_t *server,
struct bsmp_curve *curve);

# 130
enum bsmp_err bsmp_register_function (bsmp_server_t *server,
struct bsmp_func *func);

# 159
enum bsmp_err bsmp_register_hook (bsmp_server_t *server, bsmp_hook_t hook);

# 174
enum bsmp_err bsmp_process_packet (bsmp_server_t *server,
struct bsmp_raw_packet *request,
struct bsmp_raw_packet *response);


# 23 "main_sync.c"
#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = OFF

#pragma config CP = OFF
#pragma config CPD = OFF
#pragma config BOREN = OFF
#pragma config IESO = OFF
#pragma config FCMEN = OFF
#pragma config LVP = OFF


#pragma config BOR4V = BOR40V
#pragma config WRT = OFF

# 64
void __interrupt () interruption(void);

void main (void);
void inicializar (void);
void compara (void);
int protocolo (void);
uint16_t ad (void);
void enviar (char *envia, int n);
uint8_t getEEPROM (uint8_t c);
void setEEPROM (uint8_t addr, uint8_t valor);
void initLT (void);
void escreveLT (uint8_t regAddr, uint8_t data);
int SelectChannel(int ch);

char idPlaca;
char cSum = 0;
int fator = 985;
char recebidos = 0;
char serial[40] = {0};
int valorAD[8];
int media2[2] = {0};
float temp[8];
int erros[8] = {0};
uint8_t k1, k2, b1, b2, fator1, fator2;
float k, b;
uint8_t lerADpuro = 0;
uint8_t modoLeitura = 8;



void main()
{
int i, j, p, channel = 8;
int valorNovo, valorAntigo;
uint32_t sum = 0;
float t, ti, t2;
float difTemp = 1;

inicializar();
compara();

k1 = getEEPROM(0);
k2 = getEEPROM(1);
b1 = getEEPROM(2);
b2 = getEEPROM(3);
fator1 = getEEPROM(4);
fator2 = getEEPROM(5);

k = ((k1 << 8) + k2)/100.0;
b = ((b1 << 8) + b2)/100.0;
fator = (fator1 << 8) + fator2;

idPlaca = (~PORTA) & 0x1F;
for (i = 0; i < 8; i++)
{
PORTB = SelectChannel(i);

for(p = 0; p < 10000; ++p);

valorAD[i] = ad();
PORTB = SelectChannel(i) + 1;
temp[i] = ((float)(valorAD[i]) / k) - b;
}

while(1){


if(modoLeitura == 8){
for (i = 0; i <= 7; ++i){


PORTB = SelectChannel(i);
for(p = 1; p < 6980; p++);
j = ad();

PORTB = SelectChannel(i) + 8;
for(p = 1; p < 6980; p++);

valorAD[i] = j - ad();

t = ((float)(valorAD[i])/ k) - b;



if((fabs(temp[i] - t) <= difTemp) || fator == 0){
temp[i] = ((temp[i] * (float)(fator)) + (t * (1000 - (float)(fator))))/1000.0;
erros[i] = 0;
}



else {
erros[i]++;
while(erros[i] < 4 && erros[i] != 0){

PORTB = SelectChannel(i);
for(p = 1; p < 6980; p++);
j = ad();

PORTB = SelectChannel(i) + 8;
for(p = 1; p < 6980; p++);
valorAD[i] = j - ad();

t = ((float)(valorAD[i])/ k) - b;

if(fabs(temp[i] - t) <= difTemp){
temp[i] = ((temp[i] * (float)(fator)) + (t * (1000 - (float)(fator))))/1000.0;
erros[i] = 0;
}
else{
erros[i]++;
if(erros[i] == 4){
temp[i] = t;
erros[i] = 0;
}
}
}
}
}
}


else{
if(channel != modoLeitura){
channel = modoLeitura;
PORTB = SelectChannel(modoLeitura);
for (p = 1; p < 10000; p++);
for (i = 0; i < 2; i++)
media2[i] = 0;
valorAD[modoLeitura] = 0;
}




if (lerADpuro == 1) {
while (1) {
PORTB = SelectChannel(modoLeitura);

for (p = 1; p < 6980; p++);
j = ad();

PORTB = SelectChannel(modoLeitura) + 8;
for (p = 1; p < 6980; p++);
valorAD[modoLeitura] = j - ad();
}
}

else{
sum = 0;
for(j=0; j<1; j++){
media2[j] = media2[j+1];
sum += media2[j];
}
media2[1] = ad();
sum = (sum + media2[1])/2;
valorAD[modoLeitura] = sum & 0xFFFF;

# 227
}

}
}
}

int SelectChannel(int ch) {
switch (ch) {
case 0:
case 1:
return ch + 6;
case 2:
case 3:
return ch - 2;
case 4:
case 5:
return ch;
case 6:
case 7:
return ch - 4;
}
return 0;
}

void inicializar (void){
ANSEL = 0;
ANSELH = 0;
TRISA = 0xFF;
TRISB = 0;
TRISC = 0xD5;
TRISD = 0;
TRISE = 0x07;

PORTA = 0xFF;
PORTB = 0;
PORTC = 0;
PORTD = 0;
PORTE = 0;
RD3 = 0;
PORTDbits.RD5 = 1;
PORTDbits.RD4 = 1;


SSPCONbits.SSPEN = 0;
SSPSTAT = 0xC0;
SSPCON = 0x20;

SPBRGH = 0;
SPBRG = 8;
BAUDCTL = 0x00;
RCSTA = 0x90;
TXSTA = 0x26;

INTCON = 0xC0;
TMR0IE = 0;
PEIE = 1;
GIE = 1;
PIE1 = 0x20;
RCIE = 1;
T0CS = 0;
PSA = 0;
PS0 = 1;
PS1 = 1;
PS2 = 1;

for(int i = 0; i < 10000; i++);

initLT();
}

void compara (void)
{
int comparar;
for(comparar = 0; comparar<5; comparar+=2)
{
if (((getEEPROM(comparar+100) << 8) + getEEPROM(comparar+101) == (int)(65535)) && ((getEEPROM(comparar+200) << 8) + getEEPROM(comparar+201) == (int)(65535)))
{
setEEPROM(comparar+100, getEEPROM(comparar));
setEEPROM(comparar+101, getEEPROM(comparar+1));
setEEPROM(comparar+200, getEEPROM(comparar));
setEEPROM(comparar+201, getEEPROM(comparar+1));
}
}
int comp;
for(comp=0; comp<5; comp+=2)
{
if (((getEEPROM(comp) << 8) + getEEPROM(comp+1)) == ((getEEPROM(comp+100) << 8) + getEEPROM(comp + 101)))
{
if (((getEEPROM(comp) << 8) + (getEEPROM(comp+1))) == ((getEEPROM(comp+200) << 8) + (getEEPROM(comp+201))))
{
continue;
}
else
{
setEEPROM((comp+200), getEEPROM(comp));
setEEPROM((comp+201), getEEPROM(comp+1));
}
continue;
}
else if (((getEEPROM(comp+100) << 8) + (getEEPROM(comp+101))) == ((getEEPROM(comp+200) << 8) + (getEEPROM(comp+201))))
{
setEEPROM(comp, getEEPROM(comp+100));
setEEPROM((comp+1), getEEPROM(comp+101));
continue;
}
else if (((getEEPROM(comp) << 8) + (getEEPROM(comp+1))) == ((getEEPROM(comp+200) << 8) + (getEEPROM(comp+201))))
{
setEEPROM(comp+100, getEEPROM(comp));
setEEPROM(comp+101, getEEPROM(comp+1));
continue;
}
}
}


enum bsmp_comando
{

CMD_QUERY_VERSION = 0x00,
CMD_VERSION,
CMD_VAR_QUERY_LIST,
CMD_VAR_LIST,
CMD_GROUP_QUERY_LIST,
CMD_GROUP_LIST,
CMD_GROUP_QUERY,
CMD_GROUP,
CMD_CURVE_QUERY_LIST,
CMD_CURVE_LIST,
CMD_CURVE_QUERY_CSUM,
CMD_CURVE_CSUM,
CMD_FUNC_QUERY_LIST,
CMD_FUNC_LIST,


CMD_VAR_READ = 0x10,
CMD_VAR_VALUE,
CMD_GROUP_READ,
CMD_GROUP_VALUES,


CMD_VAR_WRITE = 0x20,
CMD_GROUP_WRITE = 0x22,
CMD_VAR_BIN_OP = 0x24,
CMD_GROUP_BIN_OP = 0x26,
CMD_VAR_WRITE_READ = 0x28,


CMD_GROUP_CREATE = 0x30,
CMD_GROUP_REMOVE_ALL = 0x32,


CMD_CURVE_BLOCK_REQUEST = 0x40,
CMD_CURVE_BLOCK,
CMD_CURVE_RECALC_CSUM,


CMD_FUNC_EXECUTE = 0x50,
CMD_FUNC_RETURN,
CMD_FUNC_ERROR = 0x53,


CMD_OK = 0xE0,
CMD_ERR_MALFORMED_MESSAGE,
CMD_ERR_OP_NOT_SUPPORTED,
CMD_ERR_INVALID_ID,
CMD_ERR_INVALID_VALUE,
CMD_ERR_INVALID_PAYLOAD_SIZE,
CMD_ERR_READ_ONLY,
CMD_ERR_INSUFFICIENT_MEMORY,

CMD_MAX
};

void erro (enum bsmp_comando e)
{
serial[1] = e;
serial[2] = 0;
serial[3] = 0;
}

void ok (void)
{
erro(CMD_OK);
}

void __interrupt () interruption (void)
{
char uByte;
while (RCIF) {
uByte = RCREG;
serial[recebidos] = uByte;
cSum += uByte;
recebidos++;
TMR0 = 0x68;
TMR0IF = 0;
TMR0IE = 1;
}

if(TMR0IF){
RD3 = 1;
TMR0IE = 0;
if (!cSum)
{
if (serial[0] == idPlaca)
{
uByte = protocolo();
enviar(serial,uByte);
}
}
recebidos = 0;
cSum = 0;
RD3 = 0;
TMR0IF = 0;
}
PIR1 = 0;
}

void le_temp (char *p, int temp_index)
{
int valor;

if(lerADpuro)
valor = valorAD[temp_index];
else
valor = (int) (temp[temp_index]*100);

p[0] = valor >> 8;
p[1] = valor;
}

void le_alpha (char *p, int n)
{
if(n == 1)
{
p[0] = fator1;
p[1] = fator2;
return;
}
else{
p[0] = getEEPROM(4+(n-1)*100);
p[1] = getEEPROM(5+(n-1)*100);
}
}


void escreve_alpha (char *p)
{
int w_alpha;

fator1 = p[0];
fator2 = p[1];

int novo_fator = (fator1 << 8) + fator2;
if(novo_fator < 1000)
{
fator = novo_fator;
}

for (w_alpha=0; w_alpha<3; w_alpha+=1)
{
setEEPROM(4+(w_alpha)*100, fator1);
setEEPROM(5+(w_alpha)*100, fator2);
}
}

void le_k (char *p, int n)
{
p[0] = getEEPROM(0+(n-1)*100);
p[1] = getEEPROM(1+(n-1)*100);
}

void escreve_k (char *p)
{
int write_k;

k1 = p[0];
k2 = p[1];

for (write_k=0; write_k<3; write_k+=1){
setEEPROM(0+(write_k)*100, k1);
setEEPROM(1+(write_k)*100, k2);
}
k = ((float)((k1 << 8) + k2))/100;
}

void le_b (char *p, int n)
{
p[0] = getEEPROM(2+(n-1)*100);
p[1] = getEEPROM(3+(n-1)*100);
}

void escreve_b (char *p)
{
int write_b;
b1 = p[0];
b2 = p[1];

for (write_b=0; write_b<3; write_b+=1){
setEEPROM(2+(write_b)*100, b1);
setEEPROM(3+(write_b)*100, b2);
}
b = ((float)((b1 << 8) + b2))/100;
}

int protocolo (void)
{
uint8_t comando = serial[1];
uint16_t tamanho = (serial[2] << 8) | serial[3];

serial[0] = 0;
serial[2] = 0;

char *cmd = &serial[1];
char *payload = &serial[4];
char *tam = &serial[3];

*tam = 0;

switch (comando)
{

case CMD_QUERY_VERSION:
{
if(tamanho != 0)
erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
else
{
unsigned int i;
*cmd = CMD_VERSION;
*tam = 3;

*payload++ = 0x02;
*payload++ = 0x0A;
*payload++ = 0x00;
}

break;
}

case CMD_VAR_QUERY_LIST:
{
if(tamanho != 0)
erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
else
{
unsigned int i;
*cmd = CMD_VAR_LIST;
*tam = 13;

for(i = 0; i < 8; ++i)
*payload++ = 0x02;

for(i = 0; i < 3; ++i)
*payload++ = 0x82;

*payload++ = 0x81;
*payload++ = 0x81;
}

break;
}

case CMD_GROUP_QUERY_LIST:
if(tamanho != 0)
erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
else
{
*cmd = CMD_GROUP_LIST;
*tam = 3;
*payload++ = 0x0C;
*payload++ = 0x08;
*payload++ = 0x85;
}
break;

case CMD_GROUP_QUERY:
{
uint8_t id = payload[0];

if(tamanho != 1)
erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
else if(id > 2)
erro(CMD_ERR_INVALID_ID);
else
{
*cmd = CMD_GROUP;
unsigned int i = 0;

if(id == 0 || id == 1)
{
*tam += 8;
for(i = 0; i < 8; ++i)
*payload++ = i;
}

if(id == 0 || id == 2)
{
*tam += 4;
for(i = 8; i < 13; ++i)
*payload++ = i;
}
}
break;
}

case CMD_CURVE_QUERY_LIST:
*cmd = tamanho ? CMD_ERR_INVALID_PAYLOAD_SIZE : CMD_CURVE_LIST;
break;

case CMD_FUNC_QUERY_LIST:
*cmd = tamanho ? CMD_ERR_INVALID_PAYLOAD_SIZE : CMD_FUNC_LIST;
break;

case CMD_VAR_READ:
{
uint8_t id = payload[0];

if(tamanho != 1)
erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
else if(id > 18)
erro(CMD_ERR_INVALID_ID);
else
{
*cmd = CMD_VAR_VALUE;
*tam = 2;

if(id <= 7) le_temp(payload, id);
else if(id == 8 || id == 13 || id == 16) le_alpha(payload, (id-1)/5);
else if(id == 9 || id == 14 || id == 17) le_k(payload,(id-1)/5);
else if(id == 10|| id == 15 || id == 18) le_b(payload,(id-1)/5);
else
{
*tam = 1;
if(id == 11)
payload[0] = lerADpuro;
else
payload[0] = modoLeitura;
}
}
break;
}

case CMD_GROUP_READ:
{
uint8_t id = payload[0];

if(tamanho != 1)
erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
else if(id > 2)
erro(CMD_ERR_INVALID_ID);
else
{
*cmd = CMD_GROUP_VALUES;

unsigned int i;

if(id == 0 || id == 1)
for(i = 0; i < 8; ++i)
{
le_temp(payload, i);
payload += 2;
*tam += 2;
}

if(id == 0 || id == 2)
{
le_alpha(payload,1);
le_alpha(payload,2);
le_alpha(payload,3);
payload += 2;
*tam += 2;

le_k(payload,1);
le_k(payload,2);
le_k(payload,3);
payload += 2;
*tam += 2;

le_b(payload,1);
le_b(payload,2);
le_b(payload,3);
payload += 2;
*tam += 2;

*payload = lerADpuro;
payload++;
*tam++;

*payload = modoLeitura;
*tam++;
}
}
break;
}

case CMD_VAR_WRITE:
{
*cmd = CMD_OK;
uint8_t id = payload[0];

if(id == 11 && tamanho != 2)
erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
else if(id == 12 && tamanho != 2)
erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
else if(id != 11 && id != 12 && tamanho != 3)
erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
else if(id > 18)
erro(CMD_ERR_INVALID_ID);
else if(id < 8)
erro(CMD_ERR_READ_ONLY);
else
{
if(id == 8|| id == 13 || id == 16) escreve_alpha(&payload[1]);
else if(id == 9|| id == 14 || id == 17) escreve_k(&payload[1]);
else if(id == 10|| id == 15 || id == 18) escreve_b(&payload[1]);
else if(id == 11) lerADpuro = payload[1] & 0x03;
else modoLeitura = payload[1] & 0x0F;
}
break;
}
case CMD_GROUP_WRITE:
{
uint8_t id = payload[0];

if(tamanho < 1)
erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
else if(id > 2)
erro(CMD_ERR_INVALID_ID);
else if(id < 2)
erro(CMD_ERR_READ_ONLY);
else if(tamanho != 9)
erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
else
{
escreve_alpha(&payload[1]);
escreve_k(&payload[3]);
escreve_b(&payload[5]);
lerADpuro = payload[7];
modoLeitura = payload[8];
ok();
}
break;
}

default:
erro(CMD_ERR_OP_NOT_SUPPORTED);
break;
}

return 4+*tam;
}

void enviar(char *envia, int n)
{
int i;
char cs = 0;
for (i = 0; i < n; ++i){
cs += envia[i];
TXREG = envia[i];
while (!TRMT);
}
cs = 0x100 - cs;
TXREG = cs;
while (!TRMT);
}

uint16_t ad (void)
{
uint16_t _AD, _AD0, _AD1, aux;

PORTDbits.RD0 = 1;
asm("NOP");
asm("NOP");
asm("NOP");
asm("NOP");
PORTDbits.RD0 = 0;
SSPBUF = 0;
aux = SSPBUF;
while (!BF);
_AD0 = SSPBUF;
SSPBUF = 0;
aux = SSPBUF;
while (!BF);
_AD1 = SSPBUF;
_AD = (_AD0 << 8) + _AD1;
return _AD;
}

uint8_t getEEPROM (uint8_t c)
{
uint8_t aux;

RP0 = 0;
RP1 = 0;
EEADR = c;
EEPGD = 0;
RD = 1;
while(RD);
aux = EEDAT;
return aux;
}

void setEEPROM (uint8_t addr, uint8_t valor)
{
RP0 = 0;
RP1 = 0;
EEADR = addr;
EEDAT = valor;
EEPGD = 0;
WREN = 1;
INTCONbits.GIE = 0;
EECON2 = 0x55;
EECON2 = 0xAA;
WR = 1;

while(WR);

INTCONbits.GIE = 1;
WREN = 0;
}

void initLT (void)
{
escreveLT(0x00, 0x38);
escreveLT(0x01, 0x4B);
escreveLT(0x02, 0x03);
escreveLT(0x04, 0x07);
}

void escreveLT (uint8_t regAddr, uint8_t data)
{
uint8_t chipAddr = 0x8A;
int i, j;
PORTDbits.RD4 = 0;
for(j = 0; j < 10; j++);
PORTDbits.RD5 = 0;

for (i = 0; i < 8; i++){
PORTDbits.RD4 = (chipAddr >> (7 - i));
for(j = 0; j < 3; j++);
PORTDbits.RD5 = 1;
for(j = 0; j < 3; j++);
PORTDbits.RD5 = 0;
if(i == 7)
TRISDbits.TRISD4 = 1;
}

TRISDbits.TRISD4 = 0;
for(j = 0; j < 3; j++);
PORTDbits.RD5 = 1;
for(j = 0; j < 2; j++);
PORTDbits.RD5 = 0;
for(j = 0; j < 3; j++);

PORTDbits.RD4 = 1;
for(j = 0; j < 15; j++);

for (i = 0; i < 8; i++){
PORTDbits.RD4 = (regAddr >> (7 - i));
for(j = 0; j < 3; j++);
PORTDbits.RD5 = 1;
for(j = 0; j < 3; j++);
PORTDbits.RD5 = 0;
if(i == 7)
TRISDbits.TRISD4 = 1;
}

TRISDbits.TRISD4 = 0;
for(j = 0; j < 3; j++);
PORTDbits.RD5 = 1;
for(j = 0; j < 2; j++);
PORTDbits.RD5 = 0;
for(j = 0; j < 3; j++);

PORTDbits.RD4 = 1;
for(j = 0; j < 15; j++);

for (i = 0; i < 8; i++){
PORTDbits.RD4 = (data >> (7 - i));
for(j = 0; j < 3; j++);
PORTDbits.RD5 = 1;
for(j = 0; j < 3; j++);
PORTDbits.RD5 = 0;
if(i == 7)
TRISDbits.TRISD4 = 1;
}

TRISDbits.TRISD4 = 0;
for(j = 0; j < 3; j++);
PORTDbits.RD5 = 1;
for(j = 0; j < 2; j++);
PORTDbits.RD5 = 0;
for(j = 0; j < 5; j++);

PORTDbits.RD5 = 1;
for(j = 0; j < 10; j++);
PORTDbits.RD4 = 1;

for(j = 0; j < 25; j++);
}
