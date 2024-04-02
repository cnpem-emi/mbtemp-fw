/*
 * File:    main_sync.c
 * Authors:  Marcelo Bacchetti / Bruno Luvizotto / Bruno Martins / Patricia Nallin / Robert Polli
 *
 * Created on 9 de Setembro de 2013, 14:51
 *
 * Alterada em 24/01/2017 para incluir leituras repetitivas de um unico canal e media aritmetica
 * Alterado em julho/2017: Incluso tratamento de ruido em cabos longos com Pt100 (ruido em 60Hz).
 * Alterado em Setembro/2019: Incluso salvar os coeficientes de temperatura (Alpha, b e k ) em três posições diferentes de memória
 * Alterado em 2020: Limitacao de escrita no buffer de entrada da UART - travamento do uC
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "libbsmp/server.h"

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
//#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#define adsCNVST    PORTDbits.RD0
//#define DE          PORTDbits.RD3
#define DE          RD3
#define SDA         PORTDbits.RD4
#define SDA_dir     TRISDbits.TRISD4
#define SCL         PORTDbits.RD5
#define ltADDR      0x8A

#define BSMP_DEST   0   // Destino e' o primeiro byte da mensagem
#define BSMP_CMD    1   // Comando e' o segundo byte
#define BSMP_TAM1   2   // Tamanho (mais significativo) e' o terceiro byte
#define BSMP_TAM2   3   // Tamanho (menos sig.) e' o quarto byte
#define BSMP_PLOAD  4   // Payload comeca no quinto byte

// Modos Leitura
#define TODOS       8
#define CANAL0      0
#define CANAL1      1
#define CANAL2      2
#define CANAL3      3
#define CANAL4      4
#define CANAL5      5
#define CANAL6      6
#define CANAL7      7


void __interrupt ()  interruption(void);

void     main        (void);
void     inicializar (void);
void     compara     (void);
int      protocolo   (void);
uint16_t ad          (void);
void     enviar      (char *envia, int n);
uint8_t  getEEPROM   (uint8_t c); // c => coeficiente, 0 = angular, 1 = linear; valores de calibracao;
void     setEEPROM   (uint8_t addr, uint8_t valor);
void     initLT      (void);
void     escreveLT   (uint8_t regAddr, uint8_t data);
int      SelectChannel(int ch);

char    idPlaca;
char    cSum        = 0;
int     fator       = 985;
char    recebidos   = 0;
char    serial[40]  = {0};
int     valorAD[8];
int     media2[2] = {0};
float   temp[8];
int     erros[8] = {0};
uint8_t k1, k2, b1, b2, fator1, fator2;
float   k, b;
uint8_t lerADpuro = 0;
uint8_t modoLeitura = TODOS;    // Inicialmente, le todos os canais



void main()
{
    int    i, j, p, channel = 8;
    int valorNovo, valorAntigo;
    uint32_t sum = 0;
    float  t, ti, t2;
    float  difTemp = 1;

    inicializar();
    compara();

    k1     = getEEPROM(0);
    k2     = getEEPROM(1);
    b1     = getEEPROM(2);
    b2     = getEEPROM(3);
    fator1 = getEEPROM(4);
    fator2 = getEEPROM(5);

    k      = ((k1 << 8) + k2)/100.0;
    b      = ((b1 << 8) + b2)/100.0;
    fator  = (fator1 << 8) + fator2;

    idPlaca = (~PORTA) & 0x1F;
    for (i = 0; i < 8; i++)
    {
        PORTB      = SelectChannel(i);

        for(p = 0; p < 10000; ++p);

        valorAD[i] = ad();
        PORTB      = SelectChannel(i) + 1;
        temp[i]    = ((float)(valorAD[i]) / k) - b;
    }

    while(1){

        // ----- MODO MBTEMP NORMAL
        if(modoLeitura == TODOS){
            for (i = 0; i <= 7; ++i){


                PORTB      = SelectChannel(i);
                for(p = 1; p < 6980; p++);
                j = ad();

                PORTB      = SelectChannel(i) + 8;
                for(p = 1; p < 6980; p++);

                valorAD[i] = j - ad();

                t          = ((float)(valorAD[i])/ k) - b;   //(ADs[i] / k[i]) - b[i];

                // Se diferenca de temperatura for menor que definido, faz a media.
                // Se alpha = 0, calcula com o ultimo valor do AD
                if((fabs(temp[i] - t) <= difTemp) || fator == 0){
                    temp[i] = ((temp[i] * (float)(fator)) + (t * (1000 - (float)(fator))))/1000.0;
                    erros[i] = 0;
                }

                // Se nao, le mais 4x para confirmar valor.
                // Se diferente todas as vezes, adiciona o novo valor a media.
                else {
                    erros[i]++;
                    while(erros[i] < 4 && erros[i] != 0){

                        PORTB      = SelectChannel(i);
                        for(p = 1; p < 6980; p++);
                        j = ad();

                        PORTB      = SelectChannel(i) + 8;
                        for(p = 1; p < 6980; p++);
                        valorAD[i] = j - ad();

                        t = ((float)(valorAD[i])/ k) - b;   //(ADs[i] / k[i]) - b[i];

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

        // ----- MODO LEITURA APENAS UM CANAL
        else{
            if(channel != modoLeitura){   // Zera vetor de soma se requisitado
                channel = modoLeitura; // um novo canal
                PORTB      = SelectChannel(modoLeitura);
                for (p = 1; p < 10000; p++);
                for (i = 0; i < 2; i++)
                    media2[i] = 0;
                valorAD[modoLeitura] = 0;
            }



            //            for(i=0; i<20; i++){
            if (lerADpuro == 1) {
                while (1) {
                    PORTB      = SelectChannel(modoLeitura);

                    for (p = 1; p < 6980; p++);
                    j = ad();

                    PORTB      = SelectChannel(modoLeitura) + 8;
                    for (p = 1; p < 6980; p++);
                    valorAD[modoLeitura] = j - ad();
                }
            }
                // ----- Media das ultimas 2 leituras
                else{
                    sum = 0;
                    for(j=0; j<1; j++){ // Deslocamento do vetor
                        media2[j] = media2[j+1];
                        sum += media2[j];
                    }
                    media2[1] = ad();
                    sum = (sum + media2[1])/2;
                    valorAD[modoLeitura] = sum & 0xFFFF;

                    // ----- Maior valor -----
                    /*valorNovo = ad();
                    valorAntigo = valorAD[modoLeitura];
                    if(valorNovo > valorAntigo)
                        valorAD[modoLeitura] = valorNovo; */
                }
//            }
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
            return  ch - 4;
    }
    return 0;
}

void inicializar (void){
    ANSEL  = 0;
    ANSELH = 0;
    TRISA  = 0xFF;   //Define os pinos como entrada ou saida (0 -> saida; 1 -> entrada)
    TRISB  = 0;
    TRISC  = 0xD5;
    TRISD  = 0;
    TRISE  = 0x07;

    PORTA  = 0xFF;  //Define os estados dos pinos das portas
    PORTB  = 0;
    PORTC  = 0;
    PORTD  = 0;
    PORTE  = 0;
    DE     = 0;
    SCL    = 1;
    SDA    = 1;

    //Inicializar SPI:
    SSPCONbits.SSPEN = 0;
    SSPSTAT          = 0xC0;
    SSPCON           = 0x20;
    //Inicializar Uart: 19200bps (para 115200: SPBRG = 8 e TXSTA = 0x26)(para 19200: SPBRG = 12 e TXSTA = 0x22)
    SPBRGH           = 0;
    SPBRG            = 8;
    BAUDCTL          = 0x00;
    RCSTA            = 0x90;
    TXSTA            = 0x26;
    //Inicializar Interrupcoes
    INTCON           = 0xC0;
    TMR0IE           = 0;
    PEIE             = 1;
    GIE              = 1;
    PIE1             = 0x20;
    RCIE             = 1;
    T0CS             = 0;
    PSA              = 0;
    PS0              = 1;
    PS1              = 1;
    PS2              = 1;

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
    int     comp;
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
    // Query commands
    CMD_QUERY_VERSION       = 0x00,
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

    // Read commands
    CMD_VAR_READ            = 0x10,
    CMD_VAR_VALUE,
    CMD_GROUP_READ,
    CMD_GROUP_VALUES,

    // Write commands
    CMD_VAR_WRITE           = 0x20,
    CMD_GROUP_WRITE         = 0x22,
    CMD_VAR_BIN_OP          = 0x24,
    CMD_GROUP_BIN_OP        = 0x26,
    CMD_VAR_WRITE_READ      = 0x28,

    // Group manipulation commands
    CMD_GROUP_CREATE        = 0x30,
    CMD_GROUP_REMOVE_ALL    = 0x32,

    // Curve commands
    CMD_CURVE_BLOCK_REQUEST = 0x40,
    CMD_CURVE_BLOCK,
    CMD_CURVE_RECALC_CSUM,

    // Function commands
    CMD_FUNC_EXECUTE        = 0x50,
    CMD_FUNC_RETURN,
    CMD_FUNC_ERROR          = 0x53,

    // Error codes
    CMD_OK                  = 0xE0,
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
    serial[BSMP_CMD] = e;
    serial[BSMP_TAM1] = 0;
    serial[BSMP_TAM2] = 0;
}

void ok (void)
{
    erro(CMD_OK);
}

void __interrupt ()  interruption (void)
{
    char uByte;
    while (RCIF) {
        uByte   = RCREG;
	if(recebidos < 40)
+        {
+            serial[recebidos] = uByte;
+            cSum    += uByte;
+            recebidos++;
+        }
        TMR0    = 0x68;
        TMR0IF  = 0;	//T0IF  = 0;
        TMR0IE  = 1;	//T0IE  = 1;
    }

    if(TMR0IF){  //if(T0IF){	
        DE = 1;
        TMR0IE = 0;  //T0IE = 0;
        if (!cSum)
        {
            if (serial[BSMP_DEST] == idPlaca)
            {
                uByte = protocolo();
                enviar(serial,uByte);
            }
        }
        recebidos = 0;
        cSum      = 0;
        DE        = 0;
        TMR0IF    = 0;  //T0IF    = 0;
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
    k  = ((float)((k1 << 8) + k2))/100;
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
    uint8_t  comando  =  serial[BSMP_CMD];
    uint16_t tamanho  = (serial[BSMP_TAM1] << 8) | serial[BSMP_TAM2];

    serial[BSMP_DEST] = 0;
    serial[BSMP_TAM1] = 0;  // Todos os comandos voltam tamanhos <= 255

    char *cmd     = &serial[BSMP_CMD];
    char *payload = &serial[BSMP_PLOAD];
    char *tam     = &serial[BSMP_TAM2];

    *tam = 0;

    switch (comando)
    {

    case CMD_QUERY_VERSION:            // Consulta versao do protocolo
    {
        if(tamanho != 0)
            erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
        else
        {
            unsigned int i;
            *cmd = CMD_VERSION;        // Lista de variaveis
            *tam = 3;                  // 12 variaveis

				*payload++ = 0x02; // Vers�o do protocolo: 2.10.000
				*payload++ = 0x0A;
				*payload++ = 0x00;
        }

        break;
    }

    case CMD_VAR_QUERY_LIST:            // Consulta lista de variaveis
    {
        if(tamanho != 0)
            erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
        else
        {
            unsigned int i;
            *cmd = CMD_VAR_LIST;        // Lista de variaveis
            *tam = 13;                  // 12 variaveis

            for(i = 0; i < 8; ++i)      // 8 leituras de 2 bytes (temperaturas)
                *payload++ = 0x02;

            for(i = 0; i < 3; ++i)      // 3 escritas de 2 bytes (alpha, k, b)
                *payload++ = 0x82;

            *payload++ = 0x81;          // 1 escrita de 1 byte (lerADpuro)
            *payload++ = 0x81;          // 1 escrita de 1 byte (modoLeitura)
        }

        break;
    }

    case CMD_GROUP_QUERY_LIST:          // Consulta lista de grupos
        if(tamanho != 0)
            erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
        else
        {
            *cmd = CMD_GROUP_LIST;      // Lista de grupos
            *tam = 3;
            *payload++ = 0x0C;     // Um grupo de leitura com 12 variaveis (todas)
            *payload++ = 0x08;     // Um grupo de leitura com 8 variaveis (temperaturas)
            *payload++ = 0x85;     // Um grupo de escrita com 4 variaveis (alpha, k, b, lerADpuro, modoLeitura)
        }
        break;

    case CMD_GROUP_QUERY:               // Consulta grupo
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

            if(id == 0 || id == 1)   // Grupo 1: temperaturas, id's 0 a 7
            {
                *tam += 8;
                for(i = 0; i < 8; ++i)
                    *payload++ = i;
            }

            if(id == 0 || id == 2)   // Grupo 2: alpha, k, b, lerADpuro e modoLeitura, id's 8 a 12
            {
                *tam += 4;
                for(i = 8; i < 13; ++i)
                    *payload++ = i;
            }
        }
        break;
    }

    case CMD_CURVE_QUERY_LIST:          // Consultar lista de curvas
        *cmd = tamanho ? CMD_ERR_INVALID_PAYLOAD_SIZE : CMD_CURVE_LIST;
        break;

    case CMD_FUNC_QUERY_LIST:            // Consultar lista de funcoes
        *cmd = tamanho ? CMD_ERR_INVALID_PAYLOAD_SIZE : CMD_FUNC_LIST;
        break;

    case CMD_VAR_READ:                  // Ler variavel
    {
        uint8_t id = payload[0];

        if(tamanho != 1)
            erro(CMD_ERR_INVALID_PAYLOAD_SIZE);
        else if(id > 18)
            erro(CMD_ERR_INVALID_ID);
        else
        {
            *cmd = CMD_VAR_VALUE;
            *tam = 2;                   // 2 bytes de valor

            if(id <= 7)                                le_temp(payload, id);
            else if(id == 8 || id == 13 || id == 16)   le_alpha(payload, (id-1)/5);
            else if(id == 9 || id == 14 || id == 17)   le_k(payload,(id-1)/5);
            else if(id == 10|| id == 15 || id == 18)   le_b(payload,(id-1)/5);
            else
            {
                *tam = 1;               // 1 byte de valor
                if(id == 11)
                    payload[0] = lerADpuro;
                else
                    payload[0] = modoLeitura;
            }
        }
        break;
    }

    case CMD_GROUP_READ:                // Ler grupo de variaveis
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
                    *tam    += 2;
                }

            if(id == 0 || id == 2)
            {
                le_alpha(payload,1);
                le_alpha(payload,2);
                le_alpha(payload,3);
                payload += 2;
                *tam    += 2;

                le_k(payload,1);
                le_k(payload,2);
                le_k(payload,3);
                payload += 2;
                *tam    += 2;

                le_b(payload,1);
                le_b(payload,2);
                le_b(payload,3);
                payload += 2;
                *tam    += 2;

                *payload = lerADpuro;
                payload++;
                *tam++;

                *payload = modoLeitura;
                *tam++;
            }
        }
        break;
    }

    case CMD_VAR_WRITE:              // Escrever uma variavel
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
            if(id == 8|| id == 13 || id == 16)          escreve_alpha(&payload[1]);
            else if(id == 9|| id == 14  || id == 17)    escreve_k(&payload[1]);
            else if(id == 10|| id == 15 || id == 18)    escreve_b(&payload[1]);
            else if(id == 11)   lerADpuro = payload[1] & 0x03;
            else                modoLeitura = payload[1] & 0x0F;
        }
        break;
    }
    case CMD_GROUP_WRITE:              // Escrever um grupo de variaveis
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

    adsCNVST    = 1;
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    adsCNVST    = 0;
    SSPBUF      = 0;
    
    while (!BF);
    aux         = SSPBUF;
    _AD0        = SSPBUF;
    SSPBUF      = 0;
	
    while (!BF);
    aux         = SSPBUF;
    _AD1        = SSPBUF;
    _AD         = (_AD0 << 8) + _AD1;
    return _AD;
}

uint8_t getEEPROM (uint8_t c)
{
    uint8_t aux;

    RP0     = 0;
    RP1     = 0;
    EEADR   = c;
    EEPGD   = 0;
    RD      = 1;
    while(RD);
    aux     = EEDAT;
    return aux;
}

void setEEPROM (uint8_t addr, uint8_t valor)
{
    RP0     = 0;
    RP1     = 0;
    EEADR   = addr;
    EEDAT   = valor;
    EEPGD   = 0;
    WREN    = 1;
    INTCONbits.GIE = 0;
    EECON2  = 0x55;
    EECON2  = 0xAA;
    WR      = 1;

    while(WR);

    INTCONbits.GIE = 1;
    WREN    = 0;
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
    uint8_t chipAddr = ltADDR;
    int i, j;
    SDA = 0;
    for(j = 0; j < 10; j++);
    SCL = 0;

    for (i = 0; i < 8; i++){
        SDA = (chipAddr >> (7 - i));
        for(j = 0; j < 3; j++);
        SCL = 1;
        for(j = 0; j < 3; j++);
        SCL = 0;
        if(i == 7)
            SDA_dir = 1;
    }

    SDA_dir = 0;
    for(j = 0; j < 3; j++);
    SCL = 1;
    for(j = 0; j < 2; j++);
    SCL = 0;
    for(j = 0; j < 3; j++);
    //SDA_dir = 0;
    SDA = 1;
    for(j = 0; j < 15; j++);

    for (i = 0; i < 8; i++){
        SDA = (regAddr >> (7 - i));
        for(j = 0; j < 3; j++);
        SCL = 1;
        for(j = 0; j < 3; j++);
        SCL = 0;
        if(i == 7)
            SDA_dir = 1;
    }

    SDA_dir = 0;
    for(j = 0; j < 3; j++);
    SCL = 1;
    for(j = 0; j < 2; j++);
    SCL = 0;
    for(j = 0; j < 3; j++);

    SDA = 1;
    for(j = 0; j < 15; j++);

    for (i = 0; i < 8; i++){
        SDA = (data >> (7 - i));
        for(j = 0; j < 3; j++);
        SCL = 1;
        for(j = 0; j < 3; j++);
        SCL = 0;
        if(i == 7)
            SDA_dir = 1;
    }

    SDA_dir = 0;
    for(j = 0; j < 3; j++);
    SCL = 1;
    for(j = 0; j < 2; j++);
    SCL = 0;
    for(j = 0; j < 5; j++);

    SCL = 1;
    for(j = 0; j < 10; j++);
    SDA = 1;

    for(j = 0; j < 25; j++);
}
