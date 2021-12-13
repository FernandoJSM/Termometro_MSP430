// LCD do Nokia 5110 + Sensor de temperatura
//
// Fernando José de Souza Magalhães
// Junho de 2020
//
//       MSP430G2553
//            _______________
//       VCC-|1            20|-GND
//  LCD_RST<-|2-P1.0         |
//           |3-P1.1         |
//  LCD_DIN<-|4-P1.2         |
//  PUSHBUT->|5-P1.3   RST 16|-(47Kohm Pullup + 10nF Pulldown)
//  LCD_CLK<-|6-P1.4  P1.7-15|-> LCD_CE
//           |7-P1.5  P1.6-14|-> LCD_DC
//           |8-P2.0  P2.5-13|
//           |9-P2.1  P2.4-12|
//           |10-P2.2 P2.3-11|
//           |_______________|
//
// LCD 5110      MSP430
//   ____          _______
//   RST |------->|2-P1.0  (Saída)
//   CE  |------->|15-P1.7 (Saída)
//   DC  |------->|14-P1.6 (Saída)
//   DIN |------->|4-P1.2  (UCA0SIMO)
//   CLK |------->|6-P1.4  (UCA0CLK)
//   VCC |-> VCC  |
//   LIG |-> GND  |
//   GND |-> GND  |
//   ____|        |___

#include <msp430.h>

//Pré inicialização de funções
void enviaDado(unsigned char DC, unsigned char dado);
void iniciaLCD(void);
void posXY(unsigned char X, unsigned char Y);
void limpaTela(void);
void enviaMuitos(unsigned char n, unsigned char dados[]);
void enviaSeg(unsigned char n,unsigned char segMat[][12]);
void escreveNum(unsigned char s,unsigned char d,unsigned char u);
void atualizaTemp(long temp);
void enviaMerc(unsigned char n,unsigned char y);
void atualizaMerc(long temp);
//__interrupt void Port_1(void);

// Valores de calibração do sensor de temperatura
#define CAL_ADC25T30 *((unsigned int *)0x10E8) //O valor do endereço está no datasheet
#define CAL_ADC25T85 *((unsigned int *)0x10EA)
// Valores dos bits dos pinos do LCD para facilitar alterações
#define LCD_CE 0x80  // Chip Enable do LCD
#define LCD_RST 0x01 // Reset do LCD
#define LCD_DC 0x40  // Data/Command, seleção de tipo de entrada do LCD
#define LCD_DIN 0x04 // Data in , entrada de dados do LCD
#define LCD_CLK 0x10 // Entrada de clock do LCD
// Valores de saída/entrada
#define PUSHBT 0x08  //Botão S2 do módulo

// Variáveis globais
long adcval = 0,tempC = 0; //A princípio tempC não precisa ser float.
char tempo = 0;         //Tempo pra atualizar o termômetro e mostrar a tela
static const char txt_temp[65]={0x02,0x02,0xFE,0x02,0x02,0x00,0x70,0xA8,0xA8,0xA8,0x30,0x00,0xF8,0x08,0x30,
                                0x08,0xF0,0x00,0xF8,0x28,0x28,0x28,0x10,0x00,0x70,0xA8,0xA8,0xA8,0xB0,0x00,
                                0xF8,0x10,0x08,0x08,0x10,0x00,0x40,0xA8,0xA8,0xA8,0xF0,0x00,0x08,0x7E,0x88,
                                0x80,0x40,0x00,0x78,0x80,0x80,0x40,0xF8,0x00,0xF8,0x10,0x08,0x08,0x10,0x00,
                                0x40,0xA8,0xA8,0xA8,0xF0}; // Texto "Temperatura"
static const char segtop[5][12]={{0xFC,0xFA,0xF6,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0xF6,0xFA,0xFC}, // 0-0,8,9,º
                                {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xF8,0xFC},  // 1-1
                                {0x00,0x02,0x06,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0xF6,0xFA,0xFC},  // 2-2,3,7
                                {0xFC,0xF8,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xF8,0xFC},  // 3-4
                                {0xFC,0xFA,0xF6,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x06,0x02,0x00}}; // 4-5,6,C - Segmentos do topo
static const char segmid[11][12]={{0xEF,0xC7,0x83,0x00,0x00,0x00,0x00,0x00,0x00,0x83,0xC7,0xEF}, // 0-0
                                  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x83,0xC7,0xEF}, // 1-1,7
                                  {0xE0,0xD0,0xB8,0x38,0x38,0x38,0x38,0x38,0x38,0x3B,0x17,0x0F}, // 2-2
                                  {0x00,0x10,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0xBB,0xD7,0xEF}, // 3-3
                                  {0x0F,0x17,0x3B,0x38,0x38,0x38,0x38,0x38,0x38,0xBB,0xD7,0xEF}, // 4-4,9
                                  {0x0F,0x17,0x3B,0x38,0x38,0x38,0x38,0x38,0x38,0xB8,0xD0,0xE0}, // 5-5
                                  {0xEF,0xD7,0xBB,0x38,0x38,0x38,0x38,0x38,0x38,0xB8,0xD0,0xE0}, // 6-6
                                  {0xEF,0xD7,0xBB,0x38,0x38,0x38,0x38,0x38,0x38,0xBB,0xD7,0xEF}, // 7-8
                                  {0x0F,0x17,0x3B,0x38,0x38,0x38,0x38,0x38,0x38,0x3B,0x17,0x0F}, // 8-º
                                  {0xEF,0xC7,0x83,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 9-C
                                  {0x00,0x10,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x10,0x00}};// 10- - - Segmentos do meio
static const char segbot[4][12] ={{0x7F,0xBF,0xDF,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xDF,0xBF,0x7F}, // 0-0,6,8
                                 {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x3F,0x7F}, // 1-1,4,7
                                 {0x7F,0xBF,0xDF,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xC0,0x80,0x00}, // 2-2,C
                                 {0x00,0x80,0xC0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xDF,0xBF,0x7F}};// 3-3,5,9 - Segmentos de baixo
static const char termtop[5]={0x80,0x40,0x40,0x40,0x80}; // Topo do termometro
static const char termmid[7]={0xFF,0x00,0x00,0x00,0x00,0x00,0xFF}; // Parede vazia do termometro
static const char termbot[9]={0x0E,0x11,0x2E,0X5F,0x5F,0x5F,0x2E,0x11,0x0E}; //Base do termometro
static const char termmerc[9][3]={{0x55,0x00,0x00},
                                  {0x55,0x80,0x80},
                                  {0x55,0xC0,0xC0},
                                  {0x55,0xE0,0xE0},
                                  {0x55,0xF0,0xF0},
                                  {0x55,0xF8,0xF8},
                                  {0x55,0xFC,0xFC},
                                  {0x55,0xFE,0xFE},
                                  {0x55,0xFF,0xFF}}; //Mercúrio do termômetro

int main(void) {
    WDTCTL = WDTPW+WDTHOLD; //desabilita o watchdog
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;   //Calibração de fábrica do DCO para 1MHz
    BCSCTL2 = 0x00;         //MCLK=DCO;SMCLK=DCO
    //------------------------- INICIALIZAÇÃO DAS PORTAS
    P1DIR = LCD_CE + LCD_RST + LCD_DC; //Define entradas, o restante é saída
    P1REN = PUSHBT; // Habilita resistor de pull up/down no botão
    P1SEL = LCD_DIN + LCD_CLK; //Habilito a porta de clock e SIMO (Slave in Master OUT) da interface SPI UCA0
    P1SEL2 = LCD_DIN + LCD_CLK;
    P1OUT = LCD_CE + PUSHBT; // Reseta o LCD / Desabilita chip enable do LCD / Define o resistor do botão como pullup
    P1IE = PUSHBT; //Ativa interrupção
    P1IFG = 0x00;  //Zera a interrupção
    //------------------------- CONFIGURAÇÃO DA COMUNICAÇÃO SPI COM O LCD
    UCA0CTL1 = UCSWRST; // Desabilita o módulo SPI para configurar
    UCA0CTL0 |= UCCKPH + UCMSB + UCMST + UCSYNC; // UCCKPH Pega dado no meio do clock
                                               // UCMSB Bit mais significativo primeiro
                                               // UCMST MSP430 será o mestre
                                               // UCSYNC Modo síncrono com clock
    UCA0CTL1 |= UCSSEL_2; // Seleciona o SMCLK como clock
    UCA0BR0 = 0x01;       // Divisor do clock fonte para a geração de bits = 1
    UCA0BR1 = 0;
    UCA0MCTL = 0;         //Tipo de modulação = Sem modulação
    UCA0CTL1 &= ~UCSWRST; // Inicializa o módulo SPI
    //------------------------- CONFIGURAÇÃO DO ADC
    ADC10CTL0 = SREF_1 + ADC10SHT_2 + REF2_5V + REFON + ADC10ON; //Referência interna, ClkS&H=8xClk, Referência de 2.5V, Adc ligado
    ADC10CTL1 = INCH_10 + ADC10DIV_4 + ADC10SSEL_3; //Sensor de temperatura interno, Divisor de clock x5, SM clock
    // Tempo do ADC = (1/(1MHz/5))*8 = 40us
    //-------------------------
    __delay_cycles(2000); //Tempo do Reset do LCD (Pelo menos 100ns)
    P1OUT |= LCD_RST;    //Desabilita Reset do LCD
    iniciaLCD();         //Inicializa LCD
    // Escreve 'Temperatura' e 'ºC'
    //'Temperatura'
    posXY(10,0);
    enviaMuitos(65,txt_temp);
    //'º'
    posXY(41,2);
    enviaSeg(0,segtop);
    posXY(41,3);
    enviaSeg(8,segmid);
    //'C'
    posXY(54,2);
    enviaSeg(4,segtop);
    posXY(54,3);
    enviaSeg(9,segmid);
    posXY(54,4);
    enviaSeg(2,segbot);
    // Desenha o termometro
    posXY(72,1);
    enviaMuitos(5,termtop);
    posXY(71,2);
    enviaMuitos(7,termmid);
    posXY(71,3);
    enviaMuitos(7,termmid);
    posXY(71,4);
    enviaMuitos(7,termmid);
    posXY(70,5);
    enviaMuitos(9,termbot);
    _BIS_SR(GIE); //Modo de baixo consumo e interrupcoes habilitadas
    for(;;)
    {
        //-------- Rotina do ADC
        __delay_cycles(100000); //Atraso antes de usar o ADC para estabilizar a Vref
        ADC10CTL0 |= ENC+ADC10SC; //ENC=Enable conversion; ADC10SC=StartConversion
        while (ADC10CTL1 & ADC10BUSY);
        //__bis_SR_register(CPUOFF+GIE); //Entra no modo de baixo consumo mas mantem as interrupções ativas
        adcval = ADC10MEM; //Captura o valor do ADC
        tempC = (adcval - CAL_ADC25T30) * 55/(CAL_ADC25T85 - CAL_ADC25T30) + 30;
        ADC10CTL0 &= ~ENC;
        //--------
        if (tempo == 0)
        {
            atualizaTemp(tempC);
            atualizaMerc(tempC);
            tempo = 255;
            enviaDado(0,0x08); // Apaga a tela
        }
        tempo--;
    }
}

void enviaDado(unsigned char DC, unsigned char dado){
    P1OUT &= ~LCD_CE; //Habilita LCD
    if(DC==1){
        P1OUT |= LCD_DC; //Se DC=1; Dado
    }
    else{
        P1OUT &= ~LCD_DC; //Se DC=0; Comando
    }
    //while (!(IFG2 & UCA0TXIFG)); // Buffer do módulo SPI está livre?
    UCA0TXBUF = dado; // Coloca o dado no buffer
    while (!(IFG2 & UCA0TXIFG)); // Buffer do módulo SPI está livre?
    P1OUT |= LCD_CE;  //Desabilita LCD
}

void iniciaLCD(void){
    enviaDado(0,0x21); //Define função: V=0 (Entrada horizontal); PD=0 (Liga LCD); H=1 (Modo de entrada extendido)
    enviaDado(0,0xC0); //Define Vop (tensão de operação/contraste) = ~5V
    enviaDado(0,0x06); //Defina a curva de temperatura para o padrão (típico)
    enviaDado(0,0x13); //Define a tensão de bias do multiplexador para 1:48 (padrão)
    enviaDado(0,0x20); //Define função: V=0 (Entrada horizontal); PD=0 (Liga LCD); H=0 (Modo de entrada básico)
    enviaDado(0,0x0C);  //Configura o LCD para o modo de exibição normal D=1 E=0
    limpaTela();
}

void posXY(unsigned char X, unsigned char Y){
    //Define X (0 a 83)
    enviaDado(0,0x80+X);
    //Define Y (0 a 6)
    enviaDado(0,0x40+Y);
}

void limpaTela(void){
    unsigned int i;
    posXY(0,0);
    for(i=0;i<504;i++){
        enviaDado(1,0x00);
    }
    posXY(0,0);
}

void enviaMuitos(unsigned char n, unsigned char dados[]){
    unsigned int i;
    for(i=0;i<n;i++){
        enviaDado(1,dados[i]);
    }
}

void enviaSeg(unsigned char n,unsigned char segMat[][12]){
    unsigned char i;
    for(i=0;i<12;i++){
        enviaDado(1,segMat[n][i]);
    }
}

void escreveNum(unsigned char s,unsigned char d,unsigned char u){
    unsigned char i,v,px,t,m,b;
    posXY(4,3);
    if (s==1){
        for(i=0;i<12;i++){
            enviaDado(1,0x00);
        }
    }
    else{
        enviaSeg(10,segmid);
    }
    for(i=0;i<2;i++){
        if(i==0){
            v = d;
            px = 15;
        }
        else{
            v = u;
            px = 28;
        }
        switch (v)
        {
           case 0:
               t = 0;
               m = 0;
               b = 0;
           break;
           case 1:
               t = 1;
               m = 1;
               b = 1;
           break;
           case 2:
               t = 2;
               m = 2;
               b = 2;
           break;
           case 3:
               t = 2;
               m = 3;
               b = 3;
           break;
           case 4:
               t = 3;
               m = 4;
               b = 1;
           break;
           case 5:
               t = 4;
               m = 5;
               b = 3;
           break;
           case 6:
               t = 4;
               m = 6;
               b = 0;
           break;
           case 7:
               t = 2;
               m = 1;
               b = 1;
           break;
           case 8:
               t = 0;
               m = 7;
               b = 0;
           break;
           case 9:
               t = 0;
               m = 4;
               b = 3;
           break;
           default: //Faz nada
               break;
        }
        posXY(px,2);
        enviaSeg(t,segtop);
        posXY(px,3);
        enviaSeg(m,segmid);
        posXY(px,4);
        enviaSeg(b,segbot);
    }
}

void atualizaTemp(long temp){
    unsigned char dez,uni;
    if(temp>0){
        if (temp<10){
            dez = 0;
            uni = temp;
        }
        else{
            dez = temp/10;
            uni = temp - dez*10;
        }
        escreveNum(1,dez,uni);
    }
    else{
        temp = - temp;
        if (temp<10){
            dez = 0;
            uni = temp;
        }
        else{
            dez = temp/10;
            uni = temp - dez*10;
        }
        escreveNum(0,dez,uni);
    }
}

void enviaMerc(unsigned char n,unsigned char y){
    unsigned char i;
    posXY(73,4-y);
    for(i=0;i<3;i++){
        enviaDado(1,termmerc[n][i]);
    }
}

void atualizaMerc(long temp){
    long var = 0;
    enviaMerc(0,0);
    enviaMerc(0,1);
    enviaMerc(0,2);
    if (temp>36){ //Se for maior que a escala, enche tudo
        enviaMerc(8,0);
        enviaMerc(8,1);
        enviaMerc(8,2);
    }
    else if(temp>22){ //Enche os 2 primeiros
        enviaMerc(8,0);
        enviaMerc(8,1);
        var = (temp - 22)/2;
        enviaMerc(var,2);
    }
    else if(temp>6){ //Enche o primeiro
        enviaMerc(8,0);
        var = (temp - 6)/2;
        enviaMerc(var,1);
    }
    else if(temp>-9){ //Preenche o primeiro
        var = (temp + 9)/2 + 1;
        enviaMerc(var,0);
    }
}

#pragma vector=PORT1_VECTOR //Rotina de interrupção na porta 1
__interrupt void Port_1(void){
    tempo = 0;
    enviaDado(0,0x0C); // Liga a tela
    while((P1IN & PUSHBT)==0){
    }
    P1IFG = 0x00; //zera a interrupção
}
