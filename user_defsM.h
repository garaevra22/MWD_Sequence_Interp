#include "stdint.h"
#include "MDR32F9Qx_ssp.h"

#define OneMCU 0   //1 - имитация на единственном MCU (Master/Slave), 0 - два MCU с разными ролями
#define ADM485 	   //Есть ИМС ADM485
#define Use_CP2102 	//использовать ИМС cp2102 для отладочного обмена
#define SysFreq 80  //system frequency in MHz
#define BaudRate 9600  //снижение до 9600 с 20.03.23   19200

#define minVib_ 10 //  //0 23.08.21 временно   //минимальная ампл. вибрации
#define MinFrq 5  //5    //мин частота пика в наборе гармоник вибрации
#define Chks 4   ///пока число проверок тишины в Stat3, каждая по 2,5 с

//#define DMAuart   //использовать DMA
//17.07.2020 #define IrqDmaADC  //использовать DMAIrq for ADC
//-----------------------------------------------------
/// Usefull macros
#define Toggle_LED7() {if (PORT_ReadInputData(MDR_PORTB) & (1<<7)) PORT_ResetBits(MDR_PORTB, PORT_Pin_7); else  PORT_SetBits(MDR_PORTB, PORT_Pin_7);}
#define Toggle_LED8() {if (PORT_ReadInputData(MDR_PORTB) & (1<<8)) PORT_ResetBits(MDR_PORTB, PORT_Pin_8); else  PORT_SetBits(MDR_PORTB, PORT_Pin_8);}
#define Toggle_LED9() {if (PORT_ReadInputData(MDR_PORTB) & (1<<9)) PORT_ResetBits(MDR_PORTB, PORT_Pin_9); else  PORT_SetBits(MDR_PORTB, PORT_Pin_9);}
#define Toggle_LED10() {if (PORT_ReadInputData(MDR_PORTB) & (1<<10)) PORT_ResetBits(MDR_PORTB, PORT_Pin_10); else  PORT_SetBits(MDR_PORTB, PORT_Pin_10);}
///---------------------------------

#define PinBat2Sw PORT_Pin_6
#define Mdltr_PORT MDR_PORTB

///#define Pilot1986 металлокерамический корпус с пробитым PB7! Заменен на PB9

#if defined Pilot1986

#define PinOpnCls PORT_Pin_9
/* Frequencies setup using HSI */
///#define trimCoeff 29  ///for Pilot
#define trimCoeff 25  ///for Pilot для нового МК (телесистема)

#else
#define PinOpnCls PORT_Pin_7
#define trimCoeff 25 ///for Demo board или пластмассовый корпус  34
#endif
///---------------------------------
#define PwrOff() { PORT_ResetBits(MDR_PORTA, PinBat2Sw | PORT_Pin_7 | PORT_Pin_5); \
      PORT_ResetBits(MDR_PORTB, PORT_Pin_2 | PORT_Pin_3 | PORT_Pin_5);} ///Откл.батарей и ADM3072
#define PwrOn2() PORT_SetBits(MDR_PORTA, PinBat2Sw); ///вкл. бат.2
#define PwrOn1() PORT_SetBits(MDR_PORTA, PORT_Pin_7); ///вкл. бат.1

//с 5.09.22
#define MDR_RR52_CS_SetActive() {PORT_ResetBits(U_1636RR52_SS_PORT, U_1636RR52_SS_PIN); \
            MDR_Delay(_RR52_Delay.CS_20ns);}

#define  MDR_RR52_CS_SetInactive() {PORT_SetBits(U_1636RR52_SS_PORT, U_1636RR52_SS_PIN); \
          MDR_Delay(_RR52_Delay.CS_20ns);}

//#define F1UART2() {Port1D2Mdltr.PORT_OE = PORT_OE_IN; PORT_Init(MDR_PORTD, &Port1D2Mdltr);Port1F2cnsl.PORT_FUNC  = PORT_FUNC_OVERRID; PORT_Init(MDR_PORTF, &Port1F2cnsl);} //F1 - UART2
//#define D1Uart2() {Port1F2cnsl.PORT_OE = PORT_OE_IN; PORT_Init(MDR_PORTF, &Port1F2cnsl); Port1D2Mdltr.PORT_OE = PORT_OE_OUT; PORT_Init(MDR_PORTD, &Port1D2Mdltr); } //D1 - UART2
#define D1Uart2() {Port1F2cnsl.PORT_FUNC  = PORT_FUNC_MAIN; PORT_Init(MDR_PORTF, &Port1F2cnsl); Port1D2Mdltr.PORT_FUNC = PORT_FUNC_ALTER; PORT_Init(MDR_PORTD, &Port1D2Mdltr); } //D1 - UART2
#define F1UART2() {Port1D2Mdltr.PORT_FUNC  = PORT_FUNC_MAIN; PORT_Init(MDR_PORTD, &Port1D2Mdltr);Port1F2cnsl.PORT_FUNC  = PORT_FUNC_OVERRID; PORT_Init(MDR_PORTF, &Port1F2cnsl);} //F1 - UART2

#define SendUp(MsgIndx) {Itm2Q.F.msg = SndU; Itm2Q.F.strt = MsgIndx; xQueueSendToBack(xQueueMang,(long*) &Itm2Q, 5); }

///uDelay functions
#define strtDlay(uSecs) TIMER_Cmd(MDR_TIMER3, DISABLE); MDR_TIMER3->STATUS=0; TIMER_SetCounter(MDR_TIMER3, uSecs); TIMER_Cmd(MDR_TIMER3,ENABLE);
  /* the counter is started. uSecs=uDlay в мкс  */
#define wtDlayFin() while (TIMER_GetFlagStatus(MDR_TIMER3, TIMER_STATUS_CNT_ZERO) == RESET) {;} TIMER_Cmd(MDR_TIMER3, DISABLE);
 /* wait till CNT = 0 */
#define Tim3Hlt() TIMER_Cmd(MDR_TIMER3, DISABLE)
 /* Halt TIMER3 */
#define ifDlNfin() if (TIMER_GetFlagStatus(MDR_TIMER3, TIMER_STATUS_CNT_ZERO)== RESET) ///ifDlNfin() надо ... else Tim3Hlt(); и т.п.
 /* check if CNT != 0 */
#define strtTout(uSecs) TIMER_Cmd(MDR_TIMER2, DISABLE); MDR_TIMER2->STATUS=0; TIMER_SetCounter(MDR_TIMER2, uSecs); TIMER_Cmd(MDR_TIMER2,ENABLE);
  /* the counter is started. uSecs=uDlay в мкс  */
#define Tim2Hlt() TIMER_Cmd(MDR_TIMER2, DISABLE)
 /* Halt TIMER2 */
#define ifToNfin() if (TIMER_GetFlagStatus(MDR_TIMER2, TIMER_STATUS_CNT_ZERO)== RESET) ///ifDlNfin() надо ... else Tim2Hlt(); и т.п.
 /* check if CNT != 0 */
 //просто разблокировать себя же (поток менеджер) в новом состоянии!
#define deBlkMngr(NewStat) {Itm2Q3.F.msg=0; mngState=NewStat; xQueueSendToBack(xQueueMang,&Itm2Q3, 0); }
#define loadARTimer(uiARTimerPeriod) xTimerChangePeriod(xAutoReloadTimer, uiARTimerPeriod, 5); \
                   xTimerStart(xAutoReloadTimer, 0);

#if defined (ADM485)
#define ADM485RE() PORT_ResetBits(MDR_PORTB, PORT_Pin_3);
#define ADM485RD() PORT_SetBits(MDR_PORTB, PORT_Pin_3);          //Read disable
#else
#define ADM485RE() {;}
#define ADM485RD() {;}
#endif

#define ADM485In()  {if (!OneMCU) PORT_ResetBits(MDR_PORTB, PORT_Pin_2);else PORT_SetBits(MDR_PORTB, PORT_Pin_2);} //иначе прием по FB
#define ADM485Out() {PORT_SetBits(MDR_PORTB, PORT_Pin_2);} // {PORT_ResetBits(MDR_PORTB, PORT_Pin_2);} //{PORT_SetBits(MDR_PORTB, PORT_Pin_2);}
#define ChkADM485Out() PORT_ReadInputDataBit(MDR_PORTB, PORT_Pin_2) /// if 0 - Bit_RESET ?
//-------------------------------------------------------

#define comb(who, msg) (who | msg<<8)
#define  SetIFLS(UART_IFLS, Rx, Tx) UART_IFLS = (Rx <<3)|(Tx) //(FIFO14 <<3)|(FIFO14); /* Порог FIFO при приеме (<<3), по передаче! */

#define OwnAddr 0xF000 //own modbus address
#define attmts 3  //число повторных попыток в modbus
#define TimOmdbus 400 //950  550    //таймаут для modbus
///Типы элементов очередей
///Элементы очереди к программе

	struct dlgQitm
	{  uint8_t who; uint8_t msg; uint8_t strt; uint8_t nmb;
	};
	struct LowHigh
	{  uint16_t Lo; uint16_t Hi;
	};

	typedef union DialQueueItem
	{
		char octets[4];
		long itm32;
		signed long s32;
        unsigned long u32;
		struct dlgQitm F;
		struct LowHigh LH;
	} DLGQ_ITEM;
	/// Структуры для интерпретации последовательностей передачи параметров
	typedef struct
	{  uint8_t parm; uint8_t szPty;   //param (aTF, gamma ..), SizeParity
	} SEQ_ITM;                      //szPty |=0x80; /*need parity*/
typedef union
	{	uint8_t octets[2];
		uint16_t WD;
		SEQ_ITM F;
	} SeqItm;
//++++++++++++++++++++++++++++++++++++++++++
///descriptors for values in block taken from DS750 or analogue
	typedef struct   // Структура дескриптора
	{  uint8_t bfn;  ///номер байтового буферного массива с данным парам.
	   uint8_t idx;  ///индекс нач.байта параметра в данном буфере
	   uint8_t sze;  ///число байт в записи параметра в буфере
	   uint8_t rsv;  ///reserved
	   int16_t FctMx;  //(множитель для исходного значения параметра) * (максимум параметра)
	} ItmDscrtr;
//++++++++++++++++++++++++++++++++++++++++++

	typedef union DataPar
	{	uint8_t octs[4];
		uint16_t wd[2];
		uint32_t P32;
	} CMD_PAR;

    typedef union
	{
	uint8_t Bt[2];
	uint16_t Wd;
	int16_t SW;
	} TVar16;

	typedef volatile union
	{
	uint32_t DW;
	float Fl;
	uint8_t Bt[4];
	uint16_t Wd[2];
	} TVar32;

	typedef volatile union
	{
	uint32_t DW;
	uint8_t Bt[4];
	uint16_t Wd[2];
	} TVar32i;

//с 5.09.22
typedef struct {
  uint32_t        Erase_55ms;   //  SectorErase / ChipErase
  uint32_t        Program_45us; //  ByteProgram
  uint32_t        Reset_30us;   //  Reset
  uint32_t        CS_20ns;      //  CS - Задержка между активацией SPI и передачей данных
  uint32_t        RD_30ns;      //  CS - удержание между чтениями
  uint32_t        WR_1us;       //  CS - удержание между записями
} MDR_RR52_Delays;


///Индексы объектов для функции 6 modbus:
#define oVoid  0    //Void deleted object
#define oTx_0  1   //первая упр. последовательность
#define oTx_1  2   //вторая...
#define oFp_0  3   //array of float
#define oHx_0  4   //arr. of hex
#define oIn_0  5  //arr. of signed integer
/// types of blocks in flash memory, not 0!
#define SeqS 1  //arr. of first sequence
#define SeqT 2  //arr. of second sequence
#define VibT 3  //пороги по вибрации

/// размеры массивов
/*-----------------------------------------------------*/
#define BUFFER_LENGTH 64  //120 Буферы для UART2
#define BufferSize   64   //Буферы для UART1 RS485

#define SPIBufferSize  6

#define adcBufSize  7
      //мест в массиве результатов ADC за 1 цикл 0-Ay, 3-Az, 4-Ax,5-бат2, 7-бат1,INT_VREF,TEMP_SENSOR
#define AChnls adcBufSize + 6+1+2	  //аналоговых и цифровых каналов ввода, с учетом дублирования в cas200 по одной из осей!
/// Codes for Interpreting
#define Fin 0xff       ///signature of end for resulting sequence
#define Ket 0xfe       ///signature of '}'
#define Cyc 0xfd       ///signature of Nmb of cycles for {}
///must be indices of descriptors for values in block taken from DS750 or analogue
#define iInc    0//'I'
#define iAzm    1//'A'
#define iaTFA   2//'a'
#define iMagF   4//'M'
#define iGrav   5 //'G'
#define igama   6 //'g'
#define iBatV   7 //'B'
#define iTemp   8 //'T'
#define iBat2   9 //'b'
#define iGX    10 //'X'
#define iGY    11 //'Y'
#define iGZ    12 //'Z'
#define iHX    13 //'u'
#define iHY    14 //'v'
#define iHZ    15 //'w'

///Номера состояний of State machines (разных)
#define Stat0  0/*Start*/
#define Stat1  1/*rdPROM*/
#define Stat2  2/*wtVib*/
#define Stat3  3/*wtSlnce*/
#define Stat4  4/*wrMdbS1*/
#define Stat5  5/*rdMdbS1*/
#define Stat6  6/*wtSync*/
#define Stat7  7/*TL*/
#define Stat8  8/*stSSq1*/
#define Stat9  9/*sndPars1*/
#define Stat9b 20/*TL1*/
#define Stat10 10/*GtPar2*/
#define Stat10b 21/*Freshn*/
#define Stat11 11/*gtPars2*/
#define Stat12 12/*Reset*/
#define Stat13 13
#define Stat14 14
#define Stat15 15
#define Stat16 16
#define Stat17 17
#define Stat18 18

///Индексы значений в gAv
#define ixAx  0
#define ixAz  2
#define ixAy  1
#define ixBt2 3
#define ixBt1 4
#define ixVRf 5
#define ixTmpS 6

 /** число pi  ***/
 #define pi 3.141592653


#define VRef 30  //ADC_CH_INT_VREF   Selects the ADC channel 30 (Internal VRef).
#define TmpS 31  //ADC_CH_TEMP_SENSOR) Selects the ADC channel 31 (Temperature Sensor).

#define TmpCa    9     //0     ///Index of temperature from CAS200
#define TmpSn    10    //2
#define VRefn    11    //1

#define dADCTr   adcTrig - adcTrigD
#define maxSS_id 4
#define SPI_noSS 0xff   ///initial state for PEx


#define defADCcycs 256 //1 ///5.01.21 8 //число повторов в цикле АЦП по умолчанию
//#define defADCmode 0 //разовая серия АЦП
#define defADCrepT 1; //2   период повторения циклов АЦП по умолчанию в тиках -1
#define PerMod 1  ///периодический режим ADC
#define nmrWrMask 0xff    //  //mask to limit nmrWr 0x7 0xf 0x1f 0x3f
//пришлось менять FreeRTOSConfig.h !! Иначе - зависание!!
#define nmrUpMask 0x7  //mask to limit nmrUpDn from Trigon
#define nmb2mdlt 3    ///кол-во элементов массива слотовых наборов по 64 бита -1

/// тип структур для сбора данных с последующих усреднений
	typedef struct
	{  uint32_t tMrk;
	   uint32_t AV[AChnls];
	} eAvr;

	typedef struct
	{  //uint32_t tMrk;
		int16_t U[3]; //float  //x,y,z
	} eInt4FFT;    //eFlo4FFT;
/// тип структуры для массива кодов, обрабатываемых модулятором
		typedef struct
	{  uint8_t Slts; //mnb of slots
	   uint8_t HiByt;
	   uint8_t LoByt;
	} toMdltr;

///пороги прерываний для FIFO
//#define F_lvl = { 2, 4, 8, 12, 14 }
#define FIFO2  0    //пороги прерываний для FIFO
#define FIFO4  1
#define FIFO8  2
#define FIFO12 3
#define FIFO14 4
#define maxFIFO 16	//Включать прерывание по передаче?


#define MxAtmps 3    //количество повторов при отсутствии отклика
///коды для элементов очередей
///источник who
#define Comm   0x01    // callback USB
#define ARTim  0x02    // callback AutoReloadTimer
#define THook  0x03    // callback vApplicationTickHook, истек таймаут
#define ISR485 0x04    // ISR UART1_IRQHandler
#define ISRuart2 0x05    // ISR UART2_IRQHandler
///#define ISRDma   0x07    // ISR DMA_IRQHandler
#define Mnger 0x09   // Задача vManager
#define Cnl485 0x0a   // Задача vCnl485
#define Measur 0x0c   // Задача vMeasur
#define Interp 0x0d   //Задача vInterp

///сообщения/команды

#define stADC 3  //запустить преобразование (серию, последовательность серий и т.д.)
#define TOut 4   //TimeOut
#define SndU 9   //отправить наверх

#define Rcv 0xa  ///принять сверху и обработать   USB/UART2
#define sent485 0xb  ///report: отправлена в FIFO rs485 последняя порция
#define rx485FB 0xc ///report: принято по FB из FIFO rs485
#define rcv485 0xd  ///report: принято от aliens из FIFO rs485
#define Alarm 0xe   ///report: сработал программный таймер
#define sntUART2 0xf ///report: отправлена в FIFO UART2 последняя порция
#define sntRS485 0x10  ///report: отправлена в FIFO UART1 (RS485) последняя порция  ??
#define finAD    0x11  ///report: ADC DAC cycle finished
#define SSq1  0x12     ///Выбрать последовательность SSq1
#define TSq1  0x13     ///Выбрать TSq1. Изменить таймаут.

#define NExtPar 0x14   ///Принять очередной параметр из последовательности
#define bgnTrnz 0x15   ///начать транзакцию Modbus
#define finTrnz 0x16   ///завeршить транзакцию Modbus
#define chFsUp  0x17 	///изменить в globFlags sUp (от Atmega168), нужно и для разблокировки потока!
#define Dly 0x18   ///устроить задержку в тиках, переданных через Itm....LH.Hi
#define Mdlt 0x19  ///выполнить модуляцию
#define Alm 0x1a   ///устроить задержку в тиках, переданных через Itm....LH.Hi
#define Awk 0x1b   ///устроить задержку для PwrOff в тиках, переданных через Itm....LH.Hi


#define ACK 0xfd
#define NAK 0xfe
#define sinMSG 0xff  //в очереди нет команды
#define SndB 0x13 ///отправить наверх бинарный пакет

/// коды "сквозных" команд, совпадающих для МК и ПК
#define SndO 0xf4 //запустить АЦП, вернуть oAngls! От ПК принять значение ADCcycs(длина серии АЦП)
#define SndN 0xf5 //запустить АЦП, вернуть Un! От ПК принять значение ADCcycs(длина серии АЦП)
#define SndV 0xf6 //запустить АЦП, вернуть Uv! От ПК принять значение ADCcycs(длина серии АЦП)
#define SndM 0xf7 //запустить АЦП, вернуть коды! От ПК принять значение ADCcycs(длина серии АЦП
#define MVib 0xf7 ///запустить АЦП, вернуть уровень вибрации и замерить уровень в батареях

///Флаги в составе uint16_t globFlags;
#define Rst  1   ///need to do operations for software restart
#define Trp  2   ///С остановом на контрольной точке (точка задается отдельным параметром)
#define Gnd  4   ///подключена наземная часть
#define sUp  8   ///модулятор готов

#define Pmp  16  ///насос работает (вибрация)    0x10
#define PmA  32   ///повторно определено "насос работает"  0x20
#define Msr  64  ///есть результат замера        0x40
#define uFn  128 ///прерванная передача наверх   0x80

#define Mtf  256  /// Mtf / Gtf  Режим Mtf при зенитном <5 градусов, >5 - переход в GTF из MTF    0x100
                   ///Режим Gtf при зенитном >4 градусов, <4 - переход из GTF в MTF
#define FdGm  512  ///найден gama в первой послед.                                     0x200
#define Seq2  1024 ///идет работа со 2-й послед.                                       0x400
#define ByCmn 2048 ///есть покомпонентная передача в 1-й последов                     0x800

#define ByCm2 4096 ///есть покомпонентная передача во 2-й последов                    0x1000
#define B2on  8192  ///перешли на вторую батарею                                       0x2000
#define MsBsy 0x4000   ///идет измерение вибрации и т.п.                               0x4000
#define DS750Up 0x8000 ///истекло время выхода на режим DS750
#define wSync 0x10000  ///программа доходила до состояния Stat6/*wtSync*/


///режимы работы (состояния stage485)
//
#define reqSt   1
#define rspSt   2
///режимы работы (состояния msrState потока Measure)
#define No2do 0

///***************** коэффициенты от Глобуса (29.03.2020 до внешней флеш!) *****************
#define t0      1.3800
#define tm      50.000
#define t_op    25.000
#define gxt1    0.0000
#define gxt2    0.0000
#define gyt1    0.0000
#define gyt2    0.0000
#define gzt1    0.0000
#define gzt2    0.0000
#define hxt1    0.0000
#define hxt2    0.0000
#define hyt1    0.0000
#define hyt2    0.0000
#define hzt1    0.0000
#define hzt2    0.0000
#define gx0     0.0102
#define gy0     0.0354
#define gz0     0.0123
#define gxm    -1.1047
#define gym    -1.1093
#define gzm     1.1035
#define e12     0.0175
#define e13    -0.0000
#define e21     0.0006
#define e23     0.0061
#define e31     0.0072
#define e32     0.0170
#define hx0     1.5172
#define hy0     1.5106
#define hz0     1.5069
#define hxm    -0.3248
#define hym    -0.3245
#define hzm     0.3243
#define m12     0.0099
#define m13     0.0255
#define m21    -0.0125
#define m23     0.0214
#define m31     0.0037
#define m32     0.0033
#define axt1    0.0000
#define axt2    0.0000
#define ayt1    0.0000
#define ayt2    0.0000
#define azt1    0.0000
#define azt2    0.0000
#define ax0     0.0027
#define ay0     0.0039
#define az0     0.0042
#define axm    -0.2815
#define aym    -0.2823
#define azm     0.2809
#define ae12   -0.0254
#define ae13    0.0152
#define ae21    0.0042
#define ae23    0.0099
#define ae31    0.0052
#define ae32   -0.0267
#define kv      0.805860805E-3  /*для феррозондов  0.000805860805 */
#define kva     0.1007E-3  ///для акселерометров CAS200   0,0001007
#define fi_corp 0.00

#define BB      2.8
//********** конец блока коэффициентов от Глобуса *************
//Структура для параметров, получаемых с датчиков
   typedef struct
 { uint32_t tMrk; //маркер системного времени MCU
   float   Gx;  //  акселерометр x (2g)
   float   Gy;  //  акселерометр y (2g)
   float   Gz;  //  акселерометр z (2g)
   float   Ax;  //  акселерометр x (10g)
   float   Ay;  //  акселерометр y (10g)
   float   Az;  //  акселерометр z (10g)
   float   Hx;  //  феррозонд x
   float   Hy;  //  феррозонд y
   float   Hz;  //  феррозонд z
   float   T;   //  температура
   float   Gz1;  //  дублирующая ось акселерометр z (2g)  11.02.21
   float   Az1;  //  акселерометр z (10g) 11.02.21
   float   Tmcu;//  температура процессора
   float   Umcu;//  напряжение на процессоре
 } TParam;

/** Структура для отправки нормализованных напряжений, нужна ли??? **/
   typedef struct
 { uint32_t tMrk; //маркер системного времени MCU
   float   T;   //  температура
   float   Gx;  //  акселерометр x (2g)
   float   Gy;  //  акселерометр y (2g)
   float   Gz;  //  акселерометр z (2g)
   float   Hx;  //  феррозонд x
   float   Hy;  //  феррозонд y
   float   Hz;  //  феррозонд z
   float   Ax;  //  акселерометр x (10g)
   float   Ay;  //  акселерометр y (10g)
   float   Az;  //  акселерометр z (10g)
 //  float   Tmcu;//  температура процессора
 //  float   Umcu;//  напряжение на процессоре
 } resU;

/** Структура для отправки наверх углов **/
   typedef struct
 { uint32_t tMrk;    //маркер системного времени MCU
   float   fi_out;   //визир (градусы)  по акселерометрам 2g
   float   teta_out; //зенитный (градусы)
   float   alf_out;  //азимут (градусы)
   float   G;        //полный вектор тяжести (должен быть=1)
   float   bb;       //магнитное наклонение
   float   Hbb;      ///  полный вектор магнитного поля Земли (условные ед-цы)
   float   nfi_out;  //визир (градусы) по акселерометрам 10g
   float   nteta_out;//зенитный (градусы)
   float   nalf_out; //азимут (градусы)
   float   nG;       //полный вектор тяжести (должен быть=1) от 10g
   float   nbb;      //для варианта от 10g
   float   Hnbb1;    /// полный вектор магнитного поля Земли (условные ед-цы)
 } outTrigon;

///Устаревшие вещи для работы с RS485
///варианты работы с транзакциями rs485   progMode
#define manResp  0  /*"Master, manual response if oneMCU"); */
#define autoResp 1  /*"Master, Auto response if oneMCU"); */
#define manSlave 2  /*"Slave, manual resp if oneMCU"); */
#define autSlave 3  /*"Slave, auto resp if oneMCU"); */
#define autoMstr 4  /*"Master, auto repeat"); */

