 //Предназначен для системы на базе отечественного микроконтроллера (МК) K1986BE92QI (MILANDR). 
 //Автор - Р.А.Гараев: garaevra@mail.ru. 
 //Компилятор:  arm-none-eabi-gcc.exe (ARM GCC Compiler из IDE Em::Blocks 2.30).
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include "MDR32Fx.h"
#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_port.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "user_defsM.h"
#include <math.h>
/*****************  Поток интерпретации последовательностей передачи данных  *************/

extern QueueHandle_t xQueueInterp, xQueueMang, xQueueMeasur, xQueueTimO, xQueueNext; //xQueueTrigon;
extern eAvr SignADC[];
/*************************/
static DLGQ_ITEM Itm2Q_Interp;

#define unk 1
#define lac 2
#define dig 3
#define chr 4
#define bra 5
#define ket 6
#define cln 7

///Порядок в пакете от DS750:  TF, MTF, Inc, MagF, Azm, Grav, Temp, BatV
//см. Преобразование данных DS750 для кодера.docx
///Т.о. для DS750 начальные адреса в пакете Modbus от DS750 (через AtMega). HighLow!!

#define  TF    0 //+3  //3 - Header
#define  MTF   2 //+3 тройка пропущена, чтобы различать TF/MTF
#define  Inc   4 //+3
#define  MagF  6 //+3
#define  Azm   8 //+3
#define  Grav 10 //+3
#define  HX    0 //+3  //3 - Header для покомпонентного пакета
#define  GX    2 //+3
#define  HY    4 //+3
#define  GY    6 //+3
#define  HZ    8 //+3
#define  GZ   10 //+3
#define  Temp 12 //+3
#define  BatV 14 //+3

//из пакета от gamma-модуля
#define  gamma 0    //С учетом заголовка
//из местного буфера
#define  Bat2 0     //тут без заголовка

///Пока не задействовано! must be indices of descriptors for values in block taken from DS750 or analogue
/*
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
#define iHZ    15 //'w'  */
const ItmDscrtr stDscrpt[] = {    ///массив дескрипторов
    {0,Inc,2,0,1799},
    {0,Azm,2,0,3599},
    {0,TF,2,0,3599},      //со ссылкой на MTF
    {0,MTF,2,0,3599},
    {0,MagF,2,0,9999},
    {0,Grav,2,0,19999},
    {1,gamma,1,0,255},  //другой пакет Modbus!   {1,gamma,2,0,255}, потом будет 2 байта!
    {2,BatV,2,0,4095}, //3599}, //{0,BatV,2,0,3600},Это для DS750, надо от Миландр!  9
    {0,Temp,2,0,14999},  //10
    {2,Bat2,2,0,0},     //это бинарное, определено на МК телесистемы (globFlags), может 1 байт?
    {0,GX,2,0,0xffff},
    {0,GY,2,0,0xffff},
    {0,GZ,2,0,0xffff},
    {0,HX,2,0,0xffff},
    {0,HY,2,0,0xffff},
    {0,HZ,2,0,0xffff}
    };

    ///descriptors for values in block taken from DS750 or analogue
 // Структура дескриптора:
///номер байтового буферного массива с данным парам.
///индекс нач.байта параметра в данном буфере
///число байт в записи параметра в буфере
///reserved
///(множитель для исходного значения параметра) * (максимум параметра)

//++++++++++++++++++++++++++++++++++++++++++
//char seq[]="20{3{aTFA:6:p}1{gama:8:p}} BatV:8:p 20{3{aTFA:6:p}1{gama:8:p}} Temp:8:p Bat2:8:p";
//char seq[]="Inc:12:p Azm:12:p aTFA:6:p  MagF:12:p Grav:12:p";
//это прототипы записей в флэш-памяти
//const uint8_t csSSq1[] = "gama:8:p gama:8:p";
const uint8_t csSSq1[] = "Inc:12:p Azm:12:p aTFA:6:p MagF:12:p Grav:12:p";

//const uint8_t csSSq1[] = "Inc:12:p Azm:12:p aTFA:6:p MagF:12:p Grav:12:p";
///const uint8_t sSSq1[] = "HX:12:p GX:12:p HY:12:p GY:12:p HZ:12:p GZ:12:p Temp:8:p";

//const uint8_t csTSq1[] = "2{1{gama:8:p}} BatV:8:p 2{1{gama:8:p}} Bat2:1:n";
const uint8_t csTSq1[] = "2{3{aTFA:6:p}1{gama:8:p}} BatV:8:p 2{3{aTFA:6:p}1{gama:8:p}} Temp:8:p Bat2:1:n";

uint8_t sSSq1[120], sTSq1[120];

uint8_t *ptr;
char subStr[8];   //strBuf[256],
uint16_t bycomp; //1 - покомпонентная передача Gx Hx...

 SeqItm prSSq1[32], prTSq1[64]; //массивы сомпилированных последовательностей
 SeqItm *PreCSeq;
 SeqItm *Sptr;

TickType_t ToWtIntrp;  //Timeout. May be changed

static 	DLGQ_ITEM Itm2QI = {{Interp, 0, 0, 0}};

extern uint32_t globFlags;  ///Глобальные флаги, если менять из разных потоков, то защитить мьютексом ?
/***** prototypes ********/
uint16_t interprSeq(void);
void setGlbFlg(const uint16_t Bits);
void rstGlbFlg(const uint16_t Bits);
/************************/
/*** Собственно потоковая фукция интерпретатора последовательностей передачи ***/
void vInterp(void *argument)
{ 	int16_t ii, M; portBASE_TYPE xStatus; //TVar32 i2P;
    uint16_t ret;

    TickType_t CT;   CT=xTaskGetTickCount();  ///системное время //SignADC[M].tMrk -=CT;
     ToWtIntrp = portMAX_DELAY; //Infinite

    while(1)
  { xStatus = xQueueReceive(xQueueInterp,(DLGQ_ITEM*) &Itm2Q_Interp,  portMAX_DELAY);  // ToWtIntrp
     if (xStatus!=pdPASS) {   /*Toggle_LED9();истек таймаут*/
                               Itm2Q_Interp.F.msg = sinMSG; /* "No msg" */
                          }
	   switch(Itm2Q_Interp.F.msg)  //приходит от потока vManager
        {  case SSq1:  /**** выполняется однократно при выключении насоса  ****/
                   Sptr = prSSq1; //Destination for prcompiled array
               do {ret = interprSeq(); /*need to analyze ret for incorrect val! */}
                while(ret!=Fin);
                break;
           case TSq1: Sptr = prTSq1; //Destination for prcompiled array
                     /* ToWtIntrp = 0;  500 / portTICK_PERIOD_MS; */  ///перейти к циклической работе по TSq1, изменить таймаут
  		   case sinMSG:      ///циклически выполнять интерпретацию TSq1
                do {ret = interprSeq(); /*need to analyze ret for incorrect val! */}
                 while(ret!=Fin);
                 break;   //-------------------------------
        }  //for switch
          if (globFlags & Rst)
        { /* Есть требование сброса */
               ToWtIntrp = portMAX_DELAY;
            while ( xQueueReceive(xQueueNext,(DLGQ_ITEM*) &Itm2Q_Interp, 0)==pdTRUE)  {;} ///опустошить
        }
   }   //end while (1
}  //for  void vInterp(void *argument)
/********************************************************/
BaseType_t extInterpCreate(void)
{ return( xTaskCreate(vInterp, "2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL) );
}
/*******************************************************/
//++++++++++++++++++++ functions ++++++++++++++++
uint16_t assChar(void)
{   uint8_t lc=*ptr;
	if (lc=='\0') return(0);
	if (lc==' ') return(lac);
	if (isdigit(lc)) return(dig);
	if (isalpha(lc)) return(chr);
	if (lc=='{') return(bra);
	if (lc=='}') return(ket);
	if (lc==':') return(cln);
	return(unk);
}
//+++++++++++++++++++++++++++++++++++
///Предварительная обработка строки с последовательностью передачи наверх
int16_t preCmpl(void)
{ int ii, jj, val, nest=0;     //nest - depth of {}
  uint8_t parm;

   for (ii=0; ii<128; ii++)    //along PreCSeq
 {
	val=0; (PreCSeq+ii)->WD = 0; //forelean

	while (*ptr==' ') ptr++;   //spaces
   switch(assChar())
	 { case 0:  //end of string
		   if (nest) return(-5);    //nesting error
                   (PreCSeq+ii)->F.parm=Fin;  //signature of end
		   return(ii); ///string processed? Normal exit
		 /*---------------------------*/
	   case dig: jj=4; //max digits
		  while (isdigit(*ptr))
			{ val = val*10+(*ptr-'0'); ptr++;
			  if (!(--jj)) return(-2);  //incorrect val
			  if (val>250) return(-2);
			}
                 (PreCSeq+ii)->F.parm=Cyc; (PreCSeq+ii)->F.szPty=val;
			  //here means nmb of repetitions of {}
               	while (*ptr==' ') ptr++; //may be spaces

			   if (*ptr!='{') return(-1);  //syntax error
				 nest++; ptr++; //to next lexem
				 break;
		 /*---------------------------*/
	   case bra:  nest++; ptr++; //just go to next
		      (PreCSeq+ii)->F.parm=Cyc; (PreCSeq+ii)->F.szPty=1;
			  // nmb of repetitions of {}
				 break;
		 /*---------------------------*/
	   case ket:  nest--; if (nest<0) return(-1);  //syntax error
			   ptr++; /*just go to next */
			   (PreCSeq+ii)->F.parm=Ket; (PreCSeq+ii)->F.szPty=0;
				break;
		 /*---------------------------*/

	   case chr: jj=0; parm = 0xff;
                    subStr[jj++] = *ptr++;
                  while (jj<7) /*max length of name +1*/
		   { //subStr[jj++] = *ptr++;
			  switch(assChar())
			 { case 0: return(-3);   //untimely end
			   case chr:
			   case dig: subStr[jj++] = *ptr++;
                                   break;
			   case cln: //: after parm's name
				  subStr[jj]=0; //ASCIZ
				   if (!strcmp(subStr,"Inc")) parm=iInc;
				   if (!strcmp(subStr,"Azm")) parm=iAzm;
				   if (!strcmp(subStr,"aTFA")) parm=iaTFA;
				   if (!strcmp(subStr,"MagF")) parm=iMagF;
				   if (!strcmp(subStr,"Grav")) parm=iGrav;
				   if (!strcmp(subStr,"gama")) { parm=igama; setGlbFlg(FdGm);}
				   if (!strcmp(subStr,"BatV")) parm=iBatV;
				   if (!strcmp(subStr,"Temp")) parm=iTemp;
				   if (!strcmp(subStr,"Bat2")) parm=iBat2;
				    ///c 06/06/21
                        if (!strcmp(subStr,"GX")) parm=iGX;
                        if (!strcmp(subStr,"GY")) parm=iGY;
                        if (!strcmp(subStr,"GZ")) parm=iGZ;
                        if (!strcmp(subStr,"HX")) parm=iHX;
                        if (!strcmp(subStr,"HY")) parm=iHY;
                        if (!strcmp(subStr,"HZ")) parm=iHZ;

				   if (parm==0xff) return(-4); //unknown param
				   switch(parm)
				   { case iGX:
				     case iGY:
				     case iGZ:
                     case iHX:
                     case iHY:
                     case iHZ:  setGlbFlg(ByCmn); break; //есть покомпонентная передача в 1-й последов.
                    default:    break;
				   }
				   (PreCSeq+ii)->F.parm=parm; ptr++; jj=7; //to cause exit!
                              break;
				   /*---------------------------*/
			   default: return(-1); //syntax error //wrong symbol
			 }              //end to switch(assChar())
		   }                //to while (jj<7)
		   //get nmb of bits  *********************
		   jj=3; //max digits +1
		  while (isdigit(*ptr))
			{ val = val*10+(*ptr-'0'); ptr++;
			  if (!(--jj)) return(-2);
			  if (val>95) return(-2);
			  (PreCSeq+ii)->F.szPty=val;
			  //here means nmb of bits
			}  	while (*ptr==' ') ptr++; //spaces
			   //next ':'
			   if (*ptr!=':') return(-1);
				  (PreCSeq+ii)->F.szPty=val; ptr++;
			   if (*ptr=='p') (PreCSeq+ii)->F.szPty |=0x80; /*need parity*/
			   ptr++;//to next lexem
			break;  //to case chr
		 /*---------------------------*/
	   default: return(-1); //wrong symbol
		 /*---------------------------*/
	 }  //switch(assChar())    //lac; bra; ket;
	}  //for ii != 0      ??pLtr = Ltr;
	return(ii);    //suspicious reach of the end of string
 }  //end of precCmpl
//*********************************
///Интерпретация обработанной строки с последовательностью передачи наверх
uint16_t interprSeq(void)
{ int reps, iii; SeqItm *locptr;
  if (Sptr->F.parm == Cyc)
  {  reps = Sptr->F.szPty; Sptr++; } //to next item
  else reps = 1;  locptr=Sptr;       //left at the 1 item
  for (iii=0; iii<reps; iii++)
  {  Sptr = locptr;
   while(1)
   {  if (Sptr->F.parm == Cyc) interprSeq();
      else
      { //if (Sptr->F.parm > Cyc) break; ///Ket or Fin
       if (Sptr->F.parm != Ket)
         { Itm2QI.F.msg = NExtPar;
           Itm2QI.F.strt = Sptr->F.parm;   ///в очередь xQueueNext может попадать и Fin, но не Ket!!
           Itm2QI.F.nmb = Sptr->F.szPty;
           while(!(globFlags & Rst))
           if (xQueueSendToBack(xQueueNext,(long*) &Itm2QI, 500 / portTICK_PERIOD_MS)==pdTRUE) break;
         ///Каждые 500 мс даже при полной очереди будет проверять, нет ли Rst
         }
         if (Sptr->F.parm > Cyc) break; ///Ket or Fin Лучше так!
         Sptr++;
      }
   }
  }   if (Sptr->F.parm == Ket) Sptr++;  //Ket swallowed
 return(Sptr->WD);    ///must be Fin
}
//*********************************
/********************************************/









