//Базовая ветка
//#define SC16IS740_UART



#include "lcd_AGM1232_uku207_3.h"
#include "rtl.h"
#include "type.h"
#include "main.h"
#include "simbol.h"
//#include "25//lc640.h"
#include "Timer.h"
#include "gran.h"
//#include "uart0.h"
//#include "uart1.h"
//#include "uart2.h"
#include "cmd.h"
#include "ret.h"
//#include "eeprom_map.h"
#include "common_func.h"
//#include "control.h"
#include "mess.h"
//#include "full_can.h"
#include "watchdog.h"
//#include "ad7705.h"
#include "beep.h"
//#include "avar_hndl.h"
#include "memo.h"
#include "simbols.h"
#include "graphic.h"
//#include "snmp_data_file.h" 
#include "net_config.h"
//#include "uart0.h"
#include <rtl.h>
//#include "stark.h"
//#include "ztt.h"
//#include "mcp2515.h"
//#include "sc16is7xx.h"

extern U8 own_hw_adr[];
extern U8  snmp_Community[];
BOOL tick;
extern LOCALM localm[];
#define MY_IP localm[NETIF_ETH].IpAdr
#define DHCP_TOUT   50

//***********************************************
//Таймер
char b10000Hz,b1000Hz,b2000Hz,b100Hz,b50Hz,b10Hz,b5Hz,b2Hz,b1Hz,b1min;
short t0cnt,t0cnt0,t0cnt1,t0cnt2,t0cnt3,t0cnt4,t0cnt5,t0cnt6,t0_cnt7,t0_cnt_min,t0cntMin;
char bFL5,bFL2,bFL,bFL_,bTPS;
signed short main_10Hz_cnt=0;
signed short main_1Hz_cnt=0;

 
//***********************************************
//Структура ИБЭПа
char cnt_of_slave=3;
//char cnt_of_wrks;   //колличество работающих источников , для индикации



//**********************************************
//Коэффициенты, отображаемые из EEPROM
signed short Ktsrc[2];
signed short Kusrc[2];
signed short Kisrc[2];
signed short Ki0src[2];
signed short Kubat[2];
signed short Kubatm[2];
unsigned short Kibat0[2];
signed short Kibat1[2];
signed short Ktbat[2];
signed short Kunet;
signed short Kunet_ext[3];
signed short Ktext[3];
signed short Kuload;
signed short KunetA;
signed short KunetB;
signed short KunetC;
signed short Kubps;
signed short Kuout=2;

signed short MAIN_IST;
signed short UMAX;
signed short UB0;
signed short UB20;
signed short TMAX;
signed short TSIGN;
signed short AV_OFF_AVT;
signed short USIGN;
signed short UMN;
signed short ZV_ON;
signed short IKB;
//signed short KVZ;
signed short UVZ;
signed short IMAX;
signed short IMIN;
signed short APV_ON;
signed short IZMAX;
signed short U0B;
signed short TZAS;
signed short VZ_HR;
signed short TBAT;
signed short U_AVT;
signed short DU;
signed short PAR;
signed short TBATMAX;
signed short TBATSIGN;
signed short UBM_AV;
signed short RELE_LOG;
signed short TBOXMAX;
signed short TBOXREG;
signed short TBOXVENTMAX;
signed short TLOADDISABLE;
signed short TLOADENABLE;
signed short TBATDISABLE;
signed short TBATENABLE;
signed short TVENTON;
signed short TVENTOFF;
signed short TWARMON;
signed short TWARMOFF;
enum_releventsign RELEVENTSIGN;
signed short TZNPN;
signed short UONPN;
signed short UVNPN;
enum_npn_out NPN_OUT;
enum_npn_sign NPN_SIGN;
signed short TERMOKOMPENS;
signed short TBOXVENTON; 
signed short TBOXVENTOFF;
signed short TBOXWARMON; 
signed short TBOXWARMOFF;
signed short BAT_TYPE;	//Тип батареи. 0 - обычная свинцовая, 1-литиевая COSLIGHT, 2-литиевая SACRED SUN , 3-литиевая ZTT
signed short DU_LI_BAT;	//Параметр, определяющий напряжение содержания литиевой батареи
signed short FORVARDBPSCHHOUR;	//Периодичностьсмены ведущего источника в часах. Если 0 - функция выключена и ведущий первый источник
signed short NUMBAT;
signed short NUMBAT_IP55;
signed short NUMIST;
signed short NUMINV;
signed short NUMDT;
signed short NUMSK;
signed short NUMEXT;
signed short NUMAVT;
signed short NUMMAKB;
signed short NUMBYPASS;
signed short U_OUT_KONTR_MAX;
signed short U_OUT_KONTR_MIN;
signed short U_OUT_KONTR_DELAY;
signed short DOP_RELE_FUNC;

enum_apv_on APV_ON1,APV_ON2;
signed short APV_ON2_TIME;

enum_bat_is_on BAT_IS_ON[2];
signed short BAT_DAY_OF_ON[2];
signed short BAT_MONTH_OF_ON[2];
signed short BAT_YEAR_OF_ON[2];
signed short BAT_C_NOM[2];
signed short BAT_RESURS[2];
signed short BAT_C_REAL[2];
//signed short BAT_TYPE[2];

unsigned short AUSW_MAIN;
unsigned long 	AUSW_MAIN_NUMBER;
unsigned short AUSW_DAY;
unsigned short AUSW_MONTH;
unsigned short AUSW_YEAR;
unsigned short AUSW_UKU;
unsigned short AUSW_UKU_SUB;
unsigned long AUSW_UKU_NUMBER;
unsigned long	AUSW_BPS1_NUMBER;
unsigned long  AUSW_BPS2_NUMBER;
unsigned short AUSW_RS232;
unsigned short AUSW_PDH;
unsigned short AUSW_SDH;
unsigned short AUSW_ETH;

signed short TMAX_EXT_EN[3];
signed short TMAX_EXT[3];
signed short TMIN_EXT_EN[3];
signed short TMIN_EXT[3];
signed short T_EXT_REL_EN[3];
signed short T_EXT_ZVUK_EN[3];
signed short T_EXT_LCD_EN[3];
signed short T_EXT_RS_EN[3];

signed short SK_SIGN[4];
signed short SK_REL_EN[4];
signed short SK_ZVUK_EN[4];
signed short SK_LCD_EN[4];
signed short SK_RS_EN[4];

enum_avz AVZ;

unsigned short HOUR_AVZ;
unsigned short MIN_AVZ;
unsigned short SEC_AVZ;
unsigned short DATE_AVZ;
unsigned short MONTH_AVZ;
unsigned short YEAR_AVZ;
unsigned short AVZ_TIME;

enum_mnemo_on MNEMO_ON;
unsigned short MNEMO_TIME;

signed short POWER_CNT_ADRESS;

signed short ETH_IS_ON;
signed short ETH_DHCP_ON;
signed short ETH_IP_1;
signed short ETH_IP_2;
signed short ETH_IP_3;
signed short ETH_IP_4;
signed short ETH_MASK_1;
signed short ETH_MASK_2;
signed short ETH_MASK_3;
signed short ETH_MASK_4;
signed short ETH_TRAP1_IP_1;
signed short ETH_TRAP1_IP_2;
signed short ETH_TRAP1_IP_3;
signed short ETH_TRAP1_IP_4;
signed short ETH_TRAP2_IP_1;
signed short ETH_TRAP2_IP_2;
signed short ETH_TRAP2_IP_3;
signed short ETH_TRAP2_IP_4;
signed short ETH_TRAP3_IP_1;
signed short ETH_TRAP3_IP_2;
signed short ETH_TRAP3_IP_3;
signed short ETH_TRAP3_IP_4;
signed short ETH_TRAP4_IP_1;
signed short ETH_TRAP4_IP_2;
signed short ETH_TRAP4_IP_3;
signed short ETH_TRAP4_IP_4;
signed short ETH_TRAP5_IP_1;
signed short ETH_TRAP5_IP_2;
signed short ETH_TRAP5_IP_3;
signed short ETH_TRAP5_IP_4;

signed short ETH_SNMP_PORT_READ;
signed short ETH_SNMP_PORT_WRITE;

signed short ETH_GW_1;
signed short ETH_GW_2;
signed short ETH_GW_3;
signed short ETH_GW_4;

signed short RELE_VENT_LOGIC;

signed short MODBUS_ADRESS;
signed short MODBUS_BAUDRATE;
signed short BAT_LINK;

signed short LI_UNECC;
signed short LI_Q;
signed short LI_IZMAX1;
signed short LI_IZMAX2;
signed short LI_K1;
signed short LI_K2;
signed short LI_K3;
signed short LI_T4;


//***********************************************
//Состояние батарей
BAT_STAT bat[2],bat_ips;
signed short		bat_u_old_cnt;
signed short 		Ib_ips_termokompensat;

//***********************************************
//Мониторы АКБ
MAKB_STAT makb[4];

//***********************************************
//Литиевые АКБ
LAKB_STAT lakb[3];
char lakb_damp[1][42];
char bLAKB_KONF_CH=0;
char bLAKB_KONF_CH_old=0;
char lakb_ison_mass[7];
short lakb_mn_ind_cnt;
char bLAKB_KONF_CH_EN;
//char bRS485ERR;
short LBAT_STRUKT;
char lakb_error_cnt;		//счетчик неправильного показания ннапряжения батареи
short numOfPacks,numOfPacks_;
short numOfCells, numOfTemperCells, baseOfData;
short lakb_stat_comm_error;	//аварийность канала связи с литиевыми батареями. 0 означает исправность платы расширения и наличие связи со всеми литиевыми батареями
short lakbNotErrorNum;		//колличество литиевых батарей с исправной связью
short lakbKanErrorCnt;		//Счетчик аварийности канала связи с платой расширения
short lakbKanErrorStat;		//Состояние аварийности канала связи с платой расширения

//#ifdef UKU_TELECORE2015
//***********************************************
//Состояние литиевой батареи  
LI_BAT_STAT li_bat;
//#endif

//***********************************************
//Телеметрия по внутренней шине
char can_slot[12][16];


//***********************************************
//Состояние источников
BPS_STAT bps[32];

//***********************************************
//Состояние инверторов
#ifdef UKU_220_V2
INV_STAT inv[3];
#endif
#ifndef UKU_220_V2
INV_STAT inv[20];
#endif
char first_inv_slot=MINIM_INV_ADRESS;

//***********************************************
//Состояние байпаса
BYPS_STAT byps;

//***********************************************
//Состояние нагрузки
signed short load_U;
signed short load_I;

//***********************************************
//Состояние выхода
signed short bps_U;
signed short out_U;
signed short bps_I;


//***********************************************
//Индикация

char lcd_buffer[LCD_SIZE+100]={"Hello World"};
signed char parol[3];
char phase;
char lcd_bitmap[1024];
char dig[5];
char dumm_ind[20];
stuct_ind a_ind,b_ind[10],c_ind;
char dumm_ind_[20];
char zero_on;
char mnemo_cnt=50;
char simax;
short av_j_si_max;
const char ABCDEF[]={"0123456789ABCDEF"};
const char sm_mont[13][4]={"   ","янв","фев","мар","апр","май","июн","июл","авг","сен","окт","ноя","дек"}; //
signed short ptr_ind=0;

signed short ind_pointer=0;

//***********************************************
//Состояние первичной сети
signed short net_U,net_Ustore,net_Ua,net_Ub,net_Uc;
char bFF,bFF_;
signed short net_F,hz_out,hz_out_cnt,net_F3;
signed char unet_drv_cnt;
char net_av;

//***********************************************
//Состояние внешних датчиков
//signed short tout[4];
char tout_max_cnt[4],tout_min_cnt[4];
enum_tout_stat tout_stat[4];
signed short t_ext[3];
BOOL ND_EXT[3];
signed char sk_cnt_dumm[4],sk_cnt[4],sk_av_cnt[4];
enum_sk_stat sk_stat[4]={ssOFF,ssOFF,ssOFF,ssOFF};
enum_sk_av_stat sk_av_stat[4]={sasOFF,sasOFF,sasOFF,sasOFF},sk_av_stat_old[4];
signed short t_box,t_box_warm,t_box_vent;

//***********************************************
//Звуки
extern char beep_cnt;
BOOL bSILENT;








signed short u_necc,u_necc_,u_necc_up,u_necc_dn;
signed short main_cnt_5Hz;
signed short num_necc;
signed short num_necc_Imax;
signed short num_necc_Imin;
signed short cnt_num_necc;
//char bSAME_IST_ON;
signed mat_temper;

//***********************************************
//АПВ
unsigned main_apv_cnt,hour_apv_cnt[2],reset_apv_cnt[2];
unsigned short apv_cnt_sec[2],apv_cnt[2];

//***********************************************
//Текстовые константы
const char sm_[]	={"                    "};
const char sm_exit[]={" Выход              "};
const char sm_time[]={" 0%:0^:0& 0</>  /0{ "};





//**********************************************
//Работа с кнопками 
char but;                            
unsigned long but_n,but_s;
char but0_cnt;
char but1_cnt;
char but_onL_temp;

//***********************************************
//Межблоковая связь
char cnt_net_drv;







//***********************************************
//Работа с кнопками
char speed,l_but,n_but;

//***********************************************
//Неразобранное
enum {wrkON=0x55,wrkOFF=0xAA}wrk;
char cnt_wrk;
signed short ibat_integr;
unsigned short av_beep,av_rele,av_stat;
char default_temp;
char ND_out[3];

//***********************************************
//Тест
enum_tst_state tst_state[15];

//***********************************************
//АЦП
//extern short adc_buff[16][16],adc_buff_[16];
//extern char adc_cnt,adc_cnt1,adc_ch;

//***********************************************

char flag=0;


extern signed short bat_ver_cnt;
signed short Isumm;
signed short Isumm_;

#include <LPC17xx.H>                        /* LPC21xx definitions */



/*
extern void lcd_init(void);
extern void lcd_on(void);
extern void lcd_clear(void);
*/

//extern short plazma_adc_cnt;
extern char net_buff_cnt;
extern unsigned short net_buff[32],net_buff_;
extern char rele_stat/*,rele_stat_*/;
extern char bRXIN0;


char cntrl_plazma;
extern char bOUT_FREE2;
extern char /*av_net,*//*av_bat[2],*/av_bps[12],av_inv[6],av_dt[4],av_sk[4];

char content[63];

//const short ptr_bat_zar_cnt[2]={EE_ZAR1_CNT,EE_ZAR2_CNT};


//unsigned short YEAR_AVZ,MONTH_AVZ,DATE_AVZ,HOUR_AVZ,MIN_AVZ,SEC_AVZ;


//**********************************************
//Самокалиброввка
extern signed short samokalibr_cnt;

//**********************************************
//Сообщения
extern char mess[MESS_DEEP],mess_old[MESS_DEEP],mess_cnt[MESS_DEEP];
extern short mess_par0[MESS_DEEP],mess_par1[MESS_DEEP],mess_data[2];












//-----------------------------------------------
//Климатконтроль и вентиляторы
signed short main_vent_pos;
signed char t_box_cnt=0;
enum_mixer_vent_stat mixer_vent_stat=mvsOFF;
INT_BOX_TEMPER ibt;
enum_tbatdisable_stat tbatdisable_stat=tbdsON;
enum_tloaddisable_stat tloaddisable_stat=tldsON;
enum_av_tbox_stat av_tbox_stat=atsOFF;
signed short av_tbox_cnt;
char tbatdisable_cmnd=20,tloaddisable_cmnd=22;
short tbatdisable_cnt,tloaddisable_cnt;
#ifdef UKU_KONTUR
short t_box_vent_on_cnt;
short t_box_warm_on_cnt;
enum_vent_stat vent_stat_k=vsON;
enum_warm_stat warm_stat_k=wsON;
#endif

#ifdef UKU_TELECORE2015
short t_box_vent_on_cnt;
short t_box_warm_on_cnt;
short t_box_vvent_on_cnt;
enum_vent_stat vent_stat_k=vsON,vvent_stat_k=vsON;
enum_warm_stat warm_stat_k=wsON;
signed short TELECORE2015_KLIMAT_WARM_ON_temp;
#endif

//-----------------------------------------------
//Состояние контролируемых автоматов нагрузки 
enum_avt_stat avt_stat[12],avt_stat_old[12];

//short sys_plazma,sys_plazma1;

char snmp_plazma;


short plazma_but_an;

char bCAN_OFF;


char max_net_slot;

//-----------------------------------------------
//Показания АЦП на плате измерения тока батареи
signed long ibat_metr_buff_[2];
short bIBAT_SMKLBR;


//-----------------------------------------------
//Климатконтроль TELECORE2015	
//#ifdef UKU_TELECORE2015
signed short TELECORE2015_KLIMAT_WARM_SIGNAL;
signed short TELECORE2015_KLIMAT_VENT_SIGNAL;
signed short TELECORE2015_KLIMAT_WARM_ON;
signed short TELECORE2015_KLIMAT_WARM_OFF;
signed short TELECORE2015_KLIMAT_CAP;
signed short TELECORE2015_KLIMAT_VENT_ON;
signed short TELECORE2015_KLIMAT_VENT_OFF;
signed short TELECORE2015_KLIMAT_VVENT_ON;
signed short TELECORE2015_KLIMAT_VVENT_OFF;
//#endif  
//-----------------------------------------------
//Управление низкоприоритетной нагрузкой
signed short npn_tz_cnt;
enum_npn_stat npn_stat=npnsON;


char ips_bat_av_vzvod=0;
char ips_bat_av_stat=0;

char rel_warm_plazma;
char can_byps_plazma0,can_byps_plazma1;

char bCAN_INV;
char plazma_can_inv[3];

unsigned short bat_drv_rx_cnt;
char bat_drv_rx_buff[512];
char bat_drv_rx_in;

short plazma_bat_drv0,plazma_bat_drv1,bat_drv_cnt_cnt;
short can_plazma;

//-----------------------------------------------
//Ускоренный заряд
signed short speedChrgCurr;			//максимальный ток ускоренного заряда, отображение из ЕЕПРОМ
signed short speedChrgVolt;			//максимальное напряжение ускоренного заряда, отображение из ЕЕПРОМ
signed short speedChrgTimeInHour; 		//максимальное время ускоренного заряда в часах, отображение из ЕЕПРОМ
signed short speedChrgAvtEn;	    		//Автоматическое включение Ускоренного заряда включено/выключено
signed short speedChrgDU;	    		//Просадка напряжения необходимая для включения ускоренного заряда
signed short speedChIsOn;			//Текущее состояние ускоренного заряда вкл/выкл
signed long  speedChTimeCnt;			//Счетчик времени прямой ускоренного заряда
signed short speedChrgBlckSrc;		//Источник сигнала блокировки, 0-выкл., 1-СК1, 2-СК2
signed short speedChrgBlckLog;		//Логика сигнала блокировки, 1 - блокировка по замкнутому СК, 0 - по разомкнутому
signed short speedChrgBlckStat;		//Сигнал блокировки для выравнивающего и ускоренного заряда.
char  	   speedChrgShowCnt;		//Счетчик показа информационного сообщения

//-----------------------------------------------
//Алгоритм содержания батареи 	
//signed short TELECORE2017_USTART;		//Напряжение включения
//signed short TELECORE2017_ULINECC;		//Напряжение содержания из установок
signed short LI_UNECC_;					//Напряжение содержания мгновенное, с учетом аварий батареи
signed short LI_AVAR_CNT;				//Счетчик аварийности батареи для снижения напряжения содержания

//signed short TELECORE2017_AVAR_CNT;		//Счетчик аварийности батареи для снижения напряжения содержания
//signed short TELECORE2017_Q;			//Заряд батари (%) при котором переходим с тока IZMAX1 на IZMAX2
//signed short TELECORE2017_IZMAX1;		//Максимальный ток заряда батареи при разряженной батарее(заряд < TELECORE2017_Q)
//signed short TELECORE2017_IZMAX2;	   	//Максимальный ток заряда батареи при заряженной батарее(заряд >= TELECORE2017_Q)
//signed short TELECORE2017_K1;			//Шаг регулирования(ед/с) при Uвыпр<(Uбат-2В)
//signed short TELECORE2017_K2;			//Шаг регулирования(ед/с) при Uвыпр>(Uбат-2В) и отсутствии токов батарей
//signed short TELECORE2017_K3;			//Шаг регулирования(ед/с) при токе батарей в интервале 0-70% от Izmax
//signed short TELECORE2017_T4;			//Период регулирования(сек) еденичными шагами при токе батарейц в интервале 70-110%от Izmax 
//#endif 
//-----------------------------------------------
//Управление низкоприоритетной нагрузкой

//-----------------------------------------------
//Блокировка ИПС
signed short ipsBlckSrc;
signed short ipsBlckLog;
signed short ipsBlckStat;


//-----------------------------------------------
//Контроль выходного напряжения
signed short outVoltContrHndlCnt;		//Счетчик, считает в плюс в случае выполнения условия аварии
signed short outVoltContrHndlCnt_;		//Счетчик, считает в плюс в случае отсутствия выполнения условия аварии
char uout_av;


short plazma_numOfCells;
short plazma_numOfTemperCells;
short plazma_numOfPacks;




char plazma_ztt[2];


//-----------------------------------------------
void rtc_init (void) 
{
LPC_RTC->CCR=0x11;
}

//-----------------------------------------------
static void timer_poll () 
{
if (SysTick->CTRL & 0x10000) 
     {
     timer_tick ();
     tick = __TRUE;
     }
}

//-----------------------------------------------
void inv_search(void)
{
char i;

first_inv_slot=8;
for(i=0;i<12;i++)
	{
	if(bps[i]._device==dINV)
		{
		first_inv_slot=i;
		break;

		}
	}
}

//-----------------------------------------------
signed short abs_pal(signed short in)
{
if(in<0)return -in;
else return in;
}

//-----------------------------------------------
void init_ETH(void)
{


}


//-----------------------------------------------
void ADC_IRQHandler(void) {
LPC_ADC->ADCR &=  ~(7<<24);


}

//-----------------------------------------------
void def_set(int umax__,int ub0__,int ub20__,int usign__,int imax__,int uob__,int numi,int _uvz)
{
;
//lc640_write_int(EE_NUMIST,numi);
//lc640_write_int(EE_NUMINV,0);
////lc640_write_int(EE_NUMDT,0);
////lc640_write_int(EE_NUMSK,0);
//lc640_write_int(EE_MAIN_IST,0);
//lc640_write_int(EE_PAR,1);
//lc640_write_int(EE_TBAT,60);
//lc640_write_int(EE_UMAX,umax__);
//lc640_write_int(EE_DU,ub20__/2);
//lc640_write_int(EE_UB0,ub0__);
//lc640_write_int(EE_UB20,ub20__);
//lc640_write_int(EE_TSIGN,70);
//lc640_write_int(EE_TMAX,80);
////lc640_write_int(EE_C_BAT,180);
//lc640_write_int(EE_USIGN,usign__);
//lc640_write_int(EE_UMN,187);
//lc640_write_int(EE_ZV_ON,0);
//lc640_write_int(EE_IKB,10);
////lc640_write_int(EE_KVZ,1030);
//lc640_write_int(EE_UVZ,_uvz);
//lc640_write_int(EE_IMAX,imax__);
//lc640_write_int(EE_IMIN,(imax__*8)/10);
////lc640_write_int(EE_APV_ON,apvON);
//lc640_write_int(EE_APV_ON1,apvON);
//lc640_write_int(EE_APV_ON2,apvON);
//lc640_write_int(EE_APV_ON2_TIME,1);
//lc640_write_int(EE_IZMAX,160);
//lc640_write_int(EE_U0B,uob__);
//lc640_write_int(EE_TZAS,3);
//lc640_write_int(EE_TBATMAX,50);  
//lc640_write_int(EE_TBATSIGN,40);
//lc640_write_int(EE_MNEMO_ON,mnON);
//lc640_write_int(EE_MNEMO_TIME,30);	
//lc640_write_int(EE_AV_OFF_AVT,1);
////lc640_write_int(EE_APV_ON1,apvOFF);



//lc640_write_int(EE_TBOXMAX,70);
//lc640_write_int(EE_TBOXVENTMAX,60);
//lc640_write_int(EE_TBOXREG,25);
//lc640_write_int(EE_TLOADDISABLE,80);
//lc640_write_int(EE_TLOADENABLE,70);
//lc640_write_int(EE_TBATDISABLE,91);
//lc640_write_int(EE_TBATENABLE,80);

//lc640_write_int(ADR_SK_SIGN[0],0);
//lc640_write_int(ADR_SK_REL_EN[0],0);
//lc640_write_int(ADR_SK_LCD_EN[0],0xffff);

//lc640_write_int(ADR_SK_SIGN[1],0);
//lc640_write_int(ADR_SK_REL_EN[1],0);
//lc640_write_int(ADR_SK_LCD_EN[1],0xffff);

//lc640_write_int(ADR_SK_SIGN[2],0);
//lc640_write_int(ADR_SK_REL_EN[2],0);
//lc640_write_int(ADR_SK_LCD_EN[2],0xffff);

//lc640_write_int(ADR_SK_SIGN[3],0);
//lc640_write_int(ADR_SK_REL_EN[3],0);
//lc640_write_int(ADR_SK_LCD_EN[3],0xffff);

//lc640_write_int(EE_UBM_AV,10);

//lc640_write_int(EE_POS_VENT,11);
}


//-----------------------------------------------
void def_ips_set(short voltage)
{
if(voltage==24)
	{
	def_set(300,voltage,voltage,22,150,240,7,0);
	}
if(voltage==48)
	{
	def_set(600,voltage,voltage,44,100,480,7,0);
	}
if(voltage==60)
	{
	def_set(750,voltage,voltage,55,100,600,7,0);
	}

if(voltage==220)
	{
	def_set(2450,2366,2315,187,100,2200,2,2346);

	//lc640_write_int(EE_NUMIST,2);
	//lc640_write_int(EE_NUMINV,0);
////lc640_write_int(EE_NUMDT,0);
////lc640_write_int(EE_NUMSK,0);
	//lc640_write_int(EE_MAIN_IST,0);
	//lc640_write_int(EE_PAR,1);
	//lc640_write_int(EE_TBAT,60);
	//lc640_write_int(EE_UMAX,2450);
	//lc640_write_int(EE_DU,2315/2);
	//lc640_write_int(EE_UB0,2366);
	//lc640_write_int(EE_UB20,2315);
	//lc640_write_int(EE_TSIGN,70);
	//lc640_write_int(EE_TMAX,80);
////lc640_write_int(EE_C_BAT,180);
	//lc640_write_int(EE_USIGN,187);
	//lc640_write_int(EE_UMN,187);
	//lc640_write_int(EE_ZV_ON,0);
	//lc640_write_int(EE_IKB,20);
////lc640_write_int(EE_KVZ,1030);
	//lc640_write_int(EE_UVZ,2346);
	//lc640_write_int(EE_IMAX,80);
	//lc640_write_int(EE_IMIN,50);
////lc640_write_int(EE_APV_ON,apvON);
	//lc640_write_int(EE_APV_ON1,apvON);
	//lc640_write_int(EE_APV_ON2,apvON);
	//lc640_write_int(EE_APV_ON2_TIME,1);
	//lc640_write_int(EE_IZMAX,160);
	//lc640_write_int(EE_U0B,2200);
	//lc640_write_int(EE_TZAS,3);
	//lc640_write_int(EE_TBATMAX,50);  
	//lc640_write_int(EE_TBATSIGN,40);
	//lc640_write_int(EE_MNEMO_ON,mnON);
	//lc640_write_int(EE_MNEMO_TIME,30);	
	//lc640_write_int(EE_AV_OFF_AVT,1);
////lc640_write_int(EE_APV_ON1,apvOFF);



	//lc640_write_int(EE_TBOXMAX,70);
	//lc640_write_int(EE_TBOXVENTMAX,60);
	//lc640_write_int(EE_TBOXREG,25);
	//lc640_write_int(EE_TLOADDISABLE,80);
	//lc640_write_int(EE_TLOADENABLE,70);
	//lc640_write_int(EE_TBATDISABLE,91);
	//lc640_write_int(EE_TBATENABLE,80);

	//lc640_write_int(ADR_SK_SIGN[0],0);
	//lc640_write_int(ADR_SK_REL_EN[0],0);
	//lc640_write_int(ADR_SK_LCD_EN[0],0xffff);

	//lc640_write_int(ADR_SK_SIGN[1],0);
	//lc640_write_int(ADR_SK_REL_EN[1],0);
	//lc640_write_int(ADR_SK_LCD_EN[1],0xffff);

	//lc640_write_int(ADR_SK_SIGN[2],0);
	//lc640_write_int(ADR_SK_REL_EN[2],0);
	//lc640_write_int(ADR_SK_LCD_EN[2],0xffff);

	//lc640_write_int(ADR_SK_SIGN[3],0);
	//lc640_write_int(ADR_SK_REL_EN[3],0);
	//lc640_write_int(ADR_SK_LCD_EN[3],0xffff);

	//lc640_write_int(EE_UBM_AV,10);

	//lc640_write_int(EE_POS_VENT,11);


	//lc640_write_int(EE_DU,2315-1870);
	//lc640_write_int(EE_U_AVT,2200);
	//lc640_write_int(EE_IZMAX,20);
	//lc640_write_int(EE_AUSW_MAIN,22033);
	//lc640_write_int(EE_PAR,1);
	//lc640_write_int(EE_MNEMO_ON,mnOFF);
	}

//lc640_write_int(ADR_EE_BAT_IS_ON[0],bisOFF);
//lc640_write_int(ADR_EE_BAT_DAY_OF_ON[0],LPC_RTC->DOM);
//lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[0],LPC_RTC->MONTH);
//lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[0],LPC_RTC->YEAR);
//lc640_write_int(ADR_EE_BAT_C_NOM[0],0);
//lc640_write_int(ADR_EE_BAT_RESURS[0],0);

//lc640_write_int(ADR_EE_BAT_IS_ON[1],bisOFF);
//lc640_write_int(ADR_EE_BAT_DAY_OF_ON[1],LPC_RTC->DOM);
//lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[1],LPC_RTC->MONTH);
//lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[1],LPC_RTC->YEAR);
//lc640_write_int(ADR_EE_BAT_C_NOM[1],0);
//lc640_write_int(ADR_EE_BAT_RESURS[1],0);
}

//-----------------------------------------------
void can_reset_hndl(void)
{


}



//-----------------------------------------------
void parol_init(void)
{
parol[0]=0;
parol[1]=0;
parol[2]=0;
sub_ind=0;
}

//-----------------------------------------------
void bitmap_hndl(void)
{
short x,ii,i;
unsigned int ptr_bitmap;
static char ptr_cnt,ptr_cnt1,ptr_cnt2,ptr_cnt3,ptr_cnt4;

for(ii=0;ii<488;ii++)
	{
	lcd_bitmap[ii]=0x00;
	}

	{
	for(i=0;i<4;i++)
		{
		ptr_bitmap=122*(unsigned)i;
		for(x=(20*i);x<((20*i)+20);x++)
	 		{
			lcd_bitmap[ptr_bitmap++]=caracter[(unsigned)lcd_buffer[x]*6];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+1];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+2];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+3];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+4];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+5];
			} 
		}
	}	
}

//-----------------------------------------------
void ind_hndl(void)
{
//const char* ptr;
const char* ptrs[50];
const char* sub_ptrs[50];
static char sub_cnt,sub_cnt1;
char i,sub_cnt_max;
char ii_;				  
static char ii_cnt,cnt_ind_bat;


	   
sub_cnt_max=5;
i=0;
	      




//cnt_of_wrks=0;
//for(i=0;i<NUMIST;i++)
 //    {
//     if(bps[i]._state==bsWRK)cnt_of_wrks++;
  //   }


sub_cnt1++;	
if(sub_cnt1>=20)
	{
	sub_cnt1=0;
	sub_cnt++;
	if(sub_cnt>=sub_cnt_max)
		{
		sub_cnt=0;
		}
	}


if(ind==iMn_IP55)
	{
	ptrs[0]	=	"                    ";

		ptrs[0]	=	"  В работе    rист. ";


	 
     i=0;
 	
 	ptrs[1]="Uбz=   ]В Iбz=    @А";
    ptrs[2]="Uн=    #В Iн=     $А";
    ptrs[3]=" 0%:0^:0& 0</>  /0{ ";
     
 	ptrs[4]=										" Батарея            ";
  //  ptrs[5]=										" Батарея N2         ";
    
								
	ptrs[4+NUMBAT_IP55]=  						" БПС N1             ";
    ptrs[5+NUMBAT_IP55]=  						" БПС N2             ";
    ptrs[6+NUMBAT_IP55]=  						" DC/DC N1           ";
    ptrs[7+NUMBAT_IP55]=  						" DC/DC N2           ";
    ptrs[8+NUMBAT_IP55]= 						" БПС N5             ";
    ptrs[9+NUMBAT_IP55]= 						" БПС N6             ";
    ptrs[10+NUMBAT_IP55]= 						" БПС N7             ";
    ptrs[11+NUMBAT_IP55]= 						" БПС N8             ";
    ptrs[12+NUMBAT_IP55]= 						" БПС N9             ";
    ptrs[13+NUMBAT_IP55]= 						" БПС N10            ";
    ptrs[14+NUMBAT_IP55]= 						" БПС N11            ";
    ptrs[15+NUMBAT_IP55]= 						" БПС N12            ";

    ptrs[4+NUMIST+NUMBAT_IP55+2]= 				" Сеть               "; 
    ptrs[5+NUMIST+NUMBAT_IP55+2]= 				" Нагрузка           "; 
    ptrs[6+NUMIST+NUMBAT_IP55+2]= 				" Внешние датчики    "; 
//	ptrs[6+NUMIST+NUMBAT_IP55]= 				" Спецфункции    	 ";
    ptrs[7+NUMIST+NUMBAT_IP55+2]= 				" Установки          "; 
    ptrs[8+NUMIST+NUMBAT_IP55+2]= 				" Журнал событий     "; 
    ptrs[9+NUMIST+NUMBAT_IP55+2]= 				" Выход              "; 
	ptrs[10+NUMIST+NUMBAT_IP55+2]=				" Тест               ";

    if(sub_ind==0)index_set=0;
	else if((index_set-sub_ind)>2)index_set=sub_ind+2;
	else if(sub_ind>index_set)index_set=sub_ind;
	
	if(sub_cnt<5)bgnd_par(ptrs[0],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
	else bgnd_par(sub_ptrs[sub_cnt-5],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
	

	/*if((!NUMBAT)&&(!NUMINV)) {
		int2lcd(byps._Uout,'#',1);
     	int2lcd(byps._Iout,'$',1); 
	} else */{
		int2lcd(load_U,'#',1);
 		int2lcd(load_I,'$',1);
	}
 	
	int2lcd(LPC_RTC->HOUR,'%',0);
	int2lcd(LPC_RTC->MIN,'^',0);
	int2lcd(LPC_RTC->SEC,'&',0);
	int2lcd(LPC_RTC->DOM,'<',0);
	int2lcd(LPC_RTC->YEAR,'{',0); 
	sub_bgnd(sm_mont[LPC_RTC->MONTH],'>',0);

 	if(bFL2)
 	     {
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     }

	if((index_set)&&(sub_ind))
	     {
	     if(index_set==sub_ind)lcd_buffer[60]=1;
	     else if((index_set-sub_ind)==1)lcd_buffer[40]=1;
	     else if((index_set-sub_ind)==2)lcd_buffer[20]=1;
	     }	
		
	cnt_ind_bat++;
	if(cnt_ind_bat>=(NUMBAT_IP55*40)) cnt_ind_bat=0;
	
/*	if((AUSW_MAIN==2400)||(AUSW_MAIN==4800)||(AUSW_MAIN==6000)||
		(AUSW_MAIN==2403)||(AUSW_MAIN==4803)||(AUSW_MAIN==6003))sub_bgnd("                    ",'z',-2);
	else*/
	 
	if(NUMBAT_IP55==0)sub_bgnd(" Работа без батарей ",'z',-2);
	else
		{
		int2lcd((cnt_ind_bat/40)+1/*+(!(BAT_IS_ON[0]==bisON))*/,'z',0);
		int2lcd((cnt_ind_bat/40)+1/*+(!(BAT_IS_ON[0]==bisON))*/,'z',0);
		if((NUMBAT_IP55==1)/*&&(!(BAT_IS_ON[0]==bisON))*/)
			{
			if(lakb[0]._communicationFullErrorStat)
				{
				sub_bgnd("Err",']',-1);
				sub_bgnd("Err",'@',-1);
				}
			else
				{
				int2lcd(lakb[0]._tot_bat_volt,']',1);
				int2lcd_mmm(lakb[0]._ch_curr,'@',2);
				}
			}
		else
			{
			if(lakb[cnt_ind_bat/40]._communicationFullErrorStat)
				{
				sub_bgnd("Err",']',-1);
				sub_bgnd("Err",'@',-1);
				}
			else
				{
				int2lcd(lakb[cnt_ind_bat/40]._tot_bat_volt,']',1);
				int2lcd_mmm(lakb[cnt_ind_bat/40]._ch_curr,'@',2);
			 	}
			}		
		}


 
//	int2lcdyx(plazma_bat_drv1,0,6,0);
//	int2lcdyx(lakb[0]._s_o_c_abs,0,19,0);
//	int2lcdyx(lakb[0]._ch_curr,0,12,0); 
	//int2lcdyx(bat_drv_rx_cnt,0,19,0);
//	int2lcdyx(bps[i+20]._buff[14],0,19,0);
/*	int2lcdyx(makb[2]._cnt,0,10,0);*/
	//int2lcdyx(first_inv_slot,0,19,0);	
 	//int2lcdyx(npn_tz_cnt,0,7,0);
	//char2lcdbyx(GET_REG(LPC_GPIO0->FIOPIN,4,8),0,19);
	//int2lcdyx(//lc640_read_int(ADR_EE_BAT_IS_ON[0]),0,4,0);
	//int2lcdyx(//lc640_read_int(ADR_EE_BAT_IS_ON[1]),0,9,0);

	
	//	long2lcdyx_mmm(power_summary_tempo,0,9,0);
	//long2lcdyx_mmm(power_current_tempo,0,19,0);
	
	  
	 
	//int2lcdyx(bCoslightPacketIn,0,3,0);
	//int2lcdyx(coslightBatteryInBuffPtr_plazma,0,7,0);
	//int2lcdyx(ad7705_buff_[0],0,9,0);
	//int2lcdyx(ad7705_buff_[1],0,15,0);
/*	int2lcdyx(plazma_numOfCells,0,19,0); */
	}

#ifndef _DEBUG_
else if (ind==iBat)
	{
	if(bat[sub_ind1]._av&1/*&0x01*/)
		{
		if(bFL2)bgnd_par("       АВАРИЯ!        ",
		                 "     Батарея №#       ",
		                 "    не подключена     ",sm_);
		else bgnd_par(sm_,sm_,sm_,sm_);
		int2lcd(sub_ind1+1,'#',0);
		}               
	else
		{
		if(bat[sub_ind1]._Ib>0)
		     {
		     ptrs[1]="    заряжается      ";
		     ptrs[4]=" Iзар=       #А     ";
		     }
		else
		     {
		     ptrs[1]=  "   разряжается      ";
		     ptrs[4]=  " Iразр=      #А     ";
		     }	
		ptrs[2]=       " Uбат=            $В";
          ptrs[3]=       " Uбат.с.т.=       zВ";
		if(UBM_AV)
		ptrs[3]=       " Uбат.с.т.=(^%)   zВ";
		
		if(bat[sub_ind1]._nd)ptrs[5]="    ДТ. неисправен  ";
		else ptrs[5]="   tбат =   ?°C     ";
		ptrs[6]="   Заряд=    w%     ";
		ptrs[7]="   Cбат=     QА*ч   ";
		ptrs[8]=sm_exit;
 
		bgnd_par("    БАТАРЕЯ N@      ",
		          ptrs[sub_ind+1],ptrs[sub_ind+2],ptrs[sub_ind+3]);
	     
	     int2lcd(sub_ind1+1,'@',0);
	     //int2lcd(bat[sub_ind1]._Ub,'$',1);
		 #ifdef UKU_220
		 int2lcd(bat[sub_ind1]._Ub/10,'$',0);
		 int2lcd(bat[sub_ind1]._Ubm/10,'z',0);
		 #else 
		 int2lcd(bat[sub_ind1]._Ub,'$',1);
         int2lcd(bat[sub_ind1]._Ubm,'z',1);
		 #endif
	     int2lcd_mmm(abs(bat[sub_ind1]._Ib),'#',2);
	     int2lcd_mmm(bat[sub_ind1]._Tb,'?',0);
	     int2lcd(bat[sub_ind1]._zar,'w',0);
		int2lcd(bat[sub_ind1]._dUbm,'^',0);
	     if(BAT_C_REAL[sub_ind1]==0x5555)sub_bgnd("------",'Q',-1);
	     else int2lcd(BAT_C_REAL[sub_ind1],'Q',1);
	     if(sub_ind==5)lcd_buffer[60]=1;
		 //int2lcdyx(bat[sub_ind1]._cnt_as,0,4,0);
		 
		}
	} 

else if (ind==iBat_simple)
	{
	if(bat[sub_ind1]._av&1)
		{
		if(bFL2)bgnd_par("       АВАРИЯ!        ",
		                 "     Батарея №#       ",
		                 "    не подключена     ",sm_);
		else bgnd_par(sm_,sm_,sm_,sm_);
		int2lcd(sub_ind1+1,'#',0);
		}               
	else
		{
		if(bat[sub_ind1]._Ib>0)
		     {
		     ptrs[1]="    заряжается      ";
		     ptrs[3]=" Iзар=       #А     ";
		     }
		else
		     {
		     ptrs[1]=  "   разряжается      ";
		     ptrs[3]=  " Iразр=      #А     ";
		     }	
		ptrs[2]=       " Uбат=            $В";
         // ptrs[3]=       " Uбат.с.т.=       zВ";
	//	if(UBM_AV)
	//	ptrs[3]=       " Uбат.с.т.=(^%)   zВ";
		
		if(bat[sub_ind1]._nd)ptrs[4]="    ДТ. неисправен  ";
		else ptrs[4]="   tбат =   ?°C     ";
		ptrs[5]="   Заряд=    w%     ";
		ptrs[6]="   Cбат=     QА*ч   ";
		ptrs[7]=sm_exit;
 
		bgnd_par("    БАТАРЕЯ N@      ",
		          ptrs[sub_ind+1],ptrs[sub_ind+2],ptrs[sub_ind+3]);
	     
	     int2lcd(sub_ind1+1,'@',0);
	     int2lcd(bat[sub_ind1]._Ub,'$',1);
          //int2lcd(bat[sub_ind1]._Ubm,'z',1);
	     int2lcd_mmm(abs(bat[sub_ind1]._Ib),'#',2);
	     int2lcd_mmm(bat[sub_ind1]._Tb,'?',0);
	     int2lcd(bat[sub_ind1]._zar,'w',0);
		//int2lcd(bat[sub_ind1]._dUbm,'^',0);
	     if(BAT_C_REAL[sub_ind1]==0x5555)sub_bgnd("------",'Q',-1);
	     else int2lcd(BAT_C_REAL[sub_ind1],'Q',1);
	     if(sub_ind==4)lcd_buffer[60]=1;
		}
	} 

else if (ind==iBat_li)
	{

		{
		ptrs[0]=								"   Тип неизвестен   ";
		if(lakb[sub_ind1]._s_o_h>=500)ptrs[0]=  "      48V50Ah       ";
		if(lakb[sub_ind1]._s_o_h>=400)ptrs[0]=  "      48V40Ah       ";
		if(lakb[sub_ind1]._ch_curr>0)
		     {
		     ptrs[1]="    заряжается      ";
		     ptrs[3]=" Iзар=       #А     ";
		     }
		else
		     {
		     ptrs[1]=  "   разряжается      ";
		     ptrs[3]=  " Iразр=      #А     ";
		     }	
		ptrs[2]=       " Uбат =    $В       ";
		ptrs[4]=		" t1   =    ?°C      ";
		ptrs[5]=		" t2   =    (°C      ";
		ptrs[6]=		" t3   =    )°C      ";
		ptrs[7]=		" t4   =    +°C      ";
		ptrs[8]=		" tокр.=    [°C      ";
		ptrs[9]=		" tсил.=    ]°C      ";
		ptrs[10]=		" SOC  =    wA*ч     ";
		ptrs[11]=		" SOH  =    >A*ч     ";
		ptrs[12]=		" Iшунт.=   <A       ";
		ptrs[13]=sm_exit;

		if(lakb[sub_ind1]._communicationFullErrorStat==1)
			{
			ptrs[0]=	" ОТСУТСТВУЕТ СВЯЗЬ  ";
			ptrs[1]=	"      С ПЛАТОЙ      ";
			ptrs[2]=	"    РАСШИРЕНИЯ!!!   ";

			bgnd_par(	"    Батарея N@      ",
						ptrs[0],
						ptrs[1],
						ptrs[2]);
			}

		else if(lakb[sub_ind1]._communicationFullErrorStat==2)
			{
			ptrs[0]=	" ОТСУТСТВУЕТ СВЯЗЬ  ";
			ptrs[1]=	"      С BMS!!!      ";
			ptrs[2]=	"                    ";

			bgnd_par(	"    Батарея N@      ",
						ptrs[0],
						ptrs[1],
						ptrs[2]);
			}

 		else 
			{
			bgnd_par(	"    Батарея N@      ",
						ptrs[sub_ind+0],
						ptrs[sub_ind+1],
						ptrs[sub_ind+2]);
		     

		    int2lcd(lakb[sub_ind1]._tot_bat_volt,'$',1);
			int2lcd_mmm(abs(lakb[sub_ind1]._ch_curr),'#',2);
		    int2lcd_mmm(lakb[sub_ind1]._cell_temp_1,'?',0);
			int2lcd_mmm(lakb[sub_ind1]._cell_temp_2,'(',0);
			int2lcd_mmm(lakb[sub_ind1]._cell_temp_3,')',0);
			int2lcd_mmm(lakb[sub_ind1]._cell_temp_4,'+',0);
			int2lcd_mmm(lakb[sub_ind1]._cell_temp_ambient,'[',0);
			int2lcd_mmm(lakb[sub_ind1]._cell_temp_power,']',0);
		    int2lcd(lakb[sub_ind1]._s_o_c,'w',1);
			if(sub_ind==10)lcd_buffer[60]=1;
			int2lcd(lakb[sub_ind1]._s_o_h,'>',1);
			int2lcd_mmm(bat[sub_ind1]._Ib/10,'<',1);
			}
		int2lcd(sub_ind1+1,'@',0);


		}
	}

else if (ind==iBat_SacredSun)
	{
		{
		if(bat[sub_ind1]._Ib/*lakb[sub_ind1]._ch_curr*/>0)
		     {
		     ptrs[1]="    заряжается      ";
		     ptrs[3]=" Iзар=       #А     ";
		     }
		else
		     {
		     ptrs[1]=  "   разряжается      ";
		     ptrs[3]=  " Iразр=      #А     ";
		     }	
		ptrs[2]=       " Uбат =    $В       ";
		ptrs[3]=		" tбат =    ?°C      ";
		ptrs[4]=		" SOC  =    w%       ";
		ptrs[5]=		" SOH  =    >%       ";
		ptrs[6]=sm_exit;


 
		bgnd_par(		"    БАТАРЕЯ N@      ",
					"   1 x FP16S4810A   ",
					ptrs[sub_ind+1],ptrs[sub_ind+2]);
	     
	     int2lcd(sub_ind1+1,'@',0);
	     int2lcd(bat[sub_ind1]._Ub,'$',1);//int2lcd(lakb[sub_ind1]._tot_bat_volt/10,'$',1);
		/*if(lakb[sub_ind1]._ch_curr>0)
			{
			int2lcd_mmm(lakb[sub_ind1]._ch_curr,'#',2);
			}
		else 
			{
			int2lcd_mmm(lakb[sub_ind1]._dsch_curr,'#',2);
			}*/
		int2lcd_mmm(abs(bat[sub_ind1]._Ib),'#',2);
	     int2lcd_mmm(bat[sub_ind1]._Tb,'?',0);
	     //int2lcd_mmm(lakb[sub_ind1]._max_cell_temp,'?',0);
	     int2lcd(lakb[sub_ind1]._s_o_c,'w',0);
		int2lcd((short)(((long)lakb[sub_ind1]._rat_cap*(long)lakb[sub_ind1]._s_o_h)/1000L),'Q',1);
	     if(sub_ind==8)lcd_buffer[60]=1;

		int2lcd(lakb[sub_ind1]._rat_cap,'Q',1);
		//int2lcd(lakb[sub_ind1]._s_o_c,'w',0);
		int2lcd(lakb[sub_ind1]._c_c_l_v/10,'<',1);
		int2lcd(lakb[sub_ind1]._s_o_h,'>',0);
		int2lcd(lakb[sub_ind1]._r_b_t,'[',1);

		//int2lcdyx(lakb[sub_ind1]._battCommState,0,19,0);
		}
	}

else if (ind==iBat_universe)
	{
	if(BAT_TYPE==0)	//Обычная свинцово-кислотная батарея
		{			//TODO следующий блок написан только для батареи без средней точки. Дописать автоизменение если в системе есть контроль СТ
		if(bat[0]._av&1)
			{
			if(bFL2)bgnd_par(	"       АВАРИЯ!        ",
			                 	"     Батарея №#       ",
			                 	"    не подключена     ",
							sm_);
			else bgnd_par(sm_,sm_,sm_,sm_);
			int2lcd(1,'#',0);
			}               
		else
			{
			if(bat[0]._Ib>0)
			     {
			     ptrs[1]=		"    заряжается      ";
			     ptrs[3]=		" Iзар=       #А     ";
			     }
			else
			     {
			     ptrs[1]=  "   разряжается      ";
			     ptrs[3]=  " Iразр=      #А     ";
			     }	
			ptrs[2]=       " Uбат=            $В";
			
			if(bat[0]._nd)ptrs[4]="    ДТ. неисправен  ";
			else ptrs[4]="   tбат =   ?°C     ";
			ptrs[5]="   Заряд=    w%     ";
			ptrs[6]="   Cбат=     QА*ч   ";
			ptrs[7]=sm_exit;
	 
			bgnd_par("    БАТАРЕЯ N@      ",
			          ptrs[sub_ind+1],
					ptrs[sub_ind+2],
					ptrs[sub_ind+3]);
		     
		     int2lcd(1,'@',0);
		     int2lcd(bat[0]._Ub,'$',1);
		     int2lcd_mmm(abs(bat[0]._Ib),'#',2);
		     int2lcd_mmm(bat[0]._Tb,'?',0);
		     int2lcd(bat[0]._zar,'w',0);
		     if(BAT_C_REAL[0]==0x5555)sub_bgnd("------",'Q',-1);
		     else int2lcd(BAT_C_REAL[0],'Q',1);
		     if(sub_ind==4)lcd_buffer[60]=1;
			}
		} 
	else if(BAT_TYPE==1)//Батарея	COSLIGHT
		{
		char *ptr;

		/*if(bat[sub_ind1]._av&1)
			{
			if(bFL2)bgnd_par("       АВАРИЯ!        ",
			                 "     Батарея №#       ",
			                 "    не подключена     ",sm_);
			else bgnd_par(sm_,sm_,sm_,sm_);
			int2lcd(sub_ind1+1,'#',0);
			}               
		else*/	
		if(li_bat._batStat==bsOFF)
			{
			if(bFL)
				{
				bgnd_par(		"  Сигналы телеметрии  ",
			                 	"     не передаются    ",
			                 	"                      ",
							sm_);
				}
			else 
				{
				if(li_bat._Ib>0)
					{
					ptrs[0]=	"    заряжается      ";
					ptrs[2]=	" Iзар=       #А     ";
					}
				else
				     {
				     ptrs[0]=  "   разряжается      ";
				     ptrs[2]=  " Iразр=      #А     ";
				     }	
				ptrs[1]=       " Uбат=            $В";
			     if(li_bat._nd)
					ptrs[3]=	"    ДТ. неисправен  ";
				else ptrs[3]=	"   tбат =   ?°C     ";
					ptrs[4]=sm_exit;
			 
				bgnd_par(		"      БАТАРЕЯ       ",
					          ptrs[sub_ind],
							ptrs[sub_ind+1],
							ptrs[sub_ind+2]);
				     
				int2lcd(li_bat._Ub,'$',1);
			     
				int2lcd_mmm(abs(li_bat._Ib),'#',1);
				int2lcd_mmm(li_bat._Tb,'?',0);
				} 

			}               
		else
			{
			if(li_bat._Ib>0)
			     {
			     ptrs[1]="    заряжается      ";
			     ptrs[3]=" Iзар=       #А     ";
			     }
			else
			     {
			     ptrs[1]=  "   разряжается      ";
			     ptrs[3]=  " Iразр=      #А     ";
			     }	
			ptrs[2]=       " Uбат =    $В       ";
			ptrs[4]=		" tбат =    ?°C      ";
			ptrs[5]=		" SOC  =    w%       ";
			ptrs[6]=		" Cбат =    QА*ч     ";
			ptrs[7]=		" SOH  =    >%       ";
			ptrs[8]=		" Imax =    <A       ";
			ptrs[9]=		" RBT  =    [ч       ";
			ptrs[10]=sm_exit;
	
			ptr=			"      GYFP4875      ";
			if(li_bat._ratCap==50) 
			ptr=			"      GYFP4850      ";		    	

	 
			bgnd_par(		"      БАТАРЕЯ       ",
						ptr,
						ptrs[sub_ind+1],
						ptrs[sub_ind+2]);
		     
		     int2lcd(sub_ind1+1,'@',0);
		     int2lcd(li_bat._Ub,'$',1);
			int2lcd_mmm(abs(li_bat._Ib),'#',1);
			int2lcd_mmm(li_bat._Tb,'?',0);
		     int2lcd(li_bat._soc,'w',0);
			if(sub_ind==8)lcd_buffer[60]=1;
			int2lcd(li_bat._ratCap,'Q',0);
			int2lcd(li_bat._cclv,'<',1);
			int2lcd(li_bat._soh,'>',0);
			int2lcd(li_bat._rbt,'[',1);
	
			//int2lcdyx(lakb[i]._flags1,0,19,0);
			}
		//int2lcdyx(lakb[0]._flags1,0,10,0);
		//int2lcdyx(lakb[0]._flags2,0,15,0);
		//int2lcdyx(lakb[0]._b_p_ser_num,0,19,0);
		}
	else if(BAT_TYPE==2)//Батарея	SACRED SUN
		{
		if(bat[sub_ind1]._Ib/*lakb[sub_ind1]._ch_curr*/>0)
		     {
		     ptrs[1]="    заряжается      ";
		     ptrs[3]=" Iзар=       #А     ";
		     }
		else
		     {
		     ptrs[1]=  "   разряжается      ";
		     ptrs[3]=  " Iразр=      #А     ";
		     }	
		ptrs[2]=       " Uбат =    $В       ";
		ptrs[4]=		" tбат =    ?°C      ";
		ptrs[5]=		" SOC  =    w%       ";
		ptrs[6]=		" SOH  =    >%       ";
		ptrs[7]=sm_exit;


 
		bgnd_par(		"    Батарея N@      ",
					"   1 x FP16S4810A   ",
					ptrs[sub_ind+1],ptrs[sub_ind+2]);
	     
	     int2lcd(sub_ind1+1,'@',0);
	     int2lcd(bat[sub_ind1]._Ub,'$',1);//int2lcd(lakb[sub_ind1]._tot_bat_volt/10,'$',1);
		/*if(lakb[sub_ind1]._ch_curr>0)
			{
			int2lcd_mmm(lakb[sub_ind1]._ch_curr,'#',2);
			}
		else 
			{
			int2lcd_mmm(lakb[sub_ind1]._dsch_curr,'#',2);
			}*/
		int2lcd_mmm(abs(bat[sub_ind1]._Ib),'#',2);
	     int2lcd_mmm(bat[sub_ind1]._Tb,'?',0);
	     //int2lcd_mmm(lakb[sub_ind1]._max_cell_temp,'?',0);
	     int2lcd(lakb[sub_ind1]._s_o_c,'w',0);
		int2lcd((short)(((long)lakb[sub_ind1]._rat_cap*(long)lakb[sub_ind1]._s_o_h)/1000L),'Q',1);
	     if(sub_ind==8)lcd_buffer[60]=1;

		int2lcd(lakb[sub_ind1]._rat_cap,'Q',1);
		//int2lcd(lakb[sub_ind1]._s_o_c,'w',0);
		int2lcd(lakb[sub_ind1]._c_c_l_v/10,'<',1);
		int2lcd(lakb[sub_ind1]._s_o_h,'>',0);
		int2lcd(lakb[sub_ind1]._r_b_t,'[',1);

		//int2lcdyx(lakb[sub_ind1]._battCommState,0,19,0);
		}
	else if(BAT_TYPE==3)//Батарея	ZTT
		{
		ptrs[0]=								"   Тип неизвестен   ";
		if(lakb[sub_ind1]._s_o_h>=500)ptrs[0]=  "      48V50Ah       ";
		if(lakb[sub_ind1]._s_o_h>=400)ptrs[0]=  "      48V40Ah       ";
		if(lakb[sub_ind1]._ch_curr>0)
		     {
		     ptrs[1]="    заряжается      ";
		     ptrs[3]=" Iзар=       #А     ";
		     }
		else
		     {
		     ptrs[1]=  "   разряжается      ";
		     ptrs[3]=  " Iразр=      #А     ";
		     }	
		ptrs[2]=       " Uбат =    $В       ";
		ptrs[4]=		" t1   =    ?°C      ";
		ptrs[5]=		" t2   =    (°C      ";
		ptrs[6]=		" t3   =    )°C      ";
		ptrs[7]=		" t4   =    +°C      ";
		ptrs[8]=		" tокр.=    [°C      ";
		ptrs[9]=		" tсил.=    ]°C      ";
		ptrs[10]=		" SOC  =    wA*ч     ";
		ptrs[11]=		" SOH  =    >A*ч     ";
		ptrs[12]=sm_exit;

		if(lakb[sub_ind1]._communicationFullErrorStat==1)
			{
			ptrs[0]=	" ОТСУТСТВУЕТ СВЯЗЬ  ";
			ptrs[1]=	"      С ПЛАТОЙ      ";
			ptrs[2]=	"    РАСШИРЕНИЯ!!!   ";

			bgnd_par(	"    Батарея N@      ",
						ptrs[0],
						ptrs[1],
						ptrs[2]);
			}

		else if(lakb[sub_ind1]._communicationFullErrorStat==2)
			{
			ptrs[0]=	" ОТСУТСТВУЕТ СВЯЗЬ  ";
			ptrs[1]=	"      С BMS!!!      ";
			ptrs[2]=	"                    ";

			bgnd_par(	"    Батарея N@      ",
						ptrs[0],
						ptrs[1],
						ptrs[2]);
			}

 		else 
			{
			bgnd_par(	"    Батарея N@      ",
						ptrs[sub_ind+0],
						ptrs[sub_ind+1],
						ptrs[sub_ind+2]);
		     

		    int2lcd(lakb[sub_ind1]._tot_bat_volt,'$',1);
			int2lcd_mmm(abs(lakb[sub_ind1]._ch_curr),'#',2);
		    int2lcd_mmm(lakb[sub_ind1]._cell_temp_1,'?',0);
			int2lcd_mmm(lakb[sub_ind1]._cell_temp_2,'(',0);
			int2lcd_mmm(lakb[sub_ind1]._cell_temp_3,')',0);
			int2lcd_mmm(lakb[sub_ind1]._cell_temp_4,'+',0);
			int2lcd_mmm(lakb[sub_ind1]._cell_temp_ambient,'[',0);
			int2lcd_mmm(lakb[sub_ind1]._cell_temp_power,']',0);
		    int2lcd(lakb[sub_ind1]._s_o_c,'w',1);
			if(sub_ind==10)lcd_buffer[60]=1;
			int2lcd(lakb[sub_ind1]._s_o_h,'>',1);
			
			}
		int2lcd(sub_ind1+1,'@',0);

		}
	}

else if(ind==iInv_tabl)
     {
     if(sub_ind==0)
     	{
     	bgnd_par("N     U     I    P  ",
     	         "!      ^     @     #",
     	         "!      ^     @     #",
     	         "!      ^     @     #");
      

     	}     

    	else if(sub_ind==1) 
     	{
     	bgnd_par("N   I      P     t  ",
     	         "!    @      #     $ ",
     	         "!    @      #     $ ",
     	         "!    @      #     $ ");

		}

	int2lcd(sub_ind1+1,'!',0);
	int2lcd(sub_ind1+2,'!',0);
	int2lcd(sub_ind1+3,'!',0);
		
		
	int2lcd(inv[sub_ind1]._Uio,'^',1);
	int2lcd(inv[sub_ind1+1]._Uio,'^',1);
	int2lcd(inv[sub_ind1+2]._Uio,'^',1);

     int2lcd(inv[sub_ind1]._Ii,'@',1); 
	int2lcd(inv[sub_ind1+1]._Ii,'@',1); 
	int2lcd(inv[sub_ind1+2]._Ii,'@',1); 

	int2lcd_mmm(inv[sub_ind1]._Pio,'#',0);
	int2lcd_mmm(inv[sub_ind1+1]._Pio,'#',0);
	int2lcd_mmm(inv[sub_ind1+2]._Pio,'#',0);

	int2lcd_mmm(inv[sub_ind1]._Ti,'$',0);
	int2lcd_mmm(inv[sub_ind1+1]._Ti,'$',0); 
   	int2lcd_mmm(inv[sub_ind1+2]._Ti,'$',0);

	}
else if(ind==iMakb)
	{
	const char* ptr[12];
 
	simax=10;

	ptr[0]=			" Uб1    =     @В    ";
	ptr[1]=			" Uб2    =     #В    ";
	ptr[2]=			" Uб3    =     $В    ";
	ptr[3]=			" Uб4    =     %В    ";
	ptr[4]=			" Uб5    =     ^В    ";
	ptr[5]=			" tб1    =     &°С   ";
	ptr[6]=			" tб2    =     *°С   ";
	ptr[7]=			" tб3    =     (°С   ";
	ptr[8]=			" tб4    =     )°С   ";
	ptr[9]=			" tб5    =     +°С   ";
	ptr[10]=			sm_exit;

 	if(makb[sub_ind1]._cnt>=5)
	 	{
		bgnd_par(		"   МОНИТОР АКБ N<   ",
					"   НЕ ПОДКЛЮЧЕН!!!  ",
					"                    ",
					"                    ");
		}


	else 
		{
		bgnd_par(		"   МОНИТОР АКБ N<   ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

		if(sub_ind-index_set>2)index_set=sub_ind-2;
		else if (sub_ind<index_set)index_set=sub_ind;

		if(sub_ind>=simax)	pointer_set(1);
		
		int2lcd(makb[sub_ind1]._Ub[0],'@',1);
		int2lcd(makb[sub_ind1]._Ub[1],'#',1);
		int2lcd(makb[sub_ind1]._Ub[2],'$',1);
		int2lcd(makb[sub_ind1]._Ub[3],'%',1);
		int2lcd(makb[sub_ind1]._Ub[4],'^',1);
		if(makb[sub_ind1]._T_nd[0])sub_bgnd("НЕПОДКЛЮЧЕН",'&',-5);
		else int2lcd_mmm(makb[sub_ind1]._T[0],'&',0); 
 		if(makb[sub_ind1]._T_nd[1])sub_bgnd("НЕПОДКЛЮЧЕН",'*',-5);
		else int2lcd_mmm(makb[sub_ind1]._T[1],'*',0); 
		if(makb[sub_ind1]._T_nd[2])sub_bgnd("НЕПОДКЛЮЧЕН",'(',-5);
		else int2lcd_mmm(makb[sub_ind1]._T[2],'(',0); 
		if(makb[sub_ind1]._T_nd[3])sub_bgnd("НЕПОДКЛЮЧЕН",')',-5);
		else int2lcd_mmm(makb[sub_ind1]._T[3],')',0); 
		if(makb[sub_ind1]._T_nd[4])sub_bgnd("НЕПОДКЛЮЧЕН",'+',-5);
		else int2lcd_mmm(makb[sub_ind1]._T[4],'+',0); 
		}
	int2lcd(sub_ind1+1,'<',0);
    	}

 else if(ind==iBps)
	{
	const char* ptr[8];
 
	simax=5;

	ptr[1]=			" Uист =        (В   ";
	ptr[2]=			" Iист =        [A   ";
	ptr[3]=			" tист =        ]°С  ";
	ptr[4]=			" Сброс аварий       ";
	ptr[5]=			sm_exit;

	if(bps[sub_ind1]._state==bsWRK)
		{
		ptr[0]=		"      в работе      ";
		if((bps[sub_ind1]._flags_tm&0x08)&&(bFL2))
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
 	 else if(bps[sub_ind1]._state==bsRDY)
	 	{
		ptr[0]=		"      в резерве     ";	
		}

 	 else if(bps[sub_ind1]._state==bsBL)
	 	{
		ptr[0]=		" заблокирован извне ";	
		}

	 else if(bps[sub_ind1]._state==bsAPV)
	 	{
		ptr[0]=		"    Работает АПВ    ";
		}
	 
	 else if(bps[sub_ind1]._state==bsAV)
	 	{
		if(bps[sub_ind1]._av&(1<<0))
		ptr[0]=		" Авария - перегрев! ";
		else if(bps[sub_ind1]._av&(1<<1))
		ptr[0]=		"Авария - завыш.Uвых!";
		else if(bps[sub_ind1]._av&(1<<2))	 
		ptr[0]=		"Авария - заниж.Uвых!";
		else if(bps[sub_ind1]._av&(1<<3))
			{
			ptr[0]=	"  Авария - потеряна ";
			ptr[1]=	"      связь!!!      ";
			ptr[2]=	"                    ";
			simax=0;
			}
		}

	 else if(bps[sub_ind1]._state==bsOFF_AV_NET)
	 	{
		ptr[0]=		"      ВЫКЛЮЧЕН      ";
		ptr[1]=		"     Отсутствует    ";
		ptr[2]=		" первичное питание! ";
		simax=0;
		}

	bgnd_par(			"       БПС N&       ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=4)	pointer_set(1);


		

	int2lcd(sub_ind1+1,'&',0);
	int2lcd(bps[sub_ind1]._Uii,'(',1);
     int2lcd(bps[sub_ind1]._Ii,'[',1);  
   	int2lcd_mmm(bps[sub_ind1]._Ti,']',0); 
   			 
    // char2lcdhxy(bps[sub_ind1]._state,0x32);
    
    //	int2lcdyx(sub_ind,0,2,0);
//	int2lcdyx(index_set,0,4,0);
	//int2lcdyx(u_necc,0,4,0);
	//int2lcdyx(cntrl_stat,0,8,0);	
     }  
else if(ind==iDcdc)
	{
	const char* ptr[8];
 
	simax=5;

	ptr[1]=			" Uист =        (В   ";
	ptr[2]=			" Iист =        [A   ";
	ptr[3]=			" tист =        ]°С  ";
	ptr[4]=			" Сброс аварий       ";
	ptr[5]=			sm_exit;

	if(bps[sub_ind1+2]._state==bsWRK)
		{
		ptr[0]=		"      в работе      ";
		if((bps[sub_ind1+2]._flags_tm&0x08)&&(bFL2))
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
 	 else if(bps[sub_ind1+2]._state==bsRDY)
	 	{
		ptr[0]=		"      в резерве     ";	
		}

 	 else if(bps[sub_ind1+2]._state==bsBL)
	 	{
		ptr[0]=		" заблокирован извне ";	
		}

	 else if(bps[sub_ind1+2]._state==bsAPV)
	 	{
		ptr[0]=		"    Работает АПВ    ";
		}
	 
	 else if(bps[sub_ind1+2]._state==bsAV)
	 	{
		if(bps[sub_ind1+2]._av&(1<<0))
		ptr[0]=		" Авария - перегрев! ";
		else if(bps[sub_ind1+2]._av&(1<<1))
		ptr[0]=		"Авария - завыш.Uвых!";
		else if(bps[sub_ind1+2]._av&(1<<2))	 
		ptr[0]=		"Авария - заниж.Uвых!";
		else if(bps[sub_ind1+2]._av&(1<<3))
			{
			ptr[0]=	"  Авария - потеряна ";
			ptr[1]=	"      связь!!!      ";
			ptr[2]=	"                    ";
			simax=0;
			}
		}

	 else if(bps[sub_ind1+2]._state==bsOFF_AV_NET)
	 	{
		ptr[0]=		"      ВЫКЛЮЧЕН      ";
		ptr[1]=		"     Отсутствует    ";
		ptr[2]=		" первичное питание! ";
		simax=0;
		}

	bgnd_par(			"      DCDC N&       ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=4)	pointer_set(1);


		

	int2lcd(sub_ind1+1,'&',0);
	int2lcd(bps[sub_ind1+2]._Uii,'(',1);
     int2lcd(bps[sub_ind1+2]._Ii,'[',1);  
   	int2lcd_mmm(bps[sub_ind1+2]._Ti,']',0); 
   			 
    // char2lcdhxy(bps[sub_ind1]._state,0x32);
    
    //	int2lcdyx(sub_ind,0,2,0);
//	int2lcdyx(index_set,0,4,0);
	//int2lcdyx(u_necc,0,4,0);
	//int2lcdyx(cntrl_stat,0,8,0);	
     }  
else if(ind==iInv)
	{
	const char* ptr[8];
 
	simax=5;

	ptr[1]=			" Uинв =        (В   ";
	ptr[2]=			" Iинв =        [A   ";
	ptr[3]=			" tинв =        ]°С  ";
	ptr[4]=			" Сброс аварий       ";
	ptr[5]=			sm_exit;

	if((inv[sub_ind1]._flags_tm==0)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"      в работе      ";
		}
	else if((inv[sub_ind1]._flags_tm==0x04)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
	else if((inv[sub_ind1]._flags_tm==0x24)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"отключился,перегрев ";	      
		}
	else if((inv[sub_ind1]._flags_tm&0x20)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"  отсутствует Uвых  ";
		}
	else if(inv[sub_ind1]._cnt!=0)
	 	{
		ptr[0]=		"    не подключен    ";	
		}


	bgnd_par(			"     ИНВЕРТОР N&    ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=4)	pointer_set(1);


		

	int2lcd(sub_ind1+1,'&',0);
	int2lcd(inv[sub_ind1]._Uio,'(',1);
     int2lcd(inv[sub_ind1]._Ii,'[',1);  
   	int2lcd_mmm(inv[sub_ind1]._Ti,']',0); 
   	int2lcdyx(inv[sub_ind1]._flags_tm,0,19,0);		 
    // char2lcdhxy(bps[sub_ind1]._state,0x32);
    
//int2lcdyx(bps[sub_ind1]._flags_tm&0xFF,0,18,0);

//int2lcdyx(ava,0,4,0);	
//int2lcdyx(plazma_inv[0],0,2,0);
//int2lcdyx(plazma_inv[1],0,5,0);
//int2lcdyx(plazma_inv[2],0,8,0);
//int2lcdyx(plazma_inv[3],0,11,0);
	//int2lcdyx(inv[sub_ind1]._flags_tm,0,14,0);
     } 
	 
else if(ind==iInv_v2)
	{
	const char* ptr[8];
 
	simax=7;

	ptr[1]=			" Uвых =        (В   ";
	ptr[2]=			" Iвых =        )A   ";
	ptr[3]=			" tинв =        [°С  ";
	ptr[4]=			" Pвых =        ]Вт  ";
	ptr[5]=			" Uсети =       <В   ";
	ptr[6]=			" Uшины =       >В   ";
	ptr[7]=			sm_exit;

	//if((inv[sub_ind1]._flags_tm==0)&&(inv[sub_ind1]._cnt==0))
	//	{
		ptr[0]=		"      в работе      ";
	//	}
	/*else */if((inv[sub_ind1]._flags_tm==0x04)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
	else if((inv[sub_ind1]._flags_tm==0x24)&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"отключился,перегрев ";	      
		}
	else if((!(inv[sub_ind1]._flags_tm&0x20))&&(inv[sub_ind1]._cnt==0))
		{
		ptr[0]=		"  отсутствует Uвых  ";
		}
	else if(inv[sub_ind1]._cnt!=0)
	 	{
		ptr[0]=		"    не подключен    ";	
		}


	bgnd_par(			"     ИНВЕРТОР N&    ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=simax)	pointer_set(1);


		

	int2lcd(sub_ind1+1,'&',0);
	int2lcd(inv[sub_ind1]._Uio,'(',1);
     int2lcd(inv[sub_ind1]._Ii,')',1);  
   	int2lcd_mmm(inv[sub_ind1]._Ti,'[',0); 
	int2lcd_mmm(inv[sub_ind1]._Pio,']',0);
	int2lcd(inv[sub_ind1]._Uin,'<',1);
	int2lcd(inv[sub_ind1]._Uil,'>',1);
	//int2lcdyx(inv[sub_ind1]._flags_tm,0,19,0);
	//int2lcdyx(inv[sub_ind1]._cnt,0,15,0);
   	//int2lcd(inv[sub_ind1]._flags_tm,'[',1);		 
    // char2lcdhxy(bps[sub_ind1]._state,0x32);
    
//int2lcdyx(bps[sub_ind1]._flags_tm&0xFF,0,18,0);

//int2lcdyx(ava,0,4,0);	
//int2lcdyx(plazma_inv[0],0,2,0);
//int2lcdyx(plazma_inv[1],0,5,0);
//int2lcdyx(plazma_inv[2],0,8,0);
//int2lcdyx(plazma_inv[3],0,11,0);
	//int2lcdyx(inv[sub_ind1]._Ii,0,2,0);
	//int2lcdyx(bps[sub_ind1+20]._cnt,0,19,0);
    	}

else if(ind==iByps)
	{
	const char* ptr[8];

	static char iByps_ind_cnt;
	
	if(++iByps_ind_cnt>=40)iByps_ind_cnt=0;



	simax=7;

	ptr[1]=			" Uвых =        (В   ";
	ptr[2]=			" Iвых =        )A   ";
	ptr[3]=			" Pвых =        ]Вт  ";
	ptr[4]=			" tбп  =        [°С  ";
	ptr[5]=			" Uсети =       <В   ";
	ptr[6]=			" Uшины =       >В   ";
	ptr[7]=			sm_exit;

	ptr[0]=		"      в работе      ";
	
	if(iByps_ind_cnt<=20)
		{
		if(byps._flags&0x40)ptr[0]=		"Приоритет инверторы ";
		else ptr[0]=					"Приоритет сеть      ";
		}

	if(iByps_ind_cnt>20)
		{
		if(byps._flags&0x80)ptr[0]=		"Работа от инверторов";
		else ptr[0]=					"Работа от сети      ";
		}

	if((byps._flags&0x04)&&(byps._cnt<5))
		{
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
	else if((byps._flags&0x02)&&(byps._cnt<5))
		{
		ptr[0]=		"отключился,перегрев ";	      
		}
	else if(byps._cnt>10)
	 	{
		ptr[0]=		"    не подключен    ";	
		}


	bgnd_par(			"       БАЙПАС       ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=simax)	pointer_set(1);


	int2lcd(byps._Uout,'(',1);
     int2lcd(byps._Iout,')',1);  
   	int2lcd_mmm(byps._T,'[',0); 
	int2lcd_mmm(byps._Pout,']',0);
	int2lcd(byps._Unet,'<',1);
	int2lcd(byps._Uin,'>',1);
	//int2lcdyx(iByps_ind_cnt,0,2,0);
	//int2lcdyx(byps._flags,0,6,0);
    	}
	 	  
else if(ind==iNet)
	{
	bgnd_par(		"        СЕТЬ        ",
				" U   =     [В       ",
				" f   =     ]Гц      ",
				sm_exit);
	lcd_buffer[60]=1;
	     	
     int2lcd(net_U,'[',0);
     int2lcd(net_F,']',1);

 //temp_SL=(signed long)net_buff_;
//temp_SL*=Kunet;    
//int2lcdyx(net_buff_,0,4,0);
//int2lcdyx(Kunet,0,9,0);
                  	      	   	    		
     }

else if(ind==iNet3)
	{


	ptrs[0]=  		" UфA           !В   ";
    ptrs[1]=  		" UфB           @В   ";
    ptrs[2]=  	    " UфC           #В   ";
	ptrs[3]=  	    " f   =     ]Гц      ";           
	ptrs[4]=  		" Выход              ";


	bgnd_par(		"        СЕТЬ        ",
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

    int2lcd(net_Ua,'!',0);
	int2lcd(net_Ub,'@',0);
	int2lcd(net_Uc,'#',0);
	#ifdef UKU_220_IPS_TERMOKOMPENSAT
	int2lcd(net_F3,']',1);
	#else
    int2lcd(net_F,']',1);
    #endif

 //temp_SL=(signed long)net_buff_;
//temp_SL*=Kunet;    
//int2lcdyx(net_buff_,0,4,0);
//int2lcdyx(Kunet,0,9,0);
                  	      	   	    		
     }




else if(ind==iLoad)
	{
	bgnd_par(		"      НАГРУЗКА      ",
				" Uнагр =     [В     ",
				" Iнагр =     ]А     ",
				sm_exit);
	lcd_buffer[60]=1;
	     	
     int2lcd(load_U,'[',1);
     int2lcd(load_I,']',1);

     
                   	      	   	    		
     }

else if(ind==iExtern_GLONASS)
	{

	ptrs[0]=  		" tвнеш.возд.    !°С ";
     ptrs[1]=  		" tотсек ЭПУ     @°С ";
     ptrs[2]=  	     " Сух.конт.№1  $     ";            
     ptrs[3]=  	     " Сух.конт.№2  %     ";
	ptrs[4]=  	     " Сух.конт.№3  ^     ";
	ptrs[5]=  	     " Сух.конт.№4  &     ";
	ptrs[2+NUMSK]=  	" Выход              ";
	ptrs[3+NUMSK]=  	"                    ";
	ptrs[4+NUMSK]=  	"                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);


     if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
     else int2lcd_mmm(t_ext[0],'!',0);

	if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
     else int2lcd_mmm(t_ext[1],'@',0);


	if(sk_av_stat[0]==sasON)	sub_bgnd("АВАРИЯ",'$',0);
	else                     sub_bgnd("НОРМА",'$',0);
	

	if(sk_av_stat[1]==sasON) sub_bgnd("АВАРИЯ",'%',0);
	else                     sub_bgnd("НОРМА",'%',0);

	if(sk_av_stat[2]==sasON) sub_bgnd("АВАРИЯ",'^',0);
	else                     sub_bgnd("НОРМА",'^',0);

	if(sk_av_stat[3]==sasON) sub_bgnd("АВАРИЯ",'&',0);
	else                     sub_bgnd("НОРМА",'&',0);

     }

else if(ind==iExtern_3U)
	{

	ptrs[0]=  		" tвнеш.возд.    !°С ";
     ptrs[1]=  		" tотсек ЭПУ     @°С ";
     ptrs[5]=  	     " Сух.конт.№1      $ ";            
     ptrs[6]=  	     " Сух.конт.№2      % ";
	ptrs[7]=  	     " Сух.конт.№3      ^ ";
	ptrs[8]=  	     " Сух.конт.№4      & ";
	ptrs[5+NUMSK]=  	" Выход              ";
	ptrs[6+NUMSK]=  	"                    ";
	ptrs[7+NUMSK]=  	"                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);


     if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
     else int2lcd_mmm(t_ext[0],'!',0);

	if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
     else int2lcd_mmm(t_ext[1],'@',0);


	if(sk_av_stat[0]==sasON)	sub_bgnd("АВАРИЯ",'$',0);
	else                     sub_bgnd("НОРМА",'$',0);
	

	if(sk_av_stat[1]==sasON) sub_bgnd("АВАРИЯ",'%',0);
	else                     sub_bgnd("НОРМА",'%',0);

	if(sk_av_stat[2]==sasON) sub_bgnd("АВАРИЯ",'^',0);
	else                     sub_bgnd("НОРМА",'^',0);

	if(sk_av_stat[3]==sasON) sub_bgnd("АВАРИЯ",'&',0);
	else                     sub_bgnd("НОРМА",'&',0);

     }

else if(ind==iExtern_KONTUR)
	{

	ptrs[0]=  		" tвнеш.возд.    !°С ";
     ptrs[1]=  		" tотсек ЭПУ     @°С ";
     ptrs[2]=  		" tотсек MSAN    #°С ";
     ptrs[3]=  		" tбат1          <°C ";
     ptrs[4]=  		" tбат2          >°C ";
     ptrs[5]=  	     " Датч.двери  $      ";            
     ptrs[6]=  	     " Датч.дыма   %      ";
	ptrs[7]=  	     " Датч.удара  ^      ";
	//ptrs[6]=  	     " Датч.перев. &      ";
     //ptrs[6]=  	     " Fвент.       ?     ";
	ptrs[8]=  	     " Выход              ";
	ptrs[9]=  	     "                    ";
	ptrs[10]=  	     "                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

	//int2lcdyx(t_box,0,19,0);
/*     int2lcd_mmm(t_ext[1],'@',0);
     int2lcd_mmm(t_ext[2],'#',0);*/

     if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
     else int2lcd_mmm(t_ext[0],'!',0);

	if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
     else int2lcd_mmm(t_ext[1],'@',0);

	if(ND_EXT[2])sub_bgnd("неиспр.",'#',-3);
     else int2lcd_mmm(t_ext[2],'#',0);


	if(bat[0]._nd)sub_bgnd("неиспр.",'<',-3);
     else int2lcd_mmm(bat[0]._Tb,'<',0);

	if(bat[1]._nd)sub_bgnd("неиспр.",'>',-3);
     else int2lcd_mmm(bat[1]._Tb,'>',0);

	if(bat[sub_ind1]._nd)ptrs[5]="    ДТ. неисправен  ";
		else ptrs[5]="   tбат =   ?°C     ";

	if(sk_av_stat[0]==sasON)	sub_bgnd("ОТКР.",'$',0);
	else                     sub_bgnd("ЗАКР.",'$',0);
	

	if(sk_av_stat[1]==sasON) sub_bgnd("АВАРИЯ",'%',0);
	else                     sub_bgnd("НОРМА",'%',0);

	if(sk_av_stat[2]==sasON) sub_bgnd("АВАРИЯ",'^',0);
	else                     sub_bgnd("НОРМА",'^',0);

	//if(sk_av_stat[3]==sasON) sub_bgnd("АВАРИЯ",'&',0);
	//else                     sub_bgnd("НОРМА",'&',0);

     //int2lcd(pos_vent,'?',0);
	//ptrs[7]=  	     " Pсумм.       <кВт*ч";
	//ptrs[8]=  	     " Pтекущ.      >Вт   ";
	int2lcdyx(sk_av_stat[0],0,4,0);
	int2lcdyx(sk_av_stat[1],0,8,0);
	int2lcdyx(sk_av_stat[2],0,12,0);
	int2lcdyx(sk_av_stat[3],0,16,0);
     }

else if(ind==iExtern_TELECORE2015)
	{

	ptrs[0]=		" Дt1            #°С ";
	ptrs[1]=		" Дt2            %°С ";
    ptrs[2]=  	    " Датч.двери  $      ";            
	ptrs[3]=  	    " Выход              ";
	ptrs[4]=  	    "                    ";
	ptrs[5]=  	    "                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

	//int2lcdyx(t_box,0,19,0);
/*     int2lcd_mmm(t_ext[1],'@',0);
     int2lcd_mmm(t_ext[2],'#',0);*/

	if(ND_EXT[0])sub_bgnd("неиспр.",'#',-3);
    else int2lcd_mmm(t_ext[0],'#',0);
	if(ND_EXT[1])sub_bgnd("неиспр.",'%',-3);
    else int2lcd_mmm(t_ext[1],'%',0);

	if(sk_av_stat[0]==sasON)	sub_bgnd("ОТКР.",'$',0);
	else                     sub_bgnd("ЗАКР.",'$',0);
	

/*	if(sk_av_stat[1]==sasON) sub_bgnd("АВАРИЯ",'%',0);
	else                     sub_bgnd("НОРМА",'%',0);

	if(sk_av_stat[2]==sasON) sub_bgnd("АВАРИЯ",'^',0);
	else                     sub_bgnd("НОРМА",'^',0);*/

	//if(sk_av_stat[3]==sasON) sub_bgnd("АВАРИЯ",'&',0);
	//else                     sub_bgnd("НОРМА",'&',0);

     //int2lcd(pos_vent,'?',0);  sk_cnt
	//ptrs[7]=  	     " Pсумм.       <кВт*ч";
	//ptrs[8]=  	     " Pтекущ.      >Вт   ";
	//int2lcdyx(sk_av_stat[0],0,4,0);
	//int2lcdyx(sk_av_stat[1],0,8,0);
	//int2lcdyx(sk_av_stat[2],0,12,0);
	//int2lcdyx(sk_cnt[0],1,4,0);
	//int2lcdyx(sk_cnt[1],1,8,0);
	//int2lcdyx(sk_cnt[2],1,12,0);
	//int2lcdyx(adc_buff_[sk_buff_TELECORE2015[0]],2,4,0);
	//int2lcdyx(adc_buff_[sk_buff_TELECORE2015[1]],2,8,0);
	//int2lcdyx(adc_buff_[sk_buff_TELECORE2015[2]],2,12,0);
     }

else if(ind==iExtern_6U)
	{

	ptrs[0]=  			" t1             !°С ";
	ptrs[1]=  			" t2             @°С ";
	ptrs[2]=  			" t3             #°С ";
	ptrs[NUMDT]=  		" СК1        $       ";
	ptrs[NUMDT+1]= 		" СК2        %       ";
	ptrs[NUMDT+2]= 		" СК3        ^       ";
	ptrs[NUMDT+3]= 		" СК4        &       ";		
	ptrs[NUMDT+NUMSK]=	" Выход              ";
	ptrs[NUMDT+NUMSK+1]="                    ";
	ptrs[NUMDT+NUMSK+2]="                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

    if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
    else if(index_set==0)int2lcd_mmm(t_ext[0],'!',0);

    if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
    else int2lcd_mmm(t_ext[1],'@',0);

    if(ND_EXT[2])sub_bgnd("неиспр.",'#',-3);
    else int2lcd_mmm(t_ext[2],'#',0);

	if(sk_av_stat[0]==sasON) sub_bgnd("АВАРИЯ",'$',0);
	else                     sub_bgnd("НОРМА",'$',0);

	if(sk_av_stat[1]==sasON) sub_bgnd("АВАРИЯ",'%',0);
	else                     sub_bgnd("НОРМА",'%',0);

	if(sk_av_stat[2]==sasON) sub_bgnd("АВАРИЯ",'^',0);
	else                     sub_bgnd("НОРМА",'^',0);

	if(sk_av_stat[3]==sasON) sub_bgnd("АВАРИЯ",'&',0);
	else                     sub_bgnd("НОРМА",'&',0);

     }

else if(ind==iExtern_220)
	{
	char temp;

	ptrs[0]=  			" t1             !°С ";
	ptrs[1]=  			" t2             @°С ";
	ptrs[2]=  			" t3             #°С ";
	ptrs[NUMDT]=  		" СК1              $ ";
	ptrs[NUMDT+1]= 		" СК2              % ";
	ptrs[NUMDT+2]= 		" СК3              ^ ";
	ptrs[NUMDT+3]= 		" СК4              & ";		
	ptrs[NUMDT+NUMSK]=	" Выход              ";
	ptrs[NUMDT+NUMSK+1]="                    ";
	ptrs[NUMDT+NUMSK+2]="                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

    if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
    else int2lcd_mmm(t_ext[0],'!',0);

    if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
    else int2lcd_mmm(t_ext[1],'@',0);

    if(ND_EXT[2])sub_bgnd("неиспр.",'#',-3);
    else int2lcd_mmm(t_ext[2],'#',0);

	temp=-11;
	if(sk_stat[0]==ssON)		temp=-9;
	if(sk_av_stat[0]==sasON) 	sub_bgnd("АВАРИЯ",'$',temp-2);
	else                     	sub_bgnd("НОРМА",'$',temp-1);
	if(sk_stat[0]==ssON)		sub_bgnd("ЗАМКН.",'$',-4);
	if(sk_stat[0]==ssOFF)		sub_bgnd("РАЗОМКН.",'$',-6);

	temp=-11;
	if(sk_stat[1]==ssON)		temp=-9;
	if(sk_av_stat[1]==sasON) 	sub_bgnd("АВАРИЯ",'%',temp-2);
	else                     	sub_bgnd("НОРМА",'%',temp-1);
	if(sk_stat[1]==ssON)		sub_bgnd("ЗАМКН.",'%',-4);
	if(sk_stat[1]==ssOFF)		sub_bgnd("РАЗОМКН.",'%',-6);

	temp=-11;
	if(sk_stat[2]==ssON)		temp=-9;
	if(sk_av_stat[2]==sasON) 	sub_bgnd("АВАРИЯ",'^',temp-2);
	else                     	sub_bgnd("НОРМА",'^',temp-1);
	if(sk_stat[2]==ssON)		sub_bgnd("ЗАМКН.",'^',-4);
	if(sk_stat[2]==ssOFF)		sub_bgnd("РАЗОМКН.",'^',-6);

	temp=-11;
	if(sk_stat[3]==ssON)		temp=-9;
	if(sk_av_stat[3]==sasON) 	sub_bgnd("АВАРИЯ",'&',temp-2);
	else                     	sub_bgnd("НОРМА",'&',temp-1);
 	if(sk_stat[3]==ssON)		sub_bgnd("ЗАМКН.",'&',-4);
	if(sk_stat[3]==ssOFF)		sub_bgnd("РАЗОМКН.",'&',-6);

     }


else if(ind==iVent)
	{

	ptrs[0]=  		" Fвент.текущ.     !%";
     ptrs[1]=  		" Fвент.max. (  @%) #";
	ptrs[2]=  	     " Выход              ";

	bgnd_par(			"     ВЕНТИЛЯТОР     ",
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);

	pointer_set(1);

     int2lcd(main_vent_pos*5,'!',0);
	//int2lcd(pos_vent,'#',0);
	//int2lcd(pos_vent*5+45,'@',0);     
	}

else if(ind==iAvt)
	{
     ptrs[0]=  		"  АВТОМАТЫ НАГРУЗОК ";
	ptrs[1]=  		" Автомат №1       ! ";
	ptrs[2]=  		" Автомат №2       @ ";
	ptrs[3]=  		" Автомат №3       # ";
	ptrs[4]=  		" Автомат №4       $ ";
	ptrs[5]=  		" Автомат №5       % ";
	ptrs[6]=  		" Автомат №6       ^ ";
	ptrs[7]=  		" Автомат №7       & ";
	ptrs[8]=  		" Автомат №8       * ";
	ptrs[9]=  		" Автомат №9       ( ";
	ptrs[10]=  		" Автомат №10      ) ";
	ptrs[11]=  		" Автомат №11      + ";
	ptrs[12]=  		" Автомат №12      = ";

	ptrs[1+NUMAVT]=  	" Выход              ";
	ptrs[2+NUMAVT]=  	"                    ";
	ptrs[3+NUMAVT]=  	"                    ";

	bgnd_par(		ptrs[0],
				ptrs[index_set+1],
				ptrs[index_set+2],
				ptrs[index_set+3]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

	//int2lcdyx(eb2_data_short[6],0,6,0);

	if(avt_stat[0]==avtON)	sub_bgnd("ВКЛ.",'!',-3);
	else 				sub_bgnd("ВЫКЛ.",'!',-4);
	if(avt_stat[1]==avtON)	sub_bgnd("ВКЛ.",'@',-3);
	else 				sub_bgnd("ВЫКЛ.",'@',-4);
	if(avt_stat[2]==avtON)	sub_bgnd("ВКЛ.",'#',-3);
	else 				sub_bgnd("ВЫКЛ.",'#',-4);
	if(avt_stat[3]==avtON)	sub_bgnd("ВКЛ.",'$',-3);
	else 				sub_bgnd("ВЫКЛ.",'$',-4);
	if(avt_stat[4]==avtON)	sub_bgnd("ВКЛ.",'%',-3);
	else 				sub_bgnd("ВЫКЛ.",'%',-4);
	if(avt_stat[5]==avtON)	sub_bgnd("ВКЛ.",'^',-3);
	else 				sub_bgnd("ВЫКЛ.",'^',-4);
	if(avt_stat[6]==avtON)	sub_bgnd("ВКЛ.",'&',-3);
	else 				sub_bgnd("ВЫКЛ.",'&',-4);
	if(avt_stat[7]==avtON)	sub_bgnd("ВКЛ.",'*',-3);
	else 				sub_bgnd("ВЫКЛ.",'*',-4);
	if(avt_stat[8]==avtON)	sub_bgnd("ВКЛ.",'(',-3);
	else 				sub_bgnd("ВЫКЛ.",'(',-4);
	if(avt_stat[9]==avtON)	sub_bgnd("ВКЛ.",')',-3);
	else 				sub_bgnd("ВЫКЛ.",')',-4);
	if(avt_stat[10]==avtON)	sub_bgnd("ВКЛ.",'+',-3);
	else 				sub_bgnd("ВЫКЛ.",'+',-4); 
	if(avt_stat[11]==avtON)	sub_bgnd("ВКЛ.",'=',-3);
	else 				sub_bgnd("ВЫКЛ.",'=',-4);
     //int2lcd(Uvv[1],'$',0);
     //int2lcd(Uvv[2],'$',0);

     //long2lcd_mmm(power_summary,'%',2);
     //int2lcd(power_current,'^',0);

     //int2lcdyx(adc_buff_ext_[0],0,4,0);
     //int2lcdyx(adc_buff_ext_[1],0,10,0);
     //int2lcdyx(adc_buff_ext_[2],0,16,0);
     }




else if(ind==iSpc)
	{

 	ptrs[0]=	" Выр.заряд          ";
 	ptrs[1]=	" Авт.выр.заряд      ";
 	ptrs[2]=	" К.Е. батареи N1    ";
 	ptrs[3]=	" К.Е. батареи N2    ";
 //	ptrs[4]=	" А.К.Е.  бат. N1    ";
 //	ptrs[5]=	" А.К.Е.  бат. N2    ";
 	ptrs[4]=	" Выход              ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
    	bgnd_par( "     СПЕЦФУНКЦИИ    ",
    	          ptrs[index_set],
    	          ptrs[index_set+1],
    	          ptrs[index_set+2]);
	pointer_set(1);
	}    		

else if(ind==iSpc_termocompensat)
	{

 	ptrs[0]=	" Выр.заряд          ";
 	ptrs[1]=	" Авт.выр.заряд      ";
 	ptrs[2]=	" К.Е. батареи       ";
 	ptrs[3]=	" Выход              ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
    	bgnd_par( "     СПЕЦФУНКЦИИ    ",
    	          ptrs[index_set],
    	          ptrs[index_set+1],
    	          ptrs[index_set+2]);
	pointer_set(1);
	}    		


	

	

else if(ind==iLog)
	{
	//char dt[4],dt_[4],dt__[4];
//	char iii;

	//av_j_si_max=lc640_read_int(CNT_EVENT_LOG);
	if(av_j_si_max>64)av_j_si_max=0;

	if(av_j_si_max==0)
		{
		bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," Журнал пуст        ",sm_exit,sm_);
		//lcd_buffer[33]=1;
		sub_ind=1;
		index_set=0;
		}       
		
	else if(av_j_si_max==1)
		{
		bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  ",sm_exit," Очистить журнал    ");
		//if(sub_ind==0)lcd_buffer[16]=1;
		//else if(sub_ind==1)lcd_buffer[33]=1;
		//else if(sub_ind==2)lcd_buffer[50]=1;		
		index_set=0;
		}

	else if(av_j_si_max==2)
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>2) index_set=sub_ind-2;		
		if(index_set==0) bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  "," [                  ",sm_exit);
		else if(index_set==1) bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  ",sm_exit," Очистить журнал    ");
		
		//if((sub_ind-index_set)==0) lcd_buffer[16]=1; 
		//else if((sub_ind-index_set)==1) lcd_buffer[33]=1;
		//else if((sub_ind-index_set)==2) lcd_buffer[50]=1;
		}
		
	else if(av_j_si_max>2)
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>2) index_set=sub_ind-2;  
		if(index_set==(av_j_si_max-1)) bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  ",sm_exit," Очистить журнал    ");
		else if(index_set==(av_j_si_max-2)) bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  "," [                  ",sm_exit);
		else bgnd_par("   ЖУРНАЛ СОБЫТИЙ   "," (                  "," [                  "," {                  ");
		
		//if((sub_ind-index_set)==0) lcd_buffer[16]=1; 
		//else if((sub_ind-index_set)==1) lcd_buffer[33]=1;
		//else if((sub_ind-index_set)==2) lcd_buffer[50]=1;

		}
	pointer_set(1);
     event2ind(index_set,'(');
     event2ind(index_set+1,'[');	
     event2ind(index_set+2,'{');	  
     
	}



else if(ind==iLog_)
	{	
	unsigned short tempUI/*,tempUI_*/;
//	unsigned long tempUL;
	char av_head[4],av_data_on[8],av_data_off[8],av_data[4];
	short av_head_int[2];
	
	bgnd_par(sm_,sm_,sm_,sm_);

     
     //lc640_read_long_ptr(tempUI,av_head);
     //lc640_read_long_ptr(tempUI+4,(char*)av_head_int);
     //lc640_read_long_ptr(tempUI+8,av_data_on);
     //lc640_read_long_ptr(tempUI+12,&(av_data_on[4])); 
     //lc640_read_long_ptr(tempUI+16,av_data_off);
     //lc640_read_long_ptr(tempUI+20,&(av_data_off[4]));      
	//lc640_read_long_ptr(tempUI+24,av_data);
	
	//av_head_int[0]=123;  
//av_head_int[1]=456;	

	if((av_head[0]=='U')&&(av_head[2]=='R'))
		{
		if(index_set==0) {
		
		bgnd_par(	"    Перезагрузка    ",
				"   или включение    ",
				"        ИБЭП        ",
				"  0%(  0^ 0@:0#:0$  ");
		} else if(index_set==1) {

		bgnd_par(	"    Перезагрузка    ",
				"   или включение    ",
				"        ИБЭП        ",
				"  код источника  [  ");		
		
		} else if(index_set==2) {

		bgnd_par(	"    Перезагрузка    ",
				"   или включение    ",
				"        ИБЭП        ",
				"                 ]  ");		
		
		}					
				  	
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		int2lcd(av_data_on[7],'[',0);
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		//int2lcd(av_data_on[1],'(',0);
		//int2lcdyx(av_data_on[1],2,1,0);
		if(av_data_on[7]&0x08)sub_bgnd("superwiser  ",']',-12);
		else if(av_data_on[7]&0x04)sub_bgnd("watchdog      ",']',-12);
		else if(av_data_on[7]&0x02)sub_bgnd("ext.reset    ",']',-12);
		else if(av_data_on[7]&0x01)sub_bgnd("power on     ",']',-12);
		av_j_si_max=2;

		
		}

	else if((av_head[0]=='P')&&(av_head[2]=='A'))
		{  
		ptrs[0]="   Авария сети!!!   ";
		ptrs[1]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[2]="    не устранена    ";
			ptrs[3]="     Uсети=  +В     ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			int2lcd(net_U,'+',0);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[2]="      устранена     ";
			ptrs[3]="  0[]  0< 0>:0=:0)  ";
			ptrs[4]="     Uмин=  +В      ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
			int2lcd(av_data[0]+(av_data[1]*256),'+',0);			
			}	
		
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='C'))
		{  
		ptrs[0]="       Авария       ";
		ptrs[1]="     батареи N+     ";
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      устранена     ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='S'))
		{  
		ptrs[0]="       Авария       ";
		ptrs[1]="    несимметрии     ";
		ptrs[2]="     батареи N+     ";
		ptrs[3]="  0%(  0^ 0@:0#:0$  ";
		bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=0;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='Z'))
		{  
		ptrs[0]="   Выравнивающий    ";
		ptrs[1]="       заряд        ";
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не завершен     ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      завершен      ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}



	else if((av_head[0]=='B')&&(av_head[2]=='W'))
		{  
		ptrs[0]="       Разряд       ";
		ptrs[1]="     батареи N!     ";
		ptrs[2]="   Начало           ";
		ptrs[3]="  0%(  0^ 0@:0#:0$  ";
		ptrs[4]="       Uбат=  <В";
		ptrs[5]="   Конец            ";
		ptrs[6]="  0qw  0r 0i:0l:0s  ";
		ptrs[7]="         Uбат=  >В  ";
		ptrs[8]="   Отдано    /а*ч.  ";
		
		bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
		
		int2lcd(av_head[1]+1,'!',0);

		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
				
		int2lcd(av_data_off[4],'i',0);
		int2lcd(av_data_off[5],'l',0);
		int2lcd(av_data_off[6],'s',0);
		int2lcd(av_data_off[2],'q',0);
		int2lcd(av_data_off[0],'r',0); 
		if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
		sub_bgnd(sm_mont[av_data_off[1]],'w',0);
		
		
		int2lcd(av_head_int[0]/10,'/',1);
		int2lcd(av_data_on[3]+(av_data_on[7]*256),'<',1);
		int2lcd(av_head_int[1],'>',1);	
		av_j_si_max=5;				

		
		}

	else if((av_head[0]=='B')&&(av_head[2]=='K'))
		{  
		ptrs[0]="  Контроль емкости  ";
		ptrs[1]="       батареи      ";
		ptrs[2]="   Начало           ";
		ptrs[3]="  0%(  0^ 0@:0#:0$  ";
		ptrs[4]="         Uбат=  <В  ";
		ptrs[5]="   Конец            ";
		ptrs[6]="  0qw  0r 0i:0l:0s  ";
		ptrs[7]="         Uбат=  >В  ";
		ptrs[8]="   Ёмкость   /а*ч.  ";
		
		bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
		
		int2lcd(av_head[1]+1,'!',0);

		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
				
		int2lcd(av_data_off[4],'i',0);
		int2lcd(av_data_off[5],'l',0);
		int2lcd(av_data_off[6],'s',0);
		int2lcd(av_data_off[2],'q',0);
		int2lcd(av_data_off[0],'r',0); 
		if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
		sub_bgnd(sm_mont[av_data_off[1]],'w',0);
		
		
		int2lcd(av_head_int[0],'/',1);
		int2lcd(av_data_on[3]+(av_data_on[7]*256),'<',1);
		int2lcd(av_head_int[1],'>',1);	
		av_j_si_max=5;				

		
		}



	else if((av_head[0]=='S')||(av_head[0]=='I'))
		{  
		ptrs[0]="   Авария БПС N+    ";
		
		if(av_head[2]=='L')
			{
			ptrs[1]="     отключился     ";
			}
		else if(av_head[2]=='T')
			{
			ptrs[1]="      перегрев      ";
			}		
		else if(av_head[2]=='U')
			{
			ptrs[1]="   завышено Uвых.   ";
			}		
		else if(av_head[2]=='u')
			{
			ptrs[1]="   занижено Uвых.   ";
			}								
		else if(av_head[2]=='O')
			{
			ptrs[1]="    завышено Iвых   ";
			}		
		
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    не устранена    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      устранена     ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		}

	
	}
		 
else if(ind==iBatLog)
	{
	if(BAT_IS_ON[sub_ind1]==bisON)ptrs[0]=" Введена  0!/@  /0# ";
	else ptrs[0]=" Выведена 0!/@  /0# ";
     ptrs[1]=" Номин.емк.     $A*ч";
     ptrs[2]=" Наработка      %ч. ";
     ptrs[3]=" Контроль емкости   ";
     ptrs[4]=" Выравнивающий заряд";
     ptrs[5]=" Разряды            ";
     ptrs[6]=sm_exit;	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(	" БАТАРЕЙНЫЙ ЖУРНАЛ  ",
			"     БАТАРЕЯ N^     ",
			ptrs[index_set],
			ptrs[index_set+1]);
	pointer_set(2);	

	int2lcd(sub_ind1+1,'^',0); 
	int2lcd(BAT_DAY_OF_ON[sub_ind1],'!',0);
	sub_bgnd(sm_mont[BAT_MONTH_OF_ON[sub_ind1]],'@',0);
	int2lcd(BAT_YEAR_OF_ON[sub_ind1],'#',0); 
	int2lcd(BAT_C_NOM[sub_ind1],'$',0);
	int2lcd(BAT_RESURS[sub_ind1],'%',0);

	/*int2lcdyx(BAT_IS_ON[0],0,2,0);
	int2lcdyx(BAT_IS_ON[1],0,6,0); 
	int2lcdyx(//lc640_read_int(EE_BAT1_IS_ON),0,10,0);
	int2lcdyx(//lc640_read_int(EE_BAT2_IS_ON),0,14,0);*/
	}

else if(ind==iBatLogKe)
	{             
	if(av_j_si_max==0)
		{
		bgnd_par(	"  КОНТРОЛИ ЕМКОСТИ  ",
				"     БАТАРЕИ N!     ",
				" Журнал пуст        ",
				sm_exit);
		pointer_set(3);
		sub_ind=0;
		index_set=0;
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"  КОНТРОЛИ ЕМКОСТИ  ",
				"     БАТАРЕИ N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	
	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1)) 
			{
			bgnd_par( "  КОНТРОЛИ ЕМКОСТИ  ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					sm_exit);
			}
		else
			{
			bgnd_par(	"  КОНТРОЛИ ЕМКОСТИ  ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					" [                  ");
			}
		pointer_set(2);			 
		}
		
   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');
	}

else if(ind==iBatLogVz)
	{
	if(av_j_si_max==0)
		{
		bgnd_par(	"ВЫРАВНИВАЮЩИЕ ЗАРЯДЫ",
				"     БАТАРЕИ N!     ",
				" Журнал пуст        ",
				sm_exit);
		sub_ind=0;
		index_set=0;
		pointer_set(3);
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"ВЫРАВНИВАЮЩИЕ ЗАРЯДЫ",
				"     БАТАРЕИ N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	
	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1)) 
			{
			bgnd_par(	"ВЫРАВНИВАЮЩИЕ ЗАРЯДЫ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					sm_exit);
			}

		else bgnd_par(	"ВЫРАВНИВАЮЩИЕ ЗАРЯДЫ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					" [                  "); 
		pointer_set(2);			        
		}
   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');
	
	}
   
else if(ind==iBatLogWrk)
	{
	if(av_j_si_max==0)
		{
		bgnd_par(	"      РАЗРЯДЫ       ",
				"     БАТАРЕИ N!     ",
				" Журнал пуст        ",
				sm_exit);
		sub_ind=0;
		index_set=0;
		pointer_set(3);
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"      РАЗРЯДЫ       ",
				"     БАТАРЕИ N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	

	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1))
			{
			bgnd_par(	"      РАЗРЯДЫ       ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					sm_exit);
			}
		else bgnd_par(	"      РАЗРЯДЫ       ",
					"     БАТАРЕИ N!     ",
					" (                  ",
					" [                  ");

		pointer_set(2);
		}

   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');

	

	} 
	
else if((ind==iSet_prl)||(ind==iK_prl)||(ind==iSpc_prl_vz)
	||(ind==iSpc_prl_ke)||(ind==iAusw_prl)||(ind==iPrltst))
	{
	bgnd_par("  Введите  пароль   ",sm_,sm_,sm_);
	int2lcdyx(parol[0],1,8,0);
     int2lcdyx(parol[1],1,9,0);
     int2lcdyx(parol[2],1,10,0);
     lcd_buffer[48+sub_ind]='¤';
	}	
		
else if(ind==iPrl_bat_in_out)
	{
	if(BAT_IS_ON[sub_ind1]==bisON)ptrs[0]="Для выведения бат.-и";
	else  ptrs[0]="Для введения батареи";
	bgnd_par(ptrs[0],"  наберите пароль   ",sm_,sm_);
	
     int2lcdyx(parol[0],2,8,0);
     int2lcdyx(parol[1],2,9,0);
     int2lcdyx(parol[2],2,10,0);
     lcd_buffer[68+sub_ind]='¤';	
	}

else if(ind==iPrl_bat_in_sel)
	{
	
	bgnd_par(	"Для введения батареи",
			"   выбеите ее тип   ",
			" Свинцово-кислотная ",
			" GYFP4875T          ");
	
	pointer_set(2);
	}

else if(ind==iSet_bat_sel)
	{
	ptrs[0]=	" Свинцово-кислотная ";
    ptrs[1]=	" COSLIGHT POWER CO. ";
    ptrs[2]=	" SACRED SUN         ";
	ptrs[3]=	" ZTT                ";
	ptrs[4]=	" Выход              ";

	//gran(&BAT_TYPE,0,3);
	if(BAT_TYPE<0)BAT_TYPE=0;
	if(BAT_TYPE>3)BAT_TYPE=3;
	if(bFL2)ptrs[BAT_TYPE]=		"                    ";	

	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind; /**/
		
	bgnd_par(	"  ТИП ИСПОЛЬЗУЕМОЙ  ",
			"      БАТАРЕИ       ",
			ptrs[index_set],
			ptrs[index_set+1]);
	
	pointer_set(2);
	//int2lcdyx(//lc640_read_int(EE_BAT_TYPE),0,2,0);
	}
	
else if(ind==iSet)
	{
     ptrs[0]=		" Стандартные        ";
	ptrs[1]=		" Время и дата       ";
     ptrs[2]=		" Структура          ";
	ptrs[3]=		" Климатконтроль     ";
	ptrs[4]=		" Выход              ";
     ptrs[5]=		" Мнемоника         y";
	ptrs[6]=		" Зв.сигн.   (       ";
	ptrs[7]=		" Отключение сигнала ";
	ptrs[8]=		"  аварии    )       ";
	ptrs[9]=		" АПВ источников     ";
	ptrs[10]=		" Паралл.работа z    ";
	ptrs[11]=		" T проверки   цепи  ";
     ptrs[12]=		" батареи     qмин.  ";
     ptrs[13]=		" Umax=       !В     ";
     ptrs[14]=		" Umin=       ZВ     ";
     ptrs[15]=		" Uб0°=       @В     ";
     ptrs[16]=		" Uб20°=      #В     ";
     ptrs[17]=		" Uсигн=      ^В     ";
     ptrs[18]=		" Umin.сети=  &В     ";
	ptrs[19]=		" U0б=        >В     ";
	ptrs[20]=		" Iбк.=       jА     ";
     ptrs[21]=		" Iз.мах.=    JА     ";
     ptrs[22]=		" Imax =      ]A     ";
     ptrs[23]=		" Imin =      {A     ";
     ptrs[24]=		" Uвыр.зар.=   [В    ";
     ptrs[25]=		" Tз.вкл.а.с. !с     ";
	ptrs[26]=		" tи.max=     $°C    ";
	ptrs[27]=		" tи.сигн=    z°C    ";
	ptrs[28]=		" tбат.max=   b°C    ";
	ptrs[29]=		" tбат.сигн=  X°C    ";
     ptrs[30]=		" Внешние датчики    ";
	ptrs[31]=		" Ethernet           ";
     ptrs[32]=      " Адрес счетчика    +";
     ptrs[33]=      " Контроль ср.точки  ";
     ptrs[34]=      " батареи         Q% ";
	ptrs[35]=      " Серийный N        w";
     ptrs[36]=		" Выход              ";
     ptrs[37]=		" Калибровки         "; 
     ptrs[38]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     УСТАНОВКИ      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<19)
	     {
	     if(ZV_ON)sub_bgnd("ВКЛ.",'(',0);
	     else sub_bgnd("ВЫК.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("автом.",')',0);
	     else sub_bgnd("ручн.",')',0);
		if(PAR)sub_bgnd("ВКЛ.",'z',0);
	     else sub_bgnd("ВЫК.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("через yс.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("выкл.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     int2lcd((UB20-DU),'Z',1);
	     int2lcd(UB0,'@',1);
	     int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
	     int2lcd(U0B,'>',1);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
	int2lcd(UVZ,'[',1);
	int2lcd(IMAX,']',1);
	int2lcd(IMIN,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	int2lcd(TBAT,'q',0);
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("ВЫКЛ.",'Q',-2);


	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);
	}

else if(ind==iSet_RSTKM)
	{
     ptrs[0]=		" Стандартные        ";
	ptrs[1]=		" Время и дата       ";
     ptrs[2]=		" Структура          ";
	ptrs[3]=		" Климатконтроль     ";
	ptrs[4]=		" Выход              ";
     ptrs[5]=		" Мнемоника         y";
	ptrs[6]=		" Зв.сигн.   (       ";
	ptrs[7]=		" Отключение сигнала ";
	ptrs[8]=		"  аварии    )       ";
	ptrs[9]=		" АПВ источников     ";
	ptrs[10]=		" Паралл.работа z    ";
	ptrs[11]=		" T проверки   цепи  ";
     ptrs[12]=		" батареи     qмин.  ";
     ptrs[13]=		" Umax=       !В     ";
     ptrs[14]=		" Umin=       ZВ     ";
     ptrs[15]=		" Uб0°=       @В     ";
     ptrs[16]=		" Uб20°=      #В     ";
     ptrs[17]=		" Uсигн=      ^В     ";
     ptrs[18]=		" Umin.сети=  &В     ";
	ptrs[19]=		" U0б=        >В     ";
	ptrs[20]=		" Iбк.=       jА     ";
     ptrs[21]=		" Iз.мах.=    JА     ";
     ptrs[22]=		" Imax =      ]A     ";
     ptrs[23]=		" Imin =      {A     ";
     ptrs[24]=		" Uвыр.зар.=   [В    ";
     ptrs[25]=		" Tз.вкл.а.с. !с     ";
	ptrs[26]=		" tи.max=     $°C    ";
	ptrs[27]=		" tи.сигн=    z°C    ";
	ptrs[28]=		" tбат.max=   b°C    ";
	ptrs[29]=		" tбат.сигн=  X°C    ";
     ptrs[30]=		" Внешние датчики    ";
	ptrs[31]=		" Ethernet           ";
     ptrs[32]=      " Адрес счетчика    +";
     ptrs[33]=      " Контроль ср.точки  ";
     ptrs[34]=      " батареи         Q% ";
	ptrs[35]=      " Серийный N        w";
     ptrs[36]=		" Выход              ";
     ptrs[37]=		" Калибровки         "; 
     ptrs[38]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     УСТАНОВКИ      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<19)
	     {
	     if(ZV_ON)sub_bgnd("ВКЛ.",'(',0);
	     else sub_bgnd("ВЫК.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("автом.",')',0);
	     else sub_bgnd("ручн.",')',0);
		if(PAR)sub_bgnd("ВКЛ.",'z',0);
	     else sub_bgnd("ВЫК.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("через yс.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("выкл.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     int2lcd((UB20-DU),'Z',1);
	     int2lcd(UB0,'@',1);
	     int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
	     int2lcd(U0B,'>',1);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
	int2lcd(UVZ,'[',1);
	int2lcd(IMAX,']',1);
	int2lcd(IMIN,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	int2lcd(TBAT,'q',0);
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("ВЫКЛ.",'Q',-2);


	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);
	}

else if(ind==iSet_3U)
	{
     ptrs[0]=		" Стандартные        ";
	ptrs[1]=		" Время и дата       ";
     ptrs[2]=		" Структура          ";
	ptrs[3]=		" Выход              ";
     ptrs[4]=		" Мнемоника         y";
	ptrs[5]=		" Зв.сигн.   (       ";
	ptrs[6]=		" Отключение сигнала ";
	ptrs[7]=		"  аварии    )       ";
	ptrs[8]=		" АПВ источников     ";
	ptrs[9]=		" Паралл.работа z    ";
	ptrs[10]=		" T проверки   цепи  ";
     ptrs[11]=		" батареи     qмин.  ";
     ptrs[12]=		" Umax=       !В     ";
     ptrs[13]=		" Umin=       ZВ     ";
     ptrs[14]=		" Uб0°=       @В     ";
     ptrs[15]=		" Uб20°=      #В     ";
     ptrs[16]=		" Uсигн=      ^В     ";
     ptrs[17]=		" Umin.сети=  &В     ";
	ptrs[18]=		" U0б=        >В     ";
	ptrs[19]=		" Iбк.=       jА     ";
     ptrs[20]=		" Iз.мах.=    JА     ";
     ptrs[21]=		" Imax =      ]A     ";
     ptrs[22]=		" Imin =      {A     ";
     ptrs[23]=		" Uвыр.зар.=   [В    ";
     ptrs[24]=		" Tз.вкл.а.с. !с     ";
	ptrs[25]=		" tи.max=     $°C    ";
	ptrs[26]=		" tи.сигн=    z°C    ";
	ptrs[27]=		" tбат.max=   b°C    ";
	ptrs[28]=		" tбат.сигн=  X°C    ";
     ptrs[29]=		" Внешние датчики    ";
	ptrs[30]=		" Ethernet           ";
	ptrs[31]=      " Серийный N        w";
     ptrs[32]=		" Выход              ";
     ptrs[33]=		" Калибровки         "; 
     ptrs[34]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     УСТАНОВКИ      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<19)
	     {
	     if(ZV_ON)sub_bgnd("ВКЛ.",'(',0);
	     else sub_bgnd("ВЫК.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("автом.",')',0);
	     else sub_bgnd("ручн.",')',0);
		if(PAR)sub_bgnd("ВКЛ.",'z',0);
	     else sub_bgnd("ВЫК.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("через yс.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("выкл.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     int2lcd((UB20-DU),'Z',1);
	     int2lcd(UB0,'@',1);
	     int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
	     int2lcd(U0B,'>',1);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
	int2lcd(UVZ,'[',1);
	int2lcd(IMAX,']',1);
	int2lcd(IMIN,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	int2lcd(TBAT,'q',0);
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("ВЫКЛ.",'Q',-2);


	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);
	}

else if(ind==iSet_GLONASS)
	{
     ptrs[0]=		" Стандартные        ";
	ptrs[1]=		" Время и дата       ";
     ptrs[2]=		" Структура          ";
	ptrs[3]=		" Выход              ";
     ptrs[4]=		" Мнемоника         y";
	ptrs[5]=		" Зв.сигн.   (       ";
	ptrs[6]=		" Отключение сигнала ";
	ptrs[7]=		"  аварии    )       ";
	ptrs[8]=		" АПВ источников     ";
	ptrs[9]=		" Паралл.работа z    ";
	ptrs[10]=		" T проверки   цепи  ";
     ptrs[11]=		" батареи     qмин.  ";
     ptrs[12]=		" Umax=       !В     ";
     ptrs[13]=		" Umin=       ZВ     ";
     ptrs[14]=		" Uб0°=       @В     ";
     ptrs[15]=		" Uб20°=      #В     ";
     ptrs[16]=		" Uсигн=      ^В     ";
     ptrs[17]=		" Umin.сети=  &В     ";
	ptrs[18]=		" U0б=        >В     ";
	ptrs[19]=		" Iбк.=       jА     ";
     ptrs[20]=		" Iз.мах.=    JА     ";
     ptrs[21]=		" Imax =      ]A     ";
     ptrs[22]=		" Imin =      {A     ";
     ptrs[23]=		" Uвыр.зар.=   [В    ";
     ptrs[24]=		" Tз.вкл.а.с. !с     ";
	ptrs[25]=		" tи.max=     $°C    ";
	ptrs[26]=		" tи.сигн=    z°C    ";
	ptrs[27]=		" tбат.max=   b°C    ";
	ptrs[28]=		" tбат.сигн=  X°C    ";
     ptrs[29]=		" Внешние датчики    ";
	ptrs[30]=		" Ethernet           ";
	ptrs[31]=      " Серийный N        w";
     ptrs[32]=		" Выход              ";
     ptrs[33]=		" Калибровки         "; 
     ptrs[34]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     УСТАНОВКИ      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<19)
	     {
	     if(ZV_ON)sub_bgnd("ВКЛ.",'(',0);
	     else sub_bgnd("ВЫК.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("автом.",')',0);
	     else sub_bgnd("ручн.",')',0);
		if(PAR)sub_bgnd("ВКЛ.",'z',0);
	     else sub_bgnd("ВЫК.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("через yс.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("выкл.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     int2lcd((UB20-DU),'Z',1);
	     int2lcd(UB0,'@',1);
	     int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
	     int2lcd(U0B,'>',1);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
	int2lcd(UVZ,'[',1);
	int2lcd(IMAX,']',1);
	int2lcd(IMIN,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	int2lcd(TBAT,'q',0);
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("ВЫКЛ.",'Q',-2);


	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);
	}

else if(ind==iSet_KONTUR)
	{
     ptrs[0]=		" Стандартные        ";
	ptrs[1]=		" Время и дата       ";
     ptrs[2]=		" Структура          ";
	ptrs[3]=		" Климатконтроль     ";
	ptrs[4]=		" Выход              ";
     ptrs[5]=		" Мнемоника         y";
	ptrs[6]=		" Зв.сигн.   (       ";
	ptrs[7]=		" Отключение сигнала ";
	ptrs[8]=		"  аварии    )       ";
	ptrs[9]=		" АПВ источников     ";
	ptrs[10]=		" Паралл.работа z    ";
	ptrs[11]=		" T проверки   цепи  ";
     ptrs[12]=		" батареи     qмин.  ";
     ptrs[13]=		" Umax=       !В     ";
     ptrs[14]=		" Umin=       ZВ     ";
     ptrs[15]=		" Uб0°=       @В     ";
     ptrs[16]=		" Uб20°=      #В     ";
     ptrs[17]=		" Uсигн=      ^В     ";
     ptrs[18]=		" Umin.сети=  &В     ";
	ptrs[19]=		" U0б=        >В     ";
	ptrs[20]=		" Iбк.=       jА     ";
     ptrs[21]=		" Iз.мах.=    JА     ";
     ptrs[22]=		" Imax =      ]A     ";
     ptrs[23]=		" Imin =      {A     ";
     ptrs[24]=		" Uвыр.зар.=   [В    ";
     ptrs[25]=		" Tз.вкл.а.с. !с     ";
	ptrs[26]=		" tи.max=     $°C    ";
	ptrs[27]=		" tи.сигн=    z°C    ";
	ptrs[28]=		" tбат.max=   b°C    ";
	ptrs[29]=		" tбат.сигн=  X°C    ";
     ptrs[30]=		" Внешние датчики    ";
	ptrs[31]=		" Ethernet           ";
     ptrs[32]=      " Адрес счетчика    +";
     ptrs[33]=      " Контроль ср.точки  ";
     ptrs[34]=      " батареи         Q% ";
	ptrs[35]=      " Серийный N        w";
     ptrs[36]=      " Управление реле    ";
     ptrs[37]=      "                  ( ";
     ptrs[38]=		" Выход              ";
     ptrs[39]=		" Калибровки         "; 
     ptrs[40]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     УСТАНОВКИ      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<19)
	     {
	     if(ZV_ON)sub_bgnd("ВКЛ.",'(',0);
	     else sub_bgnd("ВЫК.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("автом.",')',0);
	     else sub_bgnd("ручн.",')',0);
		if(PAR)sub_bgnd("ВКЛ.",'z',0);
	     else sub_bgnd("ВЫК.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("через yс.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("выкл.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     int2lcd((UB20-DU),'Z',1);
	     int2lcd(UB0,'@',1);
	     int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
	     int2lcd(U0B,'>',1);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
	int2lcd(UVZ,'[',1);
	int2lcd(IMAX,']',1);
	int2lcd(IMIN,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	int2lcd(TBAT,'q',0);
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("ВЫКЛ.",'Q',-2);

	if(index_set>19)
	     {
		if(RELE_LOG)sub_bgnd("отопитель",'(',-8);
		else sub_bgnd("вентилятор",'(',-9);
		}

	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);
	}

else if(ind==iSet_6U)
	{
    	ptrs[0]=		" Стандартные        ";
	ptrs[1]=		" Время и дата       ";
    	ptrs[2]=		" Структура          ";
	ptrs[3]=		" Выход              ";
    	ptrs[4]=		" Мнемоника         y";
	ptrs[5]=		" Зв.сигн.   (       ";
	ptrs[6]=		" Отключение сигнала ";
	ptrs[7]=		"  аварии    )       ";
	ptrs[8]=		" АПВ источников     ";
	ptrs[9]=		" Паралл.работа z    ";
	ptrs[10]=		" T проверки   цепи  ";
    	ptrs[11]=		" батареи     qмин.  ";
    	ptrs[12]=		" Umax=       !В     ";
    	ptrs[13]=		" Umin=       ZВ     ";
    	ptrs[14]=		" Uб0°=       @В     ";
    	ptrs[15]=		" Uб20°=      #В     ";
    	ptrs[16]=		" Uсигн=      ^В     ";
    	ptrs[17]=		" Umin.сети=  &В     ";
	ptrs[18]=		" U0б=        >В     ";
	ptrs[19]=		" Iбк.=       jА     ";
    	ptrs[20]=		" Iз.мах.=    JА     ";
    	ptrs[21]=		" Imax =      ]A     ";
    	ptrs[22]=		" Imin =      {A     ";
    	ptrs[23]=		" Uвыр.зар.=   [В    ";
    	ptrs[24]=		" Tз.вкл.а.с. !с     ";
	ptrs[25]=		" tи.max=     $°C    ";
	ptrs[26]=		" tи.сигн=    z°C    ";
	ptrs[27]=		" tбат.max=   b°C    ";
	ptrs[28]=		" tбат.сигн=  X°C    ";
	ptrs[29]=		" tвент.вкл.  =  (°C ";
	ptrs[30]=		" tвент.выкл. =  )°C ";
	ptrs[31]=		" Сигнал для вентиля-";
	ptrs[32]=		" тора          >    ";
	ptrs[33]=		" Отключение низко-  ";
	ptrs[34]=		" приоритетной нагр. ";
    	ptrs[35]=		" Внешние датчики    ";
	ptrs[36]=		" Ethernet           ";
	ptrs[37]=      " Серийный N        w";
	ptrs[38]=      " Тип батареи        ";
	ptrs[39]=      " Инверторы          ";
	ptrs[40]=      " Время ротации      ";
	ptrs[41]=      " источников    lчас.";
    	ptrs[42]=		" Выход              ";
    	ptrs[43]=		" Калибровки         "; 
    	ptrs[44]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     УСТАНОВКИ      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<19)
	     {
	     if(ZV_ON)sub_bgnd("ВКЛ.",'(',0);
	     else sub_bgnd("ВЫК.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("автом.",')',0);
	     else sub_bgnd("ручн.",')',0);
		if(PAR)sub_bgnd("ВКЛ.",'z',0);
	     else sub_bgnd("ВЫК.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("через yс.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("выкл.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     int2lcd((UB20-DU),'Z',1);
	     int2lcd(UB0,'@',1);
	     int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
	     int2lcd(U0B,'>',1);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
	int2lcd(UVZ,'[',1);
	int2lcd(IMAX,']',1);
	int2lcd(IMIN,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	int2lcd(TBAT,'q',0);
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
	int2lcd(TVENTON,'(',0); 
	int2lcd(TVENTOFF,')',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("ВЫКЛ.",'Q',-2);

	if(RELEVENTSIGN==rvsAKB)sub_bgnd("Tакб.макс.",'>',-5);
	else if(RELEVENTSIGN==rvsBPS)sub_bgnd("Tбпс.макс.",'>',-5);
	else sub_bgnd("Tвн.датч.",'>',-5);

	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);

	if((FORVARDBPSCHHOUR<=0)||(FORVARDBPSCHHOUR>500)) {
		sub_bgnd("ВЫКЛ.",'l',0);
	} else {
		int2lcd(FORVARDBPSCHHOUR,'l',0);	
	}

	//int2lcdyx(sub_ind,0,3,0);
	//int2lcdyx(index_set,0,1,0);
	}

else if(ind==iSet_IP55)
	{
    ptrs[0]=		" Стандартные        ";
	ptrs[1]=		" Время и дата       ";
    ptrs[2]=		" Структура          ";
	ptrs[3]=		" Выход              ";
    ptrs[4]=		" Мнемоника         y";
	ptrs[5]=		" Зв.сигн.   (       ";
	ptrs[6]=		" Отключение сигнала ";
	ptrs[7]=		"  аварии    )       ";
	ptrs[8]=		" АПВ источников     ";
	ptrs[9]=		" Паралл.работа z    ";
	ptrs[10]=		" T проверки   цепи  ";
    ptrs[11]=		" батареи     qмин.  ";
   	ptrs[12]=		" Umax=       !В     ";
    ptrs[13]=		" Umin=       ZВ     ";
    ptrs[14]=		" Uб0°=       @В     ";
    ptrs[15]=		" Uб20°=      #В     ";
    ptrs[16]=		" Uсигн=      ^В     ";
    ptrs[17]=		" Umin.сети=  &В     ";
	ptrs[18]=		" U0б=        >В     ";
	ptrs[19]=		" Iбк.=       jА     ";
    ptrs[20]=		" Iз.мах.=    JА     ";
    ptrs[21]=		" Imax =      ]A     ";
    ptrs[22]=		" Imin =      {A     ";
    ptrs[23]=		" Uвыр.зар.=   [В    ";
    ptrs[24]=		" Tз.вкл.а.с. !с     ";
	ptrs[25]=		" tи.max=     $°C    ";
	ptrs[26]=		" tи.сигн=    z°C    ";
	ptrs[27]=		" tбат.max=   b°C    ";
	ptrs[28]=		" tбат.сигн=  X°C    ";
	ptrs[29]=		" Климатконтроль     ";
    ptrs[30]=		" Внешние датчики    ";
	ptrs[31]=		" Ethernet           ";
	ptrs[32]=      	" Серийный N        w";
	ptrs[33]=		" Параметры литиевой ";
	ptrs[34]=		" батареи            ";
	ptrs[35]=      	" Инверторы          ";
	ptrs[36]=      	" Время ротации      ";
	ptrs[37]=      	" источников    lчас.";
    ptrs[38]=		" Выход              ";
    ptrs[39]=		" Калибровки         "; 
    ptrs[40]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     УСТАНОВКИ      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<19)
	     {
	     if(ZV_ON)sub_bgnd("ВКЛ.",'(',0);
	     else sub_bgnd("ВЫК.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("автом.",')',0);
	     else sub_bgnd("ручн.",')',0);
		if(PAR)sub_bgnd("ВКЛ.",'z',0);
	     else sub_bgnd("ВЫК.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("через yс.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("выкл.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     int2lcd((UB20-DU),'Z',1);
	     int2lcd(UB0,'@',1);
	     int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
	     int2lcd(U0B,'>',1);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
	int2lcd(UVZ,'[',1);
	int2lcd(IMAX,']',1);
	int2lcd(IMIN,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	int2lcd(TBAT,'q',0);
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
	int2lcd(TVENTON,'(',0); 
	int2lcd(TVENTOFF,')',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
	int2lcd(DU_LI_BAT,'W',1);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("ВЫКЛ.",'Q',-2);

	if(RELEVENTSIGN==rvsAKB)sub_bgnd("Tакб.макс.",'>',-5);
	else if(RELEVENTSIGN==rvsBPS)sub_bgnd("Tбпс.макс.",'>',-5);
	else sub_bgnd("Tвн.датч.",'>',-5);

	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);


	if((FORVARDBPSCHHOUR<=0)||(FORVARDBPSCHHOUR>500)) {
		sub_bgnd("ВЫКЛ.",'l',0);
	} else {
		int2lcd(FORVARDBPSCHHOUR,'l',0);	
	}


	//int2lcdyx(sub_ind,0,3,0);
	//int2lcdyx(index_set,0,1,0);
	}

else if((ind==iSet_220))
	{
     ptrs[0]=		" Стандартные        ";
	ptrs[1]=		" Время и дата       ";
     ptrs[2]=		" Структура          ";
	ptrs[3]=		" Выход              ";
     ptrs[4]=		" Мнемоника         y";
	ptrs[5]=		" Зв.сигн.   (       ";
	ptrs[6]=		" Отключение сигнала ";
	ptrs[7]=		"  аварии    )       ";
	ptrs[8]=		" АПВ источников     ";
	ptrs[9]=		" Паралл.работа z    ";
	ptrs[10]=		" T проверки   цепи  ";
     ptrs[11]=		" батареи     qмин.  ";
     ptrs[12]=		" Umax=       !В     ";
     ptrs[13]=		" Umin=       ZВ     ";
     ptrs[14]=		" Uб0°=       @В     ";
     ptrs[15]=		" Uб20°=      #В     ";
     ptrs[16]=		" Uсигн=      ^В     ";
     ptrs[17]=		" Umin.сети=  &В     ";
	ptrs[18]=		" U0б=        >В     ";
	ptrs[19]=		" Iбк.=       jА     ";
     ptrs[20]=		" Iз.мах.=    JА     ";
     ptrs[21]=		" Imax =      ]A     ";
     ptrs[22]=		" Imin =      {A     ";
     ptrs[23]=		" Uвыр.зар.=   [В    ";
     ptrs[24]=		" Tз.вкл.а.с. !с     ";
	ptrs[25]=		" tи.max=     $°C    ";
	ptrs[26]=		" tи.сигн=    z°C    ";
	ptrs[27]=		" tбат.max=   b°C    ";
	ptrs[28]=		" tбат.сигн=  X°C    ";
     ptrs[29]=		" Внешние датчики    ";
	ptrs[30]=		" Ethernet           ";
	ptrs[31]=      " Серийный N        w";
     ptrs[32]=      " Контроль ср.точки  ";
     ptrs[33]=      " батареи         Q% ";
	ptrs[34]=		" MODBUS ADRESS     <";
	ptrs[35]=		" MODBUS BAUDRATE    ";
	ptrs[36]=		"                  >0";
     ptrs[37]=		" Выход              ";
     ptrs[38]=		" Калибровки         "; 
     ptrs[39]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     УСТАНОВКИ      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<19)
	     {
	     if(ZV_ON)sub_bgnd("ВКЛ.",'(',0);
	     else sub_bgnd("ВЫК.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("автом.",')',0);
	     else sub_bgnd("ручн.",')',0);
		if(PAR)sub_bgnd("ВКЛ.",'z',0);
	     else sub_bgnd("ВЫК.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("через yс.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("выкл.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     int2lcd((UB20-DU),'Z',1);
	     int2lcd(UB0,'@',1);
	     int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
	     int2lcd(U0B,'>',1);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
	int2lcd(UVZ,'[',1);
	int2lcd(IMAX,']',1);
	int2lcd(IMIN,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	int2lcd(TBAT,'q',0);
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("ВЫКЛ.",'Q',-2);


	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);

	int2lcd(MODBUS_ADRESS,'<',0);
	int2lcd(MODBUS_BAUDRATE,'>',0);
	}

else if((ind==iSet_220_V2))
	{
     ptrs[0]=		" Стандартные        ";
	ptrs[1]=		" Время и дата       ";
     ptrs[2]=		" Структура          ";
	ptrs[3]=		" Выход              ";
     ptrs[4]=		" Мнемоника         y";
	ptrs[5]=		" Зв.сигн.   (       ";
	ptrs[6]=		" Отключение сигнала ";
	ptrs[7]=		"  аварии    )       ";
	ptrs[8]=		" АПВ источников     ";
	ptrs[9]=		" Паралл.работа z    ";
	ptrs[10]=		" T проверки   цепи  ";
     ptrs[11]=		" батареи     qмин.  ";
     ptrs[12]=		" Umax=       !В     ";
     ptrs[13]=		" Umin=       ZВ     ";
     ptrs[14]=		" Uб0°=       @В     ";
     ptrs[15]=		" Uб20°=      #В     ";
     ptrs[16]=		" Uсигн=      ^В     ";
     ptrs[17]=		" Umin.сети=  &В     ";
	ptrs[18]=		" U0б=        >В     ";
	ptrs[19]=		" Iбк.=       jА     ";
     ptrs[20]=		" Iз.мах.=    JА     ";
     ptrs[21]=		" Imax =      ]A     ";
     ptrs[22]=		" Imin =      {A     ";
     ptrs[23]=		" Uвыр.зар.=   [В    ";
     ptrs[24]=		" Tз.вкл.а.с. !с     ";
	ptrs[25]=		" tи.max=     $°C    ";
	ptrs[26]=		" tи.сигн=    z°C    ";
	ptrs[27]=		" tбат.max=   b°C    ";
	ptrs[28]=		" tбат.сигн=  X°C    ";
     ptrs[29]=		" Внешние датчики    ";
	ptrs[30]=		" Ethernet           ";
	ptrs[31]=      " Серийный N        w";
 /*    ptrs[32]=      " Контроль ср.точки  ";
     ptrs[33]=      " батареи         Q% ";*/
     ptrs[32]=		" Выход              ";
     ptrs[33]=		" Калибровки         "; 
     ptrs[34]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     УСТАНОВКИ      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<19)
	     {
	     if(ZV_ON)sub_bgnd("ВКЛ.",'(',0);
	     else sub_bgnd("ВЫК.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("автом.",')',0);
	     else sub_bgnd("ручн.",')',0);
		if(PAR)sub_bgnd("ВКЛ.",'z',0);
	     else sub_bgnd("ВЫК.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("через yс.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("выкл.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     int2lcd((UB20-DU),'Z',1);
	     int2lcd(UB0,'@',1);
	     int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
	     int2lcd(U0B,'>',1);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
	int2lcd(UVZ,'[',1);
	int2lcd(IMAX,']',1);
	int2lcd(IMIN,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	int2lcd(TBAT,'q',0);
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("ВЫКЛ.",'Q',-2);


	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);
	}

else if((ind==iSet_220_IPS_TERMOKOMPENSAT))
	{
    ptrs[0]=		" Стандартные        ";
	ptrs[1]=		" Время и дата       ";
    ptrs[2]=		" Структура          ";
	ptrs[3]=		" Выход              ";
	ptrs[4]=		" Зв.сигн.   (       ";
	ptrs[5]=		" Отключение сигнала ";
	ptrs[6]=		"  аварии    )       ";
	ptrs[7]=		" АПВ источников     ";
	ptrs[8]=		" Паралл.работа z    ";
	ptrs[9]=		" T проверки   цепи  ";
    ptrs[10]=		" батареи     qмин.  ";
    ptrs[11]=		" Umax=       !В     ";
    ptrs[12]=		" Umin=       ZВ     ";
    ptrs[13]=		" Uб0°=       @В     ";
    ptrs[14]=		" Uб20°=      #В     ";
    ptrs[15]=		" Uб.сигн=    ^В     ";
    ptrs[16]=		" Umin.сети=  &В     ";
	ptrs[17]=		" Iбк.=       jА     ";
    ptrs[18]=		" Iз.мах.=    JА     ";
    ptrs[19]=		" Imax =      ]A     ";
    ptrs[20]=		" Imin =      {A     ";
    ptrs[21]=		" Uвыр.зар.=   [В    ";
    ptrs[22]=		" Tз.вкл.а.с. !с     ";
	ptrs[23]=		" tи.max=     $°C    ";
	ptrs[24]=		" tи.сигн=    z°C    ";
	ptrs[25]=		" tбат.max=   b°C    ";
	ptrs[26]=		" tбат.сигн=  X°C    ";
    ptrs[27]=		" Внешние датчики    ";
    ptrs[28]=      	" Контроль выходного ";
    ptrs[29]=      	" напряжения         ";
	ptrs[30]=      	" Термокомпенс.     q";
	ptrs[31]=		" Ускоренный заряд   ";
	ptrs[32]=      	" Время ротации      ";
	ptrs[33]=      	" источников    lчас.";
	ptrs[34]=      	" Автономная работа  ";
	ptrs[35]=      	" источников         ";
	ptrs[36]=      	" Дополнительное реле";
	ptrs[37]=      	" Блокировка ИПС     ";
	ptrs[38]=      	" Серийный N        w";
	ptrs[39]=		" MODBUS ADRESS     <";
	ptrs[40]=		" MODBUS BAUDRATE    ";
	ptrs[41]=		"                  >0";
	ptrs[42]=		" Ethernet           ";
    ptrs[43]=		" Выход              ";
    ptrs[44]=		" Калибровки         "; 
    ptrs[45]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     УСТАНОВКИ      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<19)
	     {
	     if(ZV_ON)sub_bgnd("ВКЛ.",'(',0);
	     else sub_bgnd("ВЫК.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("автом.",')',0);
	     else sub_bgnd("ручн.",')',0);
		if(PAR)sub_bgnd("ВКЛ.",'z',0);
	     else sub_bgnd("ВЫК.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("через yс.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("выкл.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     int2lcd((UB20-DU),'Z',1);
	     int2lcd(UB0,'@',1);
	     int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
		if(TBAT==0)sub_bgnd("выкл.",'q',0);	
		else int2lcd(TBAT,'q',0);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
	int2lcd(UVZ,'[',1);
	int2lcd(IMAX,']',1);
	int2lcd(IMIN,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("ВЫКЛ.",'Q',-2);


	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);
	if(index_set>19)
		{
		if(TERMOKOMPENS)sub_bgnd("ВКЛ.",'q',-3);
		else sub_bgnd("ВЫКЛ.",'q',-4);
		}
	int2lcd(MODBUS_ADRESS,'<',0);
	int2lcd(MODBUS_BAUDRATE,'>',0);

	if((FORVARDBPSCHHOUR<=0)||(FORVARDBPSCHHOUR>500)) {
		sub_bgnd("ВЫКЛ.",'l',0);
	} else {
		int2lcd(FORVARDBPSCHHOUR,'l',0);	
	}

	//int2lcdyx(sub_ind,0,3,0);
	//int2lcdyx(index_set,0,1,0);
	}


else if (ind==iDef)

	{ 
	ptrs[0]=" ИБЭП220/48-40А     ";
	ptrs[1]=" ИБЭП220/48-60А     ";
	ptrs[2]=sm_exit;
	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par("СТАНДАРТНЫЕ УСТ.-КИ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	} 

else if (ind==iDef_RSTKM)

	{ 
	ptrs[0]=" ИБЭП220/48-40А     ";
	ptrs[1]=" ИБЭП220/48-60А     ";
	ptrs[2]=sm_exit;
	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par("СТАНДАРТНЫЕ УСТ.-КИ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	} 

else if (ind==iDef_3U)

	{ 
	ptrs[0]=" ИБЭП220/48-40А     ";
	ptrs[1]=" ИБЭП220/48-60А     ";
	ptrs[2]=" ИБЭП220/60-40А     ";
	ptrs[3]=" ИБЭП220/60-60А     ";

	ptrs[4]=sm_exit;
	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par("СТАНДАРТНЫЕ УСТ.-КИ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	} 

else if (ind==iDef_GLONASS)

	{ 
	ptrs[0]=" ИБЭП220/48-40А     ";
	ptrs[1]=" ИБЭП220/48-60А     ";
	ptrs[2]=" ИБЭП220/60-40А     ";
	ptrs[3]=" ИБЭП220/60-60А     ";

	ptrs[4]=sm_exit;
	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par("СТАНДАРТНЫЕ УСТ.-КИ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	}
	 
else if (ind==iDef_KONTUR)

	{ 
	ptrs[0]=" ИБЭП220/48-40А     ";
	ptrs[1]=" ИБЭП220/48-60А     ";
	ptrs[2]=sm_exit;
	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par("СТАНДАРТНЫЕ УСТ.-КИ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	} 
else if (ind==iDef_6U)
	{ 
	ptrs[0]=	" ИБЭП220/24-120А-2/4";
	ptrs[1]=	" ИБЭП220/24-120А-3/4";
	ptrs[2]=	" ИБЭП220/24-120А-4/4";
	ptrs[3]=	" ИБЭП220/24-210А-3/7";
	ptrs[4]=	" ИБЭП220/24-210А-4/7";
	ptrs[5]=	" ИБЭП220/24-210А-5/7";
	ptrs[6]=	" ИБЭП220/24-210А-6/7";
	ptrs[7]=	" ИБЭП220/24-210А-7/7";
	ptrs[8]=	" ИБЭП220/48-80А-2/4 ";
	ptrs[9]=	" ИБЭП220/48-80А-3/4 ";
	ptrs[10]=	" ИБЭП220/48-80А-4/4 ";
	ptrs[11]=	" ИБЭП220/48-140А-3/7";
	ptrs[12]=	" ИБЭП220/48-140А-4/7";
	ptrs[13]=	" ИБЭП220/48-140А-5/7";
	ptrs[14]=	" ИБЭП220/48-140А-6/7";
	ptrs[15]=	" ИБЭП220/48-140А-7/7";
	ptrs[16]=	" ИБЭП220/60-80А-2/4 ";
	ptrs[17]=	" ИБЭП220/60-80А-3/4 ";
	ptrs[18]=	" ИБЭП220/60-80А-4/4 ";
	ptrs[19]=	" ИБЭП220/60-140А-3/7";
	ptrs[20]=	" ИБЭП220/60-140А-4/7";
	ptrs[21]=	" ИБЭП220/60-140А-5/7";
	ptrs[22]=	" ИБЭП220/60-140А-6/7";
	ptrs[23]=	" ИБЭП220/60-140А-7/7";
	ptrs[24]=	" ИБЭП380/24-120А-4/4";
	ptrs[25]=	" ИБЭП380/24-210А-7/7";
	ptrs[26]=	" ИБЭП380/48-80А-4/4 ";
	ptrs[27]=	" ИБЭП380/48-140А-7/7";
	ptrs[28]=	" ИБЭП380/60-80А-4/4 ";
	ptrs[29]=	" ИБЭП380/60-140А-7/7";
	ptrs[30]=	" ИПС7000-24В-7/7    ";
	ptrs[31]=	" ИПС7000-48В-7/7    ";
	ptrs[32]=	" ИПС7000-60В-7/7    ";
	ptrs[33]=	" ИПС7000-380-24В-7/7";
	ptrs[34]=	" ИПС7000-380-48В-7/7";
	ptrs[35]=	" ИПС7000-380-60В-7/7";
	ptrs[36]=	sm_exit;

	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

    bgnd_par("СТАНДАРТНЫЕ УСТ.-КИ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	} 

else if (ind==iDef_220)
	{ 
	ptrs[0]=" ИБЭП220/220В-10А   ";
	ptrs[1]=" ИБЭП220/220В-10А-17";
	ptrs[2]=" ИБЭП220/220В-20А   ";
	ptrs[3]=" ИБЭП220/220В-20А-17";
	ptrs[4]=" ИБЭП380/220В-20А   ";
	ptrs[5]=" ИБЭП380/220В-20А-17";
	ptrs[6]=" ИБЭП3x220/220В-35А ";
	ptrs[7]=" ИПC3x220/220В-35А  ";
	
	ptrs[8]=sm_exit;
	ptrs[9]="                    ";
	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

    bgnd_par("СТАНДАРТНЫЕ УСТ.-КИ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	} 	

else if (ind==iDef_220_V2)
	{ 
	ptrs[0]=" AC220/220-20A-18   ";
	ptrs[1]=" AC220/220-20A-17   ";
	ptrs[2]=" AC380/220-45A-18   ";
	
	ptrs[3]=sm_exit;
	ptrs[4]="                    ";
	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

    bgnd_par("СТАНДАРТНЫЕ УСТ.-КИ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	} 

else if (ind==iDef_220_IPS_TERMOKOMPENSAT)
	{ 
	ptrs[0]=" ИПС380/220-45АТКИ17";
	ptrs[1]=" ИПС220/220-10АТКИ17";
	ptrs[2]=sm_exit;
	ptrs[3]="                    ";
	ptrs[4]="                    ";
	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

    	bgnd_par("СТАНДАРТНЫЕ УСТ.-КИ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	} 	

else if(ind==iSet_T)
	{
	static char phase_cnt;
	if(++phase_cnt>=15)
	     {
	     phase_cnt=0;
	     if(++phase>=3)phase=0;
	     }
	ptrs[0]=sm_time;
	ptrs[1]=sm_;
	if(phase==0)ptrs[2]="     <> - выбор     ";
     if(phase==1)ptrs[2]="   ^v - установка   ";
     if(phase==2)ptrs[2]="     ¤  - выход     ";
	
	bgnd_par(" УСТАНОВКА  ВРЕМЕНИ ",ptrs[0],ptrs[1],ptrs[2]);
     if(sub_ind==0)lcd_buffer[42]='^';
     else if(sub_ind==1)lcd_buffer[45]='^';
     else if(sub_ind==2)lcd_buffer[48]='^';
     else if(sub_ind==3)lcd_buffer[51]='^';
     else if(sub_ind==4)lcd_buffer[54]='^';
     else if(sub_ind==5)lcd_buffer[58]='^';
  
 	int2lcd(LPC_RTC->SEC,'&',0);
 	int2lcd(LPC_RTC->MIN,'^',0);
 	int2lcd(LPC_RTC->HOUR,'%',0);
 	
 	int2lcd(LPC_RTC->DOM,'<',0);
 	sub_bgnd(sm_mont[LPC_RTC->MONTH],'>',0);
 	int2lcd(LPC_RTC->YEAR,'{',0);
 	if(bFL2)
 	     {
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     }  
	}  

else if(ind==iStr)
	{
	ptrs[0]=" Батарей           @";
	ptrs[1]=" Источников        !";
	ptrs[2]=" Инверторов        ^";	
	ptrs[3]=" Контролируемых     ";
	ptrs[4]=" автоматов         $";
	ptrs[5]=" Выход              ";

	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      СТРУКТУРА     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(NUMBAT,'@',0);		
	int2lcd(NUMIST,'!',0);
	int2lcd(NUMAVT,'$',0);	 
	int2lcd(NUMINV,'^',0);
	int2lcd(NUMDT,'#',0);
	int2lcd(NUMSK,'$',0);
	}    

else if(ind==iStr_RSTKM)
	{
	ptrs[0]=" Батарей           @";
	ptrs[1]=" Источников        !";
	ptrs[2]=" Инверторов        ^";	
	ptrs[3]=" Контролируемых     ";
	ptrs[4]=" автоматов         $";
	ptrs[5]=" Выход              ";

	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      СТРУКТУРА     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(NUMBAT,'@',0);		
	int2lcd(NUMIST,'!',0);
	int2lcd(NUMAVT,'$',0);	 
	int2lcd(NUMINV,'^',0);
	int2lcd(NUMDT,'#',0);
	int2lcd(NUMSK,'$',0);
	}

else if(ind==iStr_3U)
	{
	ptrs[0]=" Батарей           @";
	ptrs[1]=" Источников        !";
	ptrs[2]=" Инверторов        ^";	
	ptrs[3]=" Сухих контактов   $";
	ptrs[4]=" Выход              ";

	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      СТРУКТУРА     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(NUMBAT,'@',0);		
	int2lcd(NUMIST,'!',0);
	//int2lcd(NUMAVT,'$',0);	 
	int2lcd(NUMINV,'^',0);
	int2lcd(NUMDT,'#',0);
	int2lcd(NUMSK,'$',0);
	}  

else if(ind==iStr_6U)
	{
	ptrs[0]=" Батарей           @";
	ptrs[1]=" Источников        !";
	ptrs[2]=" Инверторов        ^";
	ptrs[3]=" Байпасов          [";		
	ptrs[4]=" Датчиков темпер.  #";
	ptrs[5]=" Сухих контактов   $";
	ptrs[6]=" Мониторов АКБ     %";
	ptrs[7]=" Выход              ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      СТРУКТУРА     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(NUMBAT,'@',0);		
	int2lcd(NUMIST,'!',0); 
	int2lcd(NUMINV,'^',0);
	int2lcd(NUMBYPASS,'[',0);
	int2lcd(NUMDT,'#',0);
	int2lcd(NUMSK,'$',0);
	int2lcd(NUMMAKB,'%',0);
	}    

else if(ind==iStr_220_IPS_TERMOKOMPENSAT)
	{
	ptrs[0]=" Источников        !";
	ptrs[1]=" Датчиков темпер.  #";
	ptrs[2]=" Мониторов АКБ     %";
	ptrs[3]=" Сухих контактов   $";
	ptrs[4]=" Выход              ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      СТРУКТУРА     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(NUMIST,'!',0); 
	int2lcd(NUMDT,'#',0);
	int2lcd(NUMMAKB,'%',0);
	int2lcd(NUMSK,'$',0);
	}    

else if(ind==iStr_GLONASS)
	{
	ptrs[0]=" Батарей           @";
	ptrs[1]=" Источников        !";
	ptrs[2]=" Инверторов        ^";	
	ptrs[3]=" Сухих контактов   $";
	ptrs[4]=" Выход              ";

	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      СТРУКТУРА     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(NUMBAT,'@',0);		
	int2lcd(NUMIST,'!',0);
	//int2lcd(NUMAVT,'$',0);	 
	int2lcd(NUMINV,'^',0);
	int2lcd(NUMDT,'#',0);
	int2lcd(NUMSK,'$',0);
	}

else if(ind==iStr_KONTUR)
	{
	ptrs[0]=" Батарей           @";
	ptrs[1]=" Источников        !";
	ptrs[2]=" Инверторов        ^";	
//	ptrs[3]=" Датчиков темпер.  #";
//	ptrs[4]=" Сухих контактов   $";
	ptrs[2]=" Выход                   ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      СТРУКТУРА     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(NUMBAT,'@',0);		
	int2lcd(NUMIST,'!',0); 
	int2lcd(NUMINV,'^',0);
	int2lcd(NUMDT,'#',0);
	int2lcd(NUMSK,'$',0);
	}  

else if(ind==iStr_IP55)
	{
	ptrs[0]=" Батарей           @";
	ptrs[1]=" Источников        !";
	ptrs[2]=" Датчиков темпер.  #";
	ptrs[3]=" Сухих контактов   $";
	ptrs[4]=" Выход              ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      СТРУКТУРА     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(NUMBAT_IP55,'@',0);		
	int2lcd(NUMIST,'!',0); 
	int2lcd(NUMDT,'#',0);
	int2lcd(NUMSK,'$',0);
	}    

else if (ind==iLan_set)
	{
	char sss[10]="abcdef";
	char i/*,i_flag*/;
	 
	ptrs[0]=	" Ethernet         ! ";
	ptrs[1]=	" DHCPклиент       @ ";
	ptrs[2]=	" IPадрес            ";
	ptrs[3]=	"  000.000.000.00#   ";
	ptrs[4]=	" Маска подсети      ";
	ptrs[5]=	"  000.000.000.00$   ";
	ptrs[6]=	" Шлюз               ";
	ptrs[7]=	"  000.000.000.00)   ";
	ptrs[8]=	" Порт.чтения       [";
	ptrs[9]=	" Порт.записи       ]";
	ptrs[10]=	" Community <        ";
	ptrs[11]=	" Адресат для TRAP N1";
	ptrs[12]=	"  000.000.000.00%   ";
	ptrs[13]=	" Адресат для TRAP N2";
	ptrs[14]=	"  000.000.000.00^   ";
	ptrs[15]=	" Адресат для TRAP N3";
	ptrs[16]=	"  000.000.000.00&   ";
	ptrs[17]=	" Адресат для TRAP N4";
	ptrs[18]=	"  000.000.000.00*   ";
	ptrs[19]=	" Адресат для TRAP N5";
	ptrs[20]=	"  000.000.000.00(   ";
	ptrs[21]=	" Выход              ";

	
	if(!ETH_IS_ON)
		{
		ptrs[1]=" Выход              ";
		ptrs[2]="                    ";
		ptrs[3]="                    ";
		}

	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par(	" УСТАНОВКИ Ethernet ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);
	
	pointer_set(1);
     if(ETH_IS_ON)
     	{
     	sub_bgnd("ВКЛ.",'!',-3);   
     	}
     else 
     	{
     	sub_bgnd("ВЫКЛ.",'!',-4);   
     	}

     if(ETH_DHCP_ON)
     	{
     	sub_bgnd("ВКЛ.",'@',-3);   
     	}
     else 
     	{
     	sub_bgnd("ВЫКЛ.",'@',-4);   
     	}
		  
	if(sub_ind==2)	ip2lcd(ETH_IP_1,ETH_IP_2,ETH_IP_3,ETH_IP_4,'#',(sub_ind1+1));
	else ip2lcd(ETH_IP_1,ETH_IP_2,ETH_IP_3,ETH_IP_4,'#',0);
	if(sub_ind==4)	ip2lcd(ETH_MASK_1,ETH_MASK_2,ETH_MASK_3,ETH_MASK_4,'$',(sub_ind1+1));
	else ip2lcd(ETH_MASK_1,ETH_MASK_2,ETH_MASK_3,ETH_MASK_4,'$',0);
	if(sub_ind==6)	ip2lcd(ETH_GW_1,ETH_GW_2,ETH_GW_3,ETH_GW_4,')',(sub_ind1+1));
	else ip2lcd(ETH_GW_1,ETH_GW_2,ETH_GW_3,ETH_GW_4,')',0);

	int2lcd(ETH_SNMP_PORT_READ,'[',0);
	int2lcd(ETH_SNMP_PORT_WRITE,']',0);

	if( (ETH_TRAP1_IP_1==255) && (ETH_TRAP1_IP_2==255) && (ETH_TRAP1_IP_3==255) && (ETH_TRAP1_IP_4==255) ) sub_bgnd("    неактивен    ",'%',-14);
	else
		{
		if(sub_ind==11)	ip2lcd(ETH_TRAP1_IP_1,ETH_TRAP1_IP_2,ETH_TRAP1_IP_3,ETH_TRAP1_IP_4,'%',(sub_ind1+1));
		else ip2lcd(ETH_TRAP1_IP_1,ETH_TRAP1_IP_2,ETH_TRAP1_IP_3,ETH_TRAP1_IP_4,'%',0);
		}

	if( (ETH_TRAP2_IP_1==255) && (ETH_TRAP2_IP_2==255) && (ETH_TRAP2_IP_3==255) && (ETH_TRAP2_IP_4==255) ) sub_bgnd("    неактивен    ",'^',-14);
	else
		{
		if(sub_ind==13)	ip2lcd(ETH_TRAP2_IP_1,ETH_TRAP2_IP_2,ETH_TRAP2_IP_3,ETH_TRAP2_IP_4,'^',(sub_ind1+1));
		else ip2lcd(ETH_TRAP2_IP_1,ETH_TRAP2_IP_2,ETH_TRAP2_IP_3,ETH_TRAP2_IP_4,'^',0);
		}

	if( (ETH_TRAP3_IP_1==255) && (ETH_TRAP3_IP_2==255) && (ETH_TRAP3_IP_3==255) && (ETH_TRAP3_IP_4==255) ) sub_bgnd("    неактивен    ",'&',-14);
	else
		{
		if(sub_ind==15)	ip2lcd(ETH_TRAP3_IP_1,ETH_TRAP3_IP_2,ETH_TRAP3_IP_3,ETH_TRAP3_IP_4,'&',(sub_ind1+1));
		else ip2lcd(ETH_TRAP3_IP_1,ETH_TRAP3_IP_2,ETH_TRAP3_IP_3,ETH_TRAP3_IP_4,'&',0);
		}

	if( (ETH_TRAP4_IP_1==255) && (ETH_TRAP4_IP_2==255) && (ETH_TRAP4_IP_3==255) && (ETH_TRAP4_IP_4==255) ) sub_bgnd("    неактивен    ",'*',-14);
	else
		{
		if(sub_ind==17)	ip2lcd(ETH_TRAP4_IP_1,ETH_TRAP4_IP_2,ETH_TRAP4_IP_3,ETH_TRAP4_IP_4,'*',(sub_ind1+1));
		else ip2lcd(ETH_TRAP4_IP_1,ETH_TRAP4_IP_2,ETH_TRAP4_IP_3,ETH_TRAP4_IP_4,'*',0);
		}

	if( (ETH_TRAP5_IP_1==255) && (ETH_TRAP5_IP_2==255) && (ETH_TRAP5_IP_3==255) && (ETH_TRAP5_IP_4==255) ) sub_bgnd("    неактивен    ",'(',-14);
	else
		{
		if(sub_ind==19)	ip2lcd(ETH_TRAP5_IP_1,ETH_TRAP5_IP_2,ETH_TRAP5_IP_3,ETH_TRAP5_IP_4,'(',(sub_ind1+1));
		else ip2lcd(ETH_TRAP5_IP_1,ETH_TRAP5_IP_2,ETH_TRAP5_IP_3,ETH_TRAP5_IP_4,'(',0);
		}

/*	if((sub_ind==2)&&(sub_ind1==0)&&(bFL2))
		{
		sub_bgnd("   ",'#',-2);
		}
	else int2lcd(ETH_IP_1,'#',0);

	if((sub_ind==2)&&(sub_ind1==1)&&(bFL2))
		{
		sub_bgnd("   ",'$',-2);
		}
	else int2lcd(ETH_IP_2,'$',0);

	if((sub_ind==2)&&(sub_ind1==2)&&(bFL2))
		{
		sub_bgnd("   ",'%',-2);
		}
	else int2lcd(ETH_IP_3,'%',0);

	if((sub_ind==2)&&(sub_ind1==3)&&(bFL2))
		{
		sub_bgnd("   ",'^',-2);
		}
	else int2lcd(ETH_IP_4,'^',0);*/


	//int2lcdyx(sub_ind,0,1,0);	
	//int2lcdyx(index_set,0,3,0);
	//int2lcdyx(sub_ind1,0,5,0);
	//for(i=0;(i<9)&&(snmp_community[i]))

		

	if(sub_ind==10)community2lcd(sss,'<',sub_ind1,1);
	else community2lcd(sss,'<',sub_ind1,0);
	
	//int2lcdyx(snmp_community[0],0,4,0);
	//int2lcdyx(snmp_community[11],0,9,0);
	//int2lcdyx(snmp_community[2],0,14,0);
	//int2lcdyx(snmp_community[sub_ind1],0,19,0);	
	}

else if (ind==iSpch_set)
	{
	char sss[10]="abcdef";
	char i/*,i_flag*/;
	 
	ptrs[0]=	" Iуск.зар.        !А";
	ptrs[1]=	" Uуск.зар.        @В";
	ptrs[2]=	" Tуск.зар.        #ч";
	ptrs[3]=	" Автоматический     ";
	ptrs[4]=	" ускор.заряд.      $";
	ptrs[5]=	" dUуск.зар.       %В";
	ptrs[6]=	" Блокирование      ^";
	ptrs[7]=	" Сигнал блокирования";
	ptrs[8]=	"                   &";
	ptrs[9]=	" Выход              ";

	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par(	"  УСКОРЕННЫЙ ЗАРЯД  ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(speedChrgCurr,'!',1);
	int2lcd(speedChrgVolt,'@',1);
	int2lcd(speedChrgTimeInHour,'#',0);
	if(speedChrgAvtEn)sub_bgnd("ВКЛ.",'$',-3);   
    else sub_bgnd("ВЫКЛ.",'$',-4); 
	int2lcd(speedChrgDU,'%',0);
	if(speedChrgBlckSrc==1)sub_bgnd("СК1",'^',-2);
	else if(speedChrgBlckSrc==2)sub_bgnd("СК2",'^',-2);   
    else if(speedChrgBlckSrc==0)sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("НЕКОРР..",'^',-6); 
	if(speedChrgBlckLog==0)sub_bgnd("РАЗОМКН.",'&',-7);
	else if(speedChrgBlckLog==1) sub_bgnd("ЗАМКН.",'&',-5);  
    else sub_bgnd("НЕКОРР.",'&',-6); 	  

	
	}

else if (ind==iBlok_ips_set)
	{
	char sss[10]="abcdef";
	char i/*,i_flag*/;
	 
	ptrs[0]=	" Блокирование      ^";
	ptrs[1]=	" Сигнал блокирования";
	ptrs[2]=	"                   &";
	ptrs[3]=	" Выход              ";

	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par(	"   БЛОКИРОВКА ИПС   ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);
	
	pointer_set(1);

	if(ipsBlckSrc==1)sub_bgnd("СК1",'^',-2);
	else if(ipsBlckSrc==2)sub_bgnd("СК2",'^',-2);   
    else if(ipsBlckSrc==0)sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("НЕКОРР..",'^',-6); 
	if(ipsBlckLog==0)sub_bgnd("РАЗОМКН.",'&',-7);
	else if(ipsBlckLog==1) sub_bgnd("ЗАМКН.",'&',-5);  
    else sub_bgnd("НЕКОРР.",'&',-6); 	  

	
	}


else if (ind==iApv)
	{ 
	ptrs[0]=			" АПВ 1й уровень !   ";
	if(APV_ON1!=apvON)
	     {
	     ptrs[1]=		" Выход              ";
	     ptrs[2]=sm_;
	     ptrs[3]=sm_;
	     ptrs[4]=sm_;
	     simax=1;
	     }
	else
	     {
	     if(APV_ON2!=apvON)
	          {
	          ptrs[1]=" АПВ 2й уровень @   ";
	          ptrs[2]=" Выход              ";
	          ptrs[3]=sm_;
	          ptrs[4]=sm_;
	          simax=2;
	          }
	     else 
	          {
               ptrs[1]=" АПВ 2й уровень @   ";
	          ptrs[2]=" Период АПВ2     #ч.";
	          ptrs[3]=" Выход              ";
	          ptrs[4]=sm_;
	          simax=3;	          
	          }     
	     }     
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;	
     bgnd_par("   АПВ ИСТОЧНИКОВ   ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	if(APV_ON1==apvON)sub_bgnd("ВКЛ.",'!',0);
	else sub_bgnd("ВЫКЛ.",'!',-1);
	
	if(APV_ON2==apvON)
	     {
	     sub_bgnd("ВКЛ.",'@',0);
	     int2lcd(APV_ON2_TIME,'#',0);
	     }
	else sub_bgnd("ВЫКЛ.",'@',-1);	
     
 	} 
/*
     ptrs[0+NUMDT]=  	" СК1        $       ";            
     ptrs[1+NUMDT]=  	" СК2        %       ";
	ptrs[2+NUMDT]=  	" СК3        ^       ";
	ptrs[3+NUMDT]=  	" СК4        &       ";
	ptrs[0+NUMEXT]=  	" Выход              ";
	ptrs[1+NUMEXT]=  	"                    ";
	ptrs[2+NUMEXT]=  	"                    ";

	bgnd_par(		"  ВНЕШНИЕ ДАТЧИКИ   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

*/

else if (ind==iExt_set)
	{ 
//	ptrs[0]=			" Датчик темпер. N1  ";
//	ptrs[1]=			" Датчик темпер. N2  ";
//	ptrs[2]=			" Датчик темпер. N3  ";
	ptrs[0]=		" Датчик двери       ";
	ptrs[1]=		" Датчик дыма        ";
	ptrs[2]=		" Датчик удара       ";
//	ptrs[3]=		" Датчик переворачив.";
	ptrs[3]=  	" Выход              ";
	ptrs[4]=  	"                    ";
	ptrs[5]=  	"                    ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("     УСТАНОВКИ      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	}

else if (ind==iExt_set_3U)
	{ 
	ptrs[0]=		" Сухой контакт №1   ";
	ptrs[1]=		" Сухой контакт №2   ";
	ptrs[2]=		" Сухой контакт №3   ";
	ptrs[3]=		" Сухой контакт №4   ";
	ptrs[NUMSK]=  	" Выход              ";
	ptrs[NUMSK+1]= "                    ";
	ptrs[NUMSK+2]=	"                    ";
	ptrs[NUMSK+3]=	"                    ";
		
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("     УСТАНОВКИ      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	}

else if (ind==iExt_set_GLONASS)
	{ 
	ptrs[0]=		" Сухой контакт №1   ";
	ptrs[1]=		" Сухой контакт №2   ";
	ptrs[2]=		" Сухой контакт №3   ";
	ptrs[3]=		" Сухой контакт №4   ";
	ptrs[NUMSK]=  	" Выход              ";
	ptrs[NUMSK+1]= "                    ";
	ptrs[NUMSK+2]=	"                    ";
	ptrs[NUMSK+3]=	"                    ";
		
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("     УСТАНОВКИ      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	}
else if (ind==iExt_set_TELECORE2015)
	{ 
	ptrs[0]=	" Датчик двери       ";
	ptrs[1]=  	" Выход              ";
	ptrs[2]=  	"                    ";
	ptrs[3]=  	"                    ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
    bgnd_par("     УСТАНОВКИ      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	}
	
else if (ind==iExt_dt)
	{ 
	ptrs[0]=" температура     @°C";
	ptrs[1]=" tmax            #°C";
	ptrs[2]=" tmin            $°C";
	ptrs[3]=" Реле            [  ";
	ptrs[4]=" Звук            ]  ";
	ptrs[5]=" Дисплей         (  ";
	ptrs[6]=" RS232           )  ";
	ptrs[7]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>1) index_set=sub_ind-1;
     bgnd_par("  ВНЕШНИЙ ДАТЧИК    ","   ТЕМПЕРАТУРЫ N!   ",ptrs[index_set],ptrs[index_set+1]);
	
	pointer_set(2);
	int2lcd(sub_ind1+1,'!',0);
	int2lcd_mmm(t_ext[sub_ind1],'@',0);
	if(!TMAX_EXT_EN[sub_ind1])int2lcd_mmm(TMAX_EXT[sub_ind1],'#',0);
	else sub_bgnd("выкл.",'#',-2);
	if(!TMIN_EXT_EN[sub_ind1])int2lcd_mmm(TMIN_EXT[sub_ind1],'$',0);
	else sub_bgnd("выкл.",'$',-2);
	if(!T_EXT_REL_EN[sub_ind1])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
	if(!T_EXT_ZVUK_EN[sub_ind1])sub_bgnd("вкл.",']',-2);
	else sub_bgnd("выкл.",']',-2);
	if(!T_EXT_LCD_EN[sub_ind1])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	if(!T_EXT_RS_EN[sub_ind1])sub_bgnd("вкл.",')',-2);
	else sub_bgnd("выкл.",')',-2);	
	
	//int2lcdyx(sub_ind,0,1,0);	
	//int2lcdyx(index_set,0,3,0);
	}	
else if (ind==iExt_sk)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" аварийное          ";
	ptrs[2]=" состояние - #      ";
	ptrs[3]=" Реле            [  ";
	ptrs[4]=" Звук            ]  ";
	ptrs[5]=" Дисплей         (  ";
	ptrs[6]=" RS232           )  ";
	ptrs[7]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("  СУХОЙ КОНТАКТ N!  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	int2lcd(sub_ind1+1,'!',0);
	if(sk_stat[sub_ind1]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[sub_ind1])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(!TMIN_EXT_EN[sub_ind1])int2lcd_mmm(TMIN_EXT[sub_ind1],'$',0);
	else sub_bgnd("выкл.",'$',-6);
	if(!SK_REL_EN[sub_ind1])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
	if(!SK_ZVUK_EN[sub_ind1])sub_bgnd("вкл.",']',-2);
	else sub_bgnd("выкл.",']',-2);
	if(!SK_LCD_EN[sub_ind1])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	if(!SK_RS_EN[sub_ind1])sub_bgnd("вкл.",')',-2);
	else sub_bgnd("выкл.",')',-2);	
	
	//int2lcdyx(sub_ind,0,1,0);	
	//int2lcdyx(index_set,0,3,0);
	}		

else if (ind==iExt_sk_3U)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" аварийное          ";
	ptrs[2]=" состояние - #      ";
	ptrs[3]=" Звук            ]  ";
	ptrs[4]=" Дисплей         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("  СУХОЙ КОНТАКТ N!  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	int2lcd(sub_ind1+1,'!',0);
	if(sk_stat[sub_ind1]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[sub_ind1])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(!SK_ZVUK_EN[sub_ind1])sub_bgnd("вкл.",']',-2);
	else sub_bgnd("выкл.",']',-2);
	if(!SK_LCD_EN[sub_ind1])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	}		

else if (ind==iExt_sk_GLONASS)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" аварийное          ";
	ptrs[2]=" состояние - #      ";
	ptrs[3]=" Звук            ]  ";
	ptrs[4]=" Дисплей         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("  СУХОЙ КОНТАКТ N!  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	int2lcd(sub_ind1+1,'!',0);
	if(sk_stat[sub_ind1]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[sub_ind1])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(!SK_ZVUK_EN[sub_ind1])sub_bgnd("вкл.",']',-2);
	else sub_bgnd("выкл.",']',-2);
	if(!SK_LCD_EN[sub_ind1])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	}		

else if (ind==iExt_ddv)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" открытое состояние ";
	ptrs[2]=" двери     - #      ";
	ptrs[3]=" Реле            [  ";
	ptrs[4]=" Дисплей         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("   ДАТЧИК ДВЕРИ     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	
	if(sk_stat[0]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[0])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(SK_REL_EN[0])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
     if(SK_LCD_EN[0])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	
	}	

else if (ind==iExt_ddi)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" аварийное          ";
	ptrs[2]=" состояние - #      ";
	ptrs[3]=" Реле            [  ";
	ptrs[4]=" Дисплей         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("   ДАТЧИК ДЫМА      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	
	if(sk_stat[1]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[1])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(SK_REL_EN[1])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
	if(SK_LCD_EN[1])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	}	

else if (ind==iExt_dud)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" аварийное          ";
	ptrs[2]=" состояние - #      ";
	ptrs[3]=" Реле            [  ";
	ptrs[4]=" Дисплей         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("   ДАТЧИК УДАРА     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	
	if(sk_stat[2]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[2])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(SK_REL_EN[2])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
	if(SK_LCD_EN[2])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);
	}


else if (ind==iExt_dp)
	{ 
	ptrs[0]=" состояние - @      ";
	ptrs[1]=" аварийное          ";
	ptrs[2]=" состояние - #      ";
	ptrs[3]=" Реле            [  ";
     ptrs[4]=" Дисплей         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("ДАТЧИК ПЕРЕВОРАЧИВ. ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	if(sk_stat[3]==ssON)sub_bgnd("замкнут",'@',0);
	else sub_bgnd("разомк.",'@',0);
	if(!SK_SIGN[3])sub_bgnd("замкнут",'#',0);
	else sub_bgnd("незамк.",'#',0);
	if(!SK_REL_EN[3])sub_bgnd("вкл.",'[',-2);
	else sub_bgnd("выкл.",'[',-2);
	if(!SK_LCD_EN[3])sub_bgnd("вкл.",'(',-2);
	else sub_bgnd("выкл.",'(',-2);


    /* int2lcdyx(sk_stat[0],0,2,0);
     int2lcdyx(sk_stat[1],0,5,0);
     int2lcdyx(sk_stat[2],0,8,0);
     int2lcdyx(sk_stat[3],0,11,0);*/
	}

else if(ind==iK)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
     ptrs[i++]=" Внешние датчики    ";
     ptrs[i++]=" Силовые вводы      ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

else if(ind==iK_RSTKM)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
     ptrs[i++]=" Внешние датчики    ";
     ptrs[i++]=" Силовые вводы      ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    

else if(ind==iK_3U)
	{
	char i;
	i=0;
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
     ptrs[i++]=" Внешн.датч.темпер. ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

else if(ind==iK_GLONASS)
	{
	char i;
	i=0;
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
     ptrs[i++]=" Внешн.датч.темпер. ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    

else if(ind==iK_KONTUR)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
     ptrs[i++]=" Внешние датчики    ";
     ptrs[i++]=" Силовые вводы      ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

else if(ind==iK_6U)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	if(NUMBYPASS)
     ptrs[i++]=" Байпасс            ";
	ptrs[i++]=" Нагрузка           ";
     if(NUMDT)
     ptrs[i++]=" Внешние датчики    ";
     if(NUMMAKB)
     ptrs[i++]=" Мониторы АКБ       ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

else if(ind==iK_220)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
     ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
     if(NUMDT)
     ptrs[i++]=" Внешние датчики    ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

else if(ind==iK_220_IPS_TERMOKOMPENSAT)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
     ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	ptrs[i++]=" Выход              ";
     if(NUMDT)
     ptrs[i++]=" Внешние датчики    ";
     ptrs[i++]=" Выход              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}   


else if(ind==iK_220_IPS_TERMOKOMPENSAT_IB)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
    ptrs[i++]=" Батарея            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	ptrs[i++]=" Выходные параметры ";
    if(NUMDT)
    ptrs[i++]=" Внешние датчики    ";
    ptrs[i++]=" Выход              ";
    ptrs[i++]="                    ";
    ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}   


else if(ind==iK_220_380)
	{
	char i;
	i=0;
	
	ptrs[i++]=" Сеть               ";
	if(NUMBAT)
    ptrs[i++]=" Батареи            ";
	if(NUMIST)
	ptrs[i++]=" БПС                ";
	if(NUMINV)
    ptrs[i++]=" Инверторы          ";
	ptrs[i++]=" Нагрузка           ";
    if(NUMDT)
    ptrs[i++]=" Внешние датчики    ";
	ptrs[i++]=" Логика реле       !";
    ptrs[i++]=" Выход              ";
    ptrs[i++]="                    ";
    ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);
	
	if(RELE_VENT_LOGIC==0)sub_bgnd("Обычн.",'!',-5);
	else if(RELE_VENT_LOGIC==1)sub_bgnd("Вент.",'!',-4);
	else sub_bgnd("Ав.Б2",'!',-4);	 
	}    	

else if(ind==iK_IP55)
	{
	char i;
	i=0;
	
	ptrs[i++]=	" Сеть               ";
	if(NUMBAT_IP55)
    ptrs[i++]=	" Батареи            ";
	if(NUMIST)
	ptrs[i++]=	" БПС                ";
	ptrs[i++]=	" Нагрузка           ";
    if(NUMDT)
    ptrs[i++]=	" Внешние датчики    ";
    ptrs[i++]=" Выход              ";
    ptrs[i++]="                    ";
    ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

else if(ind==iK_net)
	{
	ptrs[0]=" U =     @В         ";
     ptrs[1]=" Выход              ";
	ptrs[2]="                    ";
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("   КАЛИБРОВКА СЕТИ  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(net_U,'@',0);
	//int2lcdyx(net_buff_,3,10,0);
	
	//int2lcdyx(Kunet,3,16,0);
     }


else if(ind==iK_net3)
	{

	ptrs[0]=  		" UфA           !В   ";
    ptrs[1]=  		" UфB           @В   ";
    ptrs[2]=  	    " UфC           #В   ";
	ptrs[3]=  	    " Выход              ";


	bgnd_par(		"   КАЛИБРОВКА СЕТИ  ",
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

    int2lcd(net_Ua,'!',0);
	int2lcd(net_Ub,'@',0);
	int2lcd(net_Uc,'#',0);

	/*int2lcdyx(KunetC,0,19,0);
	int2lcdyx(adc_buff_[10],0,13,0);
	int2lcdyx(KunetB,0,8,0);
	int2lcdyx(adc_buff_[3],0,4,0);*/

    }


else if(ind==iK_load)
	{
	ptrs[0]=" U =     @В         ";
     ptrs[1]=" Выход              ";
	ptrs[2]="                    ";
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(		" КАЛИБРОВКА НАГРУЗКИ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	pointer_set(1);
	if((load_U)>1000)int2lcd(load_U/10,'@',0);	
	else int2lcd(load_U,'@',1);
     }

else if(ind==iK_out)
	{
	ptrs[0]=" Uвыпр. =     @В    ";
    ptrs[1]=" Uшины  =     #В    ";
	ptrs[2]=" Выход              ";
	ptrs[3]="                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(	"КАЛИБРОВКА ВЫХОДНЫХ ",
				"     ПАРАМЕТРОВ     ",
				ptrs[index_set],
				ptrs[index_set+1]);

	pointer_set(2);
	
	if((bps_U)>1000)int2lcd(bps_U/10,'@',0);	
	else int2lcd(bps_U,'@',1);

	if((out_U)>1000)int2lcd(out_U/10,'#',0);	
	else int2lcd(out_U,'#',1);
    }

else if(ind==iBat_link_set)
	{
	ptrs[0]=" RS232              ";
     ptrs[1]=" RS485              ";
	ptrs[2]=" Выход              ";
	if(bFL5)ptrs[BAT_LINK]=sm_;	
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(		"    КАНАЛ СВЯЗИ     ",
				"    С БАТАРЕЕЙ      ",
				ptrs[index_set],
				ptrs[index_set+1]);

	pointer_set(2);

     }

	
else if(ind==iK_t_ext)
	{
	ptrs[0]=  	" tвнеш.возд.    !°С ";
     ptrs[1]=  	" tотсек ЭПУ     @°С ";
     ptrs[2]=  	" tотсек MSAN    #°С ";
     ptrs[3]=	     " Выход              ";
	ptrs[4]=	     "                    ";
	ptrs[5]=	     "                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(		" КАЛИБРОВКА ВНЕШНИХ ",
				" ДАТЧИКОВ ТЕМПЕРАТУР",
				ptrs[index_set],
				ptrs[index_set+1]);

	pointer_set(2);	
	if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
     else int2lcd_mmm(t_ext[0],'!',0);

	if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
     else int2lcd_mmm(t_ext[1],'@',0);

	if(ND_EXT[2])sub_bgnd("неиспр.",'#',-3);
     else int2lcd_mmm(t_ext[2],'#',0);
     }

else if(ind==iK_t_ext_6U)
	{
	ptrs[0]=  		" t1             !°С ";
    ptrs[1]=  		" t2             @°С ";
    ptrs[2]=  		" t3             #°С ";
    ptrs[NUMDT]=	" Выход              ";
	ptrs[NUMDT+1]=  "                    ";
	ptrs[NUMDT+2]=  "                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(		" КАЛИБРОВКА ВНЕШНИХ ",
				" ДАТЧИКОВ ТЕМПЕРАТУР",
				ptrs[index_set],
				ptrs[index_set+1]);

	pointer_set(2);	
	if(ND_EXT[0])sub_bgnd("неиспр.",'!',-3);
     else int2lcd_mmm(t_ext[0],'!',0);

	if(ND_EXT[1])sub_bgnd("неиспр.",'@',-3);
     else int2lcd_mmm(t_ext[1],'@',0);

	if(ND_EXT[2])sub_bgnd("неиспр.",'#',-3);
     else int2lcd_mmm(t_ext[2],'#',0);
	//int2lcdyx(u_necc,3,18,0);
     }
     
else if(ind==iK_bat_sel)
	{
	ptrs[0]=						" Батарея N1         ";
     ptrs[1]=						" Батарея N2         ";
     if(BAT_IS_ON[0]!=bisON)ptrs[0]=	" Батарея N2         ";
	ptrs[0+NUMBAT]=				" Выход              ";
	ptrs[1+NUMBAT]=				"                    ";
	ptrs[2+NUMBAT]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(" КАЛИБРОВКА БАТАРЕЙ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iK_bat_sel_TELECORE)
	{
	ptrs[0]=						" Батарея N1         ";
    ptrs[1]=						" Батарея N2         ";
    ptrs[0+NUMBAT_IP55]=		" Выход              ";
	ptrs[1+NUMBAT_IP55]=		"                    ";
	ptrs[2+NUMBAT_IP55]=		"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(" КАЛИБРОВКА БАТАРЕЙ ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }  

else if(ind==iK_bat)
	{
    ptrs[0]=		" Iбат =     #А      ";
    if(phase==0)
      	{
        ptrs[1]=	"   нажмите ¤ для    ";
        ptrs[2]=	"калибровки нуля Iбат";
        }
    else          
       	{
        ptrs[1]=	" откалибруйте Iбат  ";
        ptrs[2]=	"  нажатием љ или њ  ";
        }
	ptrs[3]=		" Выход              ";
    ptrs[4]=		"                    ";
    ptrs[5]=		"                    ";

	bgnd_par(		" КАЛИБРОВКА БАТ. N! ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);
     
   	if(sub_ind==0)
     	{
     	if(phase==0)
     		{
			mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,0xffff,10);
     		mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<sub_ind1),10);
     		//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<(1-sub_ind1)),10);
     		}
    	else if(phase==1)
     		{
			mess_send(MESS2BPS_HNDL,PARAM_BPS_ALL_OFF_AFTER_2SEC,0xffff,10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<(1-sub_ind1)),10);
     		//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<sub_ind1),10);
   			}
     		
     	}

	
	if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else index_set=3;

	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd_mmm(bat[sub_ind1]._Ib,'#',2);

	}

else if(ind==iK_inv_sel)
	{
	ptrs[0]=						" ИНВЕРТОР N1        ";
     ptrs[1]=						" ИНВЕРТОР N2        ";
     ptrs[2]=						" ИНВЕРТОР N3        ";
	ptrs[3]=						" ИНВЕРТОР N4        ";
     ptrs[4]=						" ИНВЕРТОР N5        ";
     ptrs[5]=						" ИНВЕРТОР N6        ";
	ptrs[6]=						" ИНВЕРТОР N7        ";
     ptrs[7]=						" ИНВЕРТОР N8        ";
     ptrs[8]=						" ИНВЕРТОР N9        ";
	ptrs[9]=						" ИНВЕРТОР N10       ";
     ptrs[10]=						" ИНВЕРТОР N11       ";
     ptrs[11]=						" ИНВЕРТОР N12       "; 
	ptrs[12]=						" ИНВЕРТОР N13       ";
     ptrs[13]=						" ИНВЕРТОР N14       ";
     ptrs[14]=						" ИНВЕРТОР N15       ";	              
	ptrs[NUMINV]=					" Выход              ";
	ptrs[1+NUMINV]=				"                    ";
	ptrs[2+NUMINV]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("КАЛИБРОВАТЬ ИНВЕРТОР",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iInv_set_sel)
	{
	ptrs[0]=						" ИНВЕРТОР N1        ";
     ptrs[1]=						" ИНВЕРТОР N2        ";
     ptrs[2]=						" ИНВЕРТОР N3        ";
	ptrs[3]=						" ИНВЕРТОР N4        ";
     ptrs[4]=						" ИНВЕРТОР N5        ";
     ptrs[5]=						" ИНВЕРТОР N6        ";
	ptrs[6]=						" ИНВЕРТОР N7        ";
     ptrs[7]=						" ИНВЕРТОР N8        ";
     ptrs[8]=						" ИНВЕРТОР N9        ";
	ptrs[9]=						" ИНВЕРТОР N10       ";
     ptrs[10]=						" ИНВЕРТОР N11       ";
     ptrs[11]=						" ИНВЕРТОР N12       ";  
	ptrs[12]=						" ИНВЕРТОР N13       ";
     ptrs[13]=						" ИНВЕРТОР N14       ";
     ptrs[14]=						" ИНВЕРТОР N15       ";              
	ptrs[NUMINV]=					" Выход              ";
	ptrs[1+NUMINV]=				"                    ";
	ptrs[2+NUMINV]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("УСТАНОВКИ ИНВЕРТОРОВ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iInv_set)
	{
	ptrs[0]=						" Минимальное напр.  ";
     ptrs[1]=						" выхода          <В ";
     ptrs[2]=						" Максимальное напр. ";
	ptrs[3]=						" выхода          >В ";
	ptrs[4]=					  	" Выход              ";
	ptrs[5]=						"                    ";
	ptrs[6]=						"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("    ИНВЕРТОР N!     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd(inv[sub_ind1]._Uoutmin,'<',0);
	int2lcd(inv[sub_ind1]._Uoutmax,'>',0);
	}     

else if(ind==iK_makb_sel)
	{
	ptrs[0]=						" Монитор АКБ N1     ";
     ptrs[1]=						" Монитор АКБ N2     ";
     ptrs[2]=						" Монитор АКБ N3     ";
	ptrs[3]=						" Монитор АКБ N4     ";
	ptrs[NUMMAKB]=					" Выход              ";
	ptrs[1+NUMMAKB]=				"                    ";
	ptrs[2+NUMMAKB]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("КАЛИБРОВАТЬ МОНИТОРЫ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iK_makb)
	{
	ptrs[0]=						" U1  =    !В        ";
	ptrs[1]=						" U2  =    @В        ";
	ptrs[2]=						" U3  =    #В        ";
	ptrs[3]=						" U4  =    $В        ";
	ptrs[4]=						" U5  =    %В        ";
	ptrs[5]=						" t1  =    ^°C       ";
	ptrs[6]=						" t2  =    &°C       ";
	ptrs[7]=						" t3  =    *°C       ";
	ptrs[8]=						" t4  =    (°C       ";
	ptrs[9]=						" t5  =    )°C       ";
	ptrs[10]=						" Выход              ";
	ptrs[11]=						"                    ";

	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     КАЛИБРОВКА     ","   МОНИТОР АКБ N<   ",ptrs[index_set],ptrs[index_set+1]);
	pointer_set(2);
	simax=10;

	int2lcd(sub_ind1+1,'<',0);
	int2lcd(makb[sub_ind1]._U[0],'!',1);
	int2lcd(makb[sub_ind1]._U[1],'@',1);
	int2lcd(makb[sub_ind1]._U[2],'#',1);
	int2lcd(makb[sub_ind1]._U[3],'$',1);
	int2lcd(makb[sub_ind1]._U[4],'%',1);

	if(makb[sub_ind1]._T_nd[0])sub_bgnd("НЕПОДКЛЮЧЕН",'^',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[0],'^',0); 
	if(makb[sub_ind1]._T_nd[1])sub_bgnd("НЕПОДКЛЮЧЕН",'&',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[1],'&',0); 
	if(makb[sub_ind1]._T_nd[2])sub_bgnd("НЕПОДКЛЮЧЕН",'*',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[2],'*',0); 
	if(makb[sub_ind1]._T_nd[3])sub_bgnd("НЕПОДКЛЮЧЕН",'(',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[3],'(',0); 
	if(makb[sub_ind1]._T_nd[4])sub_bgnd("НЕПОДКЛЮЧЕН",')',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[4],')',0); 


/*	int2lcd(makb[sub_ind1]._T[0],'^',0);
	int2lcd(makb[sub_ind1]._T[1],'&',0);
	int2lcd(makb[sub_ind1]._T[2],'*',0);
	int2lcd(makb[sub_ind1]._T[3],'(',0);
	int2lcd(makb[sub_ind1]._T[4],')',0);*/
	
     }   

else if(ind==iK_inv)
	{

	ptrs[0]=	"   Pвых =     <Вт   ";
	ptrs[1]=	" выберите  мощность ";
	ptrs[2]=	"  нажатием љ или њ  ";
	ptrs[3]=	" Uвых =    @В       ";
	ptrs[4]=	"откалибруйте Uвыхинв";
	ptrs[5]=	"  нажатием љ или њ  "; 
	ptrs[6]=	" Iвых =     %А      ";
	if(phase==0)
          {
          ptrs[7]=	"   нажмите ¤ для    ";
          ptrs[8]=	"калибровки нуля Iвых";
          }
     else
     	{
          ptrs[7]=" откалибруйте Iвых  ";
          ptrs[8]="  нажатием љ или њ  ";     	
     	} 
     	
	ptrs[9]=	" tинв =   ^°C       ";    
	ptrs[10]=	" откалибруйте tинв  ";
	ptrs[11]=	"  нажатием љ или њ  ";
	ptrs[12]=	" Uшины =    &В      ";
	ptrs[13]=	"откалибруйте Uшины  ";
	ptrs[14]=	"  нажатием љ или њ  "; 
	ptrs[15]=	" Uсети =    *В      ";
	ptrs[16]=	"откалибруйте Uсети  ";
	ptrs[17]=	"  нажатием љ или њ  "; 
	ptrs[18]=	" Pвых  =    (Вт     ";
	ptrs[19]=	"откалибруйте Pвых   ";
	ptrs[20]=	"  нажатием љ или њ  "; 
	ptrs[21]=	" Контроль сети     [";
	ptrs[22]=	" ШИМ               ]";
	ptrs[23]=	" Режим             /";
	ptrs[24]=	sm_exit;
	ptrs[25]=	sm_;
	ptrs[26]=	sm_;     	     	    
	

    	if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
	else if((sub_ind==9)||(sub_ind==10)||(sub_ind==11))index_set=9;
	else if((sub_ind==12)||(sub_ind==13)||(sub_ind==14))index_set=12;
	else if((sub_ind==15)||(sub_ind==16)||(sub_ind==17))index_set=15;
	else if((sub_ind==18)||(sub_ind==19)||(sub_ind==20))index_set=18;
	else if((sub_ind==21)||(sub_ind==22)||(sub_ind==23))index_set=21;
	else index_set=22;
	
	bgnd_par("КАЛИБРОВКА ИНВЕРТ N!",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	if(inv[sub_ind1]._Pnom==0)int2lcd(700,'<',0);
	else if(inv[sub_ind1]._Pnom==1)int2lcd(1000,'<',0);
	else int2lcd(2000,'<',0);
	int2lcd(inv[sub_ind1]._Uio,'@',1);
	int2lcd(inv[sub_ind1]._Ii,'%',1);
	int2lcd(inv[sub_ind1]._Ti,'^',0); 
	int2lcd(inv[sub_ind1]._Uil,'&',1);
	int2lcd(inv[sub_ind1]._Uin,'*',1);
	int2lcd_mmm(inv[sub_ind1]._Pio,'(',0); 

	if(inv[sub_ind1]._net_contr_en==0)sub_bgnd("ВЫКЛ",'[',-3);
	else if(inv[sub_ind1]._net_contr_en==1)sub_bgnd("АМПЛ.",'[',-4);
	else if(inv[sub_ind1]._net_contr_en==3)sub_bgnd("ВКЛ",'[',-2);

	if(inv[sub_ind1]._pwm_en==0)sub_bgnd("ВЫКЛ",']',-3);
	else sub_bgnd("ВКЛ",']',-2);

	if(inv[sub_ind1]._phase_mode==0)sub_bgnd("1 ФАЗА",'/',-5);
	else if(inv[sub_ind1]._phase_mode==1)sub_bgnd("3 ФАЗЫ",'/',-5);

     if((sub_ind==3))
		{
//		mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
//		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
     if(sub_ind==6)
		{
		if(phase==0)
			{
//          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<sub_ind1),10);
          	}
      	else if(phase==1)
			{
//          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
//			mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
          	}
          mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
	

	
  
	     
	//MSG_IND2PWM_SRC1=900;
	//MSG_IND2PWM_SRC2=900;         
//int2lcdyx(inv[sub_ind1]._Pnom,0,5,0);

/*int2lcdyx(sub_ind,0,1,0);
int2lcdyx(phase,0,2,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC1,0,3,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC2,0,4,0);  
int2lcdyx(MSG_IND2OUT_EN_SRC1,0,5,0);
int2lcdyx(MSG_IND2OUT_EN_SRC2,0,6,0); */

//int2lcdyx(cntrl_stat1,0,19,0); 
//int2lcdyx(load_U,0,5,0); 
//int2lcdyx(cntrl_stat,0,10,0); 
//int2lcdyx(bps[sub_ind1]._rotor,0,19,0); 
//int2lcdyx(u_necc,0,19,0);  
	 }

else if(ind==iK_byps)
	{
	
	ptrs[0]=	" Uвых =    @В       ";
	ptrs[1]=	"откалибруйте Uвых   ";
	ptrs[2]=	"  нажатием љ или њ  "; 
	ptrs[3]=	" Iвых =     %А      ";
	if(phase==0)
          {
          ptrs[4]=	"   нажмите ¤ для    ";
          ptrs[5]=	"калибровки нуля Iвых";
          }
     else
     	{
          ptrs[4]=" откалибруйте Iвых  ";
          ptrs[5]="  нажатием љ или њ  ";     	
     	} 
     	
	ptrs[6]=	" t    =   ^°C       ";    
	ptrs[7]=	" откалибруйте t     ";
	ptrs[8]=	"  нажатием љ или њ  ";
	ptrs[9]=	" Uшины =    &В      ";
	ptrs[10]=	"откалибруйте Uшины  ";
	ptrs[11]=	"  нажатием љ или њ  "; 
	ptrs[12]=	" Uсети =    *В      ";
	ptrs[13]=	"откалибруйте Uсети  ";
	ptrs[14]=	"  нажатием љ или њ  "; 
	ptrs[15]=	" Pвых  =    (Вт     ";
	ptrs[16]=	"откалибруйте Pвых   ";
	ptrs[17]=	"  нажатием љ или њ  "; 

	ptrs[18]=	sm_exit;
	ptrs[19]=	sm_;
	ptrs[20]=	sm_;     	     	    
	

    	 if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
	else if((sub_ind==9)||(sub_ind==10)||(sub_ind==11))index_set=9;
	else if((sub_ind==12)||(sub_ind==13)||(sub_ind==14))index_set=12;
	else if((sub_ind==15)||(sub_ind==16)||(sub_ind==17))index_set=15;

	else index_set=18;
	
	bgnd_par("КАЛИБРОВКА БАЙПАС   ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(byps._Uout,'@',1);
	int2lcd(byps._Iout,'%',1);
	int2lcd(byps._T,'^',0); 
	int2lcd(byps._Uin,'&',1);
	int2lcd(byps._Unet,'*',1);
	int2lcd_mmm(byps._Pout,'(',0); 

     if((sub_ind==0))
		{
//		mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
//		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
     if(sub_ind==3)
		{
		if(phase==0)
			{
//          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<sub_ind1),10);
          	}
      	else if(phase==1)
			{
//          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
//			mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
          	}
          mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
	

	
  
	     
	//MSG_IND2PWM_SRC1=900;
	//MSG_IND2PWM_SRC2=900;         
/*int2lcdyx(sub_ind1,0,0,0);
int2lcdyx(sub_ind,0,1,0);
int2lcdyx(phase,0,2,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC1,0,3,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC2,0,4,0);  
int2lcdyx(MSG_IND2OUT_EN_SRC1,0,5,0);
int2lcdyx(MSG_IND2OUT_EN_SRC2,0,6,0); */

//int2lcdyx(cntrl_stat1,0,19,0); 
//int2lcdyx(load_U,0,5,0); 
//int2lcdyx(cntrl_stat,0,10,0); 
//int2lcdyx(bps[sub_ind1]._rotor,0,19,0); 
//int2lcdyx(u_necc,0,19,0);  
	 }

else if(ind==iK_bps_sel)
	{
	ptrs[0]=						" БПС N1             ";
     ptrs[1]=						" БПС N2             ";
     ptrs[2]=						" БПС N3             ";
	ptrs[3]=						" БПС N4             ";
     ptrs[4]=						" БПС N5             ";
     ptrs[5]=						" БПС N6             ";
	ptrs[6]=						" БПС N7             ";
     ptrs[7]=						" БПС N8             ";
     ptrs[8]=						" БПС N9             ";
	ptrs[9]=						" БПС N10            ";
     ptrs[10]=						" БПС N11            ";
     ptrs[11]=						" БПС N12            ";               
	ptrs[NUMIST]=					" Выход              ";
	ptrs[1+NUMIST]=				"                    ";
	ptrs[2+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("  КАЛИБРОВКА БПСов  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iK_bps)
	{
	
	ptrs[0]=" Uист =    @В       ";
     ptrs[1]=" откалибруйте Uист  ";
     ptrs[2]="  нажатием љ или њ  "; 
	ptrs[3]=" Uнагр =   #В       ";
     ptrs[4]=" откалибруйте Uнагр ";
     ptrs[5]="  нажатием љ или њ  ";
	ptrs[6]=" Uавтон =   $В      ";
	if(bFL_)
		{
		ptrs[7]=" установите Uавтон  ";
     	ptrs[8]="  нажатием љ или њ  ";
     	}
     else 
     	{
		ptrs[7]=" удерживайте ¤ для  ";
     	ptrs[8]="    запоминания     ";     	
     	}	
	ptrs[9]=" Iист =     %А      ";
	if(phase==0)
          {
          ptrs[10]=	"   нажмите ¤ для    ";
          ptrs[11]=	"калибровки нуля Iист";
          }
     else
     	{
          ptrs[10]=" откалибруйте Iист  ";
          ptrs[11]="  нажатием љ или њ  ";     	
     	} 
     	
     ptrs[12]=" tист =   ^°C       ";    
	ptrs[13]=" откалибруйте tист  ";
     ptrs[14]="  нажатием љ или њ  ";
     ptrs[15]=sm_exit;
     ptrs[16]=sm_;
     ptrs[17]=sm_;     	     	    
	

     if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
	else if((sub_ind==9)||(sub_ind==10)||(sub_ind==11))index_set=9;
	else if((sub_ind==12)||(sub_ind==13)||(sub_ind==14))index_set=12;	
	else index_set=15;
	
	bgnd_par(" КАЛИБРОВКА БПС N! ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd(bps[sub_ind1]._Uii,'@',1);
	int2lcd(bps[sub_ind1]._Uin,'#',1);
	int2lcd(U_AVT,'$',1);
	int2lcd(bps[sub_ind1]._Ii,'%',1);
	int2lcd(bps[sub_ind1]._Ti,'^',0); 
	 
	
     if((sub_ind==0)||(sub_ind==3))
		{
		mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
     if(sub_ind==6)
		{
          mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
          mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,40);
          mess_send(MESS2UNECC_HNDL,PARAM_UNECC_SET,U_AVT,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);

          }

     if(sub_ind==9)
		{
		if(phase==0)
			{
          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<sub_ind1),10);
          	}
      	else if(phase==1)
			{
          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
          	}
          mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
	
    	if(sub_ind==12)
		{
          }	
          
          
	if(mess_find( (MESS2IND_HNDL)) && (mess_data[0]==PARAM_U_AVT_GOOD) )
		{
		show_mess("     Установка      ",
	          	"    напряжения      ",
	          	" автономной работы  ",
	          	"    произведена     ",3000);
		}	     
	     
	//MSG_IND2PWM_SRC1=900;
	//MSG_IND2PWM_SRC2=900;         
/*int2lcdyx(sub_ind1,0,0,0);
int2lcdyx(sub_ind,0,1,0);
int2lcdyx(phase,0,2,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC1,0,3,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC2,0,4,0);  
int2lcdyx(MSG_IND2OUT_EN_SRC1,0,5,0);
int2lcdyx(MSG_IND2OUT_EN_SRC2,0,6,0); */

//int2lcdyx(cntrl_stat1,0,19,0); 
//int2lcdyx(load_U,0,5,0); 
//int2lcdyx(cntrl_stat,0,10,0); 
//int2lcdyx(bps[sub_ind1]._rotor,0,19,0); 
//int2lcdyx(u_necc,0,19,0);  
	 }






#endif	 
			
if(ind==iDeb)
     {
     if(sub_ind==0)
     	{


         	bgnd_par("*0000*000000*       ",
     	         "                    ",
     	         "                    ",
     	         "      ********      ");

	int2lcdyx(SOFT_NUM,0,4,0);
	long2lcdyx_mmm(SOFT_DATE,0,11,0);

/*	int2lcdyx(a_ind.i,0,2,0);
	int2lcdyx(a_ind.s_i,0,6,0);
	int2lcdyx(a_ind.s_i1,0,10,0);
	int2lcdyx(a_ind.s_i2,0,14,0);
	int2lcdyx(a_ind.i_s,0,16,0);




	int2lcdyx(c_ind.i,1,2,0);
	int2lcdyx(c_ind.s_i,1,6,0);
	int2lcdyx(c_ind.s_i1,1,10,0);
	int2lcdyx(c_ind.s_i2,1,14,0);
	int2lcdyx(c_ind.i_s,1,16,0);*/


	int2lcdyx(t_box,2,4,0);
	//int2lcdyx(t_ext_can,3,5,0);
	//int2lcdyx(t_ext_can_nd,3,6,0);
	int2lcdyx(t_ext[1],2,10,0);
	int2lcdyx(t_ext[2],3,10,0);

	//int2lcdyx(cntrl_stat,0,19,0);

	//int2lcdyx(adc_buff_[14],3,14,0);

	/*int2lcdyx(adc_buff_[12],2,14,0);
	
	int2lcdyx(adc_buff_[5],2,19,0);
	int2lcdyx(adc_buff_[6],3,19,0);*/

	//int2lcdyx(tloaddisable_cmnd,2,14,0);
	//int2lcdyx(tloaddisable_stat,3,14,0);

/*		int2lcdyx(load_U,0,15,0);
		int2lcdyx(u_necc,0,19,0);
		
		

		int2lcdyx(sub_ind1+1,1,0,0);
		int2lcdyx(sub_ind1+2,2,0,0);
		int2lcdyx(sub_ind1+3,3,0,0);
		
		
		int2lcdyx(bps[sub_ind1  ]._cnt,1,2,0);
		int2lcdyx(bps[sub_ind1+1]._cnt,2,2,0);
		int2lcdyx(bps[sub_ind1+2]._cnt,3,2,0);*/		
		
	/*	int2lcdyx(bps[sub_ind1  ]._ist_blok_cnt,1,5,0);
		int2lcdyx(bps[sub_ind1+1]._ist_blok_cnt,2,5,0);
		int2lcdyx(bps[sub_ind1+2]._ist_blok_cnt,3,5,0);*/			
		
	/*	char2lcdhyx(bps[sub_ind1  ]._flags_tu,1,8);
		char2lcdhyx(bps[sub_ind1+1]._flags_tu,2,8);
		char2lcdhyx(bps[sub_ind1+2]._flags_tu,3,8);

		int2lcdyx(bps[sub_ind1  ]._vol_u,1,12,0);
		int2lcdyx(bps[sub_ind1+1]._vol_u,2,12,0);
		int2lcdyx(bps[sub_ind1+2]._vol_u,3,12,0);		


		char2lcdhyx(bps[sub_ind1]._flags_tm,1,15);
		char2lcdhyx(bps[sub_ind1+1]._flags_tm,2,15);
		char2lcdhyx(bps[sub_ind1+2]._flags_tm,3,15);	

		char2lcdhyx(bps[sub_ind1]._Ii,1,19);
		char2lcdhyx(bps[sub_ind1+1]._Ii,2,19);
		char2lcdhyx(bps[sub_ind1+2]._Ii,3,19);*/
	/*
		char2lcdhyx(bps[sub_ind1]._rotor>>8,1,15);
		char2lcdhyx(bps[sub_ind1+1]._rotor>>8,2,15);
		char2lcdhyx(bps[sub_ind1+2]._rotor>>8,3,15);		
		
		char2lcdhyx((char)bps[sub_ind1]._rotor,1,17);
		char2lcdhyx((char)bps[sub_ind1+1]._rotor,2,17);
		char2lcdhyx((char)bps[sub_ind1+2]._rotor,3,17);*/



     	
 /*    	bgnd_par("                    ",
     	         "                    ",
     	         "                    ",
     	         "%                   ");


		int2lcdyx(main_kb_cnt,0,3,0);
		int2lcdyx(cntrl_stat,1,3,0);
   		
		
		int2lcdyx(u_necc_up,0,7,0);
		int2lcdyx(u_necc,1,7,0);
		int2lcdyx(u_necc_dn,2,7,0);
		int2lcdyx(bat[0]._Ub,3,7,0);

		int2lcdyx(sign_U,0,10,0);
		int2lcdyx(sign_I,1,10,0);
		int2lcdyx(superviser_cnt,2,10,0);	


		int2lcdyx(bat[0]._zar,0,19,0);
		int2lcdyx(BAT_C_REAL[0],1,19,0);
		int2lcdyx(BAT_C_NOM[0],2,19,0);
		int2lcdyx(//lc640_read_int(EE_BAT1_ZAR_CNT),3,19,0);  */

		  //bat_ver_cnt



	/*	int2lcdyx(tlv_buff[1][1],0,9,0);
		int2lcdyx(tlv_buff[1][2],0,14,0);
		int2lcdyx(tlv_buff[1][3],0,19,0);

   		int2lcdyx(tlv_buff[1][4],1,4,0);
		int2lcdyx(tlv_buff[1][5],1,9,0);
		int2lcdyx(tlv_buff[1][6],1,14,0);
		int2lcdyx(tlv_buff[1][7],1,19,0);

   		int2lcdyx(tlv_buff[1][8],2,4,0);
		int2lcdyx(tlv_buff[1][9],2,9,0);
		int2lcdyx(tlv_buff[1][10],2,14,0);
		int2lcdyx(tlv_buff[1][11],2,19,0);

   		int2lcdyx(tlv_buff[1][12],3,4,0);
		int2lcdyx(tlv_buff[1][13],3,9,0);
		int2lcdyx(tlv_buff[1][14],3,14,0);
		int2lcdyx(tlv_buff[1][15],3,19,0);	*/
      

     	}     

    	else if(sub_ind==1) 
     	{
     	bgnd_par("Б                   ",
     	         "                    ",
     	         "                    ",
     	         "                    ");

		//int2lcdyx(bAVG,0,0,0);
		//int2lcdyx(LPC_CAN1->GSR,0,6,0);
		//int2lcdyx((LPC_CAN1->GSR)>>16,0,16,0);
		//int2lcdyx(avg,0,19,0);

				//int2lcdyx((((LPC_CAN1->GSR)&(0xff000000))>>24),0,19,0);
		//int2lcdyx((((LPC_CAN1->GSR)&(0x00ff0000))>>16),0,15,0);

 /*         int2lcdyx(bat[0]._Ubm,1,7,0); 	int2lcdyx(bat[0]._av,1,10,0);
		int2lcdyx(bat[0]._dUbm,2,7,0);
		int2lcdyx(bat[0]._cnt_as,3,7,0);
		
 
		int2lcdyx(bat[1]._Ub,0,14,0);
          int2lcdyx(bat[1]._Ubm,1,14,0);	int2lcdyx(bat[1]._av,1,17,0);
		int2lcdyx(bat[1]._dUbm,2,14,0);
		int2lcdyx(bat[1]._cnt_as,3,14,0);*/

		int2lcdyx(sub_ind1+0,1,0,0);
		int2lcdyx(sub_ind1+1,2,0,0);
		int2lcdyx(sub_ind1+2,3,0,0);
		
		
		int2lcdyx(bps[sub_ind1  ]._cnt,1,2,0);
		int2lcdyx(bps[sub_ind1+1]._cnt,2,2,0);
		int2lcdyx(bps[sub_ind1+2]._cnt,3,2,0);		
		
	/*	int2lcdyx(bps[sub_ind1  ]._ist_blok_cnt,1,5,0);
		int2lcdyx(bps[sub_ind1+1]._ist_blok_cnt,2,5,0);
		int2lcdyx(bps[sub_ind1+2]._ist_blok_cnt,3,5,0);*/			
		
	/*	char2lcdhyx(bps[sub_ind1  ]._flags_tu,1,8);
		char2lcdhyx(bps[sub_ind1+1]._flags_tu,2,8);
		char2lcdhyx(bps[sub_ind1+2]._flags_tu,3,8);

		int2lcdyx(bps[sub_ind1  ]._vol_u,1,12,0);
		int2lcdyx(bps[sub_ind1+1]._vol_u,2,12,0);
		int2lcdyx(bps[sub_ind1+2]._vol_u,3,12,0);		


		char2lcdhyx(bps[sub_ind1]._flags_tm,1,15);
		char2lcdhyx(bps[sub_ind1+1]._flags_tm,2,15);
		char2lcdhyx(bps[sub_ind1+2]._flags_tm,3,15);	
		*/
		int2lcdyx(bps[sub_ind1]._Ii,1,15,0);
		int2lcdyx(bps[sub_ind1+1]._Ii,2,15,0);
		int2lcdyx(bps[sub_ind1+2]._Ii,3,15,0);
	/*
		char2lcdhyx(bps[sub_ind1]._rotor>>8,1,15);
		char2lcdhyx(bps[sub_ind1+1]._rotor>>8,2,15);
		char2lcdhyx(bps[sub_ind1+2]._rotor>>8,3,15);		
		*/
		
		int2lcdyx(bps[sub_ind1]._rotor,1,19,0);
		int2lcdyx(bps[sub_ind1+1]._rotor,2,19,0);
		int2lcdyx(bps[sub_ind1+2]._rotor,3,19,0);


 		}

 

    else if(sub_ind==2)
     	{
     	bgnd_par(	"F                   ",
     		    	"                    ",
     		    	"                    ",
     		    	"                    ");


		int2lcdyx(uout_av,1,5,0);
		int2lcdyx(USIGN,2,5,0); 

		int2lcdyx(bSILENT,3,5,0);

		
		
		int2lcdyx(U_OUT_KONTR_MAX,0,19,0);
		int2lcdyx(load_U,1,19,0);
		int2lcdyx(U_OUT_KONTR_MIN,2,19,0);
		//int2lcdyx(U_OUT_KONTR_DELAY,3,19,0);
		int2lcdyx(outVoltContrHndlCnt,3,19,0);

		long2lcdhyx(0x12345678UL,1,14);
		}  

	else if(sub_ind==3)
     	{
     	bgnd_par("КЕ                  ",
     	         "                    ",
     	         "                   ^",
     	         "                   &");

/*	int2lcdyx(spc_stat,0,5,0);
	int2lcdyx(__ee_spc_stat,0,9,0);
	int2lcdyx(//lc640_read_int(EE_SPC_STAT),0,13,0);

	int2lcdyx(spc_bat,1,5,0);
	int2lcdyx(__ee_spc_bat,1,9,0);
	int2lcdyx(//lc640_read_int(EE_SPC_BAT),1,13,0);

	int2lcdyx(bat_u_old_cnt,0,19,0);
	
	
	int2lcdyx(bat[0]._zar_cnt_ke,2,5,0);
	int2lcdyx(//lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[0]),2,10,0);	
	int2lcdyx(bat[0]._u_old[0],2,14,0);
	int2lcd_mmm(bat[0]._Ib,'^',2);

	int2lcdyx(bat[1]._zar_cnt_ke,3,5,0);
	int2lcdyx(//lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[1]),3,10,0);
	int2lcdyx(bat[1]._Ub,3,14,0);
	int2lcd_mmm(bat[1]._Ib,'&',2);	

	int2lcdyx(spc_phase,1,15,0);
	int2lcdyx(__ee_spc_phase,1,17,0);
	int2lcdyx(//lc640_read_int(EE_SPC_PHASE),1,19,0);	*/

	int2lcdyx(speedChIsOn,0,4,0);
	int2lcdyx(speedChTimeCnt,1,4,0);
	int2lcdyx(speedChrgBlckStat,2,4,0);
	int2lcdyx(speedChrgBlckSrc,3,4,0);

	int2lcdyx(speedChrgBlckSrc,0,8,0);
	int2lcdyx(speedChrgBlckLog,1,8,0);
	
	
		

/*	    		int2lcdyx(adc_net_buff_cnt,0,4,0);

		    	int2lcdyx((short)(main_power_buffer[0]>>12),0,19,0);
			int2lcdyx((short)(main_power_buffer[1]>>12),1,19,0);
			int2lcdyx((short)(main_power_buffer[2]>>12),2,19,0);
			int2lcdyx((short)(main_power_buffer[3]>>12),3,19,0);

		    	int2lcdyx((net_buff_),2,5,0); */


		   
		    


/*		int2lcdyx(load_U,0,4,0);
		int2lcdyx(load_I,1,4,0);
		lcd_buffer[44]='a';
		int2lcd_mmm((bat[0]._Ib)/10,'a',1);
		lcd_buffer[64]='a';
		int2lcd_mmm((bat[1]._Ib)/10,'a',1);

 		int2lcdyx(u_necc,0,8,0);

		
		
		lcd_buffer[14]='.';
		lcd_buffer[34]='.';
		int2lcdyx(Isumm,0,15,1);		
		int2lcdyx(Isumm_,1,15,1);


		int2lcdyx(cntrl_stat,0,19,0);
		int2lcdyx(num_necc,1,19,0);
		
		
		  
//		int2lcdyx(cntrl_stat,0,15,0);
		 
		//int2lcdyx(cntrl_plazma,1,3,0);
		//lcd_buffer[30]='a';
		int2lcd_mmm(Ibmax,'a',0);
		int2lcdyx(IZMAX,1,14,0);

		lcd_buffer[65]='a';
		int2lcd_mmm(bat[0]._Ib,'a',0);

		lcd_buffer[70]='a';
		int2lcd_mmm(bat[1]._Ib,'a',0); 

		lcd_buffer[75]='a';
		int2lcd_mmm(Ibmax,'a',0); 

	//	int2lcdyx(IMAX,2,3,0);
		
		

	//	int2lcdyx(IZMAX,3,19,0);

		//int2lcdyx(num_necc_Imax,3,6,0);
		//int2lcdyx(num_necc_Imin,3,12,0);


 //    	lcd_buffer[4]='a';            
 //    	int2lcd_mmm(Ibat,'a',1);   int2lcdyx(cntrl_stat,0,9,0);          int2lcdyx(hour_apv_cnt,0,13,0);                             char2lcdhyx(St_[0],0,19);  
 //    	int2lcdyx(Ubat,1,4,0);     int2lcdyx(main_apv_cnt,1,9,0);        int2lcdyx(//lc640_read_int(bps1_AVAR_PTR),1,13,0);            char2lcdhyx(St_[1],1,19);
 //    	int2lcdyx(Us[0],2,4,0);  int2lcdyx(apv_cnt_1,2,9,0);           int2lcdyx(//lc640_read_int(SRC1_AVAR_CNT),2,13,0);                                     int2lcdhyx(av_stat,2,19);
 //    	int2lcdyx(Us[1],3,4,0);  int2lcdyx(reset_apv_cnt,3,9,0);                                            int2lcdyx(plazma,3,19,0);
     	//int2lcd(plazma,'(',0);

     	//int2lcd(Us[0],'#',1);
     	//int2lcd(Us[1],'$',1);
     	//int2lcd(Is[0],'%',1);
     	//int2lcd(Is[1],'^',1);
    // 	int2lcd(bat[0]._Ub,'<',1);
    // 	int2lcd_mmm(bat[0]._Ib,'>',2);
 //    	char2lcdhyx(St_[0],3,13);
 //    	char2lcdhyx(St_[1],3,19);
 //    	char2lcdhyx(St,3,5);  */
		}

   else if(sub_ind==4)
     	{
     	bgnd_par("LB                  ",
     		    "                    ",
     		    "        !           ",
     		    "        @           ");

     	int2lcdyx(sub_ind,0,1,0);


		int2lcdyx(lakb[0]._tot_bat_volt,2,3,0);
	   	int2lcdyx(lakb[1]._tot_bat_volt,3,3,0);
	 	int2lcd_mmm(lakb[0]._ch_curr,'!',0);
		int2lcd_mmm(lakb[1]._ch_curr,'@',0);
		//int2lcdyx(lakb[0]._max_cell_temp,2,19,0);
	   	//int2lcdyx(lakb[1]._max_cell_temp,3,19,0);
		//int2lcdyx(lakb[0]._plazma_ss,2,13,0);
	   	//int2lcdyx(lakb[1]._plazma_ss,3,13,0);
		int2lcdyx(lakb[0]._communicationFullErrorStat,2,15,0);
	   	int2lcdyx(lakb[1]._communicationFullErrorStat,3,15,0);
		int2lcdyx(lakb[0]._communication2lvlErrorCnt,2,17,0);
	   	int2lcdyx(lakb[1]._communication2lvlErrorCnt,3,17,0);
		int2lcdyx(lakb[0]._communication2lvlErrorStat,2,19,0);
	   	int2lcdyx(lakb[1]._communication2lvlErrorStat,3,19,0);

		//int2lcdyx(Ibmax,1,7,0);
		//int2lcdyx(cntrl_stat,1,11,0);
		int2lcdyx(u_necc,1,15,0);
		int2lcdyx(load_U,1,19,0);

		//int2lcdyx(plazma_ztt[1],1,17,0);
 		//int2lcdyx(zTTBatteryHndlCmnd/*zTTBatteryHndlPhase*/,0,5,0);

		//int2lcdyx(zTTButteryCnter,0,7,0);

		//int2lcdyx(numOfPacks,0,9,0);

		//int2lcdyx(numOfTemperCells,0,12,0);
		//int2lcdyx(numOfCells,0,15,0);
		//int2lcdyx(bat_drv_rx_cnt,0,19,0);
		}
/*	else if(sub_ind==4)
     	{
     	bgnd_par(" АВАРИИ             ",
     	         "                    ",
     	         "                    ",
     	         "                    ");

		int2lcdyx(main_10Hz_cnt,0,7,0);
		int2lcdyx(bat[0]._av,0,10,0);
		int2lcdyx(bat[1]._av,0,12,0);
		char2lcdhyx(rele_stat,0,19);

 		long2lcdhyx(avar_stat,1,7);
		long2lcdhyx(avar_stat_old,2,7);
		long2lcdhyx(avar_ind_stat,3,7);

		long2lcdhyx(avar_stat_new,2,19);
		long2lcdhyx(avar_stat_offed,3,19);



		}*/
 
    else if(sub_ind==5)
     	{
     	bgnd_par("                    ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    
     	/*int2lcdyx(ad7705_buff[0][0],0,4,0);
     	int2lcdyx(ad7705_buff[0][1],0,9,0);
     	int2lcdyx(ad7705_buff[0][2],0,14,0);
     	int2lcdyx(ad7705_buff[0][3],0,19,0);
     	int2lcdyx(ad7705_buff[0][4],1,4,0);
     	int2lcdyx(ad7705_buff[0][5],1,9,0);
     	int2lcdyx(ad7705_buff[0][6],1,14,0);
     	int2lcdyx(ad7705_buff[0][7],1,19,0);
     	int2lcdyx(ad7705_buff[0][8],2,4,0);
     	int2lcdyx(ad7705_buff[0][9],2,9,0);
     	int2lcdyx(ad7705_buff[0][10],2,14,0);
     	int2lcdyx(ad7705_buff[0][11],2,19,0);
     	int2lcdyx(ad7705_buff[0][12],3,4,0);
     	int2lcdyx(ad7705_buff[0][13],3,9,0);
     	int2lcdyx(ad7705_buff[0][14],3,14,0);
     	int2lcdyx(ad7705_buff[0][15],3,19,0);*/


	/*	char2lcdhyx(bat_drv_rx_buff[0],0,2);
    		char2lcdhyx(bat_drv_rx_buff[1],0,5);
     	char2lcdhyx(bat_drv_rx_buff[2],0,8);
     	char2lcdhyx(bat_drv_rx_buff[3],0,11); 
     	char2lcdhyx(bat_drv_rx_buff[4],0,14);	
     	char2lcdhyx(bat_drv_rx_buff[5],0,17);
		char2lcdhyx(bat_drv_rx_buff[6],1,2);
    		char2lcdhyx(bat_drv_rx_buff[7],1,5);
     	char2lcdhyx(bat_drv_rx_buff[8],1,8);
     	char2lcdhyx(bat_drv_rx_buff[9],1,11); 
     	char2lcdhyx(bat_drv_rx_buff[10],1,14);	
     	char2lcdhyx(bat_drv_rx_buff[11],1,17);
		char2lcdhyx(bat_drv_rx_buff[12],2,2);
    		char2lcdhyx(bat_drv_rx_buff[13],2,5);
     	char2lcdhyx(bat_drv_rx_buff[14],2,8);
     	char2lcdhyx(bat_drv_rx_buff[15],2,11); 
     	char2lcdhyx(bat_drv_rx_buff[16],2,14);	
     	char2lcdhyx(bat_drv_rx_buff[17],2,17);
     	/*int2lcdhyx(bat_drv_rx_buff[12],3,4);
     	int2lcdhyx(bat_drv_rx_buff[13],3,9);
     	int2lcdhyx(bat_drv_rx_buff[14],3,14);*/
     	
	/*	int2lcdyx(bat_drv_rx_cnt,3,15,0);
		int2lcdyx(plazma_bat_drv0,3,4,0);
		int2lcdyx(plazma_bat_drv1,3,10,0);
		int2lcdyx(bat_drv_cnt_cnt,3,19,0);*/

    	}  		  		


  else if(sub_ind==6)
     	{
     	bgnd_par("6                   ",
     		    "                 В  ",
     		    "                 Н  ",
     		    "                    ");
     		    
		int2lcdyx(sk_stat[0],0,5,0);
		int2lcdyx(sk_stat[1],0,8,0);
		int2lcdyx(sk_av_stat[0],0,11,0);
		int2lcdyx(sk_av_stat[1],0,15,0);

		int2lcdyx(t_ext[0],1,3,0);
		int2lcdyx(t_ext[1],2,3,0);
		
		int2lcdyx(t_box_warm,1,7,0);
		int2lcdyx(t_box_vent,2,7,0);

//		int2lcdyx(vent_stat_k,1,18,0);
//		int2lcdyx(warm_stat_k,2,18,0);		

//		int2lcdyx(TELECORE2015_KLIMAT_WARM_ON_temp,3,3,0);
//		int2lcdyx(TELECORE2015_KLIMAT_WARM_ON,3,7,0);
//		int2lcdyx(TELECORE2015_KLIMAT_WARM_OFF,3,11,0);
//		int2lcdyx(TELECORE2015_KLIMAT_VENT_ON,3,15,0);
//		int2lcdyx(TELECORE2015_KLIMAT_VENT_OFF,3,19,0);
		
		
		
		//int2lcdyx(li_bat._Tb,3,3,0);
		
		


//		int2lcdyx(t_box_warm_on_cnt,1,10,0);
//		int2lcdyx(t_box_vent_on_cnt,1,15,0);
//		int2lcdyx(warm_stat_k,2,10,0);
//		int2lcdyx(vent_stat_k,2,15,0);
    		}


   else if(sub_ind==7)
     	{
     	bgnd_par("7                   ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    


    		}
    else if(sub_ind==8)
     	{
     	bgnd_par("                    ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     	int2lcdyx(ibt._T[0],0,2,0);
		int2lcdyx(ibt._T[1],1,2,0);
     	int2lcdyx(ibt._T[2],2,2,0);
		int2lcdyx(ibt._T[3],3,2,0);
		
     	int2lcdyx(ibt._nd[0],0,4,0);
		int2lcdyx(ibt._nd[1],1,4,0);
     	int2lcdyx(ibt._nd[2],2,4,0);
		int2lcdyx(ibt._nd[3],3,4,0);	    

     	int2lcdyx(ibt._T_dispers[0],0,7,0);
		int2lcdyx(ibt._T_dispers[1],1,7,0);
     	int2lcdyx(ibt._T_dispers[2],2,7,0);
		int2lcdyx(ibt._T_dispers[3],3,7,0);
			    
		int2lcdyx(ibt._avg1,0,19,0);
		int2lcdyx(ibt._max_dispers_num,1,19,0);
		int2lcdyx(t_box,3,19,0);
     	}		     	

    else if(sub_ind==10)
     	{
     	bgnd_par("LB                  ",
     		    "                    ",
     		    "                    ",
     		    "                    ");

     	int2lcdyx(sub_ind1+1,0,3,0);
		int2lcdyx(lakb[sub_ind1]._cnt,0,6,0);

		int2lcdyx(lakb[sub_ind1]._max_cell_temp,0,14,0);
		int2lcdyx(lakb[sub_ind1]._min_cell_temp,0,19,0);

		int2lcdyx(lakb[sub_ind1]._max_cell_volt,1,4,0);
		int2lcdyx(lakb[sub_ind1]._min_cell_volt,1,9,0);
		int2lcdyx(lakb[sub_ind1]._tot_bat_volt,1,14,0);
		int2lcdyx(lakb[sub_ind1]._s_o_h,1,19,0);

		int2lcdyx(lakb[sub_ind1]._ch_curr,2,4,0);
		int2lcdyx(lakb[sub_ind1]._dsch_curr,2,9,0);
		int2lcdyx(lakb[sub_ind1]._rat_cap,2,14,0);
		int2lcdyx(lakb[sub_ind1]._s_o_c,2,19,0);

		int2lcdyx(lakb[sub_ind1]._c_c_l_v,3,4,0);
		int2lcdyx(lakb[sub_ind1]._r_b_t,3,9,0);
		int2lcdyx(lakb[sub_ind1]._b_p_ser_num,3,14,0);
		//int2lcdyx(lakb[sub_ind1]._bRS485ERR,3,16,0);
		//int2lcdyx(lakb[sub_ind1]._rs485_cnt,3,19,0); 
		
     	}	

    else if(sub_ind==11)
     	{
     	bgnd_par("LB                  ",
     		    "                    ",
     		    "                    ",
     		    "                    ");

     	int2lcdyx(sub_ind,0,1,0);
		
		int2lcdyx(u_necc,0,5,0);

		int2lcdyx(load_U,0,11,0);
		int2lcdyx(bps[0]._Uin,0,15,0);
		int2lcdyx(bps[1]._Uii,0,19,0);

		int2lcdyx(li_bat._Ub,1,3,0);


		int2lcdyx(lakb[0]._tot_bat_volt,1,8,0);   

		//int2lcdyx(cntrl_stat,1,19,0);
		
		int2lcdyx(lakb_error_cnt,3,19,0);



		//int2lcdyx(ccc_plazma[2],3,12,0);
		//int2lcdyx(ccc_plazma[3],1,16,0);  
		//int2lcdyx(ccc_plazma[4],2,16,0);  
		//int2lcdyx(ccc_plazma[5],3,16,0); 

		int2lcdyx(li_bat._canErrorCnt,2,5,0);
		int2lcdyx(li_bat._canError,2,8,0);
		int2lcdyx(li_bat._485ErrorCnt,3,5,0);
		int2lcdyx(li_bat._485Error,3,8,0);
			
		/*int2lcdyx(lakb[sub_ind1]._cnt,0,6,0);

		int2lcdyx(lakb[sub_ind1]._max_cell_temp,0,14,0);
		int2lcdyx(lakb[sub_ind1]._min_cell_temp,0,19,0);

		int2lcdyx(lakb[sub_ind1]._max_cell_volt,1,4,0);
		int2lcdyx(lakb[sub_ind1]._min_cell_volt,1,9,0);
		int2lcdyx(lakb[sub_ind1]._tot_bat_volt,1,14,0);
		int2lcdyx(lakb[sub_ind1]._s_o_h,1,19,0);

		int2lcdyx(lakb[sub_ind1]._ch_curr,2,4,0);
		int2lcdyx(lakb[sub_ind1]._dsch_curr,2,9,0);
		int2lcdyx(lakb[sub_ind1]._rat_cap,2,14,0);
		int2lcdyx(lakb[sub_ind1]._s_o_c,2,19,0);

		int2lcdyx(lakb[sub_ind1]._c_c_l_v,3,4,0);
		int2lcdyx(lakb[sub_ind1]._r_b_t,3,9,0);
		int2lcdyx(lakb[sub_ind1]._b_p_ser_num,3,14,0);
		int2lcdyx(lakb[sub_ind1]._bRS485ERR,3,16,0);
		 */
		
     	}	
     			
     }

#ifndef _DEBUG_	
else if(ind==iAvz)
	{
	
 	if(AVZ==AVZ_1) 		ptrs[0]=	" раз в месяц        ";
	else if(AVZ==AVZ_2) 	ptrs[0]=	" раз в 2 месяца     ";
	else if(AVZ==AVZ_3) 	ptrs[0]=	" раз в 3 месяца     "; 
	else if(AVZ==AVZ_6) 	ptrs[0]=	" раз в полгода      ";
	else if(AVZ==AVZ_12) 	ptrs[0]=	" раз в год          ";
	else 				ptrs[0]=	" выключен           "; 
	
	ptrs[1]=						" Длительность    (ч.";
	if(AVZ!=AVZ_OFF)
		{
		ptrs[2]=					" очередное включение";
		ptrs[3]=					"  0%  &0^  0@:0#:0$ ";
		ptrs[4]=					sm_exit;
		}
	else ptrs[2]=						sm_exit;

	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>1) index_set=sub_ind-1;
	if((sub_ind==2)&&(AVZ!=AVZ_OFF)) index_set=2;
	
	bgnd_par(	"   АВТОМАТИЧЕСКИЙ   ",
			"ВЫРАВНИВАЮЩИЙ ЗАРЯД ",
			ptrs[index_set],
			ptrs[index_set+1]);

	pointer_set(2);
		
	int2lcd(HOUR_AVZ,'@',0);
	int2lcd(MIN_AVZ,'#',0);
	int2lcd(SEC_AVZ,'$',0);
	int2lcd(DATE_AVZ,'%',0);
	int2lcd(YEAR_AVZ,'^',0);

	sub_bgnd(sm_mont[MONTH_AVZ],'&',-2);

	int2lcd(AVZ_TIME,'(',0);
	
	}


else if(ind==iTst_RSTKM)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле               ";
     ptrs[3]=						" освещения         @";
     ptrs[4]=						" Реле отключения    ";
     ptrs[5]=						" нагрузки          &";
     ptrs[6]=						" Перемешивающий     ";
     ptrs[7]=						" вентилятор        #";
     ptrs[8]=						" Реле аварий        "; 
     ptrs[9]=                           " общее             %";
     ptrs[10]=						" Приточный          ";
     ptrs[11]=						" вентилятор        l";

     ptrs[12]=						" Реле               ";
     ptrs[13]=						" самокалибровки    ^";

	if((sub_ind==12)&&(bFL2))
		{
		ptrs[12]=					" Iбат1 =        <A  ";
     	ptrs[13]=					" Iбат2 =        >A  ";
		}
     ptrs[14]=						" Реле бат.N1       (";
     ptrs[15]=						" Реле бат.N2       )";
	ptrs[16]=						" БПС N1             ";
     ptrs[17]=						" БПС N2             ";
     ptrs[18]=						" БПС N3             ";
	ptrs[19]=						" БПС N4             ";
     ptrs[20]=						" БПС N5             ";
     ptrs[21]=						" БПС N6             ";
	ptrs[22]=						" БПС N7             ";
     ptrs[23]=						" БПС N8             ";
     ptrs[24]=						" БПС N9             ";               
	ptrs[25]=						" БПС N10            ";
     ptrs[26]=						" БПС N11            ";
     ptrs[27]=						" БПС N12            ";               
	ptrs[16+NUMIST]=				" Сброс              ";
	ptrs[17+NUMIST]=				" Сброс пл.расш.    {";
	ptrs[18+NUMIST]=				" Выход              ";
	ptrs[19+NUMIST]=				"                    ";
	ptrs[20+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);


	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[9]==tst1) sub_bgnd("ВКЛ.",'&',-3);
	if(tst_state[9]==tst2) sub_bgnd("ВЫКЛ.",'&',-4);
	else sub_bgnd("РАБОЧ.",'&',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'%',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'%',-4);
	else sub_bgnd("РАБОЧ.",'%',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);

	if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);

	if((tst_state[10]>=1)&&(tst_state[10]<=20)) int2lcd(tst_state[10],'l',0);
	else sub_bgnd("РАБОЧ.",'l',-5);

	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_LIGHT,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_LIGHT,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[9]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_LOAD_OFF,1,5);
		else if(tst_state[9]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_LOAD_OFF,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,0,5);
		}
	else if(sub_ind==8)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_COMM,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_COMM,0,5);
		}
	else if(sub_ind==10)
		{
		if((tst_state[10]>=1)&&(tst_state[10]<=20))mess_send(MESS2VENT_HNDL,PARAM_VENT_CB,tst_state[10],5);
		}
	else if(sub_ind==12)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}



	else if(sub_ind==14)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==15)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}

	//int2lcdyx(cv_pos,0,2,0);
	//char2lcdbyx(rele_stat,0,7);
	//int2lcd(ext_can_cnt,'{',0);
 	}

/*
else if(ind==iTst_KONTUR)
	{
	ptrs[0]=						" Реле аварий        "; 
     ptrs[1]=                           " общее             %";
	ptrs[2]=						" Реле аварии        ";
     ptrs[3]=						" сети              !";
     ptrs[4]=						" Реле отключения    ";
     ptrs[5]=						" нагрузки          &";
     ptrs[6]=						" Приточный          ";
     ptrs[7]=						" вентилятор        #";
     ptrs[8]=						" Реле               ";
     ptrs[9]=						" нагревателя       @";
	ptrs[10]=						" Реле               ";
     ptrs[11]=						" самокалибровки    ^";

	if((sub_ind==10)&&(bFL2))
		{
		ptrs[10]=					" Iбат1 =        <A  ";
     	ptrs[11]=					" Iбат2 =        >A  ";
		}

     ptrs[12]=						" Реле бат.N1       (";
     ptrs[13]=						" Реле бат.N2       )";
	ptrs[14]=						" БПС N1             ";
     ptrs[15]=						" БПС N2             ";
     ptrs[16]=						" БПС N3             ";
	ptrs[17]=						" БПС N4             ";
     ptrs[18]=						" БПС N5             ";
     ptrs[19]=						" БПС N6             ";
	ptrs[20]=						" БПС N7             ";
     ptrs[21]=						" БПС N8             ";
     ptrs[22]=						" БПС N9             ";               
	ptrs[23]=						" БПС N10            ";
     ptrs[24]=						" БПС N11            ";
     ptrs[25]=						" БПС N12            ";               
	ptrs[14+NUMIST]=				" Сброс              ";
	ptrs[15+NUMIST]=				" Сброс пл.расш.    {";
	ptrs[16+NUMIST]=				" Выход              ";
	ptrs[17+NUMIST]=				"                    ";
	ptrs[18+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);


	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[9]==tst1) sub_bgnd("ВКЛ.",'&',-3);
	if(tst_state[9]==tst2) sub_bgnd("ВЫКЛ.",'&',-4);
	else sub_bgnd("РАБОЧ.",'&',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'%',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'%',-4);
	else sub_bgnd("РАБОЧ.",'%',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);

	if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);



	if(sub_ind==0)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_COMM,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_COMM,0,5);
		}	
	else if(sub_ind==2)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[9]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_LOAD_OFF,1,5);
		else if(tst_state[9]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_LOAD_OFF,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,0,5);
		}
	else if(sub_ind==8)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_WARM,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_WARM,0,5);
		}

	else if(sub_ind==10)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}



	else if(sub_ind==12)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==13)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}

	//int2lcdyx(cv_pos,0,2,0);
	//char2lcdbyx(rel_warm_plazma,0,7);
	int2lcd(ext_can_cnt,'{',0);
 	}  */

else if(ind==iTst_KONTUR)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле               ";
     ptrs[3]=						" освещения         @";
     ptrs[4]=						" Реле отключения    ";
     ptrs[5]=						" нагрузки          &";
     ptrs[6]=						" Перемешивающий     ";
     ptrs[7]=						" вентилятор        #";
     if(RELE_LOG)
		{
		ptrs[6]=					" Реле               ";
     	ptrs[7]=					" отопителя         #";
		}
     ptrs[8]=						" Реле аварий        "; 
     ptrs[9]=                           " общее             %";
     ptrs[10]=						" Приточный          ";
     ptrs[11]=						" вентилятор        l";

     ptrs[12]=						" Реле               ";
     ptrs[13]=						" самокалибровки    ^";

	if((sub_ind==12)&&(bFL))
		{
		ptrs[12]=					" Iбат1 =        <A  ";
     	ptrs[13]=					" Iбат2 =        >A  ";
		}
     ptrs[14]=						" Реле бат.N1       (";
     ptrs[15]=						" Реле бат.N2       )";
	ptrs[16]=						" БПС N1             ";
     ptrs[17]=						" БПС N2             ";
     ptrs[18]=						" БПС N3             ";
	ptrs[19]=						" БПС N4             ";
     ptrs[20]=						" БПС N5             ";
     ptrs[21]=						" БПС N6             ";
	ptrs[22]=						" БПС N7             ";
     ptrs[23]=						" БПС N8             ";
     ptrs[24]=						" БПС N9             ";               
	ptrs[25]=						" БПС N10            ";
     ptrs[26]=						" БПС N11            ";
     ptrs[27]=						" БПС N12            ";               
	ptrs[16+NUMIST]=				" Сброс              ";
	ptrs[17+NUMIST]=				" Сброс пл.расш.    {";
	ptrs[18+NUMIST]=				" Выход              ";
	ptrs[19+NUMIST]=				"                    ";
	ptrs[20+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);


	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[9]==tst1) sub_bgnd("ВКЛ.",'&',-3);
	if(tst_state[9]==tst2) sub_bgnd("ВЫКЛ.",'&',-4);
	else sub_bgnd("РАБОЧ.",'&',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'%',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'%',-4);
	else sub_bgnd("РАБОЧ.",'%',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);

	if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);

	if((tst_state[10]>=1)&&(tst_state[10]<=20)) int2lcd(tst_state[10],'l',0);
	else sub_bgnd("РАБОЧ.",'l',-5);

	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_LIGHT,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_LIGHT,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[9]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_LOAD_OFF,1,5);
		else if(tst_state[9]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_LOAD_OFF,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT_WARM,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT_WARM,0,5);
		}
	else if(sub_ind==8)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_COMM,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_COMM,0,5);
		}
	else if(sub_ind==10)
		{
		if((tst_state[10]>=1)&&(tst_state[10]<=20))mess_send(MESS2VENT_HNDL,PARAM_VENT_CB,tst_state[10],5);
		}
	else if(sub_ind==12)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}



	else if(sub_ind==14)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==15)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}

	//int2lcdyx(cv_pos,0,2,0);
	//char2lcdbyx(rele_stat,0,7);
	//int2lcd(ext_can_cnt,'{',0);
 	}

else if(ind==iTst_3U)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле аварии        ";
     ptrs[3]=						" батареи №1        @";
     ptrs[4]=						" Реле аварии        ";
     ptrs[5]=						" батареи №2        &";
     ptrs[6]=						" Реле аварии        ";
     ptrs[7]=						" источников        #";
     ptrs[8]=						" Реле               ";
     ptrs[9]=						" самокалибровки    ^";

	if((sub_ind==8)&&(bFL2))
		{
		ptrs[8]=					" Iбат1 =        <A  ";
     	ptrs[9]=					" Iбат2 =        >A  ";
		}
     ptrs[10]=						" Реле бат.N1       (";
     ptrs[11]=						" Реле бат.N2       )";
	ptrs[12]=						" БПС N1             ";
     ptrs[13]=						" БПС N2             ";
     ptrs[14]=						" БПС N3             ";
	ptrs[15]=						" БПС N4             ";
     ptrs[16]=						" БПС N5             ";
     ptrs[17]=						" БПС N6             ";
	ptrs[18]=						" БПС N7             ";
     ptrs[19]=						" БПС N8             ";
     ptrs[20]=						" БПС N9             ";               
	ptrs[21]=						" БПС N10            ";
     ptrs[22]=						" БПС N11            ";
     ptrs[23]=						" БПС N12            ";               
	ptrs[12+NUMIST]=				" Сброс              ";
	ptrs[13+NUMIST]=				" Выход              ";
	ptrs[14+NUMIST]=				"                    ";
	ptrs[15+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);


	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'&',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'&',-4);
	else sub_bgnd("РАБОЧ.",'&',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);


	if(tst_state[5]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[6]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);


	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT1,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT1,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT2,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT2,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,0,5);
		}
	else if(sub_ind==8)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}
	else if(sub_ind==10)
		{
		if(tst_state[5]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==11)
		{
		if(tst_state[6]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}
	int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(bat[0]._cnt_to_block,0,3,0);
	int2lcdyx(bat[1]._cnt_to_block,0,7,0);
	//int2lcdyx(sub_ind1,0,1,0);
 	}

else if(ind==iTst_GLONASS)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле аварии        ";
     ptrs[3]=						" батареи №1        @";
     ptrs[4]=						" Реле аварии        ";
     ptrs[5]=						" батареи №2        &";
     ptrs[6]=						" Реле аварии        ";
     ptrs[7]=						" источников        #";
     ptrs[8]=						" Реле               ";
     ptrs[9]=						" самокалибровки    ^";

	if((sub_ind==8)&&(bFL2))
		{
		ptrs[8]=					" Iбат1 =        <A  ";
     	ptrs[9]=					" Iбат2 =        >A  ";
		}
     ptrs[10]=						" Реле бат.N1       (";
     ptrs[11]=						" Реле бат.N2       )";
	ptrs[12]=						" БПС N1             ";
     ptrs[13]=						" БПС N2             ";
     ptrs[14]=						" БПС N3             ";
	ptrs[15]=						" БПС N4             ";
     ptrs[16]=						" БПС N5             ";
     ptrs[17]=						" БПС N6             ";
	ptrs[18]=						" БПС N7             ";
     ptrs[19]=						" БПС N8             ";
     ptrs[20]=						" БПС N9             ";               
	ptrs[21]=						" БПС N10            ";
     ptrs[22]=						" БПС N11            ";
     ptrs[23]=						" БПС N12            ";               
	ptrs[12+NUMIST]=				" Сброс              ";
	ptrs[13+NUMIST]=				" Выход              ";
	ptrs[14+NUMIST]=				"                    ";
	ptrs[15+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);


	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'&',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'&',-4);
	else sub_bgnd("РАБОЧ.",'&',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);


	if(tst_state[5]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[6]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);


	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT1,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT1,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT2,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT2,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,0,5);
		}
	else if(sub_ind==8)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}
	else if(sub_ind==10)
		{
		if(tst_state[5]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==11)
		{
		if(tst_state[6]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}
	//int2lcdyx(sub_ind,0,19,0);
	//int2lcdyx(index_set,0,17,0);
	//int2lcdyx(bat[0]._cnt_to_block,0,3,0);
	//int2lcdyx(bat[1]._cnt_to_block,0,7,0);
	//int2lcdyx(sub_ind1,0,1,0);
 	}

else if(ind==iTst_6U)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле аварии        ";
     ptrs[3]=						" батареи N1        @";
     ptrs[4]=						" Реле аварии        ";
     ptrs[5]=						" батареи N2        &";
     ptrs[6]=						" Реле аварии        ";
     ptrs[7]=						" БПСов             #";
     ptrs[8]=						" Реле вент.        %";
     ptrs[9]=						" Реле               ";
     ptrs[10]=						" самокалибровки    ^";
	if((sub_ind==9)&&(bFL2))
		{
		ptrs[9]=					" Iбат1 =        <A  ";
     	ptrs[10]=					" Iбат2 =        >A  ";
		}
     ptrs[11]=						" Реле бат.N1       (";
     ptrs[12]=						" Реле бат.N2       )";
	ptrs[13]=						" БПС N1             ";
     ptrs[14]=						" БПС N2             ";
     ptrs[15]=						" БПС N3             ";
	ptrs[16]=						" БПС N4             ";
     ptrs[17]=						" БПС N5             ";
     ptrs[18]=						" БПС N6             ";
	ptrs[19]=						" БПС N7             ";
     ptrs[20]=						" БПС N8             ";
     ptrs[21]=						" БПС N9          ";               
	ptrs[22]=						" БПС N10            ";
     ptrs[23]=						" БПС N11            ";
     ptrs[24]=						" БПС N12            ";
	 ptrs[13+NUMIST]=				" Сброс              ";               
	ptrs[14+NUMIST]=				" Выход              ";
	ptrs[15+NUMIST]=				"                    ";
	ptrs[16+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
/*	int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(tst_state[0],0,15,0); */

	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[9]==tst1) sub_bgnd("ВКЛ.",'&',-3);
	if(tst_state[9]==tst2) sub_bgnd("ВЫКЛ.",'&',-4);
	else sub_bgnd("РАБОЧ.",'&',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'%',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'%',-4);
	else sub_bgnd("РАБОЧ.",'%',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);

	if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);
	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT1,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT1,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[9]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT2,1,5);
		else if(tst_state[9]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT2,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,0,5);
		}
	else if(sub_ind==8)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,0,5);
		}
	else if(sub_ind==9)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}
	else if(sub_ind==11)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==12)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}

		//char2lcdbyx(GET_REG(LPC_GPIO0->FIOPIN,4,8),0,19);


 	}

else if(ind==iTst_220)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле аварии        ";
     ptrs[3]=						" батарей           @";
     ptrs[4]=						" Реле аварии        ";
     ptrs[5]=						" БПСов             #";
     ptrs[6]=						" Реле вент.        %";
     ptrs[7]=						" Реле               ";
     ptrs[8]=						" самокалибровки    ^";
	if((sub_ind==7)&&(bFL2))
		{
		ptrs[7]=					" Iбат1 =        <A  ";
     	ptrs[8]=					" Iбат2 =        >A  ";
		}
     ptrs[9]=						" Реле бат.N1       [";
     ptrs[10]=						" Реле бат.N2       ]";
	ptrs[11]=						" БПС N1             ";
     ptrs[12]=						" БПС N2             ";
     ptrs[13]=						" БПС N3             ";
	ptrs[14]=						" БПС N4             ";
     ptrs[15]=						" БПС N5             ";
     ptrs[16]=						" БПС N6             ";
	ptrs[17]=						" БПС N7             ";
     ptrs[18]=						" БПС N8             ";
     ptrs[19]=						" БПС N9             ";               
	ptrs[20]=						" БПС N10            ";
     ptrs[21]=						" БПС N11            ";
     ptrs[22]=						" БПС N12            ";               
	ptrs[11+NUMIST]=				" Выход              ";
	ptrs[12+NUMIST]=				" Проверка WDT(внутр)";
	ptrs[13+NUMIST]=				" Проверка WDT(внешн)";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
/*	int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(tst_state[0],0,15,0); */

	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'%',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'%',-4);
	else sub_bgnd("РАБОЧ.",'%',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);

	if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'[',-4);
	else sub_bgnd("РАБОЧ.",'[',-5);

	if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",']',-4);
	else sub_bgnd("РАБОЧ.",']',-5);
	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,0,5);
		}
	else if(sub_ind==7)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}
	else if(sub_ind==9)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==10)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}
	}	 

else if(ind==iTst_220_380)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле аварии        ";
     ptrs[3]=						" батареи N1        @";
     ptrs[4]=						" Реле аварии        ";
     ptrs[5]=						" БПСов             #";
	 if(RELE_VENT_LOGIC==1)
	 	{
		ptrs[6]=					" Реле вент.        %";
		}
	 else ptrs[6]=					" Реле Ав.БатN2     %";
     ptrs[7]=						" Реле               ";
     ptrs[8]=						" самокалибровки    ^";
	if((sub_ind==7)&&(bFL2))
		{
		ptrs[7]=					" Iбат1 =        <A  ";
     	ptrs[8]=					" Iбат2 =        >A  ";
		}
     ptrs[9]=						" Реле бат.N1       (";
     ptrs[10]=						" Реле бат.N2       )";
	ptrs[11]=						" БПС N1             ";
     ptrs[12]=						" БПС N2             ";
     ptrs[13]=						" БПС N3             ";
	ptrs[14]=						" БПС N4             ";
     ptrs[15]=						" БПС N5             ";
     ptrs[16]=						" БПС N6             ";
	ptrs[17]=						" БПС N7             ";
     ptrs[18]=						" БПС N8             ";
     ptrs[19]=						" БПС N9            ";               
	ptrs[20]=						" БПС N10            ";
     ptrs[21]=						" БПС N11            ";
     ptrs[22]=						" БПС N12            ";               
	ptrs[11+NUMIST]=				" Выход              ";
	ptrs[12+NUMIST]=				" Сброс              ";
	ptrs[13+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
/*	int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(tst_state[0],0,15,0); */

	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'%',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'%',-4);
	else sub_bgnd("РАБОЧ.",'%',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);

	if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);
	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,0,5);
		}
	else if(sub_ind==7)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}
	else if(sub_ind==9)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==10)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}
	}	 

else if(ind==iTst_220_IPS_TERMOKOMPENSAT)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле аварии        ";
     ptrs[3]=						" БПСов             #";
     ptrs[4]=						" Реле аварии        ";
     ptrs[5]=						" батарей           @";
	ptrs[6]=						" Реле дополнительное";
     ptrs[7]=						"                   %";
	ptrs[8]=						" БПС N1             ";
     ptrs[9]=						" БПС N2             ";
     ptrs[10]=						" БПС N3             ";
	ptrs[11]=						" БПС N4             ";
     ptrs[12]=						" БПС N5             ";
     ptrs[13]=						" БПС N6             ";
	ptrs[14]=						" БПС N7             ";
     ptrs[15]=						" БПС N8             ";
     ptrs[16]=						" БПС N9             ";               
	ptrs[17]=						" БПС N10            ";
     ptrs[18]=						" БПС N11            ";
     ptrs[19]=						" БПС N12            ";               
	ptrs[8+NUMIST]=				" Выход              ";
	ptrs[9+NUMIST]=				" Проверка WDT(внутр)";
	ptrs[10+NUMIST]=				" Проверка WDT(внешн)";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
/*	int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(tst_state[0],0,15,0); */

	//int2lcd_mmm(bat[0]._Ib,'<',2);
	//int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'%',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'%',-4);
	else sub_bgnd("РАБОЧ.",'%',-5);

	//if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	//if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	//else sub_bgnd("РАБОЧ.",'^',-5);

	//if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	//else sub_bgnd("РАБОЧ.",'(',-5);

	//if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	//else sub_bgnd("РАБОЧ.",')',-5);
	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
/*	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,0,5);
		}*/
	else if(sub_ind==2)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BPS,0,5);
		}

	else if(sub_ind==4)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_BAT,0,5);
		}

	else if(sub_ind==6)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_EXT,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_EXT,0,5);
		}
/*	else if(sub_ind==7)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}
	else if(sub_ind==9)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==10)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}*/
	}

else if(ind==iTst_TELECORE2015)
	{
	ptrs[0]=						" Реле аварии        ";
     ptrs[1]=						" сети              !";
     ptrs[2]=						" Реле               ";
     ptrs[3]=						" освещения         @";
     ptrs[4]=						" Реле               ";
     ptrs[5]=						" нагревателя       #";
     ptrs[6]=						" Реле               ";
     ptrs[7]=						" вентилятора       $";
     ptrs[8]=						" Внутренний         ";
     ptrs[9]=						" вентилятор        }";     
	ptrs[10]=						" Реле               ";
     ptrs[11]=						" самокалибровки    ^";

	if((sub_ind==10)&&(bFL))
		{
		ptrs[10]=					" Iбат  =        <A  ";
     	ptrs[11]=					"                    ";
		}
	ptrs[12]=						" БПС N1             ";
     ptrs[13]=						" БПС N2             ";
     ptrs[14]=						" БПС N3             ";
	ptrs[15]=						" БПС N4             ";
 	ptrs[12+NUMIST]=				" Сброс              ";
	ptrs[13+NUMIST]=				" Сброс пл.расш.    {";
	ptrs[14+NUMIST]=				" Выход              ";
	ptrs[15+NUMIST]=				"                    ";
	ptrs[16+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ТЕСТ        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);


	int2lcd_mmm(bat[0]._Ib,'<',2);
	int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("ВКЛ.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("ВЫКЛ.",'!',-4);
	else sub_bgnd("РАБОЧ.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("ВКЛ.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("ВЫКЛ.",'@',-4);
	else sub_bgnd("РАБОЧ.",'@',-5);

	if(tst_state[3]==tst1) sub_bgnd("ВКЛ.",'#',-3);
	if(tst_state[3]==tst2) sub_bgnd("ВЫКЛ.",'#',-4);
	else sub_bgnd("РАБОЧ.",'#',-5);

	if(tst_state[4]==tst1) sub_bgnd("ВКЛ.",'$',-3);
	if(tst_state[4]==tst2) sub_bgnd("ВЫКЛ.",'$',-4);
	else sub_bgnd("РАБОЧ.",'$',-5);

	if(tst_state[2]==tst1) sub_bgnd("ВКЛ.",'^',-3);
	if(tst_state[2]==tst2) sub_bgnd("ВЫКЛ.",'^',-4);
	else sub_bgnd("РАБОЧ.",'^',-5);

	if(tst_state[5]==tst1) sub_bgnd("ВКЛ.",'}',-3);
	if(tst_state[5]==tst2) sub_bgnd("ВЫКЛ.",'}',-4);
	else sub_bgnd("РАБОЧ.",'}',-5);

	if(tst_state[7]==tst1) sub_bgnd("ВЫКЛ.",'(',-4);
	else sub_bgnd("РАБОЧ.",'(',-5);

	if(tst_state[8]==tst1) sub_bgnd("ВЫКЛ.",')',-4);
	else sub_bgnd("РАБОЧ.",')',-5);

	if((tst_state[10]>=1)&&(tst_state[10]<=20)) int2lcd(tst_state[10],'l',0);
	else sub_bgnd("РАБОЧ.",'l',-5);

	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_NET,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_LIGHT,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_LIGHT,0,5);
		}
	else if(sub_ind==4)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_WARM,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_WARM,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[4]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,1,5);
		else if(tst_state[4]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VENT,0,5);
		}
	else if(sub_ind==8)
		{
		if(tst_state[5]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_VVENT,1,5);
		else if(tst_state[5]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_VVENT,0,5);
		}
	else if(sub_ind==10)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,0,5);
		}



/*	else if(sub_ind==14)
		{
		if(tst_state[7]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<0),10);

		}
	else if(sub_ind==15)
		{
		if(tst_state[8]==tst1)mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<1),10);

		}*/

	//int2lcdyx(cv_pos,0,2,0);
	//char2lcdbyx(rele_stat,0,7);
	//int2lcd(ext_can_cnt,'{',0);
 	}

else if(ind==iTst_bps)
	{
	if(tst_state[5]==tstOFF)ptrs[0]=		" Выключен           ";
	else if(tst_state[5]==tst1)ptrs[0]=		" Включен            ";
	else ptrs[0]=							" Автономно          ";
    ptrs[1]=								" ШИМ              @ ";
    ptrs[2]=								" U =  .$В  I =  .#A ";
	ptrs[3]=								" Выход              ";


	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ТЕСТ БПС N!    ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);

	if(tst_state[6]==tst1) sub_bgnd("Uтемпер(  .@В)",'@',-13);
	else if(tst_state[6]==tst2) sub_bgnd("Umax",'@',-3);
	else sub_bgnd("Umin",'@',-3);




	/*int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(tst_state[5],0,3,0);
	int2lcdyx(tst_state[6],0,5,0); */
	//int2lcdyx(sub_ind1,0,4,0); 

	int2lcd(sub_ind1+1,'!',0);
	if(tst_state[5]==tst2)
		{
		#ifdef UKU_220
		int2lcd(load_U/10,'$',0);
		#else 
		int2lcd(load_U,'$',1);
		#endif
		}
	else
		{
		#ifdef UKU_220
		int2lcd(bps[sub_ind1]._Uii/10,'$',0);
		#else 
		int2lcd(bps[sub_ind1]._Uii,'$',1);
		#endif
		}
	
	int2lcd(bps[sub_ind1]._Ii,'#',1);
	int2lcd(u_necc,'@',1);


	if(tst_state[5]==tstOFF) mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<sub_ind1),10);
	else mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
	
	if(tst_state[5]==tst2) mess_send(MESS2NET_DRV,PARAM_BPS_NET_OFF,1,10);
		
	if(tst_state[6]==tstOFF) mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,30,10);
	else if(tst_state[6]==tst1) 
		{
		mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,30,10);
		mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);
		}
	else if(tst_state[6]==tst2) mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1020,10);
/*		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
     if(sub_ind==6)
		{
          mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
          mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,40);
          mess_send(MESS2UNECC_HNDL,PARAM_UNECC_SET,U_AVT,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);

          }

     if(sub_ind==9)
		{
		if(phase==0)
			{
          	
          	}
      	else if(phase==1)
			{
          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
          	}
          mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
	*/




	}	 
else if(ind==iKlimat)
	{
	ptrs[0]=				" tшк.max=       !°C ";
	ptrs[1]=				" tвент.max=     @°C ";
	ptrs[2]=				" tшк.рег.=      #°C ";
	ptrs[3]=				" tоткл.нагр.    $°C ";
	ptrs[4]=				" tвкл.нагр.     %°C ";
	ptrs[5]=				" tоткл.бат.     ^°C ";
	ptrs[6]=				" tвкл.бат.      &°C ";
	ptrs[7]=				" Выход              ";



	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				"   КЛИМАТКОНТРОЛЬ   ",
						ptrs[index_set],
						ptrs[index_set+1],
						ptrs[index_set+2]);
	pointer_set(1);
	
	int2lcd(TBOXMAX,'!',0); 
	if((TBOXVENTMAX>=50)&&(TBOXVENTMAX<=80))int2lcd(TBOXVENTMAX,'@',0); 
	else sub_bgnd("ВЫКЛ.",'@',-2);
	int2lcd(TBOXREG,'#',0);
	if((TLOADDISABLE>=50)&&(TLOADDISABLE<=80))int2lcd(TLOADDISABLE,'$',0);
	else sub_bgnd("ВЫКЛ.",'$',-2);
	if((TLOADDISABLE>=50)&&(TLOADDISABLE<=80)/*&&(TLOADENABLE>=50)&&(TLOADENABLE<=80)*/)int2lcd(TLOADENABLE,'%',0);
	else sub_bgnd("ВЫКЛ.",'%',-2);
	if((TBATDISABLE>=50)&&(TBATDISABLE<=90))int2lcd(TBATDISABLE,'^',0);
	else sub_bgnd("ВЫКЛ.",'^',-2);
	if((TBATDISABLE>=50)&&(TBATDISABLE<=90)/*&&(TBATENABLE>=50)&&(TBATENABLE<=80)*/)int2lcd(TBATENABLE,'&',0);
	else sub_bgnd("ВЫКЛ.",'&',-2);
	}

else if(ind==iKlimat_kontur)
	{
	ptrs[0]=				" tшк.max=       !°C ";
	ptrs[1]=				" tвент.max=     @°C ";
	ptrs[2]=				" tшк.рег.=      #°C ";
	ptrs[3]=				" tвкл.отопит.   $°C ";
	ptrs[4]=				" tоткл.отопит.  %°C ";
	ptrs[5]=				" tоткл.нагруз.  ^°C ";
	ptrs[6]=				" tвкл.нагруз.   &°C ";
	ptrs[7]=				" tоткл.бат.     *°C ";
	ptrs[8]=				" tвкл.бат.      (°C ";
	ptrs[9]=				" Выход              ";



	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				"   КЛИМАТКОНТРОЛЬ   ",
						ptrs[index_set],
						ptrs[index_set+1],
						ptrs[index_set+2]);
	pointer_set(1);
	
	int2lcd(TBOXMAX,'!',0); 
/*	int2lcd(TBOXVENTON,'@',0); 
	int2lcd(TBOXVENTOFF,'#',0);*/
	if((TBOXVENTMAX>=50)&&(TBOXVENTMAX<=80))int2lcd(TBOXVENTMAX,'@',0); 
	else sub_bgnd("ВЫКЛ.",'@',-2);
	int2lcd(TBOXREG,'#',0);
	int2lcd_mmm(TBOXWARMON,'$',0); 
	int2lcd_mmm(TBOXWARMOFF,'%',0);
	if((TLOADDISABLE>=50)&&(TLOADDISABLE<=80))int2lcd(TLOADDISABLE,'^',0);
	else sub_bgnd("ВЫКЛ.",'^',-2);
	if((TLOADDISABLE>=50)&&(TLOADDISABLE<=80))int2lcd(TLOADENABLE,'&',0);
	else sub_bgnd("ВЫКЛ.",'&',-2);
	if((TBATDISABLE>=50)&&(TBATDISABLE<=90))int2lcd(TBATDISABLE,'*',0);
	else sub_bgnd("ВЫКЛ.",'*',-2);
	if((TBATDISABLE>=50)&&(TBATDISABLE<=90))int2lcd(TBATENABLE,'(',0);
	else sub_bgnd("ВЫКЛ.",'(',-2);

//		int2lcdyx(t_box_vent_on_cnt,0,3,0);
//	int2lcdyx(t_box_warm_on_cnt,0,7,0);
//	int2lcdyx(vent_stat_k,0,10,0);
//	int2lcdyx(warm_stat_k,0,13,0);
	
	int2lcdyx(t_box,0,19,0);	 
	}

else if(ind==iKlimat_TELECORE2015)
	{
	ptrs[0]=				" Сигнал температуры ";
	ptrs[1]=				" обогрева         ! ";
	ptrs[2]=				" Сигнал температуры ";
	ptrs[3]=				" вентилятора      @ ";
	ptrs[4]=				" tвкл.отопит.   $°C ";
	ptrs[5]=				" tоткл.отопит.  <°C ";
	ptrs[6]=				" Qперкл.        *%  ";
	ptrs[7]=				" tвкл.вент.     ^°C ";
	ptrs[8]=				" tоткл.вент.    &°C ";
	ptrs[9]=				" tвкл.вн.вент.  [°C ";
	ptrs[10]=				" tоткл.вн.вент. ]°C ";
	ptrs[11]=				" Выход              ";



	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				"   КЛИМАТКОНТРОЛЬ   ",
						ptrs[index_set],
						ptrs[index_set+1],
						ptrs[index_set+2]);
	pointer_set(1);

	if(TELECORE2015_KLIMAT_WARM_SIGNAL==0) sub_bgnd("ДТ 2",'!',-3);
	else if(TELECORE2015_KLIMAT_WARM_SIGNAL==1) sub_bgnd("ДТ 1",'!',-3);
	//else sub_bgnd("ДТ BMS",'!',-5);
	if(TELECORE2015_KLIMAT_VENT_SIGNAL==0) sub_bgnd("ДТ 2",'@',-3);
	else if(TELECORE2015_KLIMAT_VENT_SIGNAL==1) sub_bgnd("ДТ 1",'@',-3);
	//else sub_bgnd("ДТ BMS",'@',-5);
	int2lcd_mmm(TELECORE2015_KLIMAT_WARM_ON,'$',0); 
	int2lcd_mmm(TELECORE2015_KLIMAT_WARM_OFF,'<',0);
	int2lcd_mmm(TELECORE2015_KLIMAT_CAP,'*',0);	
	int2lcd_mmm(TELECORE2015_KLIMAT_VENT_ON,'^',0);
	int2lcd_mmm(TELECORE2015_KLIMAT_VENT_OFF,'&',0);
	int2lcd_mmm(TELECORE2015_KLIMAT_VVENT_ON,'[',0);
	int2lcd_mmm(TELECORE2015_KLIMAT_VVENT_OFF,']',0);

	//int2lcd(TBOXMAX,'!',0); 
	
/*	int2lcd(TBOXVENTON,'@',0); 
	int2lcd(TBOXVENTOFF,'#',0);*/
	//if((TBOXVENTMAX>=50)&&(TBOXVENTMAX<=80))int2lcd(TBOXVENTMAX,'@',0); 
	//else sub_bgnd("ВЫКЛ.",'@',-2);
	//int2lcd(TBOXREG,'#',0);




//	int2lcdyx(t_ext[0],0,3,0);
//	int2lcdyx(t_ext[1],0,7,0);
/*//	int2lcdyx(vent_stat_k,0,10,0);
	int2lcdyx(TELECORE2015_KLIMAT_WARM_ON_temp,0,13,0);
	
	int2lcdyx(t_box,0,19,0);	 */
	}


#endif

else if(ind==iBps_list)
     {
     if(sub_ind==0)
     	{
     	bgnd_par(" N  L   U    I    t " ,
     	         " !  @    ^    $    #",
     	         " !  @    ^    $    #",
     	         " !  @    ^    $    #");
      

     	}     

    	else if(sub_ind==1) 
     	{
      	bgnd_par(" N  L   U    I   Uн " ,
     	         " !  @    ^    $    %",
     	         " !  @    ^    $    %",
     	         " !  @    ^    $    %");

		} 

	int2lcd(sub_ind1+1,'!',0);
	int2lcd(sub_ind1+2,'!',0);
	if(sub_ind1==NUMIST-2) sub_bgnd("Ш",'!',0);
	else int2lcd(sub_ind1+3,'!',0);

	int2lcd(bps[sub_ind1]._cnt,'@',0);
	int2lcd(bps[sub_ind1+1]._cnt,'@',0);
	if(sub_ind1==NUMIST-2)int2lcd(bps[8]._cnt,'@',0);
	else int2lcd(bps[sub_ind1+2]._cnt,'@',0);		
		
	int2lcd(bps[sub_ind1]._Uii/10,'^',0);
	int2lcd(bps[sub_ind1+1]._Uii/10,'^',0);
	if(sub_ind1<NUMIST-2) int2lcd(bps[sub_ind1+2]._Uii/10,'^',0);
	else sub_bgnd(" ",'^',0);

     int2lcd(bps[sub_ind1]._Ii,'$',1); 
	int2lcd(bps[sub_ind1+1]._Ii,'$',1); 
	if(sub_ind1<NUMIST-2) int2lcd(bps[sub_ind1+2]._Ii,'$',1); 
	else int2lcd_mmm(Ib_ips_termokompensat,'$',2);

	int2lcd(bps[sub_ind1]._Uin/10,'%',0);
	int2lcd(bps[sub_ind1+1]._Uin/10,'%',0);
	if(sub_ind1<NUMIST-2) int2lcd(bps[sub_ind1+2]._Uin/10,'%',0);
	else sub_bgnd(" ",'%',0);

	int2lcd(bps[sub_ind1]._Ti,'#',0);
	int2lcd(bps[sub_ind1+1]._Ti,'#',0); 
   	if(sub_ind1<NUMIST-2) int2lcd(bps[sub_ind1+2]._Ti,'#',0);
	else sub_bgnd(" ",'#',0);

	}
	
else if(ind==iAvt_set_sel)
	{
	ptrs[0]=						" БПС N1             ";
     ptrs[1]=						" БПС N2             ";
     ptrs[2]=						" БПС N3             ";
	ptrs[3]=						" БПС N4             ";
     ptrs[4]=						" БПС N5             ";
     ptrs[5]=						" БПС N6             ";
	ptrs[6]=						" БПС N7             ";
     ptrs[7]=						" БПС N8             ";
     ptrs[8]=						" БПС N9             ";
	ptrs[9]=						" БПС N10            ";
     ptrs[10]=						" БПС N11            ";
     ptrs[11]=						" БПС N12            ";               
	ptrs[NUMIST]=					" Выход              ";
	ptrs[1+NUMIST]=				"                    ";
	ptrs[2+NUMIST]=				"                    ";


	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				"УСТАНОВКА НАПРЯЖЕНИЯ",
						" АВТОНОМНОЙ РАБОТЫ  ",
						ptrs[index_set],
						ptrs[index_set+1]);
	pointer_set(2);

	}		 

else if(ind==iAvt_set)
	{
	ptrs[0]=" Uавтон =   $В      ";
	if(bFL_)
		{
		ptrs[1]=" установите Uавтон  ";
     	ptrs[2]="  нажатием љ или њ  ";
     	}
     else 
     	{
		ptrs[1]=" удерживайте ¤ для  ";
     	ptrs[2]="    запоминания     ";     	
     	}	

     ptrs[3]=sm_exit;
     ptrs[4]=sm_;
     ptrs[5]=sm_;     	     	    
	

     if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else index_set=3;
	
	bgnd_par("       БПС N!      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd(U_AVT,'$',1);
	 
	
     if(sub_ind==0)
		{
        mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
        mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,40);
        mess_send(MESS2UNECC_HNDL,PARAM_UNECC_SET,U_AVT,10);
	    mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);

        }

 	if(mess_find( (MESS2IND_HNDL)) && (mess_data[0]==PARAM_U_AVT_GOOD) )
		{
		sub_ind=3;
		show_mess(	"     Установка      ",
	          		"    напряжения      ",
	          		" автономной работы  ",
	          		"    произведена     ",3000);
		
		}
	//int2lcdyx(sub_ind,0,4,0);		      
	 }
else if(ind==iOut_volt_contr)
	{
	ptrs[0]=" Uвыхmax         !В ";
    ptrs[1]=" Uвыхmin         @В ";
    ptrs[2]=" Tздрж.ав.       #с.";     	
    ptrs[3]=sm_exit;
    ptrs[4]=sm_;
    ptrs[5]=sm_;     	     	    
	

	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	
	bgnd_par(	"КОНТРОЛЬ ВЫХОДНОГО ",
				"    НАПРЯЖЕНИЯ     ",
				ptrs[index_set],
				ptrs[index_set+1]);

	pointer_set(2);	
	int2lcd(U_OUT_KONTR_MAX,'!',1);
	int2lcd(U_OUT_KONTR_MIN,'@',1);
	int2lcd(U_OUT_KONTR_DELAY,'#',0);
	}
else if(ind==iDop_rele_set)
	{
	if(DOP_RELE_FUNC==0)
		{
		ptrs[0]=	" Индикация          ";
		ptrs[1]=	" ускоренного заряда ";
		}
	else /*if(DOP_RELE_FUNC==0)*/
		{
		ptrs[0]=	" Индикация          ";
		ptrs[1]=	" разряженной батареи";
		}
	ptrs[2]=		" Выход              ";
	ptrs[3]=		"                    ";


	if((sub_ind-index_set)>1)index_set=sub_ind;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(			"ДОПОПЛНИТЕЛЬНОЕ РЕЛЕ",
						"  ФУНКЦИОНАЛЬНОСТЬ  ",
						ptrs[index_set],
						ptrs[index_set+1]);
	pointer_set(2);

	//int2lcdyx(sub_ind,0,6,0);
	//int2lcdyx(index_set,0,9,0);
	
	}

else if(ind==iSet_li_bat)
	{
	char ptr_ptrs=0;
	ptrs[ptr_ptrs++]=		"ПАРАМЕТРЫ СОДЕРЖАНИЯ";
    ptrs[ptr_ptrs++]=  		"  ЛИТИЕВОЙ БАТАРЕИ  ";
    ptrs[ptr_ptrs++]=  	    " Uсодерж.         !В"; 
	ptrs[ptr_ptrs++]=  	    " Qсодерж.         $%";
	ptrs[ptr_ptrs++]=		" Iзар.max1.       @A";	           
	ptrs[ptr_ptrs++]=		" Iзар.max2.       #A";
	ptrs[ptr_ptrs++]=  	    " K1            ^ед/с";
	ptrs[ptr_ptrs++]=  	    " K2            &ед/с";
	ptrs[ptr_ptrs++]=  	    " K3            *ед/с";
	ptrs[ptr_ptrs++]=  	    " T4             (сек";
	ptrs[ptr_ptrs++]=  	    " Выход              ";

	bgnd_par(	ptrs[0],
				ptrs[1],
				ptrs[index_set+2],
				ptrs[index_set+3]);

	if(sub_ind-index_set>1)index_set=sub_ind-1;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(2);

	int2lcd(LI_UNECC,'!',1);
    int2lcd(LI_Q,'$',0);
    int2lcd(LI_IZMAX1,'@',1);
    int2lcd(LI_IZMAX2,'#',1);
	int2lcd(LI_K1,'^',0);
    int2lcd(LI_K2,'&',0);
    int2lcd(LI_K3,'*',0);
    int2lcd(LI_T4,'(',0);
	}

/*
const char sm7[]	={" Источник N2        "}; //
const char sm8[]	={" Нагрузка           "}; //
const char sm9[]	={" Сеть               "}; //
const char sm10[]	={" Спецфункции        "}; // 
const char sm11[]	={" Журнал аварий      "}; //
const char sm12[]	=" Батарейный журнал  "}; //
const cha		=" Паспорт            "}; //
*/


//char2lcdhyx(bat_rel_stat[0],0,10);
//char2lcdhyx(bat_rel_stat[1],0,15);
//int2lcdyx(u_necc,0,19,0);
//int2lcdyx(cntrl_stat,0,5,0); 	   mess_cnt[i]

//char2lcdhyx(bat_rel_stat[0],0,5);
//char2lcdhyx(bat_rel_stat[1],0,10);
//int2lcdyx(mess_cnt[1],0,2,0);
//int2lcdyx(GET_REG(IOPIN1,21,1),0,5,0); 
//int2lcdyx(samokalibr_cnt,0,10,0);
//char2lcdhyx(rele_stat,0,19);
//char2lcdhyx(mess_cnt[1],0,16); 

//int2lcdyx(ad7705_res1,0,8,0);
//int2lcdyx(ad7705_res2,0,16,0); 
//	int2lcdyx(bat[0]._cnt_to_block,0,1,0);
//	int2lcdyx(bat[1]._cnt_to_block,0,3,0);
//	int2lcdyx(bat[0]._rel_stat,0,5,0);
/*	int2lcdyx(ind,0,3,0); 
	int2lcdyx(sub_ind,0,6,0);
	int2lcdyx(index_set,0,9,0);
	int2lcdyx(ptr_ind,0,14,0);
	;*/
/*int2lcdyx(ind,0,19,0);
int2lcdyx(retindsec,0,15,0);
int2lcdyx(retcnt,0,11,0);
int2lcdyx(retcntsec,0,7,0);	*/
//int2lcdyx(bps[0]._vol_i,0,15,0);
//int2lcdyx(AUSW_MAIN,0,19,0); 
}							    


#define BUT0	16
#define BUT1	17
#define BUT2	18
#define BUT3	19
#define BUT4	20   
#define BUT_MASK (1UL<<BUT0)|(1UL<<BUT1)|(1UL<<BUT2)|(1UL<<BUT3)|(1UL<<BUT4)

#define BUT_ON 4
#define BUT_ONL 20 

#define butLUR_  101
#define butU   253
#define butU_  125
#define butD   251
#define butD_  123
#define butL   247
#define butL_  119
#define butR   239
#define butR_  111
#define butE   254
#define butE_  126
#define butEL_  118
#define butUD  249
#define butUD_  121
#define butLR   231
#define butLR_   103
#define butED_  122
#define butDR_  107
#define butDL_  115

#define BUT_ON 4
#define BUT_ONL 20 
//-----------------------------------------------
void but_drv(void)
{
char i;
LPC_GPIO1->FIODIR|=(1<<21);
LPC_GPIO1->FIOPIN&=~(1<<21);
LPC_GPIO1->FIODIR&=~((1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<26));
LPC_PINCON->PINMODE3&=~((1<<12)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<17)|(1<<18)|(1<<19)|(1<<20)|(1<<21));

LPC_GPIO2->FIODIR|=(1<<8);
LPC_GPIO2->FIOPIN&=~(1<<8);
for(i=0;i<200;i++)
{
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
}

			LPC_GPIO2->FIODIR|=(1<<8);
			LPC_GPIO2->FIOPIN|=(1<<8);

but_n=((LPC_GPIO1->FIOPIN|(~((1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<26))))>>22)/*&0x0000001f*/;



if((but_n==1023UL)||(but_n!=but_s))
 	{
	speed=0;
 
   	if (((but0_cnt>=BUT_ON)||(but1_cnt!=0))&&(!l_but))
  		{
   	     n_but=1;
          but=but_s;

          }
   	if (but1_cnt>=but_onL_temp)
  		{
   	     n_but=1;
 
          but=but_s&0x7f;
          }
    	l_but=0;
   	but_onL_temp=BUT_ONL;
    	but0_cnt=0;
  	but1_cnt=0;          
     goto but_drv_out;
  	}
else if(but_n==but_s)
 	{
  	but0_cnt++;
  	if(but0_cnt>=BUT_ON)
  		{
   		but0_cnt=0;
   		but1_cnt++;
   		if(but1_cnt>=but_onL_temp)
   			{              
    			but=but_s&0x7f;
    			but1_cnt=0;
    			n_but=1;
    			     
    			l_but=1;
			if(speed)
				{
    				but_onL_temp=but_onL_temp>>1;
        			if(but_onL_temp<=2) but_onL_temp=2;
				}    
   			}
  		}
 	}
but_drv_out: 
but_s=but_n; 
   
}

//-----------------------------------------------
void but_an(void)
{
signed short temp_SS;
signed short deep,i,cap,ptr;
char av_head[4];
if(!n_but)goto but_an_end;
/*else  					
	{
	plazma_but_an++;
	goto but_an_end;
	}*/
av_beep=0x0000;
av_rele=0x0000;
mnemo_cnt=MNEMO_TIME;
ips_bat_av_stat=0;
//bat_ips._av&=~1;


if(but==butUD)
     {
     if(ind!=iDeb)
          {
		c_ind=a_ind;
		tree_up(iDeb,4,0,0);
		
          }
     else 
          {
		tree_down(0,0);
          }
		
		     
     }
else if(but==butLR)
	{
	bSILENT=1;
	beep_init(0x00000000,'S');
	}
else if(but==butUD_)
     {
	//avar_bat_as_hndl(0,1);
	}

else if(but==butED_)
     {
	if(!bCAN_OFF)bCAN_OFF=1;
	else bCAN_OFF=0;
	speed=0;
	}

else if(ind==iDeb)
	{
	if(but==butR)
		{
		sub_ind++;
		index_set=0;
		gran_ring_char(&sub_ind,0,11);
		}
	else if(but==butL)
		{
		sub_ind--;
		index_set=0;
		gran_ring_char(&sub_ind,0,11);
		}
		
	else if(sub_ind==1)
		{
		if(but==butU)
	     	{
	     	sub_ind1--;
	     	gran_char(&sub_ind1,0,30);
	     	}
		if(but==butD)
	     	{
	     	sub_ind1++;
	     	gran_char(&sub_ind1,0,30);
	     	}
	     
		if(but==butE)
	     	{
	     	/*SET_REG(C2GSR,3,24,8);
			C2MOD=0;
			 bOUT_FREE2=1;*/

			 // CAN interface 1, use IRQVec7, at 125kbit
//can2_init(7,8,CANBitrate250k_60MHz);

// Receive message with ID 102h on CAN 1
//FullCAN_SetFilter(2,0x18e);
			 }

		if(but==butE)
	     	{
			////lc640_write_int(EE_BAT1_ZAR_CNT,10);
			ind_pointer=0;
			ind=(i_enum)0;
			sub_ind=0;
			sub_ind1=0;
			sub_ind2=0;
			index_set=0;
			}
	     
			
		} 

	 else if(sub_ind==5)
	 	{
		//if(but==butE_)	numOfForvardBps_init();
		}
				
	 else if(sub_ind==5)
	 	{
		if(but==butE_)
		{
		//can1_init(BITRATE62_5K6_25MHZ);
		//FullCAN_SetFilter(0,0x18e);
		LPC_CAN1->MOD&=~(1<<0);
		}
		}

	else if(sub_ind==1)
		{
		if(but==butU)
	     	{
	     	sub_ind1--;
	     	gran_char(&sub_ind1,0,1);
	     	}
		if(but==butD)
	     	{
	     	sub_ind1++;
	     	gran_char(&sub_ind1,0,1);
	     	}
		}		
		
		
			
     else if(but==butU)
	     {
	     index_set--;
	     gran_char(&index_set,0,4);
	     ////lc640_write_int(ptr_ki_src[0],//lc640_read_int(ptr_ki_src[0])+10);
	     }	
     else if(but==butD)
	     {
	     index_set++;
	     gran_char(&index_set,0,4); 
	     ////lc640_write_int(ptr_ki_src[0],//lc640_read_int(ptr_ki_src[0])-10);
	     }	
     else if(but==butE)
         	{
          //a=b[--ptr_ind];
          //can1_out(1,2,3,4,5,6,7,8);
          }   
          
     else if(but==butE_)
         	{
          //a=b[--ptr_ind];
          //can1_out_adr(TXBUFF,3);
          }                      				
	}

else if(ind==iMn)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,9+NUMBAT+NUMINV+NUMIST+3+(NUMAVT!=0));
		//LPC_CAN1->GSR = 0;
		
		}
		
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,9+NUMBAT+NUMINV+NUMIST+3+(NUMAVT!=0));
		}	

	else if(but==butL)
		{
		//ind=iMn;
		sub_ind=0;
		}
	
	else if(but==butE_)
		{
		//bRESET=1;//tree_up(iK_bps_sel,0,0,0);
		}
		
	else if(but==butE)
		{
		if(sub_ind==0)
			{
																							
			}
		else if((sub_ind>0)&&(sub_ind<=NUMBAT))
		    	{
		    	if(BAT_IS_ON[0]!=bisON)tree_up(iBat,0,0,1);
		    	else tree_up(iBat,0,0,sub_ind-1);
		    	}
		else if((sub_ind>NUMBAT)&&(sub_ind<=(NUMBAT+NUMIST)))
		    	{
		    	tree_up(iBps,0,0,sub_ind-(1+NUMBAT));
		    	}
/**/		else if((sub_ind>(NUMBAT+NUMIST))&&(sub_ind<=(NUMINV+NUMBAT+NUMIST)))
		    	{
		    	tree_up(iInv,0,0,sub_ind-(1+NUMBAT));
//		    	tree_up(iInv,0,0,2); // 2 - адрес инвертора зашит как №3
/**/
		    	}
		else if(sub_ind==(1+NUMBAT+NUMIST+NUMINV))
			{
			tree_up(iNet,0,0,0);
		     ret(1000);
			}
		else if(sub_ind==(2+NUMBAT+NUMIST+NUMINV))
			{
			tree_up(iLoad,0,0,0);
		     ret(1000);
			}
		else if((sub_ind==(3+NUMBAT+NUMIST+NUMINV)))
			{
			#ifdef UKU
			tree_up(iExtern,0,0,0);
		     ret(1000);
			#endif
			#ifdef UKU_3U
			tree_up(iExtern_3U,0,0,0);
		     ret(1000);
			#endif
			}
		else if((sub_ind==(3+NUMBAT+NUMIST+NUMINV+1)))
			{
			tree_up(iVent,1,0,0);
		     ret(1000);
			}

		else if((sub_ind==(3+NUMBAT+NUMIST+NUMINV+2))&&(NUMAVT))
			{
			tree_up(iAvt,0,0,0);
		     ret(1000);
			}

		#ifdef UKU_MGTS
		else if(sub_ind==(3+NUMBAT+NUMIST+NUMINV+2)+(NUMAVT!=0))
			{
			tree_up(iEnerg3,0,0,0);
		     ret(1000);
			}
		#endif

		#ifdef UKU_3U
		else if(sub_ind==(3+NUMBAT+NUMIST+NUMINV+2)+(NUMAVT!=0))
			{
			tree_up(iEnerg,0,0,0);
		     ret(1000);
			}
		#endif

		else if(sub_ind==(4+NUMBAT+NUMIST+2)+(NUMAVT!=0))
			{
			tree_up(iSpc,0,0,0);
		     ret(1000);
			}

		else if(sub_ind==(5+NUMBAT+NUMIST+NUMINV+2)+(NUMAVT!=0))
			{
			tree_up(iSet_prl,0,0,0);
		     ret(50);
		     parol_init();
			}
		else if(sub_ind==(6+NUMBAT+NUMIST+NUMINV+2)+(NUMAVT!=0))
			{
			tree_up(iLog,0,0,0);
		     ret(1000);
			}
		else if(sub_ind==(7+NUMBAT+NUMIST+NUMINV+2)+(NUMAVT!=0))
			{
			sub_ind=0;
			}
		else if(sub_ind==(8+NUMBAT+NUMIST+NUMINV+2)+(NUMAVT!=0))
			{
			tree_up(iBatLog,0,0,0);
		     ret(1000);
			}
		else if(sub_ind==(9+NUMBAT+NUMIST+NUMINV+2)+(NUMAVT!=0))
			{
			tree_up(iBatLog,0,0,1);
		     ret(1000);
			}
		else if(sub_ind==(10+NUMBAT+NUMIST+NUMINV+2)+(NUMAVT!=0))
			{
			if(but==butE)
		     	{
		     	tree_up(iPrltst,0,0,0);
		     	parol_init();
		     	}
			}
		}
    	
 /*  else if(sub_ind==7+NUMBAT+NUMIST+NUMINV+1)
		{
		if(but==butE)
		     {
			sub_ind=0;
			}
		}	
		
	else if(sub_ind==8+NUMBAT+NUMIST+1)
		{
		if(but==butE)
		     {
			tree_up(iJ_bat,0,0,0);
		     ret(1000);
			}
		}		
     else if(sub_ind==9+NUMBAT+NUMIST+1)
		{
		if(but==butE)
		     {
			tree_up(iJ_bat,0,0,1);
		     ret(1000);
			}
		}*/			
				
	}


else if(ind==iMn_IP55)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,7+NUMBAT_IP55+NUMIST);
		//can1_init(BITRATE62_5K25MHZ);
		}
		
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,7+NUMBAT_IP55+NUMIST);
		//LPC_CAN1->CMR=0x00000022;
		}	


	else if(but==butDR_)
		{
		tree_up(iK_6U,0,0,0);
		}
	else if(but==butDL_)
		{
		tree_up(iSet_6U,0,0,0);
		}
	else if(but==butL)
		{
		//ind=iMn;
		sub_ind=0;
		}
		
	else if(but==butE)
		{
		if(sub_ind==0)
			{
																							
			}
		else if((sub_ind>0)&&(sub_ind<=NUMBAT_IP55))
		    	{
		    	tree_up(iBat_li,0,0,sub_ind-1);
				} 
		else if((sub_ind>NUMBAT_IP55)&&(sub_ind<=(NUMBAT_IP55+NUMIST)))
		    	{
		    	tree_up(iBps,0,0,sub_ind-(1+NUMBAT_IP55));
		    	}
		else if((sub_ind>(NUMBAT_IP55+2))&&(sub_ind<=(NUMBAT_IP55+NUMIST+2)))
		    	{
		    	tree_up(iDcdc,0,0,sub_ind-(3+NUMBAT_IP55));
		    	}
		else if(sub_ind==(1+NUMBAT_IP55+NUMIST+2))
			{
			tree_up(iNetEM,0,0,0);
		      ret(0);//1204//ret(1000);
			}
		
		else if(sub_ind==(2+NUMBAT_IP55+NUMIST+2))
			{
			tree_up(iLoad,0,0,0);
		    ret(1000);
			}
		else if(sub_ind==(3+NUMBAT_IP55+NUMIST+2))
			{
			tree_up(iExtern_TELECORE2015,0,0,0);
		    ret(1000);
			}

		else if(sub_ind==(3+NUMBAT_IP55+NUMIST+2))
			{
			tree_up(iSpc,0,0,0);
		    ret(1000);
			}

		else if(sub_ind==(4+NUMBAT_IP55+NUMIST+2))
			{
			tree_up(iSet_prl,0,0,0);
		    ret(50);
		    parol_init();
			}
		else if(sub_ind==(5+NUMBAT_IP55+NUMIST+2))
			{
			tree_up(iLog,0,0,0);
		    ret(1000);
			}
		else if(sub_ind==(6+NUMBAT_IP55+NUMIST+2))
			{
			sub_ind=0;
			}

		else if(sub_ind==(7+NUMBAT_IP55+NUMIST))
			{
			if(but==butE)
		     	{
		     	tree_up(iPrltst,0,0,0);
		     	parol_init();
		     	}
			}
		}
    }

else if(ind==iBat)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,5);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,5);
		}
	else if((but==butL)||((sub_ind==5)&&(but==butE)))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	else if(but==butD_)
		{
		sub_ind=5;
		}		     
     }

else if(ind==iBat_simple)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
	else if((but==butL)||((sub_ind==4)&&(but==butE)))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	else if(but==butD_)
		{
		sub_ind=4;
		}		     
     }

else if(ind==iBat_li)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,11);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,1);
		}
	else if((but==butL)||((sub_ind==8)&&(but==butE)))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	else if(but==butD_)
		{
		sub_ind=8;
		}		     
     }

else if (ind==iBat_universe)
	{
	if(BAT_TYPE==0)	//Обычная свинцово-кислотная батарея
		{			//TODO следующий блок написан только для батареи без средней точки. Дописать автоизменение если в системе есть контроль СТ
		ret(1000);
		if(but==butD)
			{
			sub_ind++;
			gran_char(&sub_ind,0,4);
			}
		else if(but==butU)
			{
			sub_ind--;
			gran_char(&sub_ind,0,4);
			}
		else if((but==butL)||((sub_ind==4)&&(but==butE)))
		     {
		     tree_down(0,0);
		     ret(0);
		     }
		else if(but==butD_)
			{
			sub_ind=4;
			}		 
		} 
	else if(BAT_TYPE==1)//Батарея	COSLIGHT
		{
		ret(1000);
		if(but==butD)
			{
			sub_ind++;
			gran_char(&sub_ind,0,8);
			if(li_bat._batStat==bsOFF) gran_char(&sub_ind,0,1);
			}
		else if(but==butU)
			{
			sub_ind--;
			gran_char(&sub_ind,0,8);
			if(li_bat._batStat==bsOFF) gran_char(&sub_ind,0,1);
			}
		else if((but==butL)||((sub_ind==8)&&(but==butE)))
		     {
		     tree_down(0,0);
		     ret(0);
		     }
		else if(but==butD_)
			{
			sub_ind=8;
			}	
		}
	else if(BAT_TYPE==2)//Батарея	SACRED SUN
		{
		ret(1000);
		if(but==butD)
			{
			sub_ind++;
			gran_char(&sub_ind,0,5);
			}
		else if(but==butU)
			{
			sub_ind--;
			gran_char(&sub_ind,0,5);
			}
		else if((but==butL)||((sub_ind==5)&&(but==butE)))
		     {
		     tree_down(0,0);
		     ret(0);
		     }
		else if(but==butD_)
			{
			sub_ind=6;
			}
		}
	else if(BAT_TYPE==3)//Батарея	ZTT
		{
		ret(1000);
		if(but==butD)
			{
			sub_ind++;
			gran_char(&sub_ind,0,10);
			}
		else if(but==butU)
			{
			sub_ind--;
			gran_char(&sub_ind,0,10);
			}
		else if((but==butL)||((sub_ind==10)&&(but==butE)))
		     {
		     tree_down(0,0);
		     ret(0);
		     }
		else if(but==butD_)
			{
			sub_ind=6;
			}
		}					     
	}

else if(ind==iInv_tabl)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind1--;
		gran_char(&sub_ind1,0,NUMINV-3);
		}
		
	else if (but==butD)
		{
		sub_ind1++;
		gran_char(&sub_ind1,0,NUMINV-3);
		}
		
	else if(but==butR)
		{
		sub_ind=1;
		}
				
	else if(but==butL)
		{
		sub_ind=0;
		}
	else if(but==butE)
		{
		tree_down(0,0);
		}				
	}

else if(ind==iMakb)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind>7)sub_ind=7;
		//else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
	else if((but==butL)||((sub_ind==simax)&&(but==butE)))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	else if(but==butD_)
		{
		sub_ind=simax;
		}		    
	}

#ifndef _DEBUG_
else if(ind==iBps)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
	else if((but==butE)&&(sub_ind==4))
		{
		//can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
				
	else if(((but==butE)&&(sub_ind==5))||(but))
		{
	     tree_down(0,0);
	     ret(0);
		}	
		
	}	
else if(ind==iDcdc)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
	else if((but==butE)&&(sub_ind==4))
		{
		//can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
				
	else if(((but==butE)&&(sub_ind==5))||(but))
		{
	     tree_down(0,0);
	     ret(0);
		}	
		
	}		
else if(ind==iNet)
	{
	ret(1000);
	if((but==butL)||(but==butE))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}

else if(ind==iNet3)
	{
	ret(1000);
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if((but==butL)||((but==butE)&&(sub_ind==4)))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}
else if(ind==iNetEM)
	{
	ret(1000);
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if((but==butL)||((but==butE)&&(sub_ind==4)))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}
else if(ind==iInv)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
	else if((but==butE)&&(sub_ind==4))
		{
		//can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
				
	else if(((but==butE)&&(sub_ind==5))||(but))
		{
	     tree_down(0,0);
	     ret(0);
		}		
	}

else if(ind==iInv_v2)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
/*	else if((but==butE)&&(sub_ind==4))
		{
		can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
		*/		
	else if(((but==butE)&&(sub_ind==simax))||(but==butL))
		{
	    tree_down(0,0);
	    ret(0);
		}		
	}

else if(ind==iByps)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
/*	else if((but==butE)&&(sub_ind==4))
		{
		can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
		*/		
	else if(((but==butE)&&(sub_ind==simax))||(but==butL))
		{
	    tree_down(0,0);
	    ret(0);
		}		
	}

else if(ind==iLoad)
	{
	ret(1000);
	if((but==butL)||(but==butE))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}	

else if(ind==iExtern)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,8);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,8);		
		}

	else if((but==butE)&&(sub_ind==8))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}
else if(ind==iExtern_KONTUR)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,8);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,8);		
		}

	else if((but==butE)&&(sub_ind==8))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}			
else if(ind==iExtern_3U)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,2+NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+NUMSK);		
		}

	else if((but==butE)&&(sub_ind==2+NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}	

else if(ind==iExtern_6U)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMDT+NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMDT+NUMSK);		
		}

	else if((but==butE)&&(sub_ind==NUMDT+NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}	

else if(ind==iExtern_220)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMDT+NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMDT+NUMSK);		
		}

	else if((but==butE)&&(sub_ind==NUMDT+NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}	

else if(ind==iExtern_GLONASS)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,2+NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+NUMSK);		
		}

	else if((but==butE)&&(sub_ind==2+NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}	

else if(ind==iExtern_TELECORE2015)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);		
		}

	else if((but==butE)&&(sub_ind==3))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}

else if(ind==iVent)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,1,2);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,1,2);		
		}

	else if (sub_ind==1)
		{
          if((but==butR)||(but==butR_))
               {
               //pos_vent++;
               }
          else if((but==butL)||(but==butL_))
               {
               //pos_vent--;
               }

		//gran(&pos_vent,1,11);
          ////lc640_write_int(EE_POS_VENT,pos_vent);		
		}
		
	else if((but==butE)&&(sub_ind==2))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}
else if(ind==iAvt)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMAVT);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMAVT);		
		}

	else if((but==butE)&&(sub_ind==NUMAVT))
		{
	     tree_down(0,0);
	     ret(0);
		}
	}
else if(ind==iEnerg)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);		
		}

	else if((but==butE)&&(sub_ind==4))
		{
	     tree_down(0,0);
	     ret(0);
		}
     }

else if(ind==iEnerg3)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,8);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,8);		
		}

	else if((but==butE)&&(sub_ind==8))
		{
	     tree_down(0,0);
	     ret(0);
		}
     }

else if((ind==iPrl_bat_in_out)||(ind==iSet_prl)||(ind==iK_prl)
	||(ind==iSpc_prl_vz)||(ind==iSpc_prl_ke)||(ind==iAusw_prl)
	||(ind==iPrltst))
	{
	ret(50);
	if(but==butR)
		{
		sub_ind++;
		gran_ring_char(&sub_ind,0,2);
		}
	else if(but==butL)
		{
		sub_ind--;
		gran_ring_char(&sub_ind,0,2);
		}	
	else if(but==butU)
		{
		parol[sub_ind]++;
		gran_ring_char(&parol[sub_ind],0,9);
		}	
	else if(but==butD)
		{
		parol[sub_ind]--;
		gran_ring_char(&parol[sub_ind],0,9);
		}	
	else if(but==butE)
		{
		unsigned short tempU;
		tempU=parol[2]+(parol[1]*10U)+(parol[0]*100U);
		
		if(ind==iPrl_bat_in_out)
		     {
		     if(BAT_IS_ON[sub_ind1]!=bisON)
		          {
		          if(tempU==PAROL_BAT_IN)
		               {
					//tree_up(iPrl_bat_in_sel,0,0,sub_ind1);
					
		               ////lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisON);
					////lc640_write_int(EE_BAT_TYPE,0);
		               ////lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],LPC_RTC->DOM);
		               ////lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],LPC_RTC->MONTH);
		               ////lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],LPC_RTC->YEAR);
		               ////lc640_write_int(ADR_EE_BAT_C_REAL[sub_ind1],0x5555);
		               ////lc640_write_int(ADR_EE_BAT_RESURS[sub_ind1],0);
					////lc640_write_int(ADR_EE_BAT_ZAR_CNT[sub_ind1],0);
		               
		               ////lc640_write(KE_PTR,0);
					////lc640_write(VZ_PTR,0);
					////lc640_write(WRK_PTR,0);
					////lc640_write(KE_CNT,0);
					////lc640_write(VZ_CNT,0);
					////lc640_write(WRK_CNT,0);
					////lc640_write(BAT_AVAR_CNT,0);
					////lc640_write(BAT_AVAR_PTR,0);					
		               
                         tree_down(0,0);
                         ret(0); 
		               }
		          else
		               {
		               tree_down(0,0);
	    	               show_mess("                    ",
	          				"       Пароль       ",
	          				"     неверный!!!    ",
	          				"                    ",1000);
     	               }
		          }      
               else		          
		          {
		          if(tempU==PAROL_BAT_OUT)
		               {
		               ////lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisOFF);
		               ////lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],LPC_RTC->DOM);
		               ////lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],LPC_RTC->MONTH);
		               ////lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],LPC_RTC->YEAR);

		               tree_down(0,0);
		               ret(0);
		               
		               }
	               else
		               {
		               tree_down(0,0);
	    	               show_mess("                    ",
	          				"       Пароль       ",
	          				"     неверный!!!    ",
	          				"                    ",1000);
		               }		               
		          }     
               }
		
		else if(ind==iSet_prl)
			{
	     	if(tempU==PAROL_SET) 
				{
				tree_down(0,0);

				tree_up(iSet_IP55,0,0,0);

				ret(1000);
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			}
		else	if(ind==iK_prl)
			{
	     	if(tempU==PAROL_KALIBR) 
				{
				tree_down(0,0);
				tree_up(iK_IP55,0,0,0);
				
				show_mess(	"Включите авт-ты СЕТЬ",
 							"  БАТАРЕЯ,НАГРУЗКА  ",
 							"   Установите ток   ",
 							"   нагрузки 4-10А   ",3000);
				
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			} 
	
		else	if(ind==iAusw_prl)
			{
	     	if(tempU==PAROL_AUSW) 
				{
				tree_down(0,0);
				tree_up(iAusw_set,1,0,0);
				default_temp=10;
				ret(0);
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			} 	
			
		else	if(ind==iSet_st_prl)
			{
	     	if(tempU==PAROL_DEFAULT) 
				{
	//			ind=iDefault;
				sub_ind=1;
				index_set=0;
				default_temp=10;
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			} 
						
		else if(ind==iPrltst)
			{
			if(tempU==PAROL_TST) 
				{
				tree_down(0,0);
				#ifdef UKU_GLONASS
				tree_up(iTst_GLONASS,0,0,0);
				#endif
				#ifdef UKU_3U
				tree_up(iTst_3U,0,0,0);
				#endif
				#ifdef UKU_RSTKM
				tree_up(iTst_RSTKM,0,0,0);
				#endif
				#ifdef UKU_KONTUR
				tree_up(iTst_KONTUR,0,0,0);
				#endif
				#ifdef UKU
				tree_up(iTst,0,0,0);
				#endif
				#ifdef UKU_6U
				tree_up(iTst_6U,0,0,0);
				#endif
				#ifdef UKU_220
				if(AUSW_MAIN==22035)
					{
					tree_up(iTst_220_380,0,0,0);
					}
				else
				tree_up(iTst_220,0,0,0);
				#endif
				#ifdef UKU_220_V2
				tree_up(iTst_220,0,0,0);
				#endif
				#ifdef UKU_220_IPS_TERMOKOMPENSAT
				tree_up(iTst_220_IPS_TERMOKOMPENSAT,0,0,0);
				#endif
				#ifdef UKU_TELECORE2015
				tree_up(iTst_TELECORE2015,0,0,0);
				#endif				
				tst_state[0]=tstOFF;
				tst_state[1]=tstOFF;
				tst_state[2]=tstOFF;
				tst_state[3]=tstOFF;
				tst_state[4]=tstOFF;
				tst_state[5]=tstOFF;
				tst_state[6]=tstOFF;
				tst_state[7]=tstOFF;
				tst_state[9]=tstOFF;
				tst_state[10]=(enum_tst_state)0;
				ret(10000);				


				}
	  		else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			}
		else if(ind==iSpc_prl_ke)
			{
			if(tempU==PAROL_KE) 
				{
				char temp;
				temp=sub_ind1;
				tree_down(0,0);
				tree_up(iKe,0,0,temp);
				ret(1000);
				}
	  		else 
				{	
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}
			}
		else if(ind==iSpc_prl_vz)
			{
			if(tempU==PAROL_VZ) 
				{
				tree_down(0,0);
				tree_up(iVz,0,0,0);
				ret(1000);
				}
	  		else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       Пароль       ",
	          			"     неверный!!!    ",
	          			"                    ",1000);
				}     	          
			}
		}
	}

else if(ind==iSet_bat_sel)
	{
	ret(1000);
	if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if(but==butE)
		{
		if(sub_ind==4)
			{
			tree_down(0,0);
          	ret(0);
			}
		}
	else if(but==butE_)
		{
		if((sub_ind>=0)&&(sub_ind<=3))
			{
			BAT_TYPE=sub_ind;
			////lc640_write_int(EE_BAT_TYPE,BAT_TYPE);
			}
		}
	else if(but==butLR)
		{

			////lc640_write_int(EE_BAT_TYPE,65535);

		}
	}

else if(ind==iPrl_bat_in_sel)
	{
	ret(1000);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,1);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,1);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			////lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisON);
		     ////lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],LPC_RTC->DOM);
		     ////lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],LPC_RTC->MONTH);
		     ////lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],LPC_RTC->YEAR);
		     ////lc640_write_int(ADR_EE_BAT_C_REAL[sub_ind1],0x5555);
		     ////lc640_write_int(ADR_EE_BAT_RESURS[sub_ind1],0);
			////lc640_write_int(ADR_EE_BAT_ZAR_CNT[sub_ind1],0);
			////lc640_write_int(ADR_EE_BAT_TYPE[sub_ind1],0);
		               
		     ////lc640_write(KE_PTR,0);
			////lc640_write(VZ_PTR,0);
			////lc640_write(WRK_PTR,0);
			////lc640_write(KE_CNT,0);
			////lc640_write(VZ_CNT,0);
			////lc640_write(WRK_CNT,0);
			////lc640_write(BAT_AVAR_CNT,0);
			////lc640_write(BAT_AVAR_PTR,0);					
		               
               tree_down(-1,0);
               ret(0);
 
			}
		else if (sub_ind==1)
			{
			////lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisON);
		     ////lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],LPC_RTC->DOM);
		     ////lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],LPC_RTC->MONTH);
		     ////lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],LPC_RTC->YEAR);
		     ////lc640_write_int(ADR_EE_BAT_C_REAL[sub_ind1],0x5555);
		     ////lc640_write_int(ADR_EE_BAT_RESURS[sub_ind1],0);
			////lc640_write_int(ADR_EE_BAT_ZAR_CNT[sub_ind1],0);
			////lc640_write_int(ADR_EE_BAT_TYPE[sub_ind1],1);
		               
		     ////lc640_write(KE_PTR,0);
			////lc640_write(VZ_PTR,0);
			////lc640_write(WRK_PTR,0);
			////lc640_write(KE_CNT,0);
			////lc640_write(VZ_CNT,0);
			////lc640_write(WRK_CNT,0);
			////lc640_write(BAT_AVAR_CNT,0);
			////lc640_write(BAT_AVAR_PTR,0);					
		               
               tree_down(-1,0);
               ret(0);
	    	     show_mess("    Не забудьте     ",
	          		"  настроить канал   ",
	          		"       связи        ",
	          		"     с батареей.    ",3000);			 
			}
		}
	}

else if(ind==iSpc)
	{
	ret(1000);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{   
               tree_up(iSpc_prl_vz,0,0,0);
			parol_init();
			}
		else if(sub_ind==1)
			{
            tree_up(iAvz,0,0,0);
            parol_init();
			}			
		else if((sub_ind==2)||(sub_ind==3))
			{
            tree_up(iSpc_prl_ke,0,0,sub_ind-2);
            parol_init();
			} 
		else if(sub_ind==4)
			{
			tree_down(0,0);
			ret(0);
			}	
		}
	else if(but==butL)
		{
		tree_down(0,0);
		ret(0);
		}			
	}
else if(ind==iSpc_termocompensat)
	{
	ret(1000);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{   
            tree_up(iSpc_prl_vz,0,0,0);
			parol_init();
			}
		else if(sub_ind==1)
			{
            tree_up(iAvz,0,0,0);
            parol_init();
			}			
		else if(sub_ind==2)
			{
            tree_up(iSpc_prl_ke,0,0,0);
            parol_init();
			} 
		else if(sub_ind==3)
			{
			tree_down(0,0);
			ret(0);
			}	
		}
	else if(but==butL)
		{
		tree_down(0,0);
		ret(0);
		}
	}




else if(ind==iSet)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==7)index_set=6;
		if(sub_ind==8)sub_ind=9;
		if(sub_ind==11)index_set=10;
		if(sub_ind==12)sub_ind=13;
          if(sub_ind==32)
               {
               index_set=31;
               }
          if(sub_ind==34)
               {
               sub_ind=35;
               //index_set=31;
               }
		
		gran_char(&sub_ind,0,37);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==8)sub_ind=7;
		if(sub_ind==12)sub_ind=9;
          if(sub_ind==33)
               {
               sub_ind=32;
		     //index_set=29;
               }
		gran_char(&sub_ind,0,37);
		}
	else if(but==butD_)
		{
		sub_ind=35;
		}
		
	else if(sub_ind==0)
	     {
	     if(but==butE)
	          {
			#ifdef UKU_GLONASS
	          tree_up(iDef_GLONASS,0,0,0);
			#endif
			#ifdef UKU_3U
	          tree_up(iDef_3U,0,0,0);
			#endif
			#ifdef UKU
	          tree_up(iDef,0,0,0);
			#endif
			#ifdef UKU_6U
	          tree_up(iDef_6U,0,0,0);
			#endif
	          ret(1000);
	          default_temp=10;
	          }


	     }	
	
     else if(sub_ind==1)
		{
		if(but==butE)
		     {
		     tree_up(iSet_T,0,0,0);
		     ret(1000);
		     phase=0;
		     }
		}	
					 
     else if(sub_ind==2)
		{
		if(but==butE)
		     {
			#ifdef UKU_3U
		     tree_up(iStr_3U,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif
			#ifdef UKU_GLONASS
		     tree_up(iStr_GLONASS,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif
			#ifdef UKU_RSTKM
		     tree_up(iStr_RSTKM,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif
			#ifdef UKU_KONTUR
		     tree_up(iStr_KONTUR,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif
			#ifdef UKU_6U
		     tree_up(iStr_6U,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif

			#ifdef UKU
		     tree_up(iStr,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif



		     }
		}	
	
	else if(sub_ind==3)
	     {
		if(but==butE)
		     {		
			tree_up(iKlimat,0,0,0);
			}
	     }

	else if(sub_ind==5)
	     {
	     if(but==butR)MNEMO_TIME++;
	     else if(but==butR_)MNEMO_TIME+=10;
	     else if(but==butL)MNEMO_TIME--;
	     else if(but==butL_)MNEMO_TIME-=10;

	     //if(((MNEMO_TIME<5)||(MNEMO_TIME>60))&&(MNEMO_ON!=mnOFF))//lc640_write_int(EE_MNEMO_ON,mnOFF);	
	     //if(((MNEMO_TIME>=5)&&(MNEMO_TIME<=60))&&(MNEMO_ON!=mnON))//lc640_write_int(EE_MNEMO_ON,mnON);
	     gran((signed short*)&MNEMO_TIME,4,61);
	     ////lc640_write_int(EE_MNEMO_TIME,MNEMO_TIME);
	     speed=1;
	     }
				     		
	else if(sub_ind==6)
	     {
		if(ZV_ON)ZV_ON=0;
		else ZV_ON=1;
	     ////lc640_write_int(EE_ZV_ON,ZV_ON);
	     speed=1;
	     }	
	
	else if(sub_ind==7)
	     {
		if(AV_OFF_AVT)AV_OFF_AVT=0;
		else AV_OFF_AVT=1;
	     ////lc640_write_int(EE_AV_OFF_AVT,AV_OFF_AVT);
	     speed=1;
	     }	

	else if(sub_ind==9)
	     {
	     if(but==butE)
	          {
	          tree_up(iApv,0,0,0);
	          ret(1000);
	          }
	     }	

	else if(sub_ind==10)
	     {
		if(PAR)PAR=0;
		else PAR=1;
	     ////lc640_write_int(EE_PAR,PAR);
	     speed=1;
	     }

	else if(sub_ind==11)
	     {
	     if(but==butR)TBAT++;
	     else if(but==butR_)TBAT+=10;
	     else if(but==butL)TBAT--;
	     else if(but==butL_)TBAT-=10;
	     gran(&TBAT,5,60);
	     ////lc640_write_int(EE_TBAT,TBAT);
	     speed=1;
	     }	
	                    	     	
	else if(sub_ind==13)
	     {
	     if(but==butR)UMAX++;
	     else if(but==butR_)UMAX+=10;
	     else if(but==butL)UMAX--;
	     else if(but==butL_)UMAX-=10;
		#ifdef UKU206_220
		gran(&UMAX,2000,3000);
		#else
	     gran(&UMAX,10,1000);
		#endif
	     ////lc640_write_int(EE_UMAX,UMAX);
	     speed=1;
	     }
	else if(sub_ind==14)
	     {
/*	     if(but==butR)DU++;
	     else if(but==butR_)DU+=10;
	     else if(but==butL)DU--;
	     else if(but==butL_)DU-=10;
	     gran(&DU,10,1000);*/
	     if(but==butL)DU++;
	     else if(but==butL_)DU+=10;
	     else if(but==butR)DU--;
	     else if(but==butR_)DU-=10;
	     gran(&DU,50,UB20-250);
	     ////lc640_write_int(EE_DU,DU);
	     speed=1;
	     }	     
	else if(sub_ind==15)
	     {
	     if(but==butR)UB0++;
	     else if(but==butR_)UB0+=10;
	     else if(but==butL)UB0--;
	     else if(but==butL_)UB0-=10;
		gran(&UB0,400,800);
          //lc640_write_int(EE_UB0,UB0);
	     speed=1;
	     }
	     
	else if(sub_ind==16)
	     {
	     if(but==butR)UB20++;
	     else if(but==butR_)UB20+=10;
	     else if(but==butL)UB20--;
	     else if(but==butL_)UB20-=10;
		gran(&UB20,400,800);
	     ////lc640_write_int(EE_UB20,UB20);
	     speed=1;
	     }	

	else if(sub_ind==17)
	     {
	     if(but==butR)USIGN++;
	     else if(but==butR_)USIGN+=10;
	     else if(but==butL)USIGN--;
	     else if(but==butL_)USIGN-=10;
		gran(&USIGN,40,100);
	     ////lc640_write_int(EE_USIGN,USIGN);
	     speed=1;
	     }	
	else if(sub_ind==18)
	     {
	     if(but==butR)UMN++;
	     else if(but==butR_)UMN+=10;
	     else if(but==butL)UMN--;
	     else if(but==butL_)UMN-=10;
	     gran(&UMN,1,220);
	     ////lc640_write_int(EE_UMN,UMN);
	     speed=1;
	     }	

	else if(sub_ind==19)
	     {
	     if(but==butR)U0B++;
	     else if(but==butR_)U0B+=10;
	     else if(but==butL)U0B--;
	     else if(but==butL_)U0B-=10;
		gran(&U0B,400,800);
	     ////lc640_write_int(EE_U0B,U0B);
	     speed=1;
	     }	
	     
	else if(sub_ind==20)
	     {
	     if(but==butR)IKB++;
	     else if(but==butR_)IKB+=10;
	     else if(but==butL)IKB--;
	     else if(but==butL_)IKB-=10;
	     gran(&IKB,1,500);
	     ////lc640_write_int(EE_IKB,IKB);
	     speed=1;
	     }		
            
	else if(sub_ind==21)
	     {
	     if(but==butR)IZMAX++;
	     else if(but==butR_)IZMAX+=10;
	     else if(but==butL)IZMAX--;
	     else if(but==butL_)IZMAX-=10;
		gran(&IZMAX,10,1000);
	     ////lc640_write_int(EE_IZMAX,IZMAX);
	     speed=1;
	     }   

	else if(sub_ind==22)
	     {
	     if(but==butR)IMAX++;
	     else if(but==butR_)IMAX+=10;
	     else if(but==butL)IMAX--;
	     else if(but==butL_)IMAX-=10;
		gran(&IMAX,10,1000);
	     ////lc640_write_int(EE_IMAX,IMAX);
	     speed=1;
	     }		
	     
	else if(sub_ind==23)
	     {
	     if(but==butR)IMIN++;
	     else if(but==butR_)IMIN+=10;
	     else if(but==butL)IMIN--;
	     else if(but==butL_)IMIN-=10;
	     gran(&IMIN,40,IMAX-10);
	     ////lc640_write_int(EE_IMIN,IMIN);
	     speed=1;
	     }
	
	else if(sub_ind==24)
	     {
	     if ((but==butR)||(but==butR_))UVZ+=1;
		if ((but==butL)||(but==butL_))UVZ-=1;
		gran(&UVZ,UB20,UMAX); 	          
		////lc640_write_int(EE_UVZ,UVZ);
	     speed=1;
	     }
	     
	else if(sub_ind==25)
		{
		if ((but==butR)||(but==butR_))TZAS++;
		if ((but==butL)||(but==butL_))TZAS--;
		gran(&TZAS,0,3);
		////lc640_write_int(EE_TZAS,TZAS);
		speed=1; 
		}	
			       	        
	else if(sub_ind==26)
	     {
	     if(but==butR)TMAX++;
	     else if(but==butR_)TMAX+=2;
	     else if(but==butL)TMAX--;
	     else if(but==butL_)TMAX-=2;
	     gran(&TMAX,10,100);
	     ////lc640_write_int(EE_TMAX,TMAX);
	     speed=1;
	     }	
	
	else if(sub_ind==27)
	     {
	     if(but==butR)TSIGN++;
	     else if(but==butR_)TSIGN+=2;
	     else if(but==butL)TSIGN--;
	     else if(but==butL_)TSIGN-=2;
	     gran(&TSIGN,10,100);
	     ////lc640_write_int(EE_TSIGN,TSIGN);
	     speed=1;
	     }	     
	else if(sub_ind==28)
	     {
	     if(but==butR)TBATMAX++;
	     else if(but==butR_)TBATMAX+=2;
	     else if(but==butL)TBATMAX--;
	     else if(but==butL_)TBATMAX-=2;
	     gran(&TBATMAX,10,100);
	     ////lc640_write_int(EE_TBATMAX,TBATMAX);
	     speed=1;
	     }	
	
	else if(sub_ind==29)
	     {
	     if(but==butR)TBATSIGN++;
	     else if(but==butR_)TBATSIGN+=2;
	     else if(but==butL)TBATSIGN--;
	     else if(but==butL_)TBATSIGN-=2;
	     gran(&TBATSIGN,10,100);
	     ////lc640_write_int(EE_TBATSIGN,TBATSIGN);
	     speed=1;
	     }	
     	     	     		     	     
    else if(sub_ind==30)
		{
		if(but==butE)
		     {
		     tree_up(iExt_set,0,0,0);
		     ret(1000);
		     }
		}

    else if(sub_ind==31)
		{
		if(but==butE)
		     {
		     tree_up(iLan_set,0,0,0);
		     ret(1000);
		     }
		}
	else if(sub_ind==32)
	     {
	     if(but==butR)POWER_CNT_ADRESS++;
	     else if(but==butR_)POWER_CNT_ADRESS+=10;
	     else if(but==butL)POWER_CNT_ADRESS--;
	     else if(but==butL_)POWER_CNT_ADRESS-=10;
	     gran(&POWER_CNT_ADRESS,0,10000);
	     ////lc640_write_int(EE_POWER_CNT_ADRESS,POWER_CNT_ADRESS);
	     speed=1;
	     } 
	else if(sub_ind==33)
	     {
	     if(but==butR)UBM_AV++;
	     else if(but==butR_)UBM_AV++;
	     else if(but==butL)UBM_AV--;
	     else if(but==butL_)UBM_AV--;
	     gran(&UBM_AV,0,50);
	     ////lc640_write_int(EE_UBM_AV,UBM_AV);
	     speed=1;
	     }

	else if(sub_ind==34)
	     {
	     if(but==butR)AUSW_MAIN_NUMBER++;
	     else if(but==butR_)AUSW_MAIN_NUMBER+=20;
	     else if(but==butL)AUSW_MAIN_NUMBER--;
	     else if(but==butL_)AUSW_MAIN_NUMBER-=20;
		else if(but==butEL_)AUSW_MAIN_NUMBER=15000;
		if(AUSW_MAIN_NUMBER<13000)AUSW_MAIN_NUMBER=100000;
		if(AUSW_MAIN_NUMBER>100000)AUSW_MAIN_NUMBER=13000;
	     ////lc640_write_int(EE_AUSW_MAIN_NUMBER,(short)(AUSW_MAIN_NUMBER&0x0000ffffUL));
		////lc640_write_int(EE_AUSW_MAIN_NUMBER+2,(short)((AUSW_MAIN_NUMBER&0xffff0000UL)>>16UL));
	     speed=1;
	     }                    		
 
     else if((sub_ind==36) || (sub_ind==4))
		{
		if(but==butE)
		     {
		     tree_down(0,0);
		     ret(0);
		     }
		}
				
	else if(sub_ind==37)
		{
		if(but==butE)
		     {		
			tree_up(iK_prl,0,0,0);
			parol_init();
			ret(50);
			}						
		}
		/*			
	else if(sub_ind==37)
		{
		if(but==butE)
		     {
		     tree_up(iPrltst,0,0,0);
		     parol_init();
		     }
		}*/	
     }

else if(ind==iSet_RSTKM)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==7)index_set=6;
		if(sub_ind==8)sub_ind=9;
		if(sub_ind==11)index_set=10;
		if(sub_ind==12)sub_ind=13;
          if(sub_ind==33)
               {
               index_set=32;
               }
          if(sub_ind==34)
               {
               sub_ind=35;
               //index_set=31;
               }
		
		gran_char(&sub_ind,0,37);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==8)sub_ind=7;
		if(sub_ind==12)sub_ind=9;
          if(sub_ind==34)
               {
               sub_ind=33;
		     //index_set=29;
               }
		gran_char(&sub_ind,0,37);
		}
	else if(but==butD_)
		{
		sub_ind=36;
		}
		
	else if(sub_ind==0)
	     {
	     if(but==butE)
	          {
	          tree_up(iDef_RSTKM,0,0,0);

	          ret(1000);
	          default_temp=10;
	          }
	     }	
	
     else if(sub_ind==1)
		{
		if(but==butE)
		     {
		     tree_up(iSet_T,0,0,0);
		     ret(1000);
		     phase=0;
		     }
		}	
					 
     else if(sub_ind==2)
		{
		if(but==butE)
		     {
		     tree_up(iStr_RSTKM,0,0,0);
		     ret(1000);
		     index_set=0;
		     }
		}	
	
	else if(sub_ind==3)
	     {
		if(but==butE)
		     {		
			tree_up(iKlimat,0,0,0);
			}
	     }

	else if(sub_ind==5)
	     {
	     if(but==butR)MNEMO_TIME++;
	     else if(but==butR_)MNEMO_TIME+=10;
	     else if(but==butL)MNEMO_TIME--;
	     else if(but==butL_)MNEMO_TIME-=10;

	     if(((MNEMO_TIME<5)||(MNEMO_TIME>60))&&(MNEMO_ON!=mnOFF))//lc640_write_int(EE_MNEMO_ON,mnOFF);	
	     if(((MNEMO_TIME>=5)&&(MNEMO_TIME<=60))&&(MNEMO_ON!=mnON))//lc640_write_int(EE_MNEMO_ON,mnON);
	     gran((signed short*)&MNEMO_TIME,4,61);
	     ////lc640_write_int(EE_MNEMO_TIME,MNEMO_TIME);
	     speed=1;
	     }
				     		
	else if(sub_ind==6)
	     {
		if(ZV_ON)ZV_ON=0;
		else ZV_ON=1;
	     ////lc640_write_int(EE_ZV_ON,ZV_ON);
	     speed=1;
	     }	
	
	else if(sub_ind==7)
	     {
		if(AV_OFF_AVT)AV_OFF_AVT=0;
		else AV_OFF_AVT=1;
	     ////lc640_write_int(EE_AV_OFF_AVT,AV_OFF_AVT);
	     speed=1;
	     }	

	else if(sub_ind==9)
	     {
	     if(but==butE)
	          {
	          tree_up(iApv,0,0,0);
	          ret(1000);
	          }
	     }	

	else if(sub_ind==10)
	     {
		if(PAR)PAR=0;
		else PAR=1;
	     ////lc640_write_int(EE_PAR,PAR);
	     speed=1;
	     }

	else if(sub_ind==11)
	     {
	     if(but==butR)TBAT++;
	     else if(but==butR_)TBAT+=10;
	     else if(but==butL)TBAT--;
	     else if(but==butL_)TBAT-=10;
	     gran(&TBAT,5,60);
	     ////lc640_write_int(EE_TBAT,TBAT);
	     speed=1;
	     }	
	                    	     	
	else if(sub_ind==13)
	     {
	     if(but==butR)UMAX++;
	     else if(but==butR_)UMAX+=10;
	     else if(but==butL)UMAX--;
	     else if(but==butL_)UMAX-=10;
		#ifdef UKU206_220
		gran(&UMAX,2000,3000);
		#else
	     gran(&UMAX,10,1000);
		#endif
	     ////lc640_write_int(EE_UMAX,UMAX);
	     speed=1;
	     }
	else if(sub_ind==14)
	     {
/*	     if(but==butR)DU++;
	     else if(but==butR_)DU+=10;
	     else if(but==butL)DU--;
	     else if(but==butL_)DU-=10;
	     gran(&DU,10,1000);*/
	     if(but==butL)DU++;
	     else if(but==butL_)DU+=10;
	     else if(but==butR)DU--;
	     else if(but==butR_)DU-=10;
	     gran(&DU,50,UB20-250);
	     ////lc640_write_int(EE_DU,DU);
	     speed=1;
	     }	     
	else if(sub_ind==15)
	     {
	     if(but==butR)UB0++;
	     else if(but==butR_)UB0+=10;
	     else if(but==butL)UB0--;
	     else if(but==butL_)UB0-=10;
		gran(&UB0,400,800);
          ////lc640_write_int(EE_UB0,UB0);
	     speed=1;
	     }
	     
	else if(sub_ind==16)
	     {
	     if(but==butR)UB20++;
	     else if(but==butR_)UB20+=10;
	     else if(but==butL)UB20--;
	     else if(but==butL_)UB20-=10;
		gran(&UB20,400,800);
	     ////lc640_write_int(EE_UB20,UB20);
	     speed=1;
	     }	

	else if(sub_ind==17)
	     {
	     if(but==butR)USIGN++;
	     else if(but==butR_)USIGN+=10;
	     else if(but==butL)USIGN--;
	     else if(but==butL_)USIGN-=10;
		gran(&USIGN,40,100);
	     ////lc640_write_int(EE_USIGN,USIGN);
	     speed=1;
	     }	
	else if(sub_ind==18)
	     {
	     if(but==butR)UMN++;
	     else if(but==butR_)UMN+=10;
	     else if(but==butL)UMN--;
	     else if(but==butL_)UMN-=10;
	     gran(&UMN,1,220);
	     ////lc640_write_int(EE_UMN,UMN);
	     speed=1;
	     }	

	else if(sub_ind==19)
	     {
	     if(but==butR)U0B++;
	     else if(but==butR_)U0B+=10;
	     else if(but==butL)U0B--;
	     else if(but==butL_)U0B-=10;
		gran(&U0B,400,800);
	     ////lc640_write_int(EE_U0B,U0B);
	     speed=1;
	     }	
	     
	else if(sub_ind==20)
	     {
	     if(but==butR)IKB++;
	     else if(but==butR_)IKB+=10;
	     else if(but==butL)IKB--;
	     else if(but==butL_)IKB-=10;
	     gran(&IKB,1,500);
	     ////lc640_write_int(EE_IKB,IKB);
	     speed=1;
	     }		
            
	else if(sub_ind==21)
	     {
	     if(but==butR)IZMAX++;
	     else if(but==butR_)IZMAX+=10;
	     else if(but==butL)IZMAX--;
	     else if(but==butL_)IZMAX-=10;
		gran(&IZMAX,10,1000);
	     //lc640_write_int(EE_IZMAX,IZMAX);
	     speed=1;
	     }   

	else if(sub_ind==22)
	     {
	     if(but==butR)IMAX++;
	     else if(but==butR_)IMAX+=10;
	     else if(but==butL)IMAX--;
	     else if(but==butL_)IMAX-=10;
		gran(&IMAX,10,1000);
	     //lc640_write_int(EE_IMAX,IMAX);
	     speed=1;
	     }		
	     
	else if(sub_ind==23)
	     {
	     if(but==butR)IMIN++;
	     else if(but==butR_)IMIN+=10;
	     else if(but==butL)IMIN--;
	     else if(but==butL_)IMIN-=10;
	     gran(&IMIN,40,IMAX-10);
	     //lc640_write_int(EE_IMIN,IMIN);
	     speed=1;
	     }
	
	else if(sub_ind==24)
	     {
	     if ((but==butR)||(but==butR_))UVZ+=1;
		if ((but==butL)||(but==butL_))UVZ-=1;
		gran(&UVZ,UB20,UMAX); 	          
		//lc640_write_int(EE_UVZ,UVZ);
	     speed=1;
	     }
	     
	else if(sub_ind==25)
		{
		if ((but==butR)||(but==butR_))TZAS++;
		if ((but==butL)||(but==butL_))TZAS--;
		gran(&TZAS,0,3);
		//lc640_write_int(EE_TZAS,TZAS);
		speed=1; 
		}	
			       	        
	else if(sub_ind==26)
	     {
	     if(but==butR)TMAX++;
	     else if(but==butR_)TMAX+=2;
	     else if(but==butL)TMAX--;
	     else if(but==butL_)TMAX-=2;
	     gran(&TMAX,10,100);
	     //lc640_write_int(EE_TMAX,TMAX);
	     speed=1;
	     }	
	
	else if(sub_ind==27)
	     {
	     if(but==butR)TSIGN++;
	     else if(but==butR_)TSIGN+=2;
	     else if(but==butL)TSIGN--;
	     else if(but==butL_)TSIGN-=2;
	     gran(&TSIGN,10,100);
	     //lc640_write_int(EE_TSIGN,TSIGN);
	     speed=1;
	     }	     
	else if(sub_ind==28)
	     {
	     if(but==butR)TBATMAX++;
	     else if(but==butR_)TBATMAX+=2;
	     else if(but==butL)TBATMAX--;
	     else if(but==butL_)TBATMAX-=2;
	     gran(&TBATMAX,10,100);
	     //lc640_write_int(EE_TBATMAX,TBATMAX);
	     speed=1;
	     }	
	
	else if(sub_ind==29)
	     {
	     if(but==butR)TBATSIGN++;
	     else if(but==butR_)TBATSIGN+=2;
	     else if(but==butL)TBATSIGN--;
	     else if(but==butL_)TBATSIGN-=2;
	     gran(&TBATSIGN,10,100);
	     //lc640_write_int(EE_TBATSIGN,TBATSIGN);
	     speed=1;
	     }	
     	     	     		     	     
    else if(sub_ind==30)
		{
		if(but==butE)
		     {
		     tree_up(iExt_set,0,0,0);
		     ret(1000);
		     }
		}

    else if(sub_ind==31)
		{
		if(but==butE)
		     {
		     tree_up(iLan_set,0,0,0);
		     ret(1000);
		     }
		}
	else if(sub_ind==32)
	     {
	     if(but==butR)POWER_CNT_ADRESS++;
	     else if(but==butR_)POWER_CNT_ADRESS+=10;
	     else if(but==butL)POWER_CNT_ADRESS--;
	     else if(but==butL_)POWER_CNT_ADRESS-=10;
	     gran(&POWER_CNT_ADRESS,0,10000);
	     //lc640_write_int(EE_POWER_CNT_ADRESS,POWER_CNT_ADRESS);
	     speed=1;
	     } 
	else if(sub_ind==33)
	     {
	     if(but==butR)UBM_AV++;
	     else if(but==butR_)UBM_AV++;
	     else if(but==butL)UBM_AV--;
	     else if(but==butL_)UBM_AV--;
	     gran(&UBM_AV,0,50);
	     //lc640_write_int(EE_UBM_AV,UBM_AV);
	     speed=1;
	     }

	else if(sub_ind==35)
	     {
	     if(but==butR)AUSW_MAIN_NUMBER++;
	     else if(but==butR_)AUSW_MAIN_NUMBER+=20;
	     else if(but==butL)AUSW_MAIN_NUMBER--;
	     else if(but==butL_)AUSW_MAIN_NUMBER-=20;
		else if(but==butEL_)AUSW_MAIN_NUMBER=15000;
		if(AUSW_MAIN_NUMBER<13000)AUSW_MAIN_NUMBER=100000;
		if(AUSW_MAIN_NUMBER>100000)AUSW_MAIN_NUMBER=13000;
	     //lc640_write_int(EE_AUSW_MAIN_NUMBER,(short)(AUSW_MAIN_NUMBER&0x0000ffffUL));
		//lc640_write_int(EE_AUSW_MAIN_NUMBER+2,(short)((AUSW_MAIN_NUMBER&0xffff0000UL)>>16UL));
	     speed=1;
	     }                    		
 
     else if((sub_ind==36) || (sub_ind==4))
		{
		if(but==butE)
		     {
		     tree_down(0,0);
		     ret(0);
		     }
		}
				
	else if(sub_ind==37)
		{
		if(but==butE)
		     {		
			tree_up(iK_prl,0,0,0);
			parol_init();
			ret(50);
			}						
		}
     }

else if(ind==iSet_IP55)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==6)index_set=5;
		if(sub_ind==7)sub_ind=8;
		if(sub_ind==10)index_set=9;
		if(sub_ind==11)sub_ind=12;
          if(sub_ind==31)
               {
               index_set=30;
               }
          if(sub_ind==27)
               {
               sub_ind=28;
               
               }
		if(sub_ind==28)
               {
               index_set=27;
               }
		if(sub_ind==36)
               {
       		index_set=35;
               }
          if(sub_ind==37)
               {
               sub_ind=38;
               
               }
		
		gran_char(&sub_ind,0,39);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==7)sub_ind=6;
		if(sub_ind==11)sub_ind=8;
       /* if(sub_ind==29)
               {
               sub_ind=28;
		       index_set=28;
               }*/ 
        if(sub_ind==32)
               {
               sub_ind=31;
		       index_set=31;
               } 
        if(sub_ind==37)
               {
               sub_ind=36;
		       index_set=36;
               }
		gran_char(&sub_ind,0,39);
		}
	else if(but==butD_)
		{
		sub_ind=38;
		}
		
	else if(sub_ind==0)
	     {
	     if(but==butE)
	          {
	          tree_up(iDef_6U,0,0,0);
	          ret(1000);
	          default_temp=10;
	          }
	     }	
	
     else if(sub_ind==1)
		{
		if(but==butE)
		     {
		     tree_up(iSet_T,0,0,0);
		     ret(1000);
		     phase=0;
		     }
		}	
					 
     else if(sub_ind==2)
		{
		if(but==butE)
		     {
		     tree_up(iStr_IP55,0,0,0);
		     ret(1000);
		     index_set=0;
 		     }
		}	
	
	else if(sub_ind==4)
	     {
	     if(but==butR)MNEMO_TIME++;
	     else if(but==butR_)MNEMO_TIME+=10;
	     else if(but==butL)MNEMO_TIME--;
	     else if(but==butL_)MNEMO_TIME-=10;

	     if(((MNEMO_TIME<5)||(MNEMO_TIME>60))&&(MNEMO_ON!=mnOFF))//lc640_write_int(EE_MNEMO_ON,mnOFF);	
	     if(((MNEMO_TIME>=5)&&(MNEMO_TIME<=60))&&(MNEMO_ON!=mnON))//lc640_write_int(EE_MNEMO_ON,mnON);
	     gran((signed short*)&MNEMO_TIME,4,61);
	     //lc640_write_int(EE_MNEMO_TIME,MNEMO_TIME);
	     speed=1;
	     }
				     		
	else if(sub_ind==5)
	     {
		if(ZV_ON)ZV_ON=0;
		else ZV_ON=1;
	     //lc640_write_int(EE_ZV_ON,ZV_ON);
	     speed=1;
	     }	
	
	else if(sub_ind==6)
	     {
		if(AV_OFF_AVT)AV_OFF_AVT=0;
		else AV_OFF_AVT=1;
	     //lc640_write_int(EE_AV_OFF_AVT,AV_OFF_AVT);
	     speed=1;
	     }	

	else if(sub_ind==8)
	     {
	     if(but==butE)
	          {
	          tree_up(iApv,0,0,0);
	          ret(1000);
	          }
	     }	

	else if(sub_ind==9)
	     {
		if(PAR)PAR=0;
		else PAR=1;
	     //lc640_write_int(EE_PAR,PAR);
	     speed=1;
	     }

	else if(sub_ind==10)
	     {
	     if(but==butR)TBAT++;
	     else if(but==butR_)TBAT+=10;
	     else if(but==butL)TBAT--;
	     else if(but==butL_)TBAT-=10;
	     gran(&TBAT,5,60);
	     //lc640_write_int(EE_TBAT,TBAT);
	     speed=1;
	     }	
	                    	     	
	else if(sub_ind==12)
	     {
	     if(but==butR)UMAX++;
	     else if(but==butR_)UMAX+=10;
	     else if(but==butL)UMAX--;
	     else if(but==butL_)UMAX-=10;
		#ifdef UKU_220
		gran(&UMAX,2000,3000);
		#else
	     gran(&UMAX,10,3000);
		#endif
	     //lc640_write_int(EE_UMAX,UMAX);
	     speed=1;
	     }
	else if(sub_ind==13)
	     {
	     if(but==butL)DU++;
	     else if(but==butL_)DU+=10;
	     else if(but==butR)DU--;
	     else if(but==butR_)DU-=10;
	     gran(&DU,50,UB20-250);
	     //lc640_write_int(EE_DU,DU);
	     speed=1;
	     }	     
	else if(sub_ind==14)
	     {
	     if(but==butR)UB0++;
	     else if(but==butR_)UB0+=10;
	     else if(but==butL)UB0--;
	     else if(but==butL_)UB0-=10;
		gran(&UB0,150,800);
          //lc640_write_int(EE_UB0,UB0);
	     speed=1;
	     }
	     
	else if(sub_ind==15)
	     {
	     if(but==butR)UB20++;
	     else if(but==butR_)UB20+=10;
	     else if(but==butL)UB20--;
	     else if(but==butL_)UB20-=10;
		gran(&UB20,150,800);
	     //lc640_write_int(EE_UB20,UB20);
	     speed=1;
	     }	

	else if(sub_ind==16)
	     {
	     if(but==butR)USIGN++;
	     else if(but==butR_)USIGN+=10;
	     else if(but==butL)USIGN--;
	     else if(but==butL_)USIGN-=10;
		gran(&USIGN,15,100);
	     //lc640_write_int(EE_USIGN,USIGN);
	     speed=1;
	     }	
	else if(sub_ind==17)
	     {
	     if(but==butR)UMN++;
	     else if(but==butR_)UMN+=10;
	     else if(but==butL)UMN--;
	     else if(but==butL_)UMN-=10;
	     gran(&UMN,1,220);
	     //lc640_write_int(EE_UMN,UMN);
	     speed=1;
	     }	

	else if(sub_ind==18)
	     {
	     if(but==butR)U0B++;
	     else if(but==butR_)U0B+=10;
	     else if(but==butL)U0B--;
	     else if(but==butL_)U0B-=10;
		gran(&U0B,150,800);
	     //lc640_write_int(EE_U0B,U0B);
	     speed=1;
	     }	
	     
	else if(sub_ind==19)
	     {
	     if(but==butR)IKB++;
	     else if(but==butR_)IKB+=10;
	     else if(but==butL)IKB--;
	     else if(but==butL_)IKB-=10;
	     gran(&IKB,1,500);
	     //lc640_write_int(EE_IKB,IKB);
	     speed=1;
	     }		
            
	else if(sub_ind==20)
	     {
	     if(but==butR)IZMAX++;
	     else if(but==butR_)IZMAX+=10;
	     else if(but==butL)IZMAX--;
	     else if(but==butL_)IZMAX-=10;
		gran(&IZMAX,10,1500);
	     //lc640_write_int(EE_IZMAX,IZMAX);
	     speed=1;
	     }   

	else if(sub_ind==21)
	     {
	     if(but==butR)IMAX++;
	     else if(but==butR_)IMAX+=10;
	     else if(but==butL)IMAX--;
	     else if(but==butL_)IMAX-=10;
		gran(&IMAX,10,1000);
	     //lc640_write_int(EE_IMAX,IMAX);
	     speed=1;
	     }		
	     
	else if(sub_ind==22)
	     {
	     if(but==butR)IMIN++;
	     else if(but==butR_)IMIN+=10;
	     else if(but==butL)IMIN--;
	     else if(but==butL_)IMIN-=10;
	     gran(&IMIN,40,IMAX-10);
	     //lc640_write_int(EE_IMIN,IMIN);
	     speed=1;
	     }
	
	else if(sub_ind==23)
	     {
	     if ((but==butR)||(but==butR_))UVZ+=1;
		if ((but==butL)||(but==butL_))UVZ-=1;
		gran(&UVZ,UB20,UMAX); 	          
		//lc640_write_int(EE_UVZ,UVZ);
	     speed=1;
	     }
	     
	else if(sub_ind==24)
		{
		if ((but==butR)||(but==butR_))TZAS++;
		if ((but==butL)||(but==butL_))TZAS--;
		gran(&TZAS,0,10);
		//lc640_write_int(EE_TZAS,TZAS);
		speed=1; 
		}	
			       	        
	else if(sub_ind==25)
	     {
	     if(but==butR)TMAX++;
	     else if(but==butR_)TMAX+=2;
	     else if(but==butL)TMAX--;
	     else if(but==butL_)TMAX-=2;
	     gran(&TMAX,10,100);
	     //lc640_write_int(EE_TMAX,TMAX);
	     speed=1;
	     }	
	
	else if(sub_ind==26)
	     {
	     if(but==butR)TSIGN++;
	     else if(but==butR_)TSIGN+=2;
	     else if(but==butL)TSIGN--;
	     else if(but==butL_)TSIGN-=2;
	     gran(&TSIGN,10,100);
	     //lc640_write_int(EE_TSIGN,TSIGN);
	     speed=1;
	     }	     
	else if(sub_ind==27)
	     {
	     if(but==butR)TBATMAX++;
	     else if(but==butR_)TBATMAX+=2;
	     else if(but==butL)TBATMAX--;
	     else if(but==butL_)TBATMAX-=2;
	     gran(&TBATMAX,10,100);
	     //lc640_write_int(EE_TBATMAX,TBATMAX);
	     speed=1;
	     }	
	
	else if(sub_ind==28)
	     {
	     if(but==butR)TBATSIGN++;
	     else if(but==butR_)TBATSIGN+=2;
	     else if(but==butL)TBATSIGN--;
	     else if(but==butL_)TBATSIGN-=2;
	     gran(&TBATSIGN,10,100);
	     //lc640_write_int(EE_TBATSIGN,TBATSIGN);
	     speed=1;
	     }
			
   	else if(sub_ind==29)
		{
		if(but==butE)
		     {
			tree_up(iKlimat_TELECORE2015,0,0,0);
		     ret(1000);
		     }
		}
		 
   	else if(sub_ind==30)
		{
		if(but==butE)
		     {
			tree_up(iExt_set_TELECORE2015,0,0,0);
		     ret(1000);
		     }
		}

  	else if(sub_ind==31)
		{
		if(but==butE)
		     {
		     tree_up(iLan_set,0,0,0);
		     ret(1000);
		     }
		}

	else if(sub_ind==32)
	     {
	     if(but==butR)AUSW_MAIN_NUMBER++;
	     else if(but==butR_)AUSW_MAIN_NUMBER+=20;
	     else if(but==butL)AUSW_MAIN_NUMBER--;
	     else if(but==butL_)AUSW_MAIN_NUMBER-=20;
		else if(but==butEL_)AUSW_MAIN_NUMBER=15000;
		if(AUSW_MAIN_NUMBER<13000)AUSW_MAIN_NUMBER=100000;
		if(AUSW_MAIN_NUMBER>100000)AUSW_MAIN_NUMBER=13000;
	     //lc640_write_int(EE_AUSW_MAIN_NUMBER,(short)(AUSW_MAIN_NUMBER&0x0000ffffUL));
		//lc640_write_int(EE_AUSW_MAIN_NUMBER+2,(short)((AUSW_MAIN_NUMBER&0xffff0000UL)>>16UL));
	     speed=1;
	     }                    		
   	else if(sub_ind==33)
		{
		if(but==butE)
		     {
		     tree_up(iSet_li_bat,0,0,0);
		     ret(1000);
		     }
		}

	else if(sub_ind==34)
	     {
	     if(but==butR)DU_LI_BAT++;
	     else if(but==butR_)DU_LI_BAT++;
	     else if(but==butL)DU_LI_BAT--;
	     else if(but==butL_)DU_LI_BAT--;
	     gran(&DU_LI_BAT,1,30);
	     //lc640_write_int(EE_DU_LI_BAT,DU_LI_BAT);
	     speed=1;
	     }

   	else if(sub_ind==35)
		{
		if(but==butE)
		     {
			if(NUMINV)
				{
		     	tree_up(iInv_set_sel,0,0,0);
		     	ret(1000);
				}
		     }
		}
  	else if(sub_ind==36)
	 	{
	    if(but==butR)FORVARDBPSCHHOUR++;
	    else if(but==butR_)FORVARDBPSCHHOUR+=2;
	    else if(but==butL)FORVARDBPSCHHOUR--;
	    else if(but==butL_)FORVARDBPSCHHOUR-=2;
	    gran(&FORVARDBPSCHHOUR,0,500);
	    //lc640_write_int(EE_FORVARDBPSCHHOUR,FORVARDBPSCHHOUR);
	    //numOfForvardBps_init();
		speed=1;
	    }
     else if((sub_ind==38) || (sub_ind==3))
		{
		if(but==butE)
		     {
		     tree_down(0,0);
		     ret(0);
		     }
		}
				
	else if(sub_ind==39)
		{
		if(but==butE)
		     {		
			tree_up(iK_prl,0,0,0);
			parol_init();
			ret(50);
			}						
		}
     }



else if(ind==iBat_link_set)

#define SIMAXIDEF 2
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			//lc640_write_int(EE_BAT_LINK,0);

			}
		else if(sub_ind==1)
			{
			//lc640_write_int(EE_BAT_LINK,1);

			}

		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }

else if(ind==iDef)

#define SIMAXIDEF 2
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			def_set(600,564,545,44,100,545,2,576);
			//lc640_write_int(EE_AUSW_MAIN,4840);

			}
		else if(sub_ind==1)
			{
			def_set(600,564,545,44,100,545,3,576);
			//lc640_write_int(EE_AUSW_MAIN,4860);

			}

		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }

else if(ind==iDef_RSTKM)

#define SIMAXIDEF 2
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			def_set(600,564,545,44,100,545,2,576);
			//lc640_write_int(EE_AUSW_MAIN,4840);

			}
		else if(sub_ind==1)
			{
			def_set(600,564,545,44,100,545,3,576);
			//lc640_write_int(EE_AUSW_MAIN,4860);

			}

		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }

else if(ind==iDef_3U)

#define SIMAXIDEF 4
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			def_set(600,564,545,44,100,545,2,576);
			//lc640_write_int(EE_AUSW_MAIN,4840);

			}
		else if(sub_ind==1)
			{
			def_set(600,564,545,44,100,545,3,576);
			//lc640_write_int(EE_AUSW_MAIN,4860);

			}

		else if(sub_ind==2)
			{
			def_set(750,705,681,55,100,680,2,720);
			//lc640_write_int(EE_AUSW_MAIN,6040);

			}
		else if(sub_ind==3)
			{
			def_set(750,705,681,55,100,680,3,720);
			//lc640_write_int(EE_AUSW_MAIN,6060);

			}

		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }

else if(ind==iDef_GLONASS)

#define SIMAXIDEF 4
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			def_set(600,564,545,44,100,545,2,576);
			//lc640_write_int(EE_AUSW_MAIN,4840);

			}
		else if(sub_ind==1)
			{
			def_set(600,564,545,44,100,545,3,576);
			//lc640_write_int(EE_AUSW_MAIN,4860);

			}

		else if(sub_ind==2)
			{
			def_set(750,705,681,55,100,680,2,720);
			//lc640_write_int(EE_AUSW_MAIN,6040);

			}
		else if(sub_ind==3)
			{
			def_set(750,705,681,55,100,680,3,720);
			//lc640_write_int(EE_AUSW_MAIN,6060);

			}

		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }

else if(ind==iDef_KONTUR)

#define SIMAXIDEF 2
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			def_set(600,564,545,44,100,545,2,576);
			//lc640_write_int(EE_AUSW_MAIN,4840);
			//lc640_write_int(EE_NUMSK,3);

			}
		else if(sub_ind==1)
			{
			def_set(600,564,545,44,100,545,3,576);
			//lc640_write_int(EE_AUSW_MAIN,4860);
			//lc640_write_int(EE_NUMSK,3);

			}

		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }
else if(ind==iDef_6U)
	{
	#define SIMAXIDEF 36
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			def_set(300,282,273,22,150,240,2,288);
			//lc640_write_int(EE_AUSW_MAIN,24120);
			}
		else if(sub_ind==1)
			{
			def_set(300,282,273,22,150,240,3,288);
			//lc640_write_int(EE_AUSW_MAIN,24120);
			}
		else if(sub_ind==2)
			{
			def_set(300,282,273,22,150,240,4,288);
			//lc640_write_int(EE_AUSW_MAIN,24120);
			}
		else if(sub_ind==3)
			{
			def_set(300,282,273,22,150,240,3,288);
			//lc640_write_int(EE_AUSW_MAIN,24210);
			}

		else if(sub_ind==4)
			{
			def_set(300,282,273,22,150,240,4,288);
			//lc640_write_int(EE_AUSW_MAIN,24210);
			}
		else if(sub_ind==5)
			{
			def_set(300,282,273,22,150,240,5,288);
			//lc640_write_int(EE_AUSW_MAIN,24210);
			}
		else if(sub_ind==6)
			{
			def_set(300,282,273,22,150,240,6,288);
			//lc640_write_int(EE_AUSW_MAIN,24210);
			}
		else if(sub_ind==7)
			{
			def_set(300,282,273,22,150,240,7,288);
			//lc640_write_int(EE_AUSW_MAIN,24210);
			}
		else if(sub_ind==8)
			{
			def_set(600,564,545,44,100,480,2,576);
			//lc640_write_int(EE_AUSW_MAIN,4880);
			}
		else if(sub_ind==9)
			{
			def_set(600,564,545,44,100,480,3,576);
			//lc640_write_int(EE_AUSW_MAIN,4880);
			}
		else if(sub_ind==10)
			{
			def_set(600,564,545,44,100,480,4,576);
			//lc640_write_int(EE_AUSW_MAIN,4880);
			}
		else if(sub_ind==11)
			{
			def_set(600,564,545,44,100,480,3,576);
			//lc640_write_int(EE_AUSW_MAIN,48140);
			}

		else if(sub_ind==12)
			{
			def_set(600,564,545,44,100,480,4,576);
			//lc640_write_int(EE_AUSW_MAIN,48140);
			}
		else if(sub_ind==13)
			{
			def_set(600,564,545,44,100,480,5,576);
			//lc640_write_int(EE_AUSW_MAIN,48140);
			}
		else if(sub_ind==14)
			{
			def_set(600,564,545,44,100,480,6,576);
			//lc640_write_int(EE_AUSW_MAIN,48140);
			}
		else if(sub_ind==15)
			{
			def_set(600,564,545,44,100,480,7,576);
			//lc640_write_int(EE_AUSW_MAIN,48140);
			}
		else if(sub_ind==16)
			{
			def_set(750,705,681,55,100,600,2,720);
			//lc640_write_int(EE_AUSW_MAIN,6080);
			}
		else if(sub_ind==17)
			{
			def_set(750,705,681,55,100,600,3,720);
			//lc640_write_int(EE_AUSW_MAIN,6080);
			}
		else if(sub_ind==18)
			{
			def_set(750,705,681,55,100,600,4,720);
			//lc640_write_int(EE_AUSW_MAIN,6080);
			}
		else if(sub_ind==19)
			{
			def_set(750,705,681,55,100,600,3,720);
			//lc640_write_int(EE_AUSW_MAIN,60140);
			}

		else if(sub_ind==20)
			{
			def_set(750,705,681,55,100,600,4,720);
			//lc640_write_int(EE_AUSW_MAIN,60140);
			}
		else if(sub_ind==21)
			{
			def_set(750,705,681,55,100,600,5,720);
			//lc640_write_int(EE_AUSW_MAIN,60140);
			}
		else if(sub_ind==22)
			{
			def_set(750,705,681,55,100,600,6,720);
			//lc640_write_int(EE_AUSW_MAIN,60140);
			}
		else if(sub_ind==23)
			{
			def_set(750,705,681,55,100,600,7,720);
			//lc640_write_int(EE_AUSW_MAIN,60140);
			}

		else if(sub_ind==24)
			{
			def_set(300,282,273,22,150,240,4,288);
			//lc640_write_int(EE_AUSW_MAIN,24123);
			}
		else if(sub_ind==25)
			{
			def_set(300,282,273,22,150,240,7,288);
			//lc640_write_int(EE_AUSW_MAIN,24213);
			}
		else if(sub_ind==26)
			{
			def_set(600,564,545,44,100,480,4,576);
			//lc640_write_int(EE_AUSW_MAIN,4883);
			}
		else if(sub_ind==27)
			{
			def_set(600,564,545,44,100,480,7,576);
			//lc640_write_int(EE_AUSW_MAIN,48143);
			}
		else if(sub_ind==28)
			{
			def_set(750,705,681,55,100,600,4,720);
			//lc640_write_int(EE_AUSW_MAIN,6083);
			}
		else if(sub_ind==29)
			{
			def_set(750,705,681,55,100,600,7,720);
			//lc640_write_int(EE_AUSW_MAIN,60143);
			}

		else if(sub_ind==30)
			{
			def_ips_set(240);
			//lc640_write_int(EE_AUSW_MAIN,2400);
			}
		else if(sub_ind==31)
			{
			def_ips_set(480);
			//lc640_write_int(EE_AUSW_MAIN,4800);
			}
		else if(sub_ind==32)
			{
			def_ips_set(600);
			//lc640_write_int(EE_AUSW_MAIN,6000);
			}

		else if(sub_ind==31)
			{
			def_ips_set(240);
			//lc640_write_int(EE_AUSW_MAIN,2403);
			}
		else if(sub_ind==32)
			{
			def_ips_set(480);
			//lc640_write_int(EE_AUSW_MAIN,4803);
			}
		else if(sub_ind==33)
			{
			def_ips_set(600);
			//lc640_write_int(EE_AUSW_MAIN,6003);
			}
		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }

else if (ind==iDef_220)
#define SIMAXIDEF 8
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			def_set(2700,2590,2450,198,30,2200,2,2590);
			//lc640_write_int(EE_DU,2450-1200);
			//lc640_write_int(EE_U_AVT,2450);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_AUSW_MAIN,22010);
			}
		else if(sub_ind==1)
			{
			def_set(2550,2397,2316,187,30,2200,2,2440);
			//lc640_write_int(EE_DU,2315-1100);
			//lc640_write_int(EE_U_AVT,2315);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_AUSW_MAIN,22010);
			}
		else if(sub_ind==2)
			{
			def_set(2700,2590,2450,198,30,2200,2,2590);
			//lc640_write_int(EE_DU,2450-1200);
			//lc640_write_int(EE_U_AVT,2450);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_AUSW_MAIN,22020);
			}
		else if(sub_ind==3)
			{
			def_set(2550,2397,2316,187,30,2200,2,2440);
			//lc640_write_int(EE_DU,2315-1100);
			//lc640_write_int(EE_U_AVT,2315);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_AUSW_MAIN,22020);
			}
		else if(sub_ind==4)
			{
			def_set(2700,2590,2450,198,30,2200,2,2590);
			//lc640_write_int(EE_DU,2450-1200);
			//lc640_write_int(EE_U_AVT,2450);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_AUSW_MAIN,22020);
			}
		else if(sub_ind==5)
			{
			def_set(2550,2397,2316,187,30,2200,2,2440);
			//lc640_write_int(EE_DU,2315-1100);
			//lc640_write_int(EE_U_AVT,2315);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_AUSW_MAIN,22035);
			}


		else if(sub_ind==6)
			{
			def_set(2700,2590,2450,198,30,2200,7,2590);
			//lc640_write_int(EE_DU,2450-1200);
			//lc640_write_int(EE_U_AVT,2450);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_AUSW_MAIN,22035);
			//lc640_write_int(EE_MNEMO_ON,mnOFF);
			}
		else if(sub_ind==7)
			{
			def_set(2700,2200,2200,198,30,2200,7,2590);
			//lc640_write_int(EE_DU,2450-1200);
			//lc640_write_int(EE_U_AVT,2450);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_AUSW_MAIN,22035);
			//lc640_write_int(EE_PAR,1);
			//lc640_write_int(EE_MNEMO_ON,mnOFF);
			}


		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }

else if (ind==iDef_220_V2)
#define SIMAXIDEF 3
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			def_set(2700,2590,2450,198,30,2200,4,2590);
			//lc640_write_int(EE_DU,2450-1200);
			//lc640_write_int(EE_U_AVT,2450);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_AUSW_MAIN,22010);
			}
		else if(sub_ind==1)
			{
			def_set(2450,2410,2315,187,30,2200,4,2410);
			//lc640_write_int(EE_DU,2315-1200);
			//lc640_write_int(EE_U_AVT,2315);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_AUSW_MAIN,22010);
			}
		else if(sub_ind==2)
			{
			def_set(2700,2590,2450,198,100,2200,3,2590);
			//lc640_write_int(EE_DU,2450-1200);
			//lc640_write_int(EE_U_AVT,2450);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_AUSW_MAIN,22033);
			}

		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }

else if (ind==iDef_220_IPS_TERMOKOMPENSAT)
#define SIMAXIDEF 2
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			def_ips_set(220);
			////lc640_write_int(EE_DU,2315-1850);
			//lc640_write_int(EE_U_AVT,2200);
			//lc640_write_int(EE_IZMAX,50);
			////lc640_write_int(EE_IMAX,12);
			////lc640_write_int(EE_IMIN,8);
			//lc640_write_int(EE_AUSW_MAIN,22043);
			//lc640_write_int(EE_NUMIST,3);

			//lc640_write_int(EE_PAR,0);
			////lc640_write_int(EE_UMAX,2550);
			//lc640_write_int(EE_DU,2315-1110);
			//lc640_write_int(EE_UB0,2397);
			//lc640_write_int(EE_UB20,2314);
			////lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_UVZ,2346);
			//lc640_write_int(EE_UBM_AV,0);
			//lc640_write_int(EE_U_OUT_KONTR_MAX,2420);
			//lc640_write_int(EE_U_OUT_KONTR_MIN,2200);

			}
		else if(sub_ind==1)
			{
			def_ips_set(220);
			//lc640_write_int(EE_DU,2315-1850);
			//lc640_write_int(EE_U_AVT,2200);
			////lc640_write_int(EE_IZMAX,50);
			//lc640_write_int(EE_AUSW_MAIN,22011);
			//lc640_write_int(EE_NUMIST,2);
			//lc640_write_int(EE_TERMOKOMP,1);
			////lc640_write_int(EE_IMAX,30);
		     ////lc640_write_int(EE_IMIN,24);
			////lc640_write_int(EE_UMAX,2450);
	/*		}
		else if(sub_ind==3)
			{ */
		//	def_ips_set(220);
			////lc640_write_int(EE_DU,2315-1850);
		//	//lc640_write_int(EE_U_AVT,2200);
			////lc640_write_int(EE_IZMAX,10);
			////lc640_write_int(EE_IMAX,30);
		//	//lc640_write_int(EE_IMIN,24);
			////lc640_write_int(EE_AUSW_MAIN,22023);
			//lc640_write_int(EE_NUMIST,2);

			//lc640_write_int(EE_PAR,0);
			//lc640_write_int(EE_UMAX,2550);
			//lc640_write_int(EE_DU,2315-1110);
			//lc640_write_int(EE_UB0,2397);
			//lc640_write_int(EE_UB20,2314);
			//lc640_write_int(EE_IZMAX,50);
			//lc640_write_int(EE_UVZ,2440);
			//lc640_write_int(EE_UBM_AV,0);
			//lc640_write_int(EE_UMAX,2450);
			//lc640_write_int(EE_U_OUT_KONTR_MAX,2420);
			//lc640_write_int(EE_U_OUT_KONTR_MIN,2200);

			}
		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }

else if(ind==iSet_T)
	{
	signed char temp;
	if(but==butR)
		{
		sub_ind++;
		gran_char(&sub_ind,0,5);
		}
	else if(but==butL)
		{
		sub_ind--;
		gran_char(&sub_ind,0,5);
		}
	else if(but==butE)
		{
		tree_down(0,0);
		}	
	else if(sub_ind==0)
	     {			    
	     temp=LPC_RTC->HOUR;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,23);
	          LPC_RTC->HOUR=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,23);
	          LPC_RTC->HOUR=temp;
	          }	
	     speed=1;               
	     }
     else if(sub_ind==1)
	     {
	     temp=LPC_RTC->MIN;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->MIN=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->MIN=temp;
	          }	
	     speed=1;               
	     }
     else if(sub_ind==2)
	     {				  
	     temp=LPC_RTC->SEC;
	     if((but==butU)||(but==butU_))
	          {
	          temp=0;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->SEC=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp=0;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->SEC=temp;
	          }	
	     speed=1;               
	     }

     else if(sub_ind==3)
	     {
	     temp=LPC_RTC->DOM;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,1,31);
	          LPC_RTC->DOM=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,1,31);
	          LPC_RTC->DOM=temp;
	          }	
	     speed=1;               
	     }
     else if(sub_ind==4)
	     {
	     temp=LPC_RTC->MONTH;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,1,12);
	          LPC_RTC->MONTH=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,1,12);
	          LPC_RTC->MONTH=temp;
	          }	
	     speed=1;               
	     }	  
     else if(sub_ind==5)
	     {
	     temp=LPC_RTC->YEAR;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,99);
	          LPC_RTC->YEAR=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,99);
	          LPC_RTC->YEAR=temp;
	          }	
	     speed=1;               
	     }		        
	}  

else if(ind==iStr)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)index_set=1;
		if(sub_ind==3)sub_ind++;
		gran_char(&sub_ind,1,5);	

		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==3)sub_ind--;
		gran_char(&sub_ind,1,5);	 
		}
	else if(but==butD_)
		{
/**/		sub_ind=5;		 /**/
		}				
     else if(sub_ind==1)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMIST++;
	     	gran(&NUMIST,0,12);
	     	//lc640_write_int(EE_NUMIST,NUMIST);
			//numOfForvardBps_init();
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMIST--;
	     	gran(&NUMIST,0,12);
	     	//lc640_write_int(EE_NUMIST,NUMIST);
			//numOfForvardBps_init();
	     	}
          }	
/**/
	  else if(sub_ind==2)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMINV++;
	     	gran(&NUMINV,0,12-NUMIST);
	     	//lc640_write_int(EE_NUMINV,NUMINV);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMINV--;
	     	gran(&NUMINV,0,12-NUMIST);
	     	//lc640_write_int(EE_NUMINV,NUMINV);
	     	}
          }	
/**/
/**/     else if(sub_ind==3)  /**/
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMAVT++;
	     	gran(&NUMAVT,0,12);
	     	//lc640_write_int(EE_NUMAVT,NUMAVT);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMAVT--;
	     	gran(&NUMAVT,0,12);
	     	//lc640_write_int(EE_NUMAVT,NUMAVT);
	     	}
          }	
          
/**/    else if(sub_ind==5)	  /**/
	     {
	     if(but==butE)
	          {
			tree_down(0,0);
	          }
          }	          
	}

else if(ind==iStr_IP55)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
	else if(but==butD_)
		{
		sub_ind=4;
		}
     else if(sub_ind==0)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMBAT_IP55++;
	     	gran(&NUMBAT_IP55,0,1);
	     	//lc640_write_int(EE_NUMBAT_IP55,NUMBAT_IP55);
			}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMBAT_IP55--;
	     	gran(&NUMBAT_IP55,0,1);
	     	//lc640_write_int(EE_NUMBAT_IP55,NUMBAT_IP55);
	     	}
          }							
     else if(sub_ind==1)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMIST++;
	     	gran(&NUMIST,0,12);
	     	//lc640_write_int(EE_NUMIST,NUMIST);
			//numOfForvardBps_init();
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMIST--;
	     	gran(&NUMIST,0,12);
	     	//lc640_write_int(EE_NUMIST,NUMIST);
			//numOfForvardBps_init();
	     	}
         }	     			          
     else if(sub_ind==2)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMDT++;
	     	gran(&NUMDT,0,3);
	     	//lc640_write_int(EE_NUMDT,NUMDT);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMDT--;
	     	gran(&NUMDT,0,3);
	     	//lc640_write_int(EE_NUMDT,NUMDT);
	     	}
          }	
     else if(sub_ind==3)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMSK++;
	     	gran(&NUMSK,0,4);
	     	//lc640_write_int(EE_NUMSK,NUMSK);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMSK--;
	     	gran(&NUMSK,0,4);
	     	//lc640_write_int(EE_NUMSK,NUMSK);
	     	}
          }              
    else if(sub_ind==4)
	     {
	     if(but==butE)
	          {
				tree_down(0,0);
	          }
          }	          
	}     

else if (ind==iSpch_set)
	{
     ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==3)index_set=2;
		if(sub_ind==4)sub_ind=5;
		if(sub_ind==8)sub_ind=9;
		gran_char(&sub_ind,0,9);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==4)sub_ind=3;
		//if(sub_ind==3)index_set=2;
		if(sub_ind==8)sub_ind=7;
		
		gran_char(&sub_ind,0,9);
		}
	else if(but==butD_)
		{
		sub_ind=6;
		}			
/*	else if(but==butE)
	     {
	     if(sub_ind==9)
	          {
	          //a=b[--ptr_ind];
	          tree_down(0,0);
	          }
          }*/
     
	else if(sub_ind==0)
		{
		if(but==butR)
			{
			speedChrgCurr++;
			gran(&speedChrgCurr,0,100);
			//lc640_write_int(EE_SPEED_CHRG_CURR,speedChrgCurr);
			}
		else if(but==butR_)
			{
			speedChrgCurr+=2;
			gran(&speedChrgCurr,0,100);
			//lc640_write_int(EE_SPEED_CHRG_CURR,speedChrgCurr);
			}
		else if(but==butL)
			{
			speedChrgCurr--;
			gran(&speedChrgCurr,0,100);
			//lc640_write_int(EE_SPEED_CHRG_CURR,speedChrgCurr);
			}
		else if(but==butL_)
			{
			speedChrgCurr-=2;
			gran(&speedChrgCurr,0,100);
			//lc640_write_int(EE_SPEED_CHRG_CURR,speedChrgCurr);
			}
		speed=1;
		}
	else if(sub_ind==1)
		{
		if(but==butR)
			{
			speedChrgVolt++;
			gran(&speedChrgVolt,0,UMAX);
			//lc640_write_int(EE_SPEED_CHRG_VOLT,speedChrgVolt);
			}
		else if(but==butR_)
			{
			speedChrgVolt=(speedChrgVolt/5+1)*5;
			gran(&speedChrgVolt,0,UMAX);
			//lc640_write_int(EE_SPEED_CHRG_VOLT,speedChrgVolt);
			}
		else if(but==butL)
			{
			speedChrgVolt--;
			gran(&speedChrgVolt,0,UMAX);
			//lc640_write_int(EE_SPEED_CHRG_VOLT,speedChrgVolt);
			}
		else if(but==butL_)
			{
			speedChrgVolt=(speedChrgVolt/5-1)*5;
			gran(&speedChrgVolt,0,UMAX);
			//lc640_write_int(EE_SPEED_CHRG_VOLT,speedChrgVolt);
			}
		speed=1;
		}
	else if(sub_ind==2)
		{
		if((but==butR)||(but==butR_))
			{
			speedChrgTimeInHour++;
			gran(&speedChrgTimeInHour,1,24);
			//lc640_write_int(EE_SPEED_CHRG_TIME,speedChrgTimeInHour);
			}
		else if((but==butL)||(but==butL_))
			{
			speedChrgTimeInHour--;
			gran(&speedChrgTimeInHour,1,24);
			//lc640_write_int(EE_SPEED_CHRG_TIME,speedChrgTimeInHour);
			}
		speed=1;
		}		
	else if(sub_ind==3)
		{
		if((but==butR)||(but==butR_))
			{
			speedChrgAvtEn=1;
			//lc640_write_int(EE_SPEED_CHRG_AVT_EN,speedChrgAvtEn);
			}
		else if((but==butL)||(but==butL_))
			{
			speedChrgAvtEn=0;
			//lc640_write_int(EE_SPEED_CHRG_AVT_EN,speedChrgAvtEn);
			}
		speed=1;
		}

	else if(sub_ind==5)
		{
		if((but==butR)||(but==butR_))
			{
			speedChrgDU++;
			gran(&speedChrgDU,1,100);
			//lc640_write_int(EE_SPEED_CHRG_D_U,speedChrgDU);
			}
		else if((but==butL)||(but==butL_))
			{
			speedChrgDU--;
			gran(&speedChrgDU,1,100);
			//lc640_write_int(EE_SPEED_CHRG_D_U,speedChrgDU);
			}
		speed=1;
		}

	else if(sub_ind==6)
		{
		if((but==butR)||(but==butR_))
			{
			speedChrgBlckSrc++;
			gran(&speedChrgBlckSrc,0,2);
			//lc640_write_int(EE_SPEED_CHRG_BLOCK_SRC,speedChrgBlckSrc);
			}
		else if((but==butL)||(but==butL_))
			{
			speedChrgBlckSrc--;
			gran(&speedChrgBlckSrc,0,2);
			//lc640_write_int(EE_SPEED_CHRG_BLOCK_SRC,speedChrgBlckSrc);
			}
		speed=1;
		}

	else if(sub_ind==7)
		{
		if((but==butR)||(but==butR_))
			{
			speedChrgBlckLog=1;
			//lc640_write_int(EE_SPEED_CHRG_BLOCK_LOG,speedChrgBlckLog);
			}
		else if((but==butL)||(but==butL_))
			{
			speedChrgBlckLog=0;
			//lc640_write_int(EE_SPEED_CHRG_BLOCK_LOG,speedChrgBlckLog);
			}
		speed=1;
		}





	else if((sub_ind==9)&&(but==butE))
		{
	     tree_down(0,0);
	     ret(0);
		} 												
	}

else if (ind==iBlok_ips_set)
	{
    ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}			
     
	else if(sub_ind==0)
		{
		if((but==butR)||(but==butR_))
			{
			ipsBlckSrc++;
			gran(&ipsBlckSrc,0,2);
			//lc640_write_int(EE_IPS_BLOCK_SRC,ipsBlckSrc);
			}
		else if((but==butL)||(but==butL_))
			{
			ipsBlckSrc--;
			gran(&ipsBlckSrc,0,2);
			//lc640_write_int(EE_IPS_BLOCK_SRC,ipsBlckSrc);
			}
		speed=1;
		}

	else if(sub_ind==1)
		{
		if((but==butR)||(but==butR_))
			{
			ipsBlckLog=1;
			//lc640_write_int(EE_IPS_BLOCK_LOG,ipsBlckLog);
			}
		else if((but==butL)||(but==butL_))
			{
			ipsBlckLog=0;
			//lc640_write_int(EE_IPS_BLOCK_LOG,ipsBlckLog);
			}
		speed=1;
		}

	else if((sub_ind==3)&&(but==butE))
		{
	     tree_down(0,0);
	     ret(0);
		} 												
	}

else if (ind==iApv)
	{
     ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butD_)
		{
		sub_ind=simax;
		}			
	else if(but==butE)
	     {
	     if(sub_ind==simax)
	          {
	          //a=b[--ptr_ind];
	          tree_down(0,0);
	          }
	     else if(sub_ind==0)   
	          {
	          //if(APV_ON1==apvON)lc640_write_int(EE_APV_ON1,apvOFF);
	          //else lc640_write_int(EE_APV_ON1,apvON);
	          }
          else if((sub_ind==1)&&(APV_ON1==apvON))   
	          {
	          //if(APV_ON2==apvON)lc640_write_int(EE_APV_ON2,apvOFF);
	          //else lc640_write_int(EE_APV_ON2,apvON);
	          }	 
          }
     
     else if((sub_ind==2)&&(APV_ON2==apvON))   
          {
	     if((but==butR)||(but==butR_))
	          {
	          signed short tempSS;
	          tempSS=APV_ON2_TIME;
	          tempSS++;
	          gran(&tempSS,1,24);
	          //lc640_write_int(EE_APV_ON2_TIME,tempSS);
	          }
          else if((but==butL)||(but==butL_))
	          {
	          signed short tempSS;
	          tempSS=APV_ON2_TIME;
	          tempSS--;
	          gran(&tempSS,1,24);
	          //lc640_write_int(EE_APV_ON2_TIME,tempSS);
	          }	          
	     speed=1;
	     }	 
  	} 

else if (ind==iExt_set)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);		
		}

	else if((but==butE)&&(sub_ind==0))
		{
	     tree_up(iExt_ddv,0,0,0);
	     ret(0);
		}

	else if((but==butE)&&(sub_ind==1))
		{
	     tree_up(iExt_ddi,0,0,0);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==2))
		{
	     tree_up(iExt_dud,0,0,0);
	     ret(0);
		}
/*	else if((but==butE)&&(sub_ind==3))
		{
	     tree_up(iExt_dp,0,0,0);
	     ret(0);
		} */
		
	else if((but==butE)&&(sub_ind==3))
		{
	     tree_down(0,0);
	     ret(0);
		}        	
	}

else if (ind==iExt_set_TELECORE2015)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,1);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,1);		
		}

	else if((but==butE)&&(sub_ind==0))
		{
	     tree_up(iExt_ddv,0,0,0);
	     ret(0);
		}

	else if((but==butE)&&(sub_ind==1))
		{
	     tree_down(0,0);
	     ret(0);
		}        	
	}


else if (ind==iExt_set_3U)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMSK);		
		}
 	else if((but==butE)&&(sub_ind==NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==0))
		{
	     tree_up(iExt_sk_3U,0,0,0);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==1))
		{
	     tree_up(iExt_sk_3U,0,0,1);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==2))
		{
	     tree_up(iExt_sk_3U,0,0,2);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==3))
		{
	     tree_up(iExt_sk_3U,0,0,3);
	     ret(0);
		} 
	}

else if (ind==iExt_set_GLONASS)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMSK);		
		}
 	else if((but==butE)&&(sub_ind==NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==0))
		{
	     tree_up(iExt_sk_GLONASS,0,0,0);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==1))
		{
	     tree_up(iExt_sk_GLONASS,0,0,1);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==2))
		{
	     tree_up(iExt_sk_GLONASS,0,0,2);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==3))
		{
	     tree_up(iExt_sk_GLONASS,0,0,3);
	     ret(0);
		} 
	}
	
else if (ind==iExt_dt)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,1,7);
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		gran_char(&sub_ind,1,7);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
		
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			//if(!TMAX_EXT_EN[sub_ind1])lc640_write_int(ADR_TMAX_EXT_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_TMAX_EXT_EN[sub_ind1],0);
			}
		else if((but==butR)||(but==butR_))
			{
			TMAX_EXT[sub_ind1]++;
			}	
		else if((but==butL)||(but==butL_))
			{
			TMAX_EXT[sub_ind1]--;
			}	
		gran(&TMAX_EXT[sub_ind1],-50,100);
		//if(lc640_read_int(ADR_TMAX_EXT[sub_ind1])!=TMAX_EXT[sub_ind1]) //lc640_write_int(ADR_TMAX_EXT[sub_ind1],TMAX_EXT[sub_ind1]);			
		speed=1;
		}
	else if(sub_ind==2) 
		{
		if(but==butE)
			{
			//if(!TMIN_EXT_EN[sub_ind1])lc640_write_int(ADR_TMIN_EXT_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_TMIN_EXT_EN[sub_ind1],0);
			}
		else if((but==butR)||(but==butR_))
			{
			TMIN_EXT[sub_ind1]++;
			}	
		else if((but==butL)||(but==butL_))
			{
			TMIN_EXT[sub_ind1]--;
			}	
		gran(&TMIN_EXT[sub_ind1],-50,100);
		//if(lc640_read_int(ADR_TMIN_EXT[sub_ind1])!=TMIN_EXT[sub_ind1]) //lc640_write_int(ADR_TMIN_EXT[sub_ind1],TMIN_EXT[sub_ind1]);			
		speed=1;
		}		
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			//if(!T_EXT_REL_EN[sub_ind1])lc640_write_int(ADR_T_EXT_REL_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_T_EXT_REL_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			//if(!T_EXT_ZVUK_EN[sub_ind1])lc640_write_int(ADR_T_EXT_ZVUK_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_T_EXT_ZVUK_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			//if(!T_EXT_LCD_EN[sub_ind1])lc640_write_int(ADR_T_EXT_LCD_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_T_EXT_LCD_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==6) 
		{
		if(but==butE)
			{
			//if(!T_EXT_RS_EN[sub_ind1])lc640_write_int(ADR_T_EXT_RS_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_T_EXT_RS_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==7) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			//a=b[--ptr_ind];
			}
		}												
	}	

else if (ind==iExt_sk)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,7);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,7);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			//if(!SK_SIGN[sub_ind1])lc640_write_int(ADR_SK_SIGN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_SK_SIGN[sub_ind1],0);
			}
		}
	else if(sub_ind==3) 
		{
	//	if(but==butE)
		//	{
	//		if(!SK_REL_EN[sub_ind1])//lc640_write_int(ADR_SK_REL_EN[sub_ind1],0xffff);
	//		else //lc640_write_int(ADR_SK_REL_EN[sub_ind1],0);
	//		}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			//if(!SK_ZVUK_EN[sub_ind1])lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			//if(!SK_LCD_EN[sub_ind1])lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==6) 
		{
		if(but==butE)
			{
			//if(!SK_RS_EN[sub_ind1])lc640_write_int(ADR_SK_RS_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_SK_RS_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==7) 
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			tree_down(0,0);
			}
		}												
	}	

else if (ind==iExt_sk_3U)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=5;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			//if(!SK_SIGN[sub_ind1])lc640_write_int(ADR_SK_SIGN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_SK_SIGN[sub_ind1],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			//if(!SK_ZVUK_EN[sub_ind1])lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			//if(!SK_LCD_EN[sub_ind1])lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}	

else if (ind==iExt_sk_GLONASS)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=5;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			//if(!SK_SIGN[sub_ind1])lc640_write_int(ADR_SK_SIGN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_SK_SIGN[sub_ind1],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			//if(!SK_ZVUK_EN[sub_ind1])lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			//if(!SK_LCD_EN[sub_ind1])lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0xffff);
			//else lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}	


else if (ind==iExt_ddv)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			//if(!SK_SIGN[0])lc640_write_int(ADR_SK_SIGN[0],0xffff);
			//else lc640_write_int(ADR_SK_SIGN[0],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			//if(SK_REL_EN[0])lc640_write_int(ADR_SK_REL_EN[0],0);
			//else lc640_write_int(ADR_SK_REL_EN[0],0xffff);
			}
		}	

	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			//if(SK_LCD_EN[0])lc640_write_int(ADR_SK_LCD_EN[0],0);
			//else lc640_write_int(ADR_SK_LCD_EN[0],0xffff);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}	

else if (ind==iExt_ddi)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			//if(!SK_SIGN[1])lc640_write_int(ADR_SK_SIGN[1],0xffff);
			//else lc640_write_int(ADR_SK_SIGN[1],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			//if(SK_REL_EN[1])lc640_write_int(ADR_SK_REL_EN[1],0);
			//else lc640_write_int(ADR_SK_REL_EN[1],0xffff);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			//if(SK_LCD_EN[1])lc640_write_int(ADR_SK_LCD_EN[1],0);
			//else lc640_write_int(ADR_SK_LCD_EN[1],0xffff);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}
 
 else if (ind==iExt_dud)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			//if(!SK_SIGN[2])lc640_write_int(ADR_SK_SIGN[2],0xffff);
			//else lc640_write_int(ADR_SK_SIGN[2],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			//if(SK_REL_EN[2])lc640_write_int(ADR_SK_REL_EN[2],0);
			//else lc640_write_int(ADR_SK_REL_EN[2],0xffff);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			//if(SK_LCD_EN[2])lc640_write_int(ADR_SK_LCD_EN[2],0);
			//else lc640_write_int(ADR_SK_LCD_EN[2],0xffff);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}
/*     
else if (ind==iExt_dp)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[3])//lc640_write_int(ADR_SK_SIGN[3],0xffff);
			else //lc640_write_int(ADR_SK_SIGN[3],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(!SK_REL_EN[3])//lc640_write_int(ADR_SK_REL_EN[3],0xffff);
			else //lc640_write_int(ADR_SK_REL_EN[3],0);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(!SK_LCD_EN[3])//lc640_write_int(ADR_SK_LCD_EN[3],0xffff);
			else //lc640_write_int(ADR_SK_LCD_EN[3],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}         	
*/		     
else if(ind==iK)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2);
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2;
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			tree_up(iK_net,0,0,0);	
			ret(1000);		
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMINV)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_inv_sel,0,0,0);	
			ret(1000);
			}
								
			else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))		 /**/
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
/**/     	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)))	   /**/
			{
			tree_up(iK_t_ext,0,0,0);	
			ret(1000);			
			}
/**/		else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+1))		/**/
			{
			tree_up(iK_power_net3,0,0,0);	
			ret(1000);
               }               				
/**/	   	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2))	   /**/
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}					
	}

else if(ind==iK_GLONASS)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2);
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2;
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			tree_up(iK_net,0,0,0);	
			ret(1000);		
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMINV)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_inv_sel,0,0,0);	
			ret(1000);
			}
		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))		 
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
     	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)))	   
			{
			tree_up(iK_t_ext,0,0,0);	
			ret(1000);			
			}
	   	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+1))	   
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}					
	}

else if(ind==iK_RSTKM)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2);
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2;
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			tree_up(iK_net,0,0,0);	
			ret(1000);		
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
/*		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
         	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)))
			{
			tree_up(iK_t_ext,0,0,0);	
			ret(1000);			
			}
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+1))
			{
			tree_up(iK_power_net3,0,0,0);	
			ret(1000);
               }               				
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+2))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}*/
/**/		else if((NUMINV)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_inv_sel,0,0,0);	
			ret(1000);
			}
								
			else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))		 /**/
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
/**/     	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)))	   /**/
			{
			tree_up(iK_t_ext,0,0,0);	
			ret(1000);			
			}
/**/		else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+1))		/**/
			{
			tree_up(iK_power_net3,0,0,0);	
			ret(1000);
               }               				
/**/	   	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2))	   /**/
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}
	}

else if(ind==iK_KONTUR)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+2);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+2);
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+2;
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			tree_up(iK_net,0,0,0);	
			ret(1000);		
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
         	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)))
			{
			tree_up(iK_t_ext,0,0,0);	
			ret(1000);			
			}
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+1))
			{
			tree_up(iK_power_net,0,0,0);	
			ret(1000);
               }               				
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+2))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}										
	}

else if(ind==iK_6U)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)+(NUMMAKB!=0)+((NUMBYPASS>0)&&(NUMBYPASS<2)));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)+(NUMMAKB!=0)+((NUMBYPASS>0)&&(NUMBYPASS<2)));
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)+(NUMMAKB!=0)+((NUMBYPASS>0)&&(NUMBYPASS<2));
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if(AUSW_MAIN%10)
				{
				tree_up(iK_net3,0,0,0);
		     	ret(1000);
				}
			else 
				{
				tree_up(iK_net,0,0,0);
		     	ret(1000);
				}
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMINV)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_inv_sel,0,0,1);	
			ret(1000);
			}		

		else if((NUMBYPASS)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))
			{
			tree_up(iK_byps,0,0,1);	
			ret(1000);
			}

		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+((NUMBYPASS>0)&&(NUMBYPASS<2)))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
         	else if((NUMDT)&&(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+((NUMBYPASS>0)&&(NUMBYPASS<2)))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}
         	else if((NUMMAKB)&&(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)+((NUMBYPASS>0)&&(NUMBYPASS<2)))))
			{
			tree_up(iK_makb_sel,0,0,0);	
			ret(1000);			
			}							
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)+(NUMMAKB!=0)+((NUMBYPASS>0)&&(NUMBYPASS<2))))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}			
	}

else if(ind==iK_220)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0);
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if((AUSW_MAIN==22035)||(AUSW_MAIN==22033))
				{
				tree_up(iK_net3,0,0,0);
		     	ret(1000);
				}
			else
				{
				tree_up(iK_net,0,0,0);	
				ret(1000);		
				}
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMINV)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_inv_sel,0,0,1);	
			ret(1000);
			}		

		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
         	else if((NUMDT)&&(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}				
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}			
	}



else if(ind==iK_220_IPS_TERMOKOMPENSAT)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if(AUSW_MAIN==22033)
				{
				tree_up(iK_net3,0,0,0);
		     	ret(1000);
				}
			else
				{
				tree_up(iK_net,0,0,0);	
				ret(1000);		
				}
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}

		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_out,0,0,0);	
			ret(1000);
			}
          
         	else if((NUMDT)&&(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}
 						
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}			
	}

else if(ind==iK_220_IPS_TERMOKOMPENSAT_IB)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3+(NUMIST!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3+(NUMIST!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=3+(NUMIST!=0)+(NUMDT!=0);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if((AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043))
				{
				tree_up(iK_net3,0,0,0);
		     	ret(1000);
				}
			else
				{
				tree_up(iK_net,0,0,0);	
				ret(1000);		
				}
			}
		else if(sub_ind==1)
			{
			tree_up(iK_bat_ips_termokompensat_ib,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==2))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}

		else if((sub_ind==(2+(NUMIST!=0))))
			{
			tree_up(iK_out,0,0,0);	
			ret(1000);
			}
          
         	else if((NUMDT)&&(sub_ind==(3+(NUMIST!=0))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}
 						
          else if(sub_ind==(3+(NUMIST!=0)+(NUMDT!=0)))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}			
	}
/*
else if(ind==iK_220_IPS_TERMOKOMPENSAT_IB)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3+(NUMIST!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3+(NUMIST!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=3+(NUMIST!=0)+(NUMDT!=0);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if((AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043))
				{
				tree_up(iK_net3,0,0,0);
		     	ret(1000);
				}
			else
				{
				tree_up(iK_net,0,0,0);	
				ret(1000);		
				}
			}
		else if(sub_ind==1)
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==2))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}

		else if((sub_ind==(2+(NUMIST!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
         	else if((NUMDT)&&(sub_ind==(3+(NUMIST!=0))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}
 						
          else if(sub_ind==(3+(NUMIST!=0)+(NUMDT!=0)))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}			
	}
*/
else if(ind==iK_220_380)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=3+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0);
		}
   	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)))
			{
			if((but==butR)||(but==butR_)||(but==butE)||(but==butE_))
				{
				if(RELE_VENT_LOGIC==0)RELE_VENT_LOGIC=1;
				else if(RELE_VENT_LOGIC==1)RELE_VENT_LOGIC=2;
				else RELE_VENT_LOGIC=0;
				//lc640_write_int(EE_RELE_VENT_LOGIC,RELE_VENT_LOGIC);
				}
			else if((but==butL)||(but==butL_))
				{
				if(RELE_VENT_LOGIC==0)RELE_VENT_LOGIC=2;
				else if(RELE_VENT_LOGIC==2)RELE_VENT_LOGIC=1;
				else RELE_VENT_LOGIC=0;
				//lc640_write_int(EE_RELE_VENT_LOGIC,RELE_VENT_LOGIC);
				}			
            }
										
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if(AUSW_MAIN==22035)
				{
				tree_up(iK_net3,0,0,0);
		     	ret(1000);
				}
			else
				{
				tree_up(iK_net,0,0,0);	
				ret(1000);		
				}
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMINV)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_inv_sel,0,0,1);	
			ret(1000);
			}		

		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
         	else if((NUMDT)&&(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}
 						
          else if(sub_ind==(3+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}			
	}

else if(ind==iK_IP55)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT_IP55!=0)+(NUMIST!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT_IP55!=0)+(NUMIST!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT_IP55!=0)+(NUMIST!=0)+(NUMDT!=0);
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			tree_up(iK_net,0,0,0);
		    ret(1000);
			}
		else if((NUMBAT_IP55)&&(sub_ind==1))
			{
			tree_up(iK_bat,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT_IP55!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}

		else if((sub_ind==(1+(NUMBAT_IP55!=0)+(NUMIST!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
    	else if((NUMDT)&&(sub_ind==(2+(NUMBAT_IP55!=0)+(NUMIST!=0))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}
    	else if(sub_ind==(2+(NUMBAT_IP55!=0)+(NUMIST!=0)+(NUMDT!=0)))
			{
	     	tree_down(0,0);
	        ret(0);
            }	               			
		}			
	}

else if(ind==iK_net)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,1);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,1);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=1;
		}				
	else if(sub_ind==0)
		{
		//temp_SS=lc640_read_int(EE_KUNET);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			//temp_SS=//lc640_read_int(EE_KUNET);
			temp_SS+=10;
			////lc640_write_int(EE_KUNET,temp_SS);
			}	
		else if(but==butL)
			{
			//temp_SS=//lc640_read_int(EE_KUNET);
			temp_SS--;
			////lc640_write_int(EE_KUNET,temp_SS);
			}
		else if(but==butL_)
			{
			//temp_SS=//lc640_read_int(EE_KUNET);
			temp_SS-=10;
			////lc640_write_int(EE_KUNET,temp_SS);
			}				
		speed=1;
		gran(&temp_SS,10,12000);
		//lc640_write_int(EE_KUNET,temp_SS);
					
		}
	else if(sub_ind==1)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_net3)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}				
	else if(sub_ind==0)
		{
		//temp_SS=lc640_read_int(EE_KUNETA);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,150,4000);
		//lc640_write_int(EE_KUNETA,temp_SS);
		}

	else if(sub_ind==1)
		{
		//temp_SS=lc640_read_int(EE_KUNETB);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,150,4000);
		//lc640_write_int(EE_KUNETB,temp_SS);
		}

	else if(sub_ind==2)
		{
		//temp_SS=lc640_read_int(EE_KUNETC);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,150,4000);
		//lc640_write_int(EE_KUNETC,temp_SS);
		}

	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_power_net)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}				
	else if(sub_ind==0)
		{
		//temp_SS=lc640_read_int(EE_KUNET_EXT0);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		//lc640_write_int(EE_KUNET_EXT0,temp_SS);
					
		}
	else if(sub_ind==1)
		{
		//temp_SS=lc640_read_int(EE_KUNET_EXT1);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		//lc640_write_int(EE_KUNET_EXT1,temp_SS);
					
		}



	else if(sub_ind==2)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_power_net3)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,6);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,6);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=6;
		}
						
	else if(sub_ind==0)
		{
		//temp_SS=lc640_read_int(EE_KVV0_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		//lc640_write_int(EE_KVV0_EB2,temp_SS);
		}

	else if(sub_ind==1)
		{
		//temp_SS=lc640_read_int(EE_KVV1_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		//lc640_write_int(EE_KVV1_EB2,temp_SS);
		}

	else if(sub_ind==2)
		{
		//temp_SS=lc640_read_int(EE_KVV2_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		//lc640_write_int(EE_KVV2_EB2,temp_SS);
		}

	else if(sub_ind==3)
		{
		//temp_SS=lc640_read_int(EE_KPES0_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		//lc640_write_int(EE_KPES0_EB2,temp_SS);
		}

	else if(sub_ind==4)
		{
		//temp_SS=lc640_read_int(EE_KPES1_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		//lc640_write_int(EE_KPES1_EB2,temp_SS);
		}

	else if(sub_ind==5)
		{
		//temp_SS=lc640_read_int(EE_KPES2_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		//lc640_write_int(EE_KPES2_EB2,temp_SS);
		}




	else if(sub_ind==6)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_bat_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMBAT);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMBAT);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMBAT;
		}	
	else if((but==butE)&&(NUMBAT)&&(BAT_IS_ON[0]==bisON)&&(sub_ind==0))
		{
		#ifdef UKU_6U
		tree_up(iK_bat_simple,0,0,0);
		#else

		#ifdef UKU_220_V2
		tree_up(iK_bat_simple,0,0,0);
		#else
		
	/*	#ifdef UKU_220 
		tree_up(iK_bat_simple,0,0,0);
		#else*/
		tree_up(iK_bat,0,0,0);	
		#endif
		#endif
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);

		ret(1000);
		}	
	else if((but==butE)&&(NUMBAT)&&(BAT_IS_ON[1]==bisON)&&(sub_ind==((BAT_IS_ON[0]==bisON))))
		{
		#ifdef UKU_6U
		tree_up(iK_bat_simple,0,0,1);
		#else
		tree_up(iK_bat,0,0,1);	
		#endif
		
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);
     		
		ret(1000);
		}	
	else if(sub_ind==(NUMBAT))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_bat_sel_TELECORE)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMBAT_IP55);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMBAT_IP55);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMBAT_IP55;
		}	
	else if((but==butE)&&(NUMBAT_IP55)&&(BAT_IS_ON[0]==bisON)&&(sub_ind==0))
		{
		tree_up(iK_bat,0,0,0);	

		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);

		ret(1000);
		}	
	else if((but==butE)&&(NUMBAT)&&(BAT_IS_ON[1]==bisON)&&(sub_ind==((BAT_IS_ON[0]==bisON))))
		{
		tree_up(iK_bat,0,0,1);	
		
     		
		ret(1000);
		}	
	else if(sub_ind==(NUMBAT_IP55))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_bat)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2)) sub_ind=0;
        gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}			
					
	else if(sub_ind==0)
		{
		if(but==butE)
		     {
		     if(phase==0)
		          {
		          ////lc640_write_int(ADR_KI0BAT[sub_ind1],ad7705_buff_[sub_ind1]);
		          phase=1;
		          }
		     }	
		else
			{
			//temp_SS=lc640_read_int(ADR_KI1BAT[sub_ind1]);
			if(but==butR)temp_SS++;
			else if(but==butR_)temp_SS+=2;
			else if(but==butL)temp_SS--;
			else if(but==butL_)temp_SS-=2;
						
			gran(&temp_SS,20,4000);
			//lc640_write_int(ADR_KI1BAT[sub_ind1],temp_SS);
			phase=1;
			}
				
		speed=1;			
		}					
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}	
		

else if(ind==iK_bps_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMIST);
		phase=0;
		//can1_out(sub_ind,sub_ind,CMND,ALRM_RES,0,0,0,0);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMIST);
		phase=0;
		//can1_out(sub_ind,sub_ind,CMND,ALRM_RES,0,0,0,0);
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMIST;
		}	
	else if((but==butE)&&(NUMIST)&&(sub_ind<NUMIST))
		{
		tree_up(iK_bps,0,0,sub_ind);	
		
		//can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);

		ret(1000);
		}	
	else if(sub_ind==(NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

		






else if(ind==iK_load)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind=1;
		}
	else if(but==butU)
		{
		sub_ind=0;
		}
	else if(sub_ind==0)
		{
		//temp_SS=lc640_read_int(EE_KULOAD);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
		#ifdef UKU_220
	    gran(&temp_SS,300,2000);
		#else 
		#ifdef UKU_220_IPS_TERMOKOMPENSAT
	    gran(&temp_SS,50,2000);
	    #else 
		gran(&temp_SS,100,5000);
		#endif
		#endif
		//lc640_write_int(EE_KULOAD,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==1)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_out)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2);
		}
	else if(sub_ind==0)
		{
		//temp_SS=lc640_read_int(EE_KUBPS);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	    gran(&temp_SS,50,2000);

		//lc640_write_int(EE_KUBPS,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==1)
		{
		//temp_SS=lc640_read_int(EE_KUOUT);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	    gran(&temp_SS,50,2000);

		//lc640_write_int(EE_KUOUT,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==2)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_t_ext)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
	else if(sub_ind==0)
		{
		//temp_SS=lc640_read_int(KT_EXT0);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		//lc640_write_int(KT_EXT0,temp_SS);					
		speed=1;	
					
		}

	else if(sub_ind==1)
		{
		//temp_SS=lc640_read_int(KT_EXT1);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		//lc640_write_int(KT_EXT1,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==2)
		{
		//temp_SS=lc640_read_int(KT_EXT2);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		//lc640_write_int(KT_EXT2,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_t_ext_6U)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMDT);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMDT);
		}
	else if(sub_ind==0)
		{
		//temp_SS=lc640_read_int(KT_EXT0);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		//lc640_write_int(KT_EXT0,temp_SS);					
		speed=1;	
					
		}

	else if(sub_ind==1)
		{
		//temp_SS=lc640_read_int(KT_EXT1);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		//lc640_write_int(KT_EXT1,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==2)
		{
		//temp_SS=lc640_read_int(KT_EXT2);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		//lc640_write_int(KT_EXT2,temp_SS);					
		speed=1;	
					
		}
 	if(sub_ind==NUMDT)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}
			
else if(ind==iBatLog)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,6);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,6);
		}
	else if(but==butD_)
		{
		sub_ind=6;
		}				
	else if((but==butL)&&((sub_ind==0)||(sub_ind==3)||(sub_ind==4)))
		{
		tree_down(0,0);
		}		
	else if(sub_ind==0)
	     {
	     if(but==butE)
	          {
	          //b[ptr_ind++]=a;
	          //if(BAT_IS_ON[sub_ind1]==bisON) ind=iPrl_bat_in_out;
	          //else 
	               //{
	               //ind=iPdp1;
	               //ret_ind(iPrl_bat_in_out,0,10);
	               //}
	          tree_up(iPrl_bat_in_out,0,0,sub_ind1);
	          if(BAT_IS_ON[sub_ind1]!=bisON) show_mess("  Введение батареи  ",
	          								 "    уничтожит все   ",
	          								 "   предшествующие   ",
	          								 "      данные!!!     ",4000);     
	          parol_init();
	          }
	     }
	else if(sub_ind==1)
	     {
	     if(but==butR)BAT_C_NOM[sub_ind1]++;
	     else if(but==butR_)BAT_C_NOM[sub_ind1]+=10;
	     else if(but==butL)BAT_C_NOM[sub_ind1]--;
	     else if(but==butL_)BAT_C_NOM[sub_ind1]-=10;
	     gran(&BAT_C_NOM[sub_ind1],0,2000);
	     //lc640_write_int(ADR_EE_BAT_C_NOM[sub_ind1],BAT_C_NOM[sub_ind1]);
	     speed=1;
	     }		     
		
	else if(sub_ind==3)
		{
		if(but==butE)
			{ 
               cap=0;
			//deep=lc640_read_int(CNT_EVENT_LOG);
			//ptr=lc640_read_int(PTR_EVENT_LOG);
			
			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			     			
			for(i=0;i<deep;i++)
				{
				//lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')&&(av_head[1]==sub_ind1)&&(av_head[2]=='K')) 	//ищем записи батарейных событий 'K'(контроли емкости)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
				ptr=ptr_carry(ptr,64,-1); 
	
				} 
				
			tree_up(iBatLogKe,0,0,sub_ind1);
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		}




	else if(sub_ind==4)
		{
		if(but==butE)
			{ 
               cap=0;
			//deep=lc640_read_int(CNT_EVENT_LOG);
			//ptr=lc640_read_int(PTR_EVENT_LOG);
			
			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			     
			for(i=0;i<deep;i++)
				{
				//lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')/*&&(av_head[1]==sub_ind1)*/&&(av_head[2]=='Z')) 	//ищем записи батарейных событий 'z'(выравнивающий заряд)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
				ptr=ptr_carry(ptr,64,-1); 
				} 
				
			tree_up(iBatLogVz,0,0,sub_ind1);   
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		else if(but==butR)
			{
			//vz_mem_hndl(0,5);
			//vz_mem_hndl(1,6);	       
			} 
		}

	else if(sub_ind==5)
		{
		if(but==butE)
			{ 
               cap=0;
			//deep=lc640_read_int(CNT_EVENT_LOG);
			//ptr=lc640_read_int(PTR_EVENT_LOG);

			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			
			//out_usart0 (8,0x11,*((char*)&deep),*(((char*)&deep)+1),*((char*)&ptr),*(((char*)&ptr)+1),cap,content[cap-1],i,0);
			
			for(i=0;i<deep;i++)
				{
				//lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')&&(av_head[1]==sub_ind1)&&(av_head[2]=='W')) 	//ищем записи батарейных событий 'W'(разряды)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
		   	/*   	out_usart0 (8,0x22,*((char*)&deep),*(((char*)&deep)+1),*((char*)&ptr),*(((char*)&ptr)+1),cap,content[cap-1],i,0); 
				delay_ms(100);
				PORTC.7=!PORTC.7;
				#asm("wdr"); 	*/
				ptr=ptr_carry(ptr,64,-1); 
				} 
				
			/*ind=iJ_bat_wrk_sel;
			sub_ind=0;*/

			tree_up(iBatLogWrk,0,0,sub_ind1);
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		else if(but==butR)
			{
			//vz_mem_hndl(0,5);
			//vz_mem_hndl(1,6);	       
			} 
		}		
		 	         	
     else if(sub_ind==6)
	     {
	     if(but==butE)
	          {
			if(BAT_IS_ON[sub_ind1]!=bisON)tree_down(0,-4);
	          else tree_down(0,0);
	          }
	     }		     
		
	} 

else if(ind==iBatLogVz)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
    //	else if(but==butR) vz_mem_hndl(sub_ind1,_sec);
	
		
	}

else if(ind==iBatLogKe)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
    //	else if(but==butR) ke_mem_hndl(sub_ind1,_sec);		
	}

else if(ind==iBatLogWrk)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		else if(sub_ind<=av_j_si_max)
			{
			//ind=iWrk;
			//sub_ind2=content[sub_ind];
			index_set=0;
			//sub_ind=0;
			}	
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
	else if(but==butR)
		{
	    //	wrk_mem_hndl(sub_ind1);

		} 
	//int2lcdyx(BAT_TYPE[sub_ind1],0,2,0);
	}



else if(ind==iAvz)
	{
	if(AVZ!=AVZ_OFF)
		{
		if(but==butU)
			{
			sub_ind--;
			if(sub_ind==3)sub_ind--;
			}
		else if(but==butD)
			{
			sub_ind++;
			if(sub_ind==3)sub_ind++;
			}
		else if(sub_ind==0)
			{
			if(but==butL)
				{
				if(AVZ==AVZ_1)AVZ=AVZ_OFF;
				else if(AVZ==AVZ_2)AVZ=AVZ_1;
				else if(AVZ==AVZ_3)AVZ=AVZ_2; 
				else if(AVZ==AVZ_6)AVZ=AVZ_3;
				else if(AVZ==AVZ_12)AVZ=AVZ_6;			
				else AVZ=AVZ_12;
				}
			else if((but==butR)||(but==butE))
				{
				if(AVZ==AVZ_1)AVZ=AVZ_2;
				else if(AVZ==AVZ_2)AVZ=AVZ_3;
				else if(AVZ==AVZ_3)AVZ=AVZ_6; 
				else if(AVZ==AVZ_6)AVZ=AVZ_12;
				else if(AVZ==AVZ_12)AVZ=AVZ_OFF;			
				else AVZ=AVZ_1;
				} 
			//lc640_write_int(EE_AVZ,AVZ);
			//if(AVZ!=AVZ_OFF)avz_next_date_hndl();		
			}      
		else if(sub_ind==1)
			{
			if((but==butR)||(but==butR_))
				{
				speed=1;
				AVZ_TIME++;
				} 
			else if((but==butL)||(but==butL_))
				{
				speed=1;
				AVZ_TIME--;
				}			
			gran((signed short*)&AVZ_TIME,1,24);
			//lc640_write_int(EE_AVZ_TIME,AVZ_TIME);
			}	
		else if(sub_ind==4)
			{
			if((but==butE))
				{
				ind=iSpc;
				sub_ind=1;
				}	
			}        
		gran_char(&sub_ind,0,4);						               
		} 
	else if(AVZ==AVZ_OFF)
		{
		if(but==butU)
			{
			sub_ind--;
			}
		else if(but==butD)
			{
			sub_ind++;
			}
		else if(sub_ind==0)
			{
			if(but==butL)
				{
				if(AVZ==AVZ_1)AVZ=AVZ_OFF;
				else if(AVZ==AVZ_2)AVZ=AVZ_1;
				else if(AVZ==AVZ_3)AVZ=AVZ_2; 
				else if(AVZ==AVZ_6)AVZ=AVZ_3;
				else if(AVZ==AVZ_12)AVZ=AVZ_6;			
				else AVZ=AVZ_12;
				}
			else if((but==butR)||(but==butE))
				{
				if(AVZ==AVZ_1)AVZ=AVZ_2;
				else if(AVZ==AVZ_2)AVZ=AVZ_3;
				else if(AVZ==AVZ_3)AVZ=AVZ_6; 
				else if(AVZ==AVZ_6)AVZ=AVZ_12;
				else if(AVZ==AVZ_12)AVZ=AVZ_OFF;			
				else AVZ=AVZ_1;
				}   
			//lc640_write_int(EE_AVZ,AVZ);
			//if(AVZ!=AVZ_OFF)avz_next_date_hndl();		
			}      
		else if(sub_ind==1)
			{
			if((but==butR)||(but==butR_))
				{
				speed=1;
				AVZ_TIME++;
				} 
			else if((but==butL)||(but==butL_))
				{
				speed=1;
				AVZ_TIME--;
				}			
			gran((signed short*)&AVZ_TIME,1,20);
			//lc640_write_int(EE_AVZ_TIME,AVZ_TIME);
			}	
		else if(sub_ind==2)
			{
			if((but==butE))
				{
				tree_down(0,0);
				}	
			}        
		gran_char(&sub_ind,0,2);						               
		} 
     }
		
else if(ind==iTst_RSTKM)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,18+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		if(sub_ind==9)
			{
               sub_ind=10;
			index_set=9;
			}
		if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}
		if(sub_ind==13)
			{
			sub_ind=14;
			index_set=13;
			}
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,18+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==13)
			{
			sub_ind=12;
			}		
		if(sub_ind==11)
			{
			sub_ind=10;
			}
		if(sub_ind==9)
			{
			sub_ind=8;
			}
		if(sub_ind==7)
			{
			sub_ind=6;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}		

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[9]==tstOFF) tst_state[9]=tst1;
			else if(tst_state[9]==tst1) tst_state[9]=tst2;
			else tst_state[9]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[9]==tst2) tst_state[9]=tst1;
			else if(tst_state[9]==tstOFF) tst_state[9]=tst2;
			else tst_state[9]=tstOFF;
			}
		}
		
	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}

	else if(sub_ind==8)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}
	else if(sub_ind==10)
		{
		if((but==butE)||(but==butR))
			{
			tst_state[10]++;
			if((tst_state[10]<(enum_tst_state)1)||(tst_state[10]>(enum_tst_state)20)) tst_state[10]=(enum_tst_state)20;
			}
		else if(but==butL)
			{
			tst_state[10]--;
			if((tst_state[10]<(enum_tst_state)1)||(tst_state[10]>(enum_tst_state)20)) tst_state[10]=(enum_tst_state)0;
			}
		}
	else if(sub_ind==12)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}
	else if(sub_ind==14)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[7]==tstOFF) tst_state[7]=tst1;
			else tst_state[7]=tstOFF;
			}
		}
	else if(sub_ind==15)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[8]==tstOFF) tst_state[8]=tst1;
			else tst_state[8]=tstOFF;
			}
		}
	else if((sub_ind>=16)&&(sub_ind<(16+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-16);
		//can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											
	else if(sub_ind==(16+NUMIST))
		{
		if(but==butE)
			{
			//bRESET=1;
			}
	
		}
	else if(sub_ind==(17+NUMIST))
		{
		if(but==butE)
			{
			//bRESET_EXT=1;
			}
	
		}			
	else if(sub_ind==(18+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
	
		}	
	}

else if(ind==iTst_KONTUR)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,18+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		if(sub_ind==9)
			{
               sub_ind=10;
			index_set=9;
			}
		if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}
		if(sub_ind==13)
			{
			sub_ind=14;
			index_set=13;
			}
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,18+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==13)
			{
			sub_ind=12;
			}		
		if(sub_ind==11)
			{
			sub_ind=10;
			}
		if(sub_ind==9)
			{
			sub_ind=8;
			}
		if(sub_ind==7)
			{
			sub_ind=6;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}		

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[9]==tstOFF) tst_state[9]=tst1;
			else if(tst_state[9]==tst1) tst_state[9]=tst2;
			else tst_state[9]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[9]==tst2) tst_state[9]=tst1;
			else if(tst_state[9]==tstOFF) tst_state[9]=tst2;
			else tst_state[9]=tstOFF;
			}
		}
		
	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}

	else if(sub_ind==8)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}
	else if(sub_ind==10)
		{
		if((but==butE)||(but==butR))
			{
			tst_state[10]++;
			if((tst_state[10]<(enum_tst_state)1)||(tst_state[10]>(enum_tst_state)20)) tst_state[10]=(enum_tst_state)20;
			}
		else if(but==butL)
			{
			tst_state[10]--;
			if((tst_state[10]<(enum_tst_state)1)||(tst_state[10]>(enum_tst_state)20)) tst_state[10]=(enum_tst_state)0;
			}
		}
	else if(sub_ind==12)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}
	else if(sub_ind==14)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[7]==tstOFF) tst_state[7]=tst1;
			else tst_state[7]=tstOFF;
			}
		}
	else if(sub_ind==15)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[8]==tstOFF) tst_state[8]=tst1;
			else tst_state[8]=tstOFF;
			}
		}
	else if((sub_ind>=16)&&(sub_ind<(16+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-16);
		//can2_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											
	else if(sub_ind==(16+NUMIST))
		{
		if(but==butE)
			{
			//bRESET=1;
			}
	
		}
	else if(sub_ind==(17+NUMIST))
		{
		if(but==butE)
			{
			//bRESET_EXT=1;
			}
	
		}			
	else if(sub_ind==(18+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
	
		}	
	}

else if(ind==iTst_3U)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,14+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		if(sub_ind==9)
			{
               sub_ind=10;
			//index_set=9;
			}
	/*	if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}*/
	/*	if(sub_ind==13)
			{
			sub_ind=14;
			index_set=13;
			}*/
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,18+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

	/*	if(sub_ind==13)
			{
			sub_ind=12;
			}*/		
	/*	if(sub_ind==11)
			{
			sub_ind=10;
			}*/
		if(sub_ind==9)
			{
			sub_ind=8;
			}
		if(sub_ind==7)
			{
			sub_ind=6;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}		

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}
		
	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}

	else if(sub_ind==8)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}

	else if(sub_ind==10)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[5]==tstOFF) tst_state[5]=tst1;
			else tst_state[5]=tstOFF;
			}
		}
	else if(sub_ind==11)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[6]==tstOFF) tst_state[6]=tst1;
			else tst_state[6]=tstOFF;
			}
		}
	else if((sub_ind>=12)&&(sub_ind<(12+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-13);
		//can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											
	else if(sub_ind==(12+NUMIST))
		{
		if(but==butE)
			{
			//bRESET=1;
			}
		}
			
	else if(sub_ind==(13+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
	
		}	
	}

else if(ind==iTst_GLONASS)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,13+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		if(sub_ind==9)
			{
               sub_ind=10;
			//index_set=9;
			}
	/*	if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}*/
	/*	if(sub_ind==13)
			{
			sub_ind=14;
			index_set=13;
			}*/
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,13+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

	/*	if(sub_ind==13)
			{
			sub_ind=12;
			}*/		
	/*	if(sub_ind==11)
			{
			sub_ind=10;
			}*/
		if(sub_ind==9)
			{
			sub_ind=8;
			}
		if(sub_ind==7)
			{
			sub_ind=6;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}		

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}
		
	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}

	else if(sub_ind==8)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}

	else if(sub_ind==10)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[5]==tstOFF) tst_state[5]=tst1;
			else tst_state[5]=tstOFF;
			}
		}
	else if(sub_ind==11)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[6]==tstOFF) tst_state[6]=tst1;
			else tst_state[6]=tstOFF;
			}
		}
/*	else if(sub_ind==12)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[7]==tstOFF) tst_state[7]=tst1;
			else tst_state[7]=tstOFF;
			}
		}*/
	else if((sub_ind>=12)&&(sub_ind<(12+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-12);
		//can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											
	else if(sub_ind==(12+NUMIST))
		{
		if(but==butE)
			{
			//bRESET=1;
			}
		}
			
	else if(sub_ind==(13+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
	
		}	
	}

else if(ind==iTst_6U)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,14+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			//index_set=7;
			}
		if(sub_ind==9)
			{
			index_set=8;
			}
		if(sub_ind==10)
			{
			sub_ind=11;
			index_set=10;
			}
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,14+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		
		if(sub_ind==10)
			{
			sub_ind=9;
			//index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=6;
			//index_set=5;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			//index_set=5;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			//index_set=5;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			//index_set=5;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}		

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[9]==tstOFF) tst_state[9]=tst1;
			else if(tst_state[9]==tst1) tst_state[9]=tst2;
			else tst_state[9]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[9]==tst2) tst_state[9]=tst1;
			else if(tst_state[9]==tstOFF) tst_state[9]=tst2;
			else tst_state[9]=tstOFF;
			}
		}
		
	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}

	else if(sub_ind==8)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}
	else if(sub_ind==9)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}
	else if(sub_ind==11)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[7]==tstOFF) tst_state[7]=tst1;
			else tst_state[7]=tstOFF;
			}
		}
	else if(sub_ind==12)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[8]==tstOFF) tst_state[8]=tst1;
			else tst_state[8]=tstOFF;
			}
		}
	else if((sub_ind>=13)&&(sub_ind<(13+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-13);
		//can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											
	else if(sub_ind==(13+NUMIST))
		{
		if(but==butE)
			{
			//bRESET=1;
			}
		}

	else if(sub_ind==(14+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
	
		}	
	}

else if((ind==iTst_220)||(ind==iTst_220_380))
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,13+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			//index_set=5;
			}
		if(sub_ind==7)
			{
			index_set=6;
			}
		if(sub_ind==8)
			{
			sub_ind=9;
			index_set=5;
			}
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,13+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		
		if(sub_ind==8)
			{
			sub_ind=7;
			//index_set=5;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			//index_set=5;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			//index_set=5;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			//index_set=5;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}		
		
	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}

	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}
	else if(sub_ind==7)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}
	else if(sub_ind==9)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[7]==tstOFF) tst_state[7]=tst1;
			else tst_state[7]=tstOFF;
			}
		}
	else if(sub_ind==10)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[8]==tstOFF) tst_state[8]=tst1;
			else tst_state[8]=tstOFF;
			}
		}
	else if((sub_ind>=11)&&(sub_ind<(11+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-11);
		//can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											

	else if(sub_ind==(11+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	else if(sub_ind==(12+NUMIST))
		{
		if(but==butE)
			{
			//bRESET_INT_WDT=1;
			}
		}
	else if(sub_ind==(13+NUMIST))
		{
		if(but==butE)
			{
			//bRESET_EXT_WDT=1;
			}
		}		
	}

else if(ind==iTst_220_IPS_TERMOKOMPENSAT)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,10+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,10+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		
		if(sub_ind==7)
			{
			sub_ind=6;
			index_set=4;
			} 
		if(sub_ind==5)
			{
			sub_ind=4;
			index_set=5;
			} 
		if(sub_ind==3)
			{
			sub_ind=2;
			//index_set=5;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			//index_set=5;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
		
		
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}
	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}
	else if((sub_ind>=8)&&(sub_ind<(8+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-8);
		//can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											

	else if(sub_ind==(8+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	else if(sub_ind==(9+NUMIST))
		{
		if(but==butE)
			{
		//	bRESET_INT_WDT=1;
			}
		}
	else if(sub_ind==(10+NUMIST))
		{
		if(but==butE)
			{
			//bRESET_EXT_WDT=1;
			}
		}					
	}

else if(ind==iTst_TELECORE2015)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,14+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		if(sub_ind==9)
			{
			sub_ind=10;
			index_set=9;
			}
		if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}
	/*	if(sub_ind==13)
			{
               sub_ind=14;
			//index_set=9;
			}*/
/*		if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}
		if(sub_ind==13)
			{
			sub_ind=14;
			index_set=13;
			}   */
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,14+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

/*		if(sub_ind==13)
			{
			sub_ind=12;
			}		
		if(sub_ind==11)
			{
			sub_ind=10;
			}*/
	/*	if(sub_ind==13)
			{
			sub_ind=12;
			}*/
		if(sub_ind==11)
			{
			sub_ind=10;
			}
		if(sub_ind==9)
			{
			sub_ind=8;
			}
		if(sub_ind==7)
			{
			sub_ind=6;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}

	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}

	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}

	else if(sub_ind==8)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[5]==tstOFF) tst_state[5]=tst1;
			else if(tst_state[5]==tst1) tst_state[5]=tst2;
			else tst_state[5]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[5]==tst2) tst_state[5]=tst1;
			else if(tst_state[5]==tstOFF) tst_state[5]=tst2;
			else tst_state[5]=tstOFF;
			}
		}

	else if(sub_ind==10)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else tst_state[2]=tstOFF;
			}
		}

	else if((sub_ind>=12)&&(sub_ind<(12+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-12);
		//can2_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											
	else if(sub_ind==(12+NUMIST))
		{
		if(but==butE)
			{
			//bRESET=1;
			}
	
		}
	else if(sub_ind==(13+NUMIST))
		{
		if(but==butE)
			{
			//bRESET_EXT=1;
			}
	
		}			
	else if(sub_ind==(14+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
	
		}	
	}

else if(ind==iTst_bps)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		
		if(sub_ind==2)
			{
			sub_ind=3;
			//index_set=2;
			}

		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		
		if(sub_ind==2)
			{
			sub_ind=1;
			//index_set=2;
			}
		}

	else if(sub_ind==0)
		{
		if(but==butR)
			{
			if(tst_state[5]==tstOFF)tst_state[5]=tst1;
			else if(tst_state[5]==tst1)tst_state[5]=tst2;
			else tst_state[5]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[5]==tstOFF)tst_state[5]=tst2;
			else if(tst_state[5]==tst1)tst_state[5]=tstOFF;
			else tst_state[5]=tst1;
			}
		}
	else if(sub_ind==1)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[6]==tstOFF) tst_state[6]=tst1;
			else if(tst_state[6]==tst1) tst_state[6]=tst2;
			else tst_state[6]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[6]==tst2) tst_state[6]=tst1;
			else if(tst_state[6]==tstOFF) tst_state[6]=tst2;
			else tst_state[6]=tstOFF;
			}
		}		
		
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}	
	}

else if(ind==iKlimat)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,7);
	
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,7);
		
		}
	else if(sub_ind==0)
	     {
	     if(but==butR)TBOXMAX++;
	     else if(but==butR_)TBOXMAX+=2;
	     else if(but==butL)TBOXMAX--;
	     else if(but==butL_)TBOXMAX-=2;
	     gran(&TBOXMAX,50,80);
	     //lc640_write_int(EE_TBOXMAX,TBOXMAX);
	     speed=1;
	     }

	else if(sub_ind==1)
	     {
	     if(but==butR)TBOXVENTMAX++;
	     else if(but==butR_)TBOXVENTMAX+=2;
	     else if(but==butL)TBOXVENTMAX--;
	     else if(but==butL_)TBOXVENTMAX-=2;
	     gran(&TBOXVENTMAX,49,81);
	     //lc640_write_int(EE_TBOXVENTMAX,TBOXVENTMAX);
	     speed=1;
	     }

	else if(sub_ind==2)
	     {
	     if(but==butR)TBOXREG++;
	     else if(but==butR_)TBOXREG+=2;
	     else if(but==butL)TBOXREG--;
	     else if(but==butL_)TBOXREG-=2;
	     gran(&TBOXREG,5,30);
	     //lc640_write_int(EE_TBOXREG,TBOXREG);
	     speed=1;
	     }

	else if(sub_ind==3)
	     {
	     if(but==butR)TLOADDISABLE++;
	     else if(but==butR_)TLOADDISABLE+=2;
	     else if(but==butL)TLOADDISABLE--;
	     else if(but==butL_)TLOADDISABLE-=2;
	     gran(&TLOADDISABLE,49,81);
	     //lc640_write_int(EE_TLOADDISABLE,TLOADDISABLE);
	     speed=1;
	     }

	else if(sub_ind==4)
	     {
	     if(but==butR)TLOADENABLE++;
	     else if(but==butR_)TLOADENABLE+=2;
	     else if(but==butL)TLOADENABLE--;
	     else if(but==butL_)TLOADENABLE-=2;
	     gran(&TLOADENABLE,44,TLOADDISABLE-5);
	     //lc640_write_int(EE_TLOADENABLE,TLOADENABLE);
	     speed=1;
	     }

	else if(sub_ind==5)
	     {
	     if(but==butR)TBATDISABLE++;
	     else if(but==butR_)TBATDISABLE+=2;
	     else if(but==butL)TBATDISABLE--;
	     else if(but==butL_)TBATDISABLE-=2;
	     gran(&TBATDISABLE,49,91);
	     //lc640_write_int(EE_TBATDISABLE,TBATDISABLE);
	     speed=1;
	     }

	else if(sub_ind==6)
	     {
	     if(but==butR)TBATENABLE++;
	     else if(but==butR_)TBATENABLE+=2;
	     else if(but==butL)TBATENABLE--;
	     else if(but==butL_)TBATENABLE-=2;
	     gran(&TBATENABLE,44,TBATDISABLE-5);
	     //lc640_write_int(EE_TBATENABLE,TBATENABLE);
	     speed=1;
	     }
	else if(sub_ind==7)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	}

else if(ind==iKlimat_kontur)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,9);
	
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,9);
		
		}
	else if(sub_ind==0)
	     {
	     if(but==butR)TBOXMAX++;
	     else if(but==butR_)TBOXMAX+=2;
	     else if(but==butL)TBOXMAX--;
	     else if(but==butL_)TBOXMAX-=2;
	     gran(&TBOXMAX,50,80);
	     //lc640_write_int(EE_TBOXMAX,TBOXMAX);
	     speed=1;
	     }

/*	else if(sub_ind==1)
	     {
	     if(but==butR)TBOXVENTON++;
	     else if(but==butR_)TBOXVENTON+=2;
	     else if(but==butL)TBOXVENTON--;
	     else if(but==butL_)TBOXVENTON-=2;
	     gran(&TBOXVENTON,TBOXVENTOFF+2,150);
	     //lc640_write_int(EE_TBOXVENTON,TBOXVENTON);
	     speed=1;
	     }

	else if(sub_ind==2)
	     {
	     if(but==butR)TBOXVENTOFF++;
	     else if(but==butR_)TBOXVENTOFF+=2;
	     else if(but==butL)TBOXVENTOFF--;
	     else if(but==butL_)TBOXVENTOFF-=2;
	     gran(&TBOXVENTOFF,TBOXWARMOFF+2,TBOXVENTON-2);
	     //lc640_write_int(EE_TBOXVENTOFF,TBOXVENTOFF);
	     speed=1;
	     }  */
	else if(sub_ind==1)
	     {
	     if(but==butR)TBOXVENTMAX++;
	     else if(but==butR_)TBOXVENTMAX+=2;
	     else if(but==butL)TBOXVENTMAX--;
	     else if(but==butL_)TBOXVENTMAX-=2;
	     gran(&TBOXVENTMAX,49,81);
	     //lc640_write_int(EE_TBOXVENTMAX,TBOXVENTMAX);
	     speed=1;
	     }

	else if(sub_ind==2)
	     {
	     if(but==butR)TBOXREG++;
	     else if(but==butR_)TBOXREG+=2;
	     else if(but==butL)TBOXREG--;
	     else if(but==butL_)TBOXREG-=2;
	     //gran(&TBOXREG,5,30);
		gran(&TBOXREG,0,50);
	     //lc640_write_int(EE_TBOXREG,TBOXREG);
	     speed=1;
	     }


	else if(sub_ind==3)
	     {
	     if(but==butR)TBOXWARMON++;
	     else if(but==butR_)TBOXWARMON+=2;
	     else if(but==butL)TBOXWARMON--;
	     else if(but==butL_)TBOXWARMON-=2;
	     //gran(&TBOXWARMON,-20,20);
		gran(&TBOXWARMON,-50,50);
	     //lc640_write_int(EE_TBOXWARMON,TBOXWARMON);
	     speed=1;
	     }

	else if(sub_ind==4)
	     {
	     if(but==butR)TBOXWARMOFF++;
	     else if(but==butR_)TBOXWARMOFF+=2;
	     else if(but==butL)TBOXWARMOFF--;
	     else if(but==butL_)TBOXWARMOFF-=2;
	     //gran(&TBOXWARMOFF,-20,20);
		gran(&TBOXWARMOFF,-50,50);
	     //lc640_write_int(EE_TBOXWARMOFF,TBOXWARMOFF);
	     speed=1;
	     }

	else if(sub_ind==5)
	     {
	     if(but==butR)TLOADDISABLE++;
	     else if(but==butR_)TLOADDISABLE+=2;
	     else if(but==butL)TLOADDISABLE--;
	     else if(but==butL_)TLOADDISABLE-=2;
	     gran(&TLOADDISABLE,49,81);
	     //lc640_write_int(EE_TLOADDISABLE,TLOADDISABLE);
	     speed=1;
	     }

	else if(sub_ind==6)
	     {
	     if(but==butR)TLOADENABLE++;
	     else if(but==butR_)TLOADENABLE+=2;
	     else if(but==butL)TLOADENABLE--;
	     else if(but==butL_)TLOADENABLE-=2;
	     gran(&TLOADENABLE,44,TLOADDISABLE-5);
	     //lc640_write_int(EE_TLOADENABLE,TLOADENABLE);
	     speed=1;
	     }

	else if(sub_ind==7)
	     {
	     if(but==butR)TBATDISABLE++;
	     else if(but==butR_)TBATDISABLE+=2;
	     else if(but==butL)TBATDISABLE--;
	     else if(but==butL_)TBATDISABLE-=2;
	     gran(&TBATDISABLE,49,91);
	     //lc640_write_int(EE_TBATDISABLE,TBATDISABLE);
	     speed=1;
	     }

	else if(sub_ind==8)
	     {
	     if(but==butR)TBATENABLE++;
	     else if(but==butR_)TBATENABLE+=2;
	     else if(but==butL)TBATENABLE--;
	     else if(but==butL_)TBATENABLE-=2;
	     gran(&TBATENABLE,44,TBATDISABLE-5);
	     //lc640_write_int(EE_TBATENABLE,TBATENABLE);
	     speed=1;
	     }
	else if(sub_ind==9)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	}

/* else if(ind==iKlimat_TELECORE2015)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==1)
			{
			sub_ind++;
			index_set=1;
			}
		gran_char(&sub_ind,0,11);
		if(sub_ind==3)sub_ind++;
		gran_char(&sub_ind,0,11);	
		}

	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==1)sub_ind--;
		gran_char(&sub_ind,0,11);
		if(sub_ind==3)sub_ind--;
		gran_char(&sub_ind,0,11);
		
		}
	else if(sub_ind==0)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_WARM_SIGNAL++;
	     else if(but==butR_)TELECORE2015_KLIMAT_WARM_SIGNAL++;
	     else if(but==butL)TELECORE2015_KLIMAT_WARM_SIGNAL--;
	     else if(but==butL_)TELECORE2015_KLIMAT_WARM_SIGNAL--;
	     gran_ring(&TELECORE2015_KLIMAT_WARM_SIGNAL,0,1);
	     //lc640_write_int(EE_TELECORE2015_KLIMAT_WARM_SIGNAL,TELECORE2015_KLIMAT_WARM_SIGNAL);
	     speed=1;
	     }
	else if(sub_ind==2)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_VENT_SIGNAL++;
	     else if(but==butR_)TELECORE2015_KLIMAT_VENT_SIGNAL++;
	     else if(but==butL)TELECORE2015_KLIMAT_VENT_SIGNAL--;
	     else if(but==butL_)TELECORE2015_KLIMAT_VENT_SIGNAL--;
	     gran_ring(&TELECORE2015_KLIMAT_VENT_SIGNAL,0,1);
	     //lc640_write_int(EE_TELECORE2015_KLIMAT_VENT_SIGNAL,TELECORE2015_KLIMAT_VENT_SIGNAL);
	     speed=1;
	     }
	else if(sub_ind==4)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_WARM_ON++;
	     else if(but==butR_)TELECORE2015_KLIMAT_WARM_ON+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_WARM_ON--;
	     else if(but==butL_)TELECORE2015_KLIMAT_WARM_ON-=2;
	     gran(&TELECORE2015_KLIMAT_WARM_ON,-20,50);
	     //lc640_write_int(EE_TELECORE2015_KLIMAT_WARM_ON,TELECORE2015_KLIMAT_WARM_ON);
	     speed=1;
	     }

	else if(sub_ind==5)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_WARM_OFF++;
	     else if(but==butR_)TELECORE2015_KLIMAT_WARM_OFF+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_WARM_OFF--;
	     else if(but==butL_)TELECORE2015_KLIMAT_WARM_OFF-=2;
	     gran(&TELECORE2015_KLIMAT_WARM_OFF,-20,50);
	     //lc640_write_int(EE_TELECORE2015_KLIMAT_WARM_OFF,TELECORE2015_KLIMAT_WARM_OFF);
	     speed=1;
	     }


	else if(sub_ind==6)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_CAP++;
	     else if(but==butR_)TELECORE2015_KLIMAT_CAP+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_CAP--;
	     else if(but==butL_)TELECORE2015_KLIMAT_CAP-=2;
	     //gran(&TBOXWARMON,-20,20);
		gran(&TELECORE2015_KLIMAT_CAP,5,95);
	     //lc640_write_int(EE_TELECORE2015_KLIMAT_CAP,TELECORE2015_KLIMAT_CAP);
	     speed=1;
	     }

	else if(sub_ind==7)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_VENT_ON++;
	     else if(but==butR_)TELECORE2015_KLIMAT_VENT_ON+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_VENT_ON--;
	     else if(but==butL_)TELECORE2015_KLIMAT_VENT_ON-=2;
	     gran(&TELECORE2015_KLIMAT_VENT_ON,0,80);
		gran(&TELECORE2015_KLIMAT_VENT_ON,TELECORE2015_KLIMAT_VENT_OFF,80);
	     //lc640_write_int(EE_TELECORE2015_KLIMAT_VENT_ON,TELECORE2015_KLIMAT_VENT_ON);
	     speed=1;
	     }

	else if(sub_ind==8)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_VENT_OFF++;
	     else if(but==butR_)TELECORE2015_KLIMAT_VENT_OFF+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_VENT_OFF--;
	     else if(but==butL_)TELECORE2015_KLIMAT_VENT_OFF-=2;
	     gran(&TELECORE2015_KLIMAT_VENT_OFF,0,80);
		gran(&TELECORE2015_KLIMAT_VENT_OFF,0,TELECORE2015_KLIMAT_VENT_OFF);
	     //lc640_write_int(EE_TELECORE2015_KLIMAT_VENT_OFF,TELECORE2015_KLIMAT_VENT_OFF);
	     speed=1;
	     }
	else if(sub_ind==9)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_VVENT_ON++;
	     else if(but==butR_)TELECORE2015_KLIMAT_VVENT_ON+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_VVENT_ON--;
	     else if(but==butL_)TELECORE2015_KLIMAT_VVENT_ON-=2;
	     gran(&TELECORE2015_KLIMAT_VVENT_ON,0,80);
		gran(&TELECORE2015_KLIMAT_VVENT_ON,TELECORE2015_KLIMAT_VVENT_OFF,80);
	     //lc640_write_int(EE_TELECORE2015_KLIMAT_VVENT_ON,TELECORE2015_KLIMAT_VVENT_ON);
	     speed=1;
	     }

	else if(sub_ind==10)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_VVENT_OFF++;
	     else if(but==butR_)TELECORE2015_KLIMAT_VVENT_OFF+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_VVENT_OFF--;
	     else if(but==butL_)TELECORE2015_KLIMAT_VVENT_OFF-=2;
	     gran(&TELECORE2015_KLIMAT_VVENT_OFF,0,80);
		gran(&TELECORE2015_KLIMAT_VVENT_OFF,0,TELECORE2015_KLIMAT_VVENT_OFF);
	     //lc640_write_int(EE_TELECORE2015_KLIMAT_VVENT_OFF,TELECORE2015_KLIMAT_VVENT_OFF);
	     speed=1;
	     }


	else if(sub_ind==11)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	}  */


else if(ind==iBps_list)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind1--;
		gran_char(&sub_ind1,0,NUMIST-2);
		}
		
	else if (but==butD)
		{
		sub_ind1++;
		gran_char(&sub_ind1,0,NUMIST-2);
		}

	else if (but==butD_)
		{
		sub_ind1=NUMIST-2;
		}
				
	else if(but==butR)
		{
		sub_ind=1;
		}
				
	else if(but==butL)
		{
		sub_ind=0;
		}
	else if(but==butE)
		{
		tree_down(0,0);
		}				
	}
else if(ind==iAvt_set_sel)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMIST);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMIST);
		}
	else if((sub_ind>=0)&&(sub_ind<NUMIST))
		{
		if(but==butE)
			{
			tree_up(iAvt_set,0,0,sub_ind);
			}
		}
	else if(sub_ind==NUMIST)
		{
		if(but==butE)
			{
			tree_down(0,0);
			}	
		}
	}
else if(ind==iAvt_set)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2))sub_ind=0;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}
 	else if (sub_ind == 0)
		{
		//temp_SS=lc640_read_int(EE_U_AVT);
		if(but==butR)temp_SS++;
		else if(but==butR_)temp_SS+=2;
		else if(but==butL)temp_SS--;
		else if(but==butL_)temp_SS-=2;
		//else if(but==butE_)can1_out(sub_ind1,sub_ind1,CMND,0xee,0xee,0,0,0);   
		
		#ifdef UKU206_220				
		gran(&temp_SS,1000,3000);
		#endif

		#ifdef UKU206_24
		gran(&temp_SS,200,300);
		#endif

		#ifdef UKU320
		gran(&temp_SS,400,800);
		#endif

		#ifdef UKU320_24
		gran(&temp_SS,200,300);
		#endif

		#ifdef UKU320_F
		gran(&temp_SS,400,800);
		#endif		
		//lc640_write_int(EE_U_AVT,temp_SS);
		
		speed=1;	
					
		}	
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			//sub_ind++;
			tree_down(0,1);
			ret(0);
			}
		}			

	}
else if(ind==iOut_volt_contr)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	else if(sub_ind==0)
		{
		if(but==butR)U_OUT_KONTR_MAX++;
		else if(but==butR_)U_OUT_KONTR_MAX=(U_OUT_KONTR_MAX/5+1)*5;
		else if(but==butL)U_OUT_KONTR_MAX--;
		else if(but==butL_)U_OUT_KONTR_MAX=(U_OUT_KONTR_MAX/5-1)*5;
		gran(&U_OUT_KONTR_MAX,10,3000);
		//lc640_write_int(EE_U_OUT_KONTR_MAX,U_OUT_KONTR_MAX);
		speed=1;
		}				

	else if(sub_ind==1)
		{
		if(but==butR)U_OUT_KONTR_MIN++;
		else if(but==butR_)U_OUT_KONTR_MIN=(U_OUT_KONTR_MIN/5+1)*5;
		else if(but==butL)U_OUT_KONTR_MIN--;
		else if(but==butL_)U_OUT_KONTR_MIN=(U_OUT_KONTR_MIN/5-1)*5;
		gran(&U_OUT_KONTR_MIN,10,3000);
		//lc640_write_int(EE_U_OUT_KONTR_MIN,U_OUT_KONTR_MIN);
		speed=1;
		}				


	else if(sub_ind==2)
		{
		if(but==butR)U_OUT_KONTR_DELAY++;
	    else if(but==butR_)U_OUT_KONTR_DELAY+=2;
	    else if(but==butL)U_OUT_KONTR_DELAY--;
	    else if(but==butL_)U_OUT_KONTR_DELAY-=2;
	    gran(&U_OUT_KONTR_DELAY,5,100);
	    //lc640_write_int(EE_U_OUT_KONTR_DELAY,U_OUT_KONTR_DELAY);
	    speed=1;
		}				


/*	
	int2lcd(U_BAT_KONTR_MAX,'!',1);
	int2lcd(U_BAT_KONTR_MIN,'@',1);
	int2lcd(U_BAT_KONTR_DELAY,'#',0);
	*/

	}

else if(ind==iDop_rele_set)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind+=2;
		gran_char(&sub_ind,0,2);
		}
	else if(but==butU)
		{
		sub_ind-=2;
		gran_char(&sub_ind,0,2);
		}
	else if(but==butD_)
		{
		sub_ind=2;
		}
	else if(sub_ind==2)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	else if(sub_ind==0)
		{
		if((but==butR)||(but==butR_))DOP_RELE_FUNC++;
		if((but==butL)||(but==butL_))DOP_RELE_FUNC--;
		gran(&DOP_RELE_FUNC,0,1);
		//lc640_write_int(EE_DOP_RELE_FUNC,DOP_RELE_FUNC);
		speed=1;
		}				
	}
else if(ind==iSet_li_bat)
	{
	ret(1000);
	if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,8);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,8);
		}
	else if(but==butE)
		{
		if(sub_ind==8)
			{
			tree_down(0,0);
          	ret(0);
			}
		}
	else if(sub_ind==0)
	     {
	     if(but==butR)LI_UNECC++;
	     else if(but==butR_)LI_UNECC+=10;
	     else if(but==butL)LI_UNECC--;
	     else if(but==butL_)LI_UNECC-=10;
	     gran(&LI_UNECC,420,540);
	     //lc640_write_int(EE_LI_UNECC,LI_UNECC);
	     speed=1;
	     }
	else if(sub_ind==1)
	     {
	     if(but==butR)LI_Q++;
	     else if(but==butR_)LI_Q++;
	     else if(but==butL)LI_Q--;
	     else if(but==butL_)LI_Q--;
	     gran(&LI_Q,10,100);
	     //lc640_write_int(EE_LI_Q,LI_Q);
	     speed=1;
	     }
	else if(sub_ind==2)
	     {
	     if(but==butR)LI_IZMAX1++;
	     else if(but==butR_)LI_IZMAX1+=10;
	     else if(but==butL)LI_IZMAX1--;
	     else if(but==butL_)LI_IZMAX1-=10;
	     gran(&LI_IZMAX1,5,500);
	     //lc640_write_int(EE_LI_IZMAX1,LI_IZMAX1);
	     speed=1;
	     }
	else if(sub_ind==3)
	     {
	     if(but==butR)LI_IZMAX2++;
	     else if(but==butR_)LI_IZMAX2+=10;
	     else if(but==butL)LI_IZMAX2--;
	     else if(but==butL_)LI_IZMAX2-=10;
	     gran(&LI_IZMAX2,5,500);
	     //lc640_write_int(EE_LI_IZMAX2,LI_IZMAX2);
	     speed=1;
	     }
	else if(sub_ind==4)
	     {
	     if(but==butR)LI_K1++;
	     else if(but==butR_)LI_K1++;
	     else if(but==butL)LI_K1--;
	     else if(but==butL_)LI_K1--;
	     gran(&LI_K1,1,30);
	     //lc640_write_int(EE_LI_K1,LI_K1);
	     speed=1;
	     }
	else if(sub_ind==5)
	     {
	     if(but==butR)LI_K2++;
	     else if(but==butR_)LI_K2++;
	     else if(but==butL)LI_K2--;
	     else if(but==butL_)LI_K2--;
	     gran(&LI_K2,1,10);
	     //lc640_write_int(EE_LI_K2,LI_K2);
	     speed=1;
	     }
	else if(sub_ind==6)
	     {
	     if(but==butR)LI_K3++;
	     else if(but==butR_)LI_K3++;
	     else if(but==butL)LI_K3--;
	     else if(but==butL_)LI_K3--;
	     gran(&LI_K3,1,10);
	     //lc640_write_int(EE_LI_K3,LI_K3);
	     speed=1;
	     }
	else if(sub_ind==7)
	     {
	     if(but==butR)LI_T4++;
	     else if(but==butR_)LI_T4++;
	     else if(but==butL)LI_T4--;
	     else if(but==butL_)LI_T4--;
	     gran(&LI_T4,1,10);
	     //lc640_write_int(EE_LI_T4,LI_T4);
	     speed=1;
	     }
	}
#endif		
but_an_end:
n_but=0;
}

//-----------------------------------------------
void watchdog_enable (void) 
{
LPC_WDT->WDTC=2000000;
LPC_WDT->WDCLKSEL=0;
LPC_WDT->WDMOD=3;
LPC_WDT->WDFEED=0xaa;
LPC_WDT->WDFEED=0x55;
}

//-----------------------------------------------
void watchdog_reset (void) 
{
LPC_WDT->WDFEED=0xaa;
LPC_WDT->WDFEED=0x55;
}


//***********************************************
//***********************************************
//***********************************************
//***********************************************
//***********************************************
void SysTick_Handler (void) 	 /* SysTick Interrupt Handler (1ms)    */
{
//sys_plazma++;
b2000Hz=1;

if(bTPS)
	{
	LPC_GPIO1->FIODIR|=(1UL<<26);
	LPC_GPIO1->FIOPIN^=(1UL<<26);
	}

if(++t0cnt4>=2)
	{
t0cnt4=0;
b1000Hz=1;

	bFF=(char)(GET_REG(LPC_GPIO0->FIOPIN, 27, 1));
	if(bFF!=bFF_) hz_out++;
	bFF_=bFF;


if(++t0cnt5>=20)
     {
     t0cnt5=0;
     b50Hz=1;
     }
     
if(++t0cnt>=10)
     {
     t0cnt=0;
     b100Hz=1;

     hz_out_cnt++;
     if(hz_out_cnt>=500)
	     {	
	     hz_out_cnt=0;
	     net_F=hz_out;
	     hz_out=0;
	     }

     if(++t0cnt0>=10)
	     {
	     t0cnt0=0;
	     b10Hz=1;
		beep_drv();
		if(main_10Hz_cnt<10000) main_10Hz_cnt++;
	     }

     if(t0cnt0==5)
	     {
		//beep_drv();
	     }

     if(++t0cnt1>=20)
	     {
	     t0cnt1=0;
	     b5Hz=1;
		if(bFL5)bFL5=0;
		else bFL5=1;     
	     }

     if(++t0cnt2>=50)
	     {
	     t0cnt2=0;
	     b2Hz=1;
		if(bFL2)bFL2=0;
		else bFL2=1;

	     }         

     if(++t0cnt3>=100)
	     {
	     t0cnt3=0;
	     b1Hz=1;
		if(main_1Hz_cnt<10000) main_1Hz_cnt++;
		if(bFL)bFL=0;
		else bFL=1;

		t0cntMin++;
		if(t0cntMin>=60)
			{
			t0cntMin=0;
			b1min=1;
			}
	     }
     }

	}




//LPC_GPIO0->FIOCLR|=0x00000001;
  return;          



//LPC_GPIO0->FIOCLR|=0x00000001;
}


//***********************************************
__irq void timer0_interrupt(void) 
{	
/*if(BPS1_spa_leave)T0EMR_bit.EM1=0; 
else T0EMR_bit.EM1=1;
if(BPS2_spa_leave)T0EMR_bit.EM3=0; 
else T0EMR_bit.EM3=1;
T0IR = 0xff;*/
}

//===============================================
//===============================================
//===============================================
//===============================================
int main (void) 
{
char ind_reset_cnt=0;
//long i;
char mac_adr[6] = { 0x00,0x73,0x04,50,60,70 };

//i=200000;
//while(--i){};

SystemInit();

bTPS=1;

SysTick->LOAD = (SystemFrequency / 2000) - 1;
SysTick->CTRL = 0x07;

//init_timer( 0,SystemFrequency/2000/4 - 1 ); // 1ms	
//enable_timer( 0 );

//rs232_data_out_1();

bps[0]._state=bsOFF_AV_NET;
bps[1]._state=bsOFF_AV_NET;
bps[2]._state=bsOFF_AV_NET;
bps[3]._state=bsOFF_AV_NET;
bps[4]._state=bsOFF_AV_NET;
bps[5]._state=bsOFF_AV_NET;
bps[6]._state=bsOFF_AV_NET;

SET_REG(LPC_GPIO0->FIODIR, 0, 27, 1);
SET_REG(LPC_GPIO2->FIODIR, 1, 7, 1);
SET_REG(LPC_GPIO2->FIODIR, 1, 8, 1);
//LPC_GPIO1->FIODIR  |= 1<<27;                
	;
//FIO1MASK = 0x00000000;	 
//LPC_GPIO0->FIODIR  |= 1<<27;
//LPC_GPIO0->FIOSET  |= 1<<27;

///SET_REG(LPC_GPIO0->FIODIR,0,10,1); //вход частоты 
#ifdef UKU2071x
SET_REG(LPC_GPIO3->FIODIR,1,SHIFT_REL_AV_NET,1);
SET_REG(LPC_GPIO3->FIOCLR,1,SHIFT_REL_AV_NET,1);  // реле аварии сети под ток
#else 
SET_REG(LPC_GPIO3->FIODIR,1,SHIFT_REL_AV_NET,1);
SET_REG(LPC_GPIO3->FIOSET,1,SHIFT_REL_AV_NET,1);  // реле аварии сети под ток
#endif

/*

ad7705_reset();
delay_ms(20);

ad7705_write(0x21);
ad7705_write(BIN8(1101)); 
ad7705_write(0x11);
ad7705_write(0x44);


ad7705_buff[0][1]=0x7fff;
ad7705_buff[0][2]=0x7fff;
ad7705_buff[0][3]=0x7fff;
ad7705_buff[0][4]=0x7fff;
ad7705_buff[0][5]=0x7fff;
ad7705_buff[0][6]=0x7fff;
ad7705_buff[0][7]=0x7fff;
ad7705_buff[0][8]=0x7fff;
ad7705_buff[0][9]=0x7fff;
ad7705_buff[0][10]=0x7fff;
ad7705_buff[0][11]=0x7fff;
ad7705_buff[0][12]=0x7fff;
ad7705_buff[0][13]=0x7fff;
ad7705_buff[0][14]=0x7fff;
ad7705_buff[0][15]=0x7fff;
ad7705_buff[1][1]=0x7fff;
ad7705_buff[1][2]=0x7fff;
ad7705_buff[1][3]=0x7fff;
ad7705_buff[1][4]=0x7fff;
ad7705_buff[1][5]=0x7fff;
ad7705_buff[1][6]=0x7fff;
ad7705_buff[1][7]=0x7fff;
ad7705_buff[1][8]=0x7fff;
ad7705_buff[1][9]=0x7fff;
ad7705_buff[1][10]=0x7fff;
ad7705_buff[1][11]=0x7fff;
ad7705_buff[1][12]=0x7fff;
ad7705_buff[1][13]=0x7fff;
ad7705_buff[1][14]=0x7fff;
ad7705_buff[1][15]=0x7fff;

ad7705_buff_[0]=0x7fff;
ad7705_buff_[1]=0x7fff;

/*
ad7705_reset();
delay_ms(20);

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44);

ad7705_reset();
delay_ms(20);  

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44); 

delay_ms(20); */




lcd_init();  
lcd_on();
lcd_clear();
		
///LPC_GPIO4->FIODIR |= (1<<29);           /* LEDs on PORT2 defined as Output    */
rtc_init();
///pwm_init();




//snmp_plazma=15;


//#ifdef ETHISON
//mac_adr[5]=*((char*)&AUSW_MAIN_NUMBER);
//mac_adr[4]=*(((char*)&AUSW_MAIN_NUMBER)+1);
//mac_adr[3]=*(((char*)&AUSW_MAIN_NUMBER)+2);
//mem_copy (own_hw_adr, mac_adr, 6);


//if(//lc640_read_int(EE_ETH_IS_ON)==1)
	//{
	bgnd_par(		"                    ",
     		"    Инициализация   ",
     		"      Ethernet      ",
     		"                    ");
	//bitmap_hndl();
	//lcd_out(lcd_bitmap);
	//init_TcpNet ();

	//init_ETH();
	//mem_copy (&localm[NETIF_ETH], &ip_config, sizeof(ip_config));

//	}
//#endif
//event2snmp(2);

//reload_hndl();
//LPC_GPIO0->FIODIR |= (0x60000000);

//adc_init();

LPC_GPIO0->FIODIR|=(1<<11);
LPC_GPIO0->FIOSET|=(1<<11);


//lc640_write_int(100,134);





//memo_read();




/*
mac_adr[5]=*((char*)&AUSW_MAIN_NUMBER);
mac_adr[4]=*(((char*)&AUSW_MAIN_NUMBER)+1);
mac_adr[3]=*(((char*)&AUSW_MAIN_NUMBER)+2);
mem_copy (own_hw_adr, mac_adr, 6);

snmp_Community[0]=(char)//lc640_read_int(EE_COMMUNITY); 
//if((snmp_Community[0]==0)||(snmp_Community[0]==' '))snmp_Community[0]=0;
snmp_Community[1]=(char)//lc640_read_int(EE_COMMUNITY+2);
if((snmp_Community[1]==0)||(snmp_Community[1]==' '))snmp_Community[1]=0;
snmp_Community[2]=(char)//lc640_read_int(EE_COMMUNITY+4);
if((snmp_Community[2]==0)||(snmp_Community[2]==' '))snmp_Community[2]=0;
snmp_Community[3]=(char)//lc640_read_int(EE_COMMUNITY+6);
if((snmp_Community[3]==0)||(snmp_Community[3]==' '))snmp_Community[3]=0;
snmp_Community[4]=(char)//lc640_read_int(EE_COMMUNITY+8);
if((snmp_Community[4]==0)||(snmp_Community[4]==' '))snmp_Community[4]=0;
snmp_Community[5]=(char)//lc640_read_int(EE_COMMUNITY+10);
if((snmp_Community[5]==0)||(snmp_Community[5]==' '))snmp_Community[5]=0;
snmp_Community[6]=(char)//lc640_read_int(EE_COMMUNITY+12);
if((snmp_Community[6]==0)||(snmp_Community[6]==' '))snmp_Community[6]=0;
snmp_Community[7]=(char)//lc640_read_int(EE_COMMUNITY+14);
if((snmp_Community[7]==0)||(snmp_Community[7]==' '))snmp_Community[7]=0;
snmp_Community[8]=(char)//lc640_read_int(EE_COMMUNITY+16);
if((snmp_Community[8]==0)||(snmp_Community[8]==' '))snmp_Community[8]=0;
snmp_Community[9]=0; 

if(//lc640_read_int(EE_ETH_IS_ON)==1)
	{
	bgnd_par(		"                    ",
     		"    Инициализация   ",
     		"      Ethernet      ",
     		"                    ");
	bitmap_hndl();
	lcd_out(lcd_bitmap);
	init_TcpNet ();
	lcd_out(lcd_bitmap);
	init_ETH();
	//mem_copy (&localm[NETIF_ETH], &ip_config, sizeof(ip_config));
//	lcd_out(lcd_bitmap);
	} */
//sys_plazma1=sys_plazma;
ind_reset_cnt=58;


//watchdog_enable();






ind=iMn_IP55;
		
while (1)  
	{
	bTPS=0; 
     //timer_poll ();
     main_TcpNet ();

	//watchdog_reset();





/*	if(bRXIN_SC16IS700) 
		{
		bRXIN_SC16IS700=0;
	
		uart_in_SC16IS700();
		}*/

	/*
	if(bRXIN1) 
		{
		bRXIN1=0;
	
		uart_in1();
		}*/ 
     if(b10000Hz)
		{
		b10000Hz=0; 
		

		}

     if(b2000Hz)
		{

		b2000Hz=0; 
		
		}

	if(b1000Hz)
		{
		b1000Hz=0;


		
						
		}
	
	if(b100Hz)
		{
		b100Hz=0;

		//LPC_GPIO2->FIODIR|=(1<<7);
		//LPC_GPIO2->FIOPIN^=(1<<7);		

		//if((!bRESET_INT_WDT)&&(!bRESET_EXT_WDT))but_drv();
		but_an();
		}
		 
	if(b50Hz)
		{
		b50Hz=0;

		}

	if(b10Hz)
		{
		char i;

     timer_tick ();
     tick = __TRUE;

		b10Hz=0;
				
	//	u_necc_hndl();
		
		//for(i=0;i<NUMIST+2;i++)bps_drv(i);
	//	bps_hndl();

		//inv_search();
		
		//if(NUMINV) {for(i=0;i<NUMINV;i++)inv_drv(i);}		  
		
		//nv[0]._Uii=123;


		//if(BAT_IS_ON[0]==bisON)bat_drv(0);
		//if(BAT_IS_ON[1]==bisON)bat_drv(1);
		//bat_hndl();

				
		//unet_drv();

		
		
		ind_hndl(); 
		#ifndef SIMULATOR
		bitmap_hndl();
		//if(!bRESET_EXT_WDT)
			{
			lcd_out(lcd_bitmap);
			}
		#endif
		//ad7705_drv();
		//ad7705_write(0x20);


		ret_hndl();  
		mess_hndl();

	//	cntrl_hndl_ip55();

		ret_hndl();
		//ext_drv();
		
		}

	if(b5Hz)
		{
		b5Hz=0;

	//	if(!bRESET_EXT_WDT)
			{
			//ad7705_drv();
			}
		//if(!bRESET_EXT_WDT)
			{
			//memo_read();
			}
		//LPC_GPIO1->FIODIR|=(1UL<<26);
		//matemat();
		
		//rele_hndl();
		//if(!bRESET_EXT_WDT)avar_hndl();
		//zar_superviser_drv();
		//snmp_data();
		//LPC_GPIO1->FIODIR|=(1UL<<31);
		//LPC_GPIO1->FIOPIN^=(1UL<<31);


  		}

	if(b2Hz)
		{
		b2Hz=0;

				//uart_out_adr1(dig,150);
		//sc16is700_wr_buff(CS16IS7xx_THR, 20);

		//sc16is700_wr_byte(CS16IS7xx_LCR, 0x80);

		//putchar_sc16is700(0x55);
  		}

	if(b1Hz)
		{
		b1Hz=0;
		//if(!bRESET_INT_WDT)
			{
			//watchdog_reset();
			}
		//can1_out_adr((char*)&net_U,21);

		//samokalibr_hndl();
		//num_necc_hndl();
		//zar_drv();
		//ubat_old_drv();
		//kb_hndl();
		beep_hndl();
		//avg_hndl();
		//vz_drv();	 
		//avz_drv();
		//ke_drv();
		//mnemo_hndl();
		//vent_hndl();

		//plazma_plazma_plazma++;

		if(++ind_reset_cnt>=60)
			{
			ind_reset_cnt=0;
			lcd_init();
			lcd_on();
			lcd_clear();
			}
               
          //vent_hndl();


		//if(t_ext_can_nd<10) t_ext_can_nd++;
		
		//if(main_1Hz_cnt<200)main_1Hz_cnt++;

		//#ifdef UKU_TELECORE2015

	/*	if(BAT_TYPE==2)sacred_san_bat_hndl();
		else if(BAT_TYPE==3)
			{*/
		/*	#ifdef UKU_TELECORE2016 
			ztt_bat_hndl_can();
			#else
			ztt_bat_hndl();
			#endif */
		//	}
		/*#endif*/

		//stark_bat_hndl();

		//can_reset_hndl();
		//npn_hndl();
/*		#ifdef UKU_220_IPS_TERMOKOMPENSAT
		if((AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043))rs232_data_out_tki();
		else if(AUSW_MAIN==22010)rs232_data_out_1();
		else rs232_data_out();
		#endif */	
 
		//modbus_registers_transmit(MODBUS_ADRESS,4,0,5);
		
	/*	putchar2(0x56);
		putchar2(0x57);
		putchar2(0x58);
		putchar2(0x59);
		putchar2(0x5a);*/

		//powerAntiAliasingHndl();

		//outVoltContrHndl();
		
		}
	if(b1min)
		{
		b1min=0;

		}
	}
}
