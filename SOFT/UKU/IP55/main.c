//������� �����
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
#include "FT_DataTypes.h"
#include "FT_GPU.h"
#include "FT_GPU_Hal.h"
#include "FT_CoPro_Cmds.h"
#include "riverdi.h"

extern U8 own_hw_adr[];
extern U8  snmp_Community[];
BOOL tick;
extern LOCALM localm[];
#define MY_IP localm[NETIF_ETH].IpAdr
#define DHCP_TOUT   50

//***********************************************
//������
char b10000Hz,b1000Hz,b2000Hz,b100Hz,b50Hz,b10Hz,b5Hz,b2Hz,b1Hz,b1min;
short t0cnt,t0cnt0,t0cnt1,t0cnt2,t0cnt3,t0cnt4,t0cnt5,t0cnt6,t0_cnt7,t0_cnt_min,t0cntMin;
char bFL5,bFL2,bFL,bFL_,bTPS;
signed short main_10Hz_cnt=0;
signed short main_1Hz_cnt=0;

char ind_reset_cnt=0; 
unsigned short slider_pos=0;
unsigned short toggle_pos=0;
unsigned short toggle_del=0;
unsigned short dial_pos=0;
//***********************************************
//��������� �����
char cnt_of_slave=3;
//char cnt_of_wrks;   //����������� ���������� ���������� , ��� ���������




//***********************************************
//��������� �������
BAT_STAT bat[2],bat_ips;
signed short		bat_u_old_cnt;
signed short 		Ib_ips_termokompensat;

//***********************************************
//�������� ���
MAKB_STAT makb[4];

//***********************************************
//�������� ���
LAKB_STAT lakb[3];
char lakb_damp[1][42];
char bLAKB_KONF_CH=0;
char bLAKB_KONF_CH_old=0;
char lakb_ison_mass[7];
short lakb_mn_ind_cnt;
char bLAKB_KONF_CH_EN;
//char bRS485ERR;
short LBAT_STRUKT;
char lakb_error_cnt;		//������� ������������� ��������� ����������� �������
short numOfPacks,numOfPacks_;
short numOfCells, numOfTemperCells, baseOfData;
short lakb_stat_comm_error;	//����������� ������ ����� � ��������� ���������. 0 �������� ����������� ����� ���������� � ������� ����� �� ����� ��������� ���������
short lakbNotErrorNum;		//����������� �������� ������� � ��������� ������
short lakbKanErrorCnt;		//������� ����������� ������ ����� � ������ ����������
short lakbKanErrorStat;		//��������� ����������� ������ ����� � ������ ����������

//#ifdef UKU_TELECORE2015
//***********************************************
//��������� �������� �������  
LI_BAT_STAT li_bat;
//#endif

//***********************************************
//���������� �� ���������� ����
char can_slot[12][16];


//***********************************************
//��������� ����������
BPS_STAT bps[32];

//***********************************************
//��������� ����������
#ifdef UKU_220_V2
INV_STAT inv[3];
#endif
#ifndef UKU_220_V2
INV_STAT inv[20];
#endif
char first_inv_slot=MINIM_INV_ADRESS;

//***********************************************
//��������� �������
BYPS_STAT byps;

//***********************************************
//��������� ��������
signed short load_U;
signed short load_I;

//***********************************************
//��������� ������
signed short bps_U;
signed short out_U;
signed short bps_I;


//***********************************************
//���������

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
const char sm_mont[13][4]={"   ","���","���","���","���","���","���","���","���","���","���","���","���"}; //
signed short ptr_ind=0;

signed short ind_pointer=0;

//***********************************************
//��������� ��������� ����
signed short net_U,net_Ustore,net_Ua,net_Ub,net_Uc;
char bFF,bFF_;
signed short net_F,hz_out,hz_out_cnt,net_F3;
signed char unet_drv_cnt;
char net_av;

//***********************************************
//��������� ������� ��������
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
//�����
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
//���
unsigned main_apv_cnt,hour_apv_cnt[2],reset_apv_cnt[2];
unsigned short apv_cnt_sec[2],apv_cnt[2];

//***********************************************
//��������� ���������
const char sm_[]	={"                    "};
const char sm_exit[]={" �����              "};
const char sm_time[]={" 0%:0^:0& 0</>  /0{ "};





//**********************************************
//������ � �������� 
char but;                            
unsigned long but_n,but_s;
char but0_cnt;
char but1_cnt;
char but_onL_temp;

//***********************************************
//����������� �����
char cnt_net_drv;







//***********************************************
//������ � ��������
char speed,l_but,n_but;

//***********************************************
//�������������
enum {wrkON=0x55,wrkOFF=0xAA}wrk;
char cnt_wrk;
signed short ibat_integr;
unsigned short av_beep,av_rele,av_stat;
char default_temp;
char ND_out[3];

//***********************************************
//����
enum_tst_state tst_state[15];

//***********************************************
//���
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
//���������������
extern signed short samokalibr_cnt;

//**********************************************
//���������
extern char mess[MESS_DEEP],mess_old[MESS_DEEP],mess_cnt[MESS_DEEP];
extern short mess_par0[MESS_DEEP],mess_par1[MESS_DEEP],mess_data[2];












//-----------------------------------------------
//�������������� � �����������
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
//��������� �������������� ��������� �������� 
enum_avt_stat avt_stat[12],avt_stat_old[12];

//short sys_plazma,sys_plazma1;

char snmp_plazma;


short plazma_but_an;

char bCAN_OFF;


char max_net_slot;

//-----------------------------------------------
//��������� ��� �� ����� ��������� ���� �������
signed long ibat_metr_buff_[2];
short bIBAT_SMKLBR;


//-----------------------------------------------
//�������������� TELECORE2015	
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
//���������� ����������������� ���������
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
//���������� �����
signed short speedChrgCurr;			//������������ ��� ����������� ������, ����������� �� ������
signed short speedChrgVolt;			//������������ ���������� ����������� ������, ����������� �� ������
signed short speedChrgTimeInHour; 		//������������ ����� ����������� ������ � �����, ����������� �� ������
signed short speedChrgAvtEn;	    		//�������������� ��������� ����������� ������ ��������/���������
signed short speedChrgDU;	    		//�������� ���������� ����������� ��� ��������� ����������� ������
signed short speedChIsOn;			//������� ��������� ����������� ������ ���/����
signed long  speedChTimeCnt;			//������� ������� ������ ����������� ������
signed short speedChrgBlckSrc;		//�������� ������� ����������, 0-����., 1-��1, 2-��2
signed short speedChrgBlckLog;		//������ ������� ����������, 1 - ���������� �� ���������� ��, 0 - �� ������������
signed short speedChrgBlckStat;		//������ ���������� ��� �������������� � ����������� ������.
char  	   speedChrgShowCnt;		//������� ������ ��������������� ���������

//-----------------------------------------------
//�������� ���������� ������� 	
//signed short TELECORE2017_USTART;		//���������� ���������
//signed short TELECORE2017_ULINECC;		//���������� ���������� �� ���������
signed short LI_UNECC_;					//���������� ���������� ����������, � ������ ������ �������
signed short LI_AVAR_CNT;				//������� ����������� ������� ��� �������� ���������� ����������

//signed short TELECORE2017_AVAR_CNT;		//������� ����������� ������� ��� �������� ���������� ����������
//signed short TELECORE2017_Q;			//����� ������ (%) ��� ������� ��������� � ���� IZMAX1 �� IZMAX2
//signed short TELECORE2017_IZMAX1;		//������������ ��� ������ ������� ��� ����������� �������(����� < TELECORE2017_Q)
//signed short TELECORE2017_IZMAX2;	   	//������������ ��� ������ ������� ��� ���������� �������(����� >= TELECORE2017_Q)
//signed short TELECORE2017_K1;			//��� �������������(��/�) ��� U����<(U���-2�)
//signed short TELECORE2017_K2;			//��� �������������(��/�) ��� U����>(U���-2�) � ���������� ����� �������
//signed short TELECORE2017_K3;			//��� �������������(��/�) ��� ���� ������� � ��������� 0-70% �� Izmax
//signed short TELECORE2017_T4;			//������ �������������(���) ���������� ������ ��� ���� �������� � ��������� 70-110%�� Izmax 
//#endif 
//-----------------------------------------------
//���������� ����������������� ���������

//-----------------------------------------------
//���������� ���
signed short ipsBlckSrc;
signed short ipsBlckLog;
signed short ipsBlckStat;


//-----------------------------------------------
//�������� ��������� ����������
signed short outVoltContrHndlCnt;		//�������, ������� � ���� � ������ ���������� ������� ������
signed short outVoltContrHndlCnt_;		//�������, ������� � ���� � ������ ���������� ���������� ������� ������
char uout_av;


short plazma_numOfCells;
short plazma_numOfTemperCells;
short plazma_numOfPacks;




char plazma_ztt[2];
short ft_plazma;

Ft_Gpu_Hal_Context_t host,*phost;

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

		ptrs[0]	=	"  � ������    r���. ";


	 
     i=0;
 	
 	ptrs[1]="U�z=   ]� I�z=    @�";
    ptrs[2]="U�=    #� I�=     $�";
    ptrs[3]=" 0%:0^:0& 0</>  /0{ ";
     
 	ptrs[4]=										" �������            ";
  //  ptrs[5]=										" ������� N2         ";
    

    if(sub_ind==0)index_set=0;
	else if((index_set-sub_ind)>2)index_set=sub_ind+2;
	else if(sub_ind>index_set)index_set=sub_ind;
	
	//if(sub_cnt<5)bgnd_par(ptrs[0],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
	//else bgnd_par(sub_ptrs[sub_cnt-5],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
	
	bgnd_par("                    ","                    ","                    ","                    ");

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
		
/*	if((AUSW_MAIN==2400)||(AUSW_MAIN==4800)||(AUSW_MAIN==6000)||
		(AUSW_MAIN==2403)||(AUSW_MAIN==4803)||(AUSW_MAIN==6003))sub_bgnd("                    ",'z',-2);
	else*/
	 



 
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
	
	  
	 
	int2lcdyx(ft_plazma,0,19,0);
	//int2lcdyx(coslightBatteryInBuffPtr_plazma,0,7,0);
	int2lcdyx(ind_reset_cnt,0,9,0);
/*	long2lcdhyx(rd32(0x300000),1,8);
	long2lcdhyx(rd32(0x300004),2,8);
	long2lcdhyx(rd32(0x300008),1,18);
	long2lcdhyx(rd32(0x30000c),2,18);
	int2lcdyx(rd8(0x0c0001),0,19,0); */
	int2lcdyx(sub_ind,0,1,0); 
//	long2lcdhyx(rd32(REG_CTOUCH_TOUCH0_XY),1,8);
//	long2lcdhyx(rd32(REG_CTOUCH_TOUCH1_XY),2,8);
//	long2lcdhyx(Ft_Gpu_Hal_Rd16(phost,REG_CMD_READ)/*rd32(REG_CTOUCH_TOUCH2_XY)*/,3,8);

/*	long2lcdhyx(rd32(RAM_CMD),1,8);
	long2lcdhyx(rd32(RAM_CMD+4),2,8);
	long2lcdhyx(rd32(RAM_CMD+8),3,8);
	//long2lcdhyx(rd32(REG_CTOUCH_TOUCH1_XY),2,8);

//	long2lcdhyx(rd32(REG_TOUCH_MODE),1,18);
//	long2lcdhyx(rd32(REG_TOUCH_ADC_MODE),2,18);

	long2lcdhyx(rd32(REG_CMD_WRITE),1,18);
	long2lcdhyx(rd32(REG_CMD_READ),2,18);

	long2lcdhyx(rd32(REG_CMD_DL),3,18);*/

	int2lcdyx(rd32(REG_TAG),1,9,0);
	int2lcdyx(rd32(REG_TAG_X),1,14,0);
	int2lcdyx(rd32(REG_TAG_Y),1,19,0);
	int2lcdyx(rd32(REG_TOUCH_TAG),2,9,0);
	long2lcdhyx(rd32(REG_TOUCH_TAG_XY),2,19);
	long2lcdhyx(rd32(REG_TOUCH_SCREEN_XY),3,19);
	long2lcdhyx(rd32(REG_TRACKER),3,9);
	if((rd32(REG_TRACKER)&0x000000ff)==1)slider_pos=/*(unsigned short)*/(((unsigned short)(rd32(REG_TRACKER)>>16/*UL*/))/660);
	if(toggle_del)toggle_del--;
	if(rd32(REG_TOUCH_TAG)==2)
		{
		if(!toggle_del)
			{
			if(toggle_pos==0)toggle_pos=65535;
			else toggle_pos=0;
			toggle_del=10;
			}
		}
	if((rd32(REG_TRACKER)&0x000000ff)==5)dial_pos=(unsigned short)((rd32(REG_TRACKER)>>16));

	if(sub_ind)
		{
		//Ft_Gpu_CoCmd_Dlstart(phost); 
		//Ft_Gpu_Hal_WrCmd32(phost, CLEAR(1, 1, 1));//Ft_Gpu_Cmd(phost,CLEAR(1,1,1)); 
		//Ft_Gpu_CoCmd_Text(phost,80, 30, 27,OPT_CENTER, "Please tap on the dot");
		//Ft_Gpu_CoCmd_Swap(phost);
		}

		{
		Ft_Gpu_CoCmd_Dlstart(phost);
		Ft_Gpu_Hal_WrCmd32(phost,CLEAR_COLOR_RGB(0, 0, 200));
		Ft_Gpu_Hal_WrCmd32(phost, CLEAR(1, 1, 1));
		Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 0, 0));
		Ft_Gpu_Hal_WrCmd32(phost,POINT_SIZE(10*16));
		Ft_Gpu_Hal_WrCmd32(phost,BEGIN(POINTS));
		Ft_Gpu_Hal_WrCmd32(phost,VERTEX2F(500 * 16, 300 * 16));
		Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		Ft_Gpu_CoCmd_FgColor(phost,0x00ff00UL);
		Ft_Gpu_CoCmd_BgColor(phost,0xff00ffUL);
		//Ft_Gpu_CoCmd_Number(phost,150, 20, 31, OPT_RIGHTX | 4, slider_pos);
		Ft_Gpu_Hal_WrCmd32(phost,TAG(1));
		Ft_Gpu_CoCmd_Slider(phost,200,180, 10, 270,0, slider_pos, 100);
		Ft_Gpu_CoCmd_Track(phost,200, 180, 10, 270,1);
		Ft_Gpu_Hal_WrCmd32(phost,TAG(2));
		Ft_Gpu_CoCmd_BgColor(phost,0xff00ffUL);
		Ft_Gpu_CoCmd_Toggle(phost,400, 100, 50, 28, 0, toggle_pos, "yes" "\xff" "no");
		Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(0, 0, 0));
		Ft_Gpu_CoCmd_FgColor(phost,0x007f00UL);
		Ft_Gpu_Hal_WrCmd32(phost,TAG(3));
		if(rd32(REG_TOUCH_TAG)==3)Ft_Gpu_CoCmd_FgColor(phost,0xffff00UL);
		Ft_Gpu_CoCmd_Button(phost,550, 50, 200, 150, 25, 0, "Press");
		Ft_Gpu_CoCmd_FgColor(phost,0xffffffUL);
		Ft_Gpu_CoCmd_BgColor(phost,0x000000UL);
		//Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_Hal_WrCmd32(phost,TAG(11));
		if(rd32(REG_TOUCH_TAG)==11)Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		else Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_Text(phost,100, 200-(slider_pos*2), 30, OPT_CENTER, "Item1");
		Ft_Gpu_Hal_WrCmd32(phost,TAG(12));
		if(rd32(REG_TOUCH_TAG)==12)Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		else Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_Text(phost,100, 250-(slider_pos*2), 30, OPT_CENTER, "Item2");
		Ft_Gpu_Hal_WrCmd32(phost,TAG(13));
		if(rd32(REG_TOUCH_TAG)==13)Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		else Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_Text(phost,100, 300-(slider_pos*2), 30, OPT_CENTER, "Item3");
		Ft_Gpu_Hal_WrCmd32(phost,TAG(14));
		if(rd32(REG_TOUCH_TAG)==14)Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		else Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_Text(phost,100, 350-(slider_pos*2), 30, OPT_CENTER, "Item4");
		Ft_Gpu_Hal_WrCmd32(phost,TAG(15));
		if(rd32(REG_TOUCH_TAG)==15)Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		else Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_Text(phost,100, 400-(slider_pos*2), 30, OPT_CENTER, "Item5");
		Ft_Gpu_Hal_WrCmd32(phost,TAG(16));
		if(rd32(REG_TOUCH_TAG)==16)Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		else Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_Text(phost,100, 450-(slider_pos*2), 30, OPT_CENTER, "Item6");
		Ft_Gpu_Hal_WrCmd32(phost,TAG(17));
		if(rd32(REG_TOUCH_TAG)==17)Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		else Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_Text(phost,100, 500-(slider_pos*2), 30, OPT_CENTER, "Item7");
		Ft_Gpu_Hal_WrCmd32(phost,TAG(18));
		if(rd32(REG_TOUCH_TAG)==18)Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		else Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_Text(phost,100, 550-(slider_pos*2), 30, OPT_CENTER, "Item8");
		Ft_Gpu_Hal_WrCmd32(phost,TAG(19));
		if(rd32(REG_TOUCH_TAG)==19)Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		else Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_Text(phost,100, 600-(slider_pos*2), 30, OPT_CENTER, "Item9");
		Ft_Gpu_Hal_WrCmd32(phost,TAG(20));
		if(rd32(REG_TOUCH_TAG)==20)Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		else Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_Text(phost,100, 650-(slider_pos*2), 30, OPT_CENTER, "Item10");
		Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(0, 0, 200));
		Ft_Gpu_Hal_WrCmd32(phost,BEGIN(RECTS));
		Ft_Gpu_Hal_WrCmd32(phost,VERTEX2F(0 * 16,0 * 16));
		Ft_Gpu_Hal_WrCmd32(phost,VERTEX2F(200 * 16,170 * 16));
		Ft_Gpu_Hal_WrCmd32(phost,END());
		Ft_Gpu_CoCmd_FgColor(phost,0xffffffUL);
		Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_Text(phost,100, 150, 30, OPT_CENTER, "Main");
		//Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		//Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 255));
		Ft_Gpu_CoCmd_FgColor(phost,0x00ff00UL);
		Ft_Gpu_CoCmd_BgColor(phost,0xff00ffUL);
		Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		Ft_Gpu_CoCmd_Number(phost,150, 20, 31, OPT_RIGHTX | 4,dial_pos);
		Ft_Gpu_Hal_WrCmd32(phost,TAG(5));
		Ft_Gpu_CoCmd_Dial(phost,600, 350, 50, 0, dial_pos);
		Ft_Gpu_CoCmd_Track(phost,600, 350, 1, 1,5);

		Ft_Gpu_Hal_WrCmd32(phost, DISPLAY());
		Ft_Gpu_CoCmd_Swap(phost);
		}
	}


/*
const char sm7[]	={" �������� N2        "}; //
const char sm8[]	={" ��������           "}; //
const char sm9[]	={" ����               "}; //
const char sm10[]	={" �����������        "}; // 
const char sm11[]	={" ������ ������      "}; //
const char sm12[]	=" ���������� ������  "}; //
const cha		=" �������            "}; //
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



else if(ind==iMn_IP55)
	{
	if(but==butE)
		{
		//Ft_Gpu_CoCmd_Dlstart(phost);
		Ft_Gpu_CoCmd_Text(phost,100, 100, 27, OPT_CENTER, "Mama");

		}
	else if(but==butD)
		{
		sub_ind++;
		//Ft_Gpu_CoCmd_Dlstart(phost);
		wr32(RAM_DL + 0,CLEAR_COLOR_RGB(0, 200, 0)); // change colour to red 
		//Ft_Gpu_Hal_WrCmd32(phost,CLEAR_COLOR_RGB(0, 0, 200));
		wr32(RAM_DL + 4, CLEAR(1, 1, 1)); // clear screen 
		//Ft_Gpu_Hal_WrCmd32(phost, CLEAR(1, 1, 1));
		wr32(RAM_DL + 8, COLOR_RGB(255, 255, 255)); 
		wr32(RAM_DL + 12,POINT_SIZE(5*16)); // set point size to 10 pixels in radius
		wr32(RAM_DL + 16,BEGIN(POINTS)); // start drawing points  
		wr32(RAM_DL + 20, VERTEX2F(800/2 * 16, 480/2 * 16)); // ascii F in font 31 

		wr32(RAM_DL + 24, DISPLAY()); // display the image
		wr8(REG_DLSWAP,DLSWAP_FRAME);	
		}
		
	else if(but==butU)
		{
		sub_ind--;
		Ft_Gpu_CoCmd_Dlstart(phost);
		Ft_Gpu_Hal_WrCmd32(phost,CLEAR_COLOR_RGB(0, 0, 200));
		Ft_Gpu_Hal_WrCmd32(phost, CLEAR(1, 1, 1));
		Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 0, 0));
		Ft_Gpu_Hal_WrCmd32(phost,POINT_SIZE(10*16));
		Ft_Gpu_Hal_WrCmd32(phost,BEGIN(POINTS));
		Ft_Gpu_Hal_WrCmd32(phost,VERTEX2F(500 * 16, 300 * 16));
		Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(255, 255, 0));
		Ft_Gpu_CoCmd_FgColor(phost,0x00ff00UL);
		/*Ft_Gpu_Hal_WrCmd32(phost,0xffffff0aUL);
		Ft_Gpu_Hal_WrCmd32(phost,0xffffffffUL);
		Ft_Gpu_Hal_WrCmd32(phost,0xffffff09UL);
		Ft_Gpu_Hal_WrCmd32(phost,0x00000000UL);
		Ft_Gpu_Hal_WrCmd32(phost,0xffffff16UL);
		Ft_Gpu_Hal_WrCmd32(phost,100UL+(100UL<<8));
		Ft_Gpu_Hal_WrCmd32(phost,0); */
		//
		Ft_Gpu_CoCmd_BgColor(phost,0xff0000UL);
		Ft_Gpu_CoCmd_Number(phost,150, 20, 31, OPT_RIGHTX | 3, 42);

		//Ft_Gpu_CoCmd_Spinner(phost,300,300,3,1);

		//Ft_Gpu_CoCmd_Slider(phost,300,100,200, 10, OPT_FLAT, 50, 100);


		//Ft_Gpu_CoCmd_Button(phost,10, 10, 140, 100, 31, 0,"Press!");

		
		Ft_Gpu_Hal_WrCmd32(phost, DISPLAY());
		Ft_Gpu_CoCmd_Swap(phost);
		}	

	else if(but==butR)
		{
		//sub_ind--;
		Ft_Gpu_CoCmd_Dlstart(phost);
		Ft_Gpu_Hal_WrCmd32(phost,CLEAR_COLOR_RGB(0, 0, 255)); 
		Ft_Gpu_Hal_WrCmd32(phost, CLEAR(1, 1, 1));//Ft_Gpu_Cmd(phost,CLEAR(1,1,1)); 
		Ft_Gpu_Hal_WrCmd32(phost, COLOR_RGB(0,255,  0));
		Ft_Gpu_CoCmd_FgColor(phost,0xff0000UL);
		Ft_Gpu_CoCmd_BgColor(phost,0x0000ffUL);
		Ft_Gpu_CoCmd_Text(phost,100, 100, 31, OPT_CENTER, "Please Tap on the dot");
		/*Ft_Gpu_Hal_WrCmd32(phost, POINT_SIZE(320)); // set point size to 20 pixels in radius 
		Ft_Gpu_Hal_WrCmd32(phost, BEGIN(POINTS)); // start drawing points 
		Ft_Gpu_Hal_WrCmd32(phost, VERTEX2II(192, 133, 0, 0)); // red point 
		Ft_Gpu_Hal_WrCmd32(phost, END());*/

		//Ft_Gpu_Hal_WrCmd32(phost, COLOR_RGB(255,255,255));
		//
		//Ft_Gpu_CoCmd_Text(phost,80, 30, 27,OPT_CENTER, "Please tap on the dot"); 
		//Ft_Gpu_CoCmd_Button(phost,100,100,200,200,31,OPT_CENTER,"AA");
		Ft_Gpu_Hal_WrCmd32(phost, DISPLAY()); // display the image	

		
		//
		//
		Ft_Gpu_CoCmd_Swap(phost);
		//wr32(RAM_CMD+rd32(REG_CMD_WRITE),4294967040UL);
		//wr32(REG_CMD_WRITE,rd32(REG_CMD_WRITE)+4);
		}
	else if(but==butL)
		{
		//ind=iMn;
		sub_ind++;
	Ft_Gpu_CoCmd_Dlstart(phost);

	Ft_Gpu_Hal_WrCmd32(phost,CLEAR_COLOR_RGB(200,200,200));
	Ft_Gpu_Hal_WrCmd32(phost,CLEAR(1,1,1));
	Ft_Gpu_Hal_WrCmd32(phost,COLOR_RGB(0xff,0xff,0xff));
	Ft_Gpu_CoCmd_FgColor(phost,0xff0000UL);
	Ft_Gpu_CoCmd_BgColor(phost,0x0000ffUL);
	/* Draw number at 0,0 location */
	//Ft_App_WrCoCmd_Buffer(phost,COLOR_A(30));
	Ft_Gpu_CoCmd_Text(phost,100, 100, 27, OPT_CENTER, "Please Tap on the dot");
	Ft_Gpu_CoCmd_Calibrate(phost,0);

	/* Download the commands into FIFIO */
	//Ft_App_Flush_Co_Buffer(phost);
	/* Wait till coprocessor completes the operation */
	//Ft_Gpu_Hal_WaitCmdfifo_empty(phost);
	/* Print the configured values */
	//Ft_Gpu_Hal_RdMem(phost,REG_TOUCH_TRANSFORM_A,(ft_uint8_t *)TransMatrix,4*6);//read all the 6 coefficients

			//Ft_Gpu_Hal_WrCmd32(phost, DISPLAY()); // display the image	

		
		Ft_Gpu_Hal_WrCmd32(phost, DISPLAY());
		Ft_Gpu_CoCmd_Swap(phost);
		}
	else if(but==butDR_)
		{
		tree_up(iK_6U,0,0,0);
		}
	else if(but==butDL_)
		{
		tree_up(iSet_6U,0,0,0);
		}

		

    }		
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
     		"    �������������   ",
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
     		"    �������������   ",
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
spi1_config();

//watchdog_enable();

phost=&host;

PD_LOW
delay_ms(20);
PD_HIGH
delay_ms(20);
command_active();

delay_ms(20);
host_command(INTERNAL_OSC);
delay_ms(10);
host_command(SLEEP);
host_command(SYSCLK_48M);
command_active();
host_command(CORE_RESET);
while (rd8(REG_ID)!=0x7c){};
 
wr16(REG_HCYCLE, DispHCycle);
wr16(REG_HOFFSET, DispHOffset);
wr16(REG_HSYNC0, DispHSync0);
wr16(REG_HSYNC1, DispHSync1);
wr16(REG_VCYCLE, DispVCycle);
wr16(REG_VOFFSET, DispVOffset);
wr16(REG_VSYNC0, DispVSync0);
wr16(REG_VSYNC1, DispVSync1);
wr8(REG_SWIZZLE, DispSwizzle);
wr8(REG_PCLK_POL, DispPCLKPol);
wr16(REG_HSIZE, DispWidth);
wr16(REG_VSIZE, DispHeight);
//wr16(REG_CSPREAD, DispCSpread);
//wr16(REG_DITHER, DispDither);
/*
wr16(REG_HCYCLE, 548); 
wr16(REG_HOFFSET, 43); 
wr16(REG_HSYNC0, 0); 
wr16(REG_HSYNC1, 41); 
wr16(REG_VCYCLE, 292); 
wr16(REG_VOFFSET, 12); 
wr16(REG_VSYNC0, 0); 
wr16(REG_VSYNC1, 10); 
wr8(REG_SWIZZLE, 0); 
wr8(REG_PCLK_POL, 1); 
wr8(REG_CSPREAD, 1); 
wr16(REG_HSIZE, 480); 
wr16(REG_VSIZE, 272);*/


wr8(REG_GPIO_DIR,0xff);
wr8(REG_GPIO,0xff);
/*
wr32(RAM_DL+0,0x020000ff); 
wr32(RAM_DL+4,0x26000007); 
wr32(RAM_DL+8,0x00000000); */



wr32(RAM_DL + 0, CLEAR(1, 1, 1)); // clear screen 
wr32(RAM_DL + 4, BEGIN(BITMAPS)); // start drawing bitmaps 
wr32(RAM_DL + 8, VERTEX2II(220, 110, 31, 'F')); // ascii F in font 31 
wr32(RAM_DL + 12,/* VERTEX2II(244, 110, 31, 'T')*/ COLOR_RGB( 22,255, 22)); // change colour to red); // ascii T 
wr32(RAM_DL + 16, VERTEX2II(270, 110, 31, 'D')); // ascii D 
wr32(RAM_DL + 20, VERTEX2II(400, 110, 31, 'I')); // ascii I 
wr32(RAM_DL + 24, END()); 
wr32(RAM_DL + 28, COLOR_RGB(255, 22, 22)); // change colour to red 
wr32(RAM_DL + 32, POINT_SIZE(320)); // set point size to 20 pixels in radius 
wr32(RAM_DL + 36, BEGIN(POINTS)); // start drawing points 
wr32(RAM_DL + 40, VERTEX2II(192, 133, 0, 0)); // red point 
wr32(RAM_DL + 44, END()); 
wr32(RAM_DL + 48, DISPLAY()); // display the image	

		

/// 		Ft_Gpu_CoCmd_Dlstart(phost); 
//wr32(RAM_DL + 0, CLEAR(1, 1, 1)); // clear screen 
///		Ft_Gpu_Hal_WrCmd32(phost, CLEAR(1, 1, 1));//Ft_Gpu_Cmd(phost,CLEAR(1,1,1)); 
///wr32(RAM_DL + 4, BEGIN(BITMAPS)); // start drawing bitmaps 
///wr32(RAM_DL + 8, VERTEX2II(220, 110, 31, 'F')); // ascii F in font 31 
///wr32(RAM_DL + 12, END());
//wr32(RAM_DL + 48, DISPLAY()); // display the image

wr8(REG_DLSWAP,DLSWAP_FRAME);




wr8(REG_PCLK,5);


ind=iMn_IP55;

delay_ms(300);
//wr8(0xc0000,0x53);		
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

		but_drv();
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


		//rd8(0xc0000);
		//RIV_CS_ON
		//spi1(0x55);
		//RIV_CS_OFF
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
