/*
 * File:   mainXC16.c
 * Author: Lenovo
 * Created on October 11, 2018, 11:44 AM
 */
#include <p33EV256GM104.h>
#include "xc.h"

#include "../../../include/lega-c/string.h"
#include "../../../include/lega-c/stdlib.h"
#include "../../../include/lega-c/ctype.h"
#include "../../../include/lega-c/stdio.h"
#include "232commu.h"
#include "485motor.h"

#define         UDmotor     0
#define         LRmotor     1

#define         Forward     0
#define         Reverse     1

#define         LEDEN       _LATB7


#define         FCAN        40000000                // Fcyc = 1/2Fpll
#define         BAUD9600    ((FCAN/9600)/16) - 1
#define         BAUD19200    ((FCAN/19200)/16) - 1
#define         BAUD115200    ((FCAN/115200)/16) - 1
//  Macros for Configuration Fuse Registers 
_FOSCSEL(FNOSC_PRIPLL);
_FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMD_XT);

_FWDT(FWDTEN_OFF);      // Watchdog Timer Enabled/disabled by user software

_FICD(ICS_PGD3);        // PGD3 for external PK3/ICD3/RealIce, use PGD2 for PKOB

_FPOR(BOREN0_OFF);      // no brownout detect
_FDMT(DMTEN_DISABLE);   // no deadman timer  <<< *** New feature, important to DISABLE

void oscConfig(void);
void clearIntrflags(void);
void InitMonitor(void);

extern Rmotordata Rmotor;

extern Myrdata rdata;
extern volatile int rindex;
extern volatile char rfinish;
extern volatile char rbegin ;
extern afterprodata pdata;

volatile int tick1;
volatile char check = 0;
int main(void)
{
    unsigned char kk[10] = {0x3E, 0xA2, 0x01, 0x04, 0xE5, 0xA0, 0x8C, 0x00, 0x00, 0x2C};
    int k;
   // Configure Oscillator Clock Source
    oscConfig();
    // Clear Interrupt Flags
    clearIntrflags();
    
    Init();
    Init_232TX();
    Init_232RX();
    Init_485TX();
    Init_485RX();
    _TRISB7 = 0;  //LED控制脚
     
    for(k = 0; k < 1500; k++)
    Delayus(1000);
    Init_clouldplaform();
   
    while(1)
    {   
        if(rbegin == 1)
        {
            Delayus(22000);     //延时22ms等待串口25个字节数据接收完成
            if(rfinish != 1)    //判断是否接收到完整的一帧数据
            {
                for(k = 0; k < 25; k++)
                {
                    rdata.data[k] = 0;
                }
            }
            else
            {
                rindex = 0;
                rfinish = 0;
                /*接收成功作处理*/
                if(rdata.commondata.axis1_dir == 'G') 
                {
                    get_position();      //向机器人发送当前的电机位置数据
                }
                if(rdata.commondata.axis1_dir == 'S') 
                {
                    set_position();     //把电机设置成机器人发送过来的对应的位置
                }
                if(rdata.commondata.axis1_dir == 'I') 
                {
                    Init_clouldplaform();   //吊舱初始化
                }
                else
                {
                    changedatatype();    //串口通讯帧数据的解析
                    Drive_motor(pdata);  //驱动电机
                }
                                              
                for(k = 0; k < 25; k++)
                {
                    rdata.data[k] = 0;
                }
            }
            rbegin = 0;
            rindex = 0;
        }
        
    }
    
}



void Init(void)
{
    _TRISB7 = 0;   //12V LED灯
    LEDEN = 0;
    
    _TRISB8 = 0;   
    Transmit485 = 1;
}
void oscConfig(void)
{

    //  Configure Oscillator to operate the device at 80MHz/40MIPs
    // 	Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // 	Fosc= 8M*40/(2*2)=80Mhz for 8M input clock
    // To be safe, always load divisors before feedback
    
   
    CLKDIVbits.PLLPOST = 0;     // N1=2
    CLKDIVbits.PLLPRE = 0;      // N2=2
    PLLFBD = 38;                // M=(40-2), Fcyc = 40MHz for ECAN baud timer


    // Disable Watch Dog Timer

    RCONbits.SWDTEN = 0;

}

void StartTM1(void)      //250ms的计时器
{
    T1CONbits.TON = 0;          // Disable Timer1
    T1CONbits.TCS = 0;          // Select internal instruction cycle clock
    T1CONbits.TGATE = 0;        // Disable Gated Timer mode
    T1CONbits.TCKPS = 0x3;      // Select 1:256 Prescaler
    PR1 = 39062;//39062 ;                // Load the period value (250ms/(256*25ns))
    IPC0bits.T1IP = 0x03;       // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0;          // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1;          // Enable Timer1 interrupt
    
    T1CONbits.TON = 1;          // Start Timer1
}

void clearIntrflags(void)
{
    /* Clear Interrupt Flags */

    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;
    IFS3 = 0;
    IFS4 = 0;
    IPC16bits.U1EIP = 6;        //service the LIN framing error before the RX
    IPC2bits.U1RXIP = 4;
}
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
   // s_tick++; // increment the 'slow tick'
//    tick1++;
//    if(tick1 == 4)
//    {
//        tick1 = 0;
//        _LATB7 = ~_LATB7 ;
//    }
//    if((LRencoder_add == 1) || (LRencoder_reduce == 1))
//    {
        check = 1;
//    }
//    
    IFS0bits.T1IF = 0; //Clear Timer1 interrupt flag

}
void Delayus(int delay)
{
    int i;
    for (i = 0; i < delay; i++)
    {
        __asm__ volatile ("repeat #39");
        __asm__ volatile ("nop");
    }
}



