#include <p33EV256GM104.h>
#include "xc.h"
#include "232commu.h"

volatile int datal;
volatile int k;
Myrdata rdata; 
afterprodata pdata;

volatile int rindex = 0;
volatile char rfinish = 0;
volatile char rbegin = 0;

void Init_232TX(void)
{

    _TRISC1 = 0;
    
    RPOR5bits.RP49R = 0x03;
    //
    // set up the UART for default baud, 1 start, 1 stop, no parity
    //
    U2MODEbits.STSEL = 0;       // 1-Stop bit
    U2MODEbits.PDSEL = 0;       // No Parity, 8-Data bits
    U2MODEbits.ABAUD = 0;       // Auto-Baud disabled
    U2MODEbits.BRGH = 0;        // Standard-Speed mode
    U2BRG = BAUD9600;          // Baud Rate setting for 38400 (default)
    U2STAbits.UTXISEL0 = 0;     // Interrupt after TX buffer done
    U2STAbits.UTXISEL1 = 1;
    IEC1bits.U2TXIE = 0;        // Enable UART TX interrupt
    U2MODEbits.UARTEN = 1;      // Enable UART (this bit must be set *BEFORE* UTXEN)
    
    U2STAbits.UTXEN = 1;
}
void Init_232RX(void)
{   
    ANSELCbits.ANSC2 = 0;     //digital
    _TRISC2 = 1;                    // digital input pin
    RPINR19 = 50;     
//     RPINR18 = 0x48;                 // map LIN receiver to pin RD8
//    _TRISD8 = 1;                    // digital input pin
    //
    // set up the UART for LIN_BRGVAL baud, 1 start, 1 stop, no parity
    //
    U2MODEbits.STSEL = 0;           // 1-Stop bit
    U2MODEbits.PDSEL = 0;           // No Parity, 8-Data bits
    U2MODEbits.ABAUD = 0;           // Auto-Baud disabled
    U2MODEbits.BRGH = 0;            // Standard-Speed mode
    U2BRG = BAUD9600;             // Baud Rate setting
    U2STAbits.URXISEL1 = 0;          // Interrupt after one RX done
    U2STAbits.URXISEL0 = 0;
    IEC1bits.U2RXIE = 1;            // Enable UART1 RX interrupt
    IEC4bits.U2EIE = 1;             // Enable Error (Framing) Interrupt for BREAK
    U2MODEbits.UARTEN = 1;          // Enable UART1
}
void changedatatype(void)
{
    pdata.axis1_dir = atoi((char*)&rdata.commondata.axis1_dir) / 1000;
    pdata.axis1_angle = atoi(rdata.commondata.axis1_angle);
    pdata.axis1_speed = atoi(rdata.commondata.axis1_speed);
    pdata.axis1_div = atoi((char*)&rdata.commondata.axis1_div);
    
    pdata.axis2_dir = atoi((char*)&rdata.commondata.axis2_dir) / 1000;
    pdata.axis2_angle = atoi(rdata.commondata.axis2_angle);
    pdata.axis2_speed = atoi(rdata.commondata.axis2_speed);
    pdata.axis2_div = atoi((char*)&rdata.commondata.axis2_div);
}

//void putsU2(char *s)
//{
//    int i = 2;
//    while (i--)
//    { // loop until *s =\0, end of string
//        putU2(*s++);
//    } // send the character and point to the next one
//}
void putsU2(char *s, char num)
{
    int i = num;
    while (i--)
    { // loop until *s =\0, end of string
        putU2(*s++);
    } // send the character and point to the next one
}
void putsU2_characters(char *s)
{
    while (*s)
    { // loop until *s =\0, end of string
        putU2(*s++);
    } // send the character and point to the next one
}
void putU2(int c)
{
    while (U2STAbits.UTXBF); // wait while Tx buffer full
    U2TXREG = c;
}

 void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void)
{
    while (U2STAbits.TRMT == 0); // wait for transmitter empty
    IFS1bits.U2TXIF = 0; // Clear TX2 Interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void)
{
    rbegin = 1;
    while(U2STAbits.URXDA == 1)
    {
        datal = U2RXREG;
        if(U2STAbits.OERR == 1) //处理串口接收溢出
        {
            while(U2STAbits.URXDA == 1)
            {
                datal = U2RXREG;      
            }
            rindex = 0;
            U2STAbits.OERR = 0;
            for(k = 0; k < 25; k++)
            {
                rdata.data[k] = 0;
            }
            break;
        }
        if( ((datal & 0xFF) == 'A') || (rdata.data[0] == 'A') )
        {
            if(rindex >= 25)
            {
                rindex = 0;
            }
            rdata.data[rindex] = datal & 0xFF;
            rindex++;
            
        }
        
    }
    

    if(rindex >= 25)
    {
        //putsU2("222222");   
        if(rdata.data[24] == 'B')
        {
            rfinish = 1;
        }     
        rindex = 0;
    }
    
    IFS1bits.U2RXIF = 0;                // Clear RX1 Interrupt flag
}

