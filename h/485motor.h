#ifndef __485MOTOR_H__
#define __485MOTOR_H__

#define         FCAN        40000000                // Fcyc = 1/2Fpll
#define         BAUD19200    ((FCAN/19200)/16) - 1
#define         BAUD115200    ((FCAN/115200)/16) - 1
#define         Transmit485 _LATB8

#define         SUCCESS     0
#define         FAILE       1

#define         ADD         2
#define         REDUCE      3

#define         UDmotor     1
#define         LRmotor     0

#define         UDzero      0
#define         LRzero      3051  
/*485通讯结构体及变量*/
typedef union 
{
    unsigned char Sdata[18];
    struct
    {
        unsigned char head;
        unsigned char cmd;
        unsigned char motorID;
        unsigned char lengh;
        unsigned char check1_4;
        unsigned char angle[8];
        unsigned char speed[4];
        unsigned char check6_17;     
    }Sdata_analyze;
}Tmotordata;

typedef union 
{
    volatile unsigned char Rdata[8];
    struct
    {
        unsigned char head;
        unsigned char cmd;
        unsigned char motorID;
        unsigned char lengh;
        unsigned char check1_4;
        unsigned char encoder1;
        unsigned char encoder2;
        unsigned char check6_7;      
    }Rdata_analyze;
}Rmotordata;

typedef union 
{
    volatile unsigned char Speeddata[10];
    struct
    {
        unsigned char head;
        unsigned char cmd;
        unsigned char motorID;
        unsigned char lengh;
        unsigned char check1_4;
        unsigned char speed[4];
        unsigned char check6_9;      
    }Speeddata_analyze;
}Speedmotordata;


char Transmit_position_to_motor(char motor_num, char dir, int speed, long int angle);
char Transmit_to_check_encoder(char motor_num);
char Check_Init_success(void);
char Transmit_speed_to_motor(char motor_num, char dir, int speed);
char Transmit_to_stop_motor(char motor_num);
void Init_485TX(void);
void Init_485RX(void);
void putsU1(char *s, char num);
void putsU1_characters(char *s);
void putU1(int c);
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void);
#endif
