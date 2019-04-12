#ifndef __232COMMU_H__
#define __232COMMU_H__

#define         FCAN        40000000                // Fcyc = 1/2Fpll
#define         BAUD19200    ((FCAN/19200)/16) - 1
#define         BAUD9600    ((FCAN/9600)/16) - 1

/*232通讯结构体及变量*/
typedef union 
{
    volatile unsigned char data[25];
    struct
    {
        char head;
        
        char axis1_dir;
        char axis1_angle[3];
        char A2;
        char axis1_speed[4];
        char A3;
        char axis1_div;
        
        char A4;
        char axis2_dir;
        char axis2_angle[3];
        char A5;
        char axis2_speed[4];
        char A6;
        char axis2_div;
        
        char end;
        
    }commondata;
}Myrdata;

typedef struct 
{
    int axis1_dir;
    int axis1_angle; 
    int axis1_speed;   
    int axis1_div;

    int axis2_dir;
    int axis2_angle;
    int axis2_speed;
    int axis2_div;
}afterprodata;

#endif
