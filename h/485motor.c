#include <p33EV256GM104.h>
#include "xc.h"
#include "485motor.h"
#include "232commu.h"

Tmotordata Tmotor[2] = 
{
    {
        .Sdata_analyze.head = 0x3E,
        .Sdata_analyze.cmd  = 0xA4,
        .Sdata_analyze.motorID = 0x04,        
        .Sdata_analyze.lengh = 0x0C,
        .Sdata_analyze.check1_4 = 0xF2,
        .Sdata_analyze.angle[0] = 0x00,
        .Sdata_analyze.angle[1] = 0x00,
        .Sdata_analyze.angle[2] = 0x00,
        .Sdata_analyze.angle[3] = 0x00,
        .Sdata_analyze.angle[4] = 0x00,
        .Sdata_analyze.angle[5] = 0x00,
        .Sdata_analyze.angle[6] = 0x00,
        .Sdata_analyze.angle[7] = 0x00,
        .Sdata_analyze.speed[0] = 0x00,
        .Sdata_analyze.speed[1] = 0x00,
        .Sdata_analyze.speed[2] = 0x00,
        .Sdata_analyze.speed[3] = 0x00,
        .Sdata_analyze.check6_17 = 0x00
    },
    
    {
        .Sdata_analyze.head = 0x3E,
        .Sdata_analyze.cmd  = 0xA4,
        .Sdata_analyze.motorID = 0x01,        
        .Sdata_analyze.lengh = 0x0C,
        .Sdata_analyze.check1_4 = 0xEF,
        .Sdata_analyze.angle[0] = 0x00,
        .Sdata_analyze.angle[1] = 0x00,
        .Sdata_analyze.angle[2] = 0x00,
        .Sdata_analyze.angle[3] = 0x00,
        .Sdata_analyze.angle[4] = 0x00,
        .Sdata_analyze.angle[5] = 0x00,
        .Sdata_analyze.angle[6] = 0x00,
        .Sdata_analyze.angle[7] = 0x00,
        .Sdata_analyze.speed[0] = 0x00,
        .Sdata_analyze.speed[1] = 0x00,
        .Sdata_analyze.speed[2] = 0x00,
        .Sdata_analyze.speed[3] = 0x00,
        .Sdata_analyze.check6_17 = 0x00
    }
};

Rmotordata Rmotor = 
{
    
    .Rdata_analyze.head = 0x00,
    .Rdata_analyze.cmd  = 0x00,
    .Rdata_analyze.motorID = 0x00,
    .Rdata_analyze.lengh   = 0x00,
    .Rdata_analyze.check1_4 = 0x00,
    .Rdata_analyze.encoder1 = 0x00,
    .Rdata_analyze.encoder2 = 0x00,
    .Rdata_analyze.check6_7 = 0x00
   
};

volatile int datal485;
volatile int p;

volatile int rindex485 = 0;
volatile char rfinish485 = 0;
volatile char rbegin485 = 0;

volatile char UDmotor_state;
volatile char LRmotor_state;

volatile int last_move_state = ADD;    //������¼���������ϴ��˶���״̬
char check_cmd[5] = {0x3E, 0x90, 0x04, 0x00, 0xD2};  //��ѯ������������ݵ�ָ��
volatile int round;             //��¼�����ǰλ�ڵڼ�Ȧ
char store_index = 0;           
extern volatile char check;
volatile char LRencoder_add = 0;        //��¼���������Ƿ�����������ӵķ����˶�
volatile char LRencoder_reduce = 0;     //��¼���������Ƿ�����������ٵķ����˶�

volatile int store_encoder_last;        //�ϴμ��ı���������

extern Myrdata rdata;
extern afterprodata pdata;
Speedmotordata Stop_motor[2] = 
{
    {
        .Speeddata_analyze.head = 0x3E,
        .Speeddata_analyze.cmd  = 0xA2,
        .Speeddata_analyze.motorID = 0x04,
        .Speeddata_analyze.lengh = 0x04,
        .Speeddata_analyze.check1_4 = 0xE8,
        .Speeddata_analyze.speed[0] = 0x00,
        .Speeddata_analyze.speed[1] = 0x00,  
        .Speeddata_analyze.speed[2] = 0x00,
        .Speeddata_analyze.speed[3] = 0x00,  
        .Speeddata_analyze.check6_9 = 0x00        
    },
    {
        .Speeddata_analyze.head = 0x3E,
        .Speeddata_analyze.cmd  = 0xA2,
        .Speeddata_analyze.motorID = 0x01,
        .Speeddata_analyze.lengh = 0x04,
        .Speeddata_analyze.check1_4 = 0xE5,
        .Speeddata_analyze.speed[0] = 0x00,
        .Speeddata_analyze.speed[1] = 0x00,  
        .Speeddata_analyze.speed[2] = 0x00,
        .Speeddata_analyze.speed[3] = 0x00,  
        .Speeddata_analyze.check6_9 = 0x00 
    }
};

Speedmotordata LR_speed_motor =
{
    .Speeddata_analyze.head = 0x3E,
    .Speeddata_analyze.cmd  = 0xA2,
    .Speeddata_analyze.motorID = 0x04,
    .Speeddata_analyze.lengh = 0x04,
    .Speeddata_analyze.check1_4 = 0xE8,
    .Speeddata_analyze.speed[0] = 0x00,
    .Speeddata_analyze.speed[1] = 0x00,  
    .Speeddata_analyze.speed[2] = 0x00,
    .Speeddata_analyze.speed[3] = 0x00,  
    .Speeddata_analyze.check6_9 = 0x00
};
void Init_clouldplaform()
{
    int time = 0;
    for( ; (Transmit_position_to_motor(UDmotor, 1, 3000, 19800) && time < 5); time++)   //�ɼ���
    {            
    }
//    for( ; (Transmit_position_to_motor(LRmotor, 1, 9000, 27000) && time < 5); time++)   //+120��
//    {            
//    }
    for( ; (Transmit_position_to_motor(LRmotor, 1, 9000, 72000 - 18000) && time < 5); time++)   //+270��
    {            
    }
    StartTM1();         //��ʼ��ʱ
           _LATB7 = ~_LATB7 ;
    while(Check_Init_success() == FAILE); //������������Ƿ��ʼ����ɣ���ΪȦ���ļ�¼����׼ȷ�����Գ�ʼ�����ǰ���ܽ����˶�ָ��
               _LATB7 = ~_LATB7 ;
    T1CONbits.TON = 0;   //��ʱֹͣ
    round = 2;
}

/*������������Ƿ��ʼ����ɣ����ʱ����Ϊ250ms��
 * ����һ�μ�⵽�ı����������뵱�μ�⵽�ı��������ݼ��С��50��
 * ����Ϊ��ʼ�����*/
char Check_Init_success(void)
{
    int k;
    char time = 0;
    unsigned int temp;
    if(check == 1)
    {
        Transmit485 = 1;
        putsU1(&check_cmd[0] ,5);    
        Delayus(2200);

        Transmit485 = 0;
        Delayus(1000);

        if(rbegin485 == 1)
        {
            Delayus(3000);
            if(rfinish485 != 1)
            {
                for(p = 0; p < 8; p++)
                {
                    Rmotor.Rdata[p] = 0;
                }
            }
            else
            {

                rindex485 = 0;
                rfinish485 = 0;
                /*���ճɹ�������*/

                temp = (Rmotor.Rdata_analyze.encoder2 * 256) + Rmotor.Rdata_analyze.encoder1;

                    if(store_index == 0)
                    {
                        store_encoder_last = temp;
                        store_index = 1;

                    }
                    else if(store_index == 1)
                    {
                        if( (((temp - store_encoder_last) <= 50) && ((temp - store_encoder_last) >= 0)) || (((store_encoder_last - temp) <= 50) && ((store_encoder_last - temp) >= 0)) )
//                        if( ((store_encoder_last - temp) < 10) || ((temp - store_encoder_last) < 10) )
                         {
                            for(p = 0; p < 8; p++)
                            {
                                Rmotor.Rdata[p] = 0;
                            }
                            return SUCCESS;
                        }
                        store_encoder_last = temp;
                    }


                for(p = 0; p < 8; p++)
                {
                    Rmotor.Rdata[p] = 0;
                }
            }


        }
        rbegin485 = 0;
        rindex485 = 0;
        check = 0;
        return FAILE ;
    }
    return FAILE ;
}

/*����������˶�������UDmotor_state��LRmotor_state������¼�˶�Ӧ��ĵ���Ƿ���յ����˶�����û��ִ�й�ֹͣ��
 last_move_state��¼���������ϴ��˶������Ǳ��������ӻ���ٵķ���
 * LRencoder_reduce��LRencoder_add��¼��ǰҪִ�е��˶������Ӧ��ʱ���������ӻ���ٵķ���
 * ���������Ĺ�ͬ�����ڿ�����ʱ���Ȧ���������*/
char Drive_motor(afterprodata prodata)
{
    int time = 0;
    if(prodata.axis1_angle == 999) 
    {
        if(prodata.axis1_dir == 1)
        {
            for( ; (Transmit_position_to_motor(UDmotor, 1, prodata.axis1_speed * 10, 19800) && (time < 5)); time++)   //+200��
            {            
            }
        }
        if(prodata.axis1_dir == 0)
        {
            for( ; (Transmit_position_to_motor(UDmotor, 1, prodata.axis1_speed * 10, 0) && (time < 5)); time++)   //+200��
            {            
            }
           // Transmit_to_motor(UDmotor, prodata.axis1_dir, prodata.axis1_speed * 10, 6000);   //-60��
        }
        UDmotor_state = 1;
    }
    if((prodata.axis1_angle == 0) && (UDmotor_state == 1))
    {
        for( ; (Transmit_to_stop_motor(UDmotor) && (time < 5)); time++)   
        {            
        }
        UDmotor_state = 0;
    }

    if(prodata.axis2_angle == 999) 
    {
        if(prodata.axis2_dir == 1)
        {
//            for( ; (Transmit_position_to_motor(LRmotor, prodata.axis2_dir, prodata.axis2_speed * 10, 36000) && time < 5); time++)   //+200��
//            {            
//            }
            LRencoder_add = 1;
            LRencoder_reduce = 0;
            Transmit_to_check_encoder(LRmotor);
            for( ; (Transmit_position_to_motor(LRmotor, 1, prodata.axis2_speed * 10, 36000 * round ) && (time < 5)); time++)   //������++
            {            
            }
            last_move_state = ADD;
            
        }
        if(prodata.axis2_dir == 0)
        {
//            for( ; (Transmit_position_to_motor(LRmotor, prodata.axis2_dir, prodata.axis2_speed * 10, 0) && time < 5); time++)   //+200��
//            {            
//            }
            LRencoder_reduce = 1;
            LRencoder_add = 0;
            Transmit_to_check_encoder(LRmotor);
            for( ; (Transmit_position_to_motor(LRmotor, 1, prodata.axis2_speed * 10, (36000 * (round - 1)) ) && (time < 5)); time++)   //������--
            {            
            }
            last_move_state = REDUCE;
        }
//        T1CONbits.TON = 0; //Stop TM1 
//        StartTM1();
        LRmotor_state = 1;
        
    }
    if((prodata.axis2_angle == 0) && (LRmotor_state == 1))
    {
        for( ; (Transmit_to_stop_motor(LRmotor) && (time < 5)); time++)   
        {            
        }
        LRmotor_state = 0;
//        
//        T1CONbits.TON = 0; //Stop TM1 
        LRencoder_add = 0;
        LRencoder_reduce = 0;
    }
}
/*�ú��������ж����������˶�֮ǰ�����Ȧ�����������������˶�ǰ���λ����㸽��λ�ã�
 * �����������ܵ�����ĸ������˶�����㣬������п�����Ϊ����������㣬��˺����е�����һ���ж�����������㸽������30�������
 *�������һ���˶�����ʱ���������ӵķ��򣬴˴ν�Ҫ�˶��ķ���Ҳʱ������Ҫ���ӵķ��򣬴�����µ��Ȧ������һȦ��
 * ������ϴ��˶������Ǳ��������ӵķ��򣬵��Ǵ˴��˶������Ǳ��������ٵķ�������Ȧ�������ӡ�
 *���˴���30�ǽ�Ϊ�ɿ��ľ��룬��Ϊ���������֮������ͨѶ���ϵ�ʱ����ʱ���⣬�����һ��ֹͣ��λ�ñ��������ݱض��������30���ϵ�λ��
 ��˲�����ֵ��Ȧ������������Ȧ�����*/
char Transmit_to_check_encoder(char motor_num)
{
    int k;
    char time = 0;
    unsigned int temp;
    Transmit485 = 1;
//    _LATB7 = 1;
    putsU1(&check_cmd[0] ,5);    
    Delayus(2200);
    
    Transmit485 = 0;
    Delayus(1000);

    if(rbegin485 == 1)
    {_LATB7 = ~_LATB7;
        Delayus(3000);
        if(rfinish485 != 1)
        {
            putsU2_characters(Rmotor.Rdata);
            
            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }
//            return FAILE;
//            _LATB7 = ~_LATB7;
        }
        else
        {
            rindex485 = 0;
            rfinish485 = 0;
            /*���ճɹ�������*/

            temp = (Rmotor.Rdata_analyze.encoder2 * 256) + Rmotor.Rdata_analyze.encoder1;
            
            //Delayus(10000);
            if(motor_num == LRmotor)
            {
                if(LRencoder_add == 1)
                {
//                    if( (((LRzero - temp) <= 100) && (LRzero - temp) >= 0) || (((temp - LRzero) <= 100) && ((temp - LRzero) >= 0)))
                     if(((LRzero - temp) <= 30) && (LRzero - temp) >= 0) 
                     {
                        if((round < 4) && (last_move_state == ADD))
                        {

//                            putsU2(&round,1);
                            round ++;
//                            putsU2(&round,1);

                        }
                    }
                }

                if(LRencoder_reduce == 1)
                {
                    if(((temp - LRzero) <= 30) && ((temp - LRzero) >= 0)) //temp
                    {
                        if((round > 1) && (last_move_state == REDUCE))
                        {
                            _LATB7 = ~_LATB7;
//                            putsU2(&round,1);
                            round --;
//                            putsU2(&round,1);
                        }
                    }
                }
            }
                
            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }
        }

                
    }
    rbegin485 = 0;
    rindex485 = 0;

}
char Transmit_to_stop_motor(char motor_num)
{
    int k;
    unsigned long int temp;
    Transmit485 = 1;
    putsU1(&(Stop_motor[motor_num].Speeddata[0]) ,10);    
    Delayus(2200);
    
    Transmit485 = 0;
    Delayus(1000);

    if(rbegin485 == 1)
    {
        Delayus(3000);
        if(rfinish485 != 1)
        {
            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }
        }
        else
        {
            rindex485 = 0;
            rfinish485 = 0;
            /*���ճɹ�������*/
            
//            for(k = 0; k < 1000; k++)
//            Delayus(1000);
            
            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }

        }
        rbegin485 = 0;
        rindex485 = 0;
    }
    rbegin485 = 0;
    rindex485 = 0;
    
    Transmit485 = 1;
    putsU1(&(Stop_motor[motor_num].Speeddata[0]) ,10);    
    Delayus(2200);
    
    Transmit485 = 0;
    Delayus(1000);

    if(rbegin485 == 1)
    {
        Delayus(3000);
        if(rfinish485 != 1)
        {
            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }
            return FAILE;
        }
        else
        {
            rindex485 = 0;
            rfinish485 = 0;
            /*���ճɹ�������*/

            temp = (Rmotor.Rdata_analyze.encoder2 * 256) + Rmotor.Rdata_analyze.encoder1;
            
//            temp = temp - (temp % 100);
//            Delayus(10000);
//            if(temp > 18000)
//                Transmit_position_to_motor(motor_num, 1, 8000, 18000);
//            if(temp < 1000)
//                Transmit_position_to_motor(motor_num, 1, 8000, 0);
//            else
            if(motor_num == LRmotor)
            {
                if(temp >LRzero)
                {
                    temp = temp -LRzero;
                    
                }               
                else if(temp == LRzero)
                {
                    if(last_move_state == ADD)
                    {
                        temp = 4095;
                    }
                    if(last_move_state == REDUCE)
                    {
                        temp = 0;
                    }
                }
                else if(temp < LRzero)
                {
                    temp = 4095 - (LRzero - temp)  ;
                }
                temp =  temp * 8.789;            //(((float)temp / 4096) * 36000);
//                if(LRencoder_reduce == 1)
//                    temp = (round - 1) * 36000 + temp;
//                
//                if(LRencoder_add == 1)
                temp = ((round - 1) * 36000) + temp;
                Transmit_position_to_motor(motor_num, 1, 2000, temp );
            }
                
            if(motor_num == UDmotor)
            {
                if(temp > UDzero)
                {
                    temp = temp - UDzero;
                    temp =  temp * 8.789;
                    Transmit_position_to_motor(motor_num, 1, 3000, temp);
                }
                else if(temp == UDzero)
                {
                    temp = temp - UDzero;
                    temp =  temp * 8.789;
                    Transmit_position_to_motor(motor_num, 1, 3000, temp);
                }
                else if(temp < UDzero)
                {
                    temp = UDzero - temp;
                    temp =  temp * 8.789;
                    Transmit_position_to_motor(motor_num, 1, 3000, temp);
                }                
            }
                
//            if(motor_num == UDmotor)
//            {
//                if(temp < 18000)
//                Transmit_position_to_motor(motor_num, 1, 8000, temp);
//                if(temp >= 18000)
//                {
//                    temp = 36000 - temp;
//                    Transmit_position_to_motor(motor_num, 0, 8000, temp);
//                }
//                
                for(p = 0; p < 8; p++)
                {
                    Rmotor.Rdata[p] = 0;
                }
//            }
            
//            if(motor_num == LRmotor)
//            {
//                Transmit_position_to_motor(motor_num, 1, 9000, temp);
//            }
            return SUCCESS;
        }
        rbegin485 = 0;
        rindex485 = 0;
    }
    rbegin485 = 0;
    rindex485 = 0;
    return FAILE;
}
/*������������ת�������Ӧ�ĽǶȷ��͸�������*/
void get_position(void)
{
    char back[25] = "AGXXXXXX0000AGXXXXXX0000B";
    char i[6];
    char inverse = 0;
    int n,  k,j;
    long int temp, encoder;
    Transmit485 = 1;
    putsU1(&(Stop_motor[UDmotor].Speeddata[0]) ,10);    
    Delayus(2200);
    
    Transmit485 = 0;
    Delayus(1000);

    if(rbegin485 == 1)
    {
        Delayus(3000);
        if(rfinish485 != 1)
        {
            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }
        }
        else
        {
            rindex485 = 0;
            rfinish485 = 0;
            /*���ճɹ�������*/

            encoder = (Rmotor.Rdata_analyze.encoder2 * 256) + Rmotor.Rdata_analyze.encoder1;
            
            if(encoder > UDzero)
            {
                encoder = encoder - UDzero;
                encoder =  encoder * 8.789;
                Transmit_position_to_motor(UDmotor, 1, 5000, encoder);
//                    encoder = 1;
            }
            else if(encoder == UDzero)
            {
                encoder = encoder - UDzero;
                encoder =  encoder * 8.789;
                Transmit_position_to_motor(UDmotor, 1, 5000, encoder);
//                    encoder = 1;
            }
            else if(encoder < UDzero)
            {
                encoder = UDzero - encoder;
                encoder =  encoder * 8.789;
                Transmit_position_to_motor(UDmotor, 1, 5000, encoder);
            }  
//            encoder =  encoder * 8.789;            //(((float)temp / 4096) * 36000);           
            
//            if(encoder > UDzero)
//                {
//                    encoder = encoder - UDzero;
//                    encoder =  encoder * 8.789;
//                    Transmit_position_to_motor(UDmotor, 1, 5000, encoder);
//
//                }
//                else if(encoder == UDzero)
//                {
//                    encoder = encoder - UDzero;
//                    encoder =  encoder * 8.789;
//                    Transmit_position_to_motor(UDmotor, 1, 5000, encoder);
//
//                }
//                else if(encoder < UDzero)
//                {
//                    encoder = UDzero - encoder;
//                    encoder =  encoder * 8.789;
//                    Transmit_position_to_motor(UDmotor, 1, 5000, encoder);
//                }  
            
            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }
        }
        rbegin485 = 0;
        rindex485 = 0;
    }
    
    temp = encoder;
    if(temp >0)
    {
        for (n = 0; temp > 0; n++)
        {
          temp /= 10;
        }
    }
    else
    {
        n = 1;  
    }
//    itoa(Axis1, i, 10);
    sprintf(i, "%ld", encoder);
//    putsU2_characters(i);
//    putsU2(&encoder,4);
//    putsU2(i,5);
    for(j = 2; j < 8; j++)
    {
        back[j] = '0';
    }

    for(k = 0; n > 0; n--,k++)
    {
        back[7 - k] = i[n - 1];
    } 
    
    
    Transmit485 = 1;
    putsU1(&(Stop_motor[LRmotor].Speeddata[0]) ,10);    
    Delayus(2200);
    
    Transmit485 = 0;
    Delayus(1000);

    if(rbegin485 == 1)
    {
        Delayus(3000);
        if(rfinish485 != 1)
        {
            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }
        }
        else
        {
            rindex485 = 0;
            rfinish485 = 0;
            /*���ճɹ�������*/

            encoder = (Rmotor.Rdata_analyze.encoder2 * 256) + Rmotor.Rdata_analyze.encoder1;
            if(encoder >LRzero)
                {
                    encoder = encoder -LRzero;
                }               
                else if(encoder == LRzero)
                {
                    encoder = 0;
                }
                else if(encoder < LRzero)
                {
                    encoder = 4096 - (LRzero - encoder)  ;
                }
                encoder =  encoder * 8.789;            //(((float)temp / 4096) * 36000);
                
                encoder = ((round - 1) * 36000) + encoder;
                Transmit_position_to_motor(LRmotor, 1, 2000, encoder );
//                Transmit_position_to_motor(LRmotor, 1, 5000, encoder);
            //encoder =  encoder * 8.789;            //(((float)temp / 4096) * 36000);
            
            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }
        }
        rbegin485 = 0;
        rindex485 = 0;
    }
    temp = encoder;
    if(temp >0)
    {
        for (n = 0; temp > 0; n++)
        {
           temp /= 10;
        }
    }
    else
    {
        n = 1;  
    }
    //itoa(Axis2, i, 10);
    sprintf(i, "%ld", encoder);
//    for(j = 14; j < 19; j++)
//    {
//        back[j] = '0';
//    }
//  
//  for(k = 0; n > 0; n--,k++)
//  {
//        back[18 - k] = i[n - 1];
//  } 
    for(j = 14; j < 20; j++)
    {
        back[j] = '0';
    }
  
  for(k = 0; n > 0; n--,k++)
  {
        back[19 - k] = i[n - 1];
  } 
  for(k = 0; k < 25; k++)
    {
        rdata.data[k] = 0;
    }
    back[11] = '0' + inverse;
//    _LATB7 = ~_LATB7 ;
  putsU2(back,25);
}
/*���ݻ����˷��͹����ĽǶ���Ϣ���õ��λ��*/
void set_position(void)
{
//    int temp;
//    unsigned int encoder;
    unsigned long int angle;
//    unsigned long int abc;
    unsigned char temp[6];
    int i;
    for(i = 0; i < 5; i++)
    {
        temp[i] = rdata.data[2 + i];
    }
    temp[5] = 'A';
    angle = atoi(temp);

//    putsU2(&angle,4);

        Transmit_position_to_motor(UDmotor, 1, 5000, angle * 10);

    
    
    for(i = 0; i < 5; i++)
    {
        temp[i] = rdata.data[14 + i];
    }

    temp[5] = 'A';
    angle = atoi(temp);
    
    angle = angle % 3600;
//    putsU2(&angle,4);
    Transmit_position_to_motor(LRmotor, 1, 5000, angle * 10 + 36000 * (round - 1));//�˴�֮����angle * 10����Ϊ�Ƕ���Ϣʵ����6λ��
                                                             //���Ǹú�����ֻ����5λ�����* 10�����Ե����һλ��Ӱ��������λ�õ�׼ȷ��                           
}

//void Sprocessdatatype(void)
//{
//    char temp[6];
//    int i;
//    for(i = 0; i < 5; i++)
//    {
//        temp[i] = rdata.data[2 + i];
//    }
//    temp[5] = 'A';
//    pdata.axis1_angle = atoi(temp);
//    
//    for(i = 0; i < 5; i++)
//    {
//        temp[i] = rdata.data[14 + i];
//    }
//    temp[5] = 'A';
//    pdata.axis2_angle = atoi(temp);
//    temp[5] = 'A';
//}

char Transmit_position_to_motor(char motor_num, char dir, int speed, long int angle)
{
    char temp = 0;
    int i;
    Transmit485 = 1;
    if(dir == 1)
    {
        angle =angle;
        Tmotor[motor_num].Sdata_analyze.angle[3] = 0x00;
        Tmotor[motor_num].Sdata_analyze.angle[4] = 0x00;
        Tmotor[motor_num].Sdata_analyze.angle[5] = 0x00;
        Tmotor[motor_num].Sdata_analyze.angle[6] = 0x00;
        Tmotor[motor_num].Sdata_analyze.angle[7] = 0x00;
    }
    if(dir == 0)
    {
        angle = 0 - angle;
        Tmotor[motor_num].Sdata_analyze.angle[3] = 0xFF;
        Tmotor[motor_num].Sdata_analyze.angle[4] = 0xFF;
        Tmotor[motor_num].Sdata_analyze.angle[5] = 0xFF;
        Tmotor[motor_num].Sdata_analyze.angle[6] = 0xFF;
        Tmotor[motor_num].Sdata_analyze.angle[7] = 0xFF;
    }
    
    Tmotor[motor_num].Sdata_analyze.angle[0] = angle & 0xFF;
    Tmotor[motor_num].Sdata_analyze.angle[1] = (angle >> 8) & 0xFF;
    Tmotor[motor_num].Sdata_analyze.angle[2] = (angle >> 16) & 0xFF;
    Tmotor[motor_num].Sdata_analyze.speed[0] = speed & 0xFF;
    Tmotor[motor_num].Sdata_analyze.speed[1] = (speed >> 8) & 0xFF;
    
    for(i = 5; i < 17; i++)
    {
        temp = temp + Tmotor[motor_num].Sdata[i] ;
    }
    
    Tmotor[motor_num].Sdata_analyze.check6_17 = temp;
    putsU1(&(Tmotor[motor_num].Sdata[0]), 18);
    Delayus(2200);
    Transmit485 = 0;
    
    Delayus(800);

    if(rbegin485 == 1)
    {
        Delayus(3000);    
        if(rfinish485 != 1)
        {
            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }
            rbegin485 = 0;
            rindex485 = 0;
            return FAILE;
        }
        else
        {
            rindex485 = 0;
            rfinish485 = 0;
            /*���ճɹ�������*/

            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }
            return SUCCESS;

        }
        
    }
    rbegin485 = 0;
    rindex485 = 0;
    return FAILE;
}

//char Transmit_speed_to_motor(char motor_num, char dir, int speed)
//{
//    char temp = 0;
//    int i;
//    Transmit485 = 1;
//    if(dir == 1)
//    {
//        LR_speed_motor.Speeddata_analyze.speed[0] = speed & 0xFF;
//        LR_speed_motor.Speeddata_analyze.speed[1] = (speed >> 8) & 0xFF;
//        LR_speed_motor.Speeddata_analyze.speed[2] = 0x00;
//        LR_speed_motor.Speeddata_analyze.speed[3] = 0x00;
//    }
//    if(dir == 0)
//    {
//        speed = 0 - speed;
//        LR_speed_motor.Speeddata_analyze.speed[0] = speed & 0xFF;
//        LR_speed_motor.Speeddata_analyze.speed[1] = (speed >> 8) & 0xFF;
//        LR_speed_motor.Speeddata_analyze.speed[2] = 0xFF;
//        LR_speed_motor.Speeddata_analyze.speed[3] = 0xFF;
//    }
//
//    for(i = 5; i < 9; i++)
//    {
//        temp = temp + LR_speed_motor.Speeddata[i] ;
//    }
//    LR_speed_motor.Speeddata_analyze.check6_9 = temp;
//    putsU1(&(LR_speed_motor.Speeddata[0]), 10);
//    Delayus(2200);
//    Transmit485 = 0;
//    
//    Delayus(1000);
//
//    if(rbegin485 == 1)
//    {
//        Delayus(3000);    
//        if(rfinish485 != 1)
//        {
//            for(p = 0; p < 8; p++)
//            {
//                Rmotor.Rdata[p] = 0;
//            }
//            rbegin485 = 0;
//            rindex485 = 0;
//            return FAILE;
//        }
//        else
//        {
//            rindex485 = 0;
//            rfinish485 = 0;
//            /*���ճɹ�������*/
//
//            for(p = 0; p < 8; p++)
//            {
//                Rmotor.Rdata[p] = 0;
//            }
//            return SUCCESS;
//
//        }
//        
//    }
//    rbegin485 = 0;
//    rindex485 = 0;
//    return FAILE;
//}

void Init_485TX(void)
{
    _TRISB3 = 0;
    
    RPOR0bits.RP35R = 0x01;
    //
    // set up the UART for default baud, 1 start, 1 stop, no parity
    //
    U1MODEbits.STSEL = 0;       // 1-Stop bit
    U1MODEbits.PDSEL = 0;       // No Parity, 8-Data bits
    U1MODEbits.ABAUD = 0;       // Auto-Baud disabled
    U1MODEbits.BRGH = 0;        // Standard-Speed mode
    U1BRG = BAUD19200;          // Baud Rate setting for 38400 (default)
    U1STAbits.UTXISEL0 = 0;     // Interrupt after TX buffer done
    U1STAbits.UTXISEL1 = 1;
    IEC0bits.U1TXIE = 0;        // Enable UART TX interrupt
    U1MODEbits.UARTEN = 1;      // Enable UART (this bit must be set *BEFORE* UTXEN)
    
    U1STAbits.UTXEN = 1;
}
void Init_485RX(void)
{    
    ANSELBbits.ANSB2 = 0;     //digital
    _TRISB2 = 1;                    // digital input pin
    RPINR18 = 34;     

    // set up the UART for LIN_BRGVAL baud, 1 start, 1 stop, no parity
    //
    U1MODEbits.STSEL = 0;           // 1-Stop bit
    U1MODEbits.PDSEL = 0;           // No Parity, 8-Data bits
    U1MODEbits.ABAUD = 0;           // Auto-Baud disabled
    U1MODEbits.BRGH = 0;            // Standard-Speed mode
    U1BRG = BAUD19200;             // Baud Rate setting
    U1STAbits.URXISEL1 = 0;          // Interrupt after one RX done
    U1STAbits.URXISEL0 = 0;
    IEC0bits.U1RXIE = 1;            // Enable UART1 RX interrupt
    IEC4bits.U1EIE = 1;             // Enable Error (Framing) Interrupt for BREAK
    U1MODEbits.UARTEN = 1;          // Enable UART1
}

void putsU12(char *s)
{
    int i = 10;
    while (i--)
    { // loop until *s =\0, end of string
        putU1(*s++);
    } // send the character and point to the next one
}

void putsU1(char *s, char num)
{
    int i = num;
    while (i--)
    { // loop until *s =\0, end of string
        putU1(*s++);
    } // send the character and point to the next one
}
void putsU1_characters(char *s)
{
    while (*s)
    { // loop until *s =\0, end of string
        putU1(*s++);
    } // send the character and point to the next one
}
void putU1(int c)
{
    while (U1STAbits.UTXBF); // wait while Tx buffer full
    U1TXREG = c;
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{
    while (U1STAbits.TRMT == 0); // wait for transmitter empty
    IFS0bits.U1TXIF = 0; // Clear TX1 Interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
    rbegin485 = 1;
    while(U1STAbits.URXDA == 1)
    {
        datal485 = U1RXREG;
        if(U1STAbits.OERR == 1) //�����ڽ������
        {
            while(U1STAbits.URXDA == 1)
            {
                datal485 = U1RXREG;      
            }
            rindex485 = 0;
            U1STAbits.OERR = 0;
            for(p = 0; p < 8; p++)
            {
                Rmotor.Rdata[p] = 0;
            }
            break;
        }
//        if( ((datal485 & 0xFF) == 0x3E) || (Rmotor.Rdata[0] == 0x3E) )
//        {
            if(rindex485 >= 8)
            {
                rindex485 = 0;
            }
            Rmotor.Rdata[rindex485] = datal485 & 0xFF;
            rindex485++;
            
//        }
        
    }
    

    if(rindex485 >= 8)
    {
        if(Rmotor.Rdata[7] == (Rmotor.Rdata[5] + Rmotor.Rdata[6]))
        {
            rfinish485 = 1;
        }     
        rindex485 = 0;
    }
    
    IFS0bits.U1RXIF = 0;                // Clear RX1 Interrupt flag
}
