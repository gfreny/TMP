#include "stm32f10x.h"
#include "platform_config.h"


//����NPN
#define SetD0Pin_H GPIO_ResetBits(WG_D0_PORT, WG_D0_PIN)
#define SetD0Pin_L GPIO_SetBits(WG_D0_PORT, WG_D0_PIN)
#define SetD1Pin_H GPIO_ResetBits(WG_D1_PORT, WG_D1_PIN)
#define SetD1Pin_L GPIO_SetBits(WG_D1_PORT, WG_D1_PIN)

//TIME2��ʼ��
volatile unsigned  short TimesetupSave=0;
void TIM3_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_DeInit(TIM3);                               
     
    /* TIM3 clock enable [TIM3��ʱ������]*/ 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /* TIM3 configuration */ 
    TIM_TimeBaseStructure.TIM_Period = 47;     //10ms�����ֽڼ�    10*36000/72000000*1000      
    TIM_TimeBaseStructure.TIM_Prescaler = 399;//35999;    	    
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 

    /* Clear TIM3 update pending flag*/ 
    TIM_ClearFlag(TIM3, TIM_FLAG_Update); 

    /* Enable TIM3 Update interrupt*/ 
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);   

    /* TIM3 enable counter */ 
	TIM_Cmd(TIM3, ENABLE); 	
}

volatile unsigned char rcr[2];
volatile unsigned int wg_data[2];
volatile unsigned short tigg_sum;//25 or 33
volatile unsigned char tigg_save;//��¼
volatile unsigned char us100_step;//Сʱ��Ƭ�μ�¼
volatile unsigned char crc_enable=0;//0Ϊ����CRC,other������CRC
void TIM3_irq(void)
{
	   if (TIM3->SR & (1<<0)) 
   // if(TIMER_GetIntBitState(TIMER2,TIMER_INT_UPDATE)==ENABLE)
    {       
       // TIMER_ClearIntBitState(TIMER2,TIMER_INT_UPDATE);
        TIM_ClearFlag(TIM3, TIM_FLAG_Update);
        //�ж��Ƿ����
        
        if(us100_step==0)//����һ��bit
        {
            if(tigg_save==0)//żУ��
            {
                if(rcr[0]&0x01)
                    SetD1Pin_L;
                else
                    SetD0Pin_L;
            }
            else if(tigg_save>=tigg_sum)//��У��
            {
               if(rcr[1]&0x01)
                    SetD1Pin_L;
                else
                    SetD0Pin_L;
            }
            else//���ݲ���
            { 
                if(wg_data[0]&(1<<(tigg_sum-1-tigg_save)))
                    SetD1Pin_L;
                else
                    SetD0Pin_L;  
                
                
            }
            us100_step+=1;    
        }
        else if(us100_step==1)//
        {
            us100_step+=1;
            SetD1Pin_H;
            SetD0Pin_H;
        }
        else if(us100_step>=4)//bit�ѷ������
        {
           us100_step=0;
           if(crc_enable==0)
           {
           	if(tigg_save>=tigg_sum)//�Ѿ���ɷ���
           	{
                //TIMER_Enable( TIMER2, DISABLE );
                TIM_Cmd(TIM3, DISABLE); 
           	}
           	else
           	{
                tigg_save+=1;
           	}      
           }
           else//������CRC
           {
           		if(tigg_save>=(tigg_sum-1))//�Ѿ���ɷ���
           		{
                //TIMER_Enable( TIMER2, DISABLE );
                TIM_Cmd(TIM3, DISABLE); 
           		}
           		else
           		{
                tigg_save+=1;
           		}  
           }
           
           
           
                
        } 
        else
        {
            us100_step+=1;
        }            
        
    }
}
/*
void Sent_WG26(unsigned char *id)
{
   unsigned char i,crc,rbuf[4];
   wg_data[0]=id[2];
   wg_data[0]=((wg_data[0]<<8)&0xff00)+id[1]; 
   wg_data[0]=((wg_data[0]<<8)&0xffff00)+id[0];
      
    //��12bitż����  
   rcr[0]=0;
   for(i=0;i<12;)  //żУ�� 
   {
       if((wg_data[0]&(1<<23-i))!=0)
       {    rcr[0]+=1;}
       i+=1;
   }  
   //��12bit��У��
   rcr[1]=1;
   for(i=0;i<12;)  //��У�� 
   {
       if((wg_data[0]&(1<<11-i))!=0)
       {    rcr[1]+=1;}
       i+=1;
   }  
   tigg_sum=25;
   tigg_save=0;
   us100_step=0;
   crc_enable=0;//0Ϊ����CRC,other������CRC
   TIM3_Init();
}
void Sent_WG34(unsigned char *id)
{
   unsigned char i,crc,rbuf[4];
   wg_data[0]=id[3];
   wg_data[0]=((wg_data[0]<<8)&0xff00)+id[2]; 
   wg_data[0]=((wg_data[0]<<8)&0xffff00)+id[1];
   wg_data[0]=((wg_data[0]<<8)&0xffffff00)+id[0];
   rcr[0]=0;
   for(i=0;i<16;)  //żУ�� 
   {
       if((wg_data[0]&(1<<31-i))!=0)
           rcr[0]+=1;
       i+=1;
   }  
   rcr[1]=1;
   for(i=0;i<16;)  //��У�� 
   {
       if((wg_data[0]&(1<<15-i))!=0)
           rcr[1]+=1;
       i+=1;
   }  
   tigg_sum=33;
   tigg_save=0;
   us100_step=0;
   crc_enable=0;//0Ϊ����CRC,other������CRC
   TIM3_Init();
}*/
void Sent_WG4noCRC(unsigned char *id)
{
   unsigned char i,crc,rbuf[4];
   wg_data[0]=id[0];  
   tigg_sum=5;
   tigg_save=1;
   us100_step=0;
   crc_enable=0xff;//0Ϊ����CRC,other������CRC
   TIM3_Init();
}
//---------------------------
//id����Ϊǰ�ߺ��
void Sent_WG26HL(unsigned char *id,unsigned char HorL)
{
   unsigned char i,crc,rbuf[4];
	
	 if(HorL==0)//ǰ�ߺ��
	 {
	 wg_data[0]=id[1];
   wg_data[0]=((wg_data[0]<<8)&0xff00)+id[2]; 
   wg_data[0]=((wg_data[0]<<8)&0xffff00)+id[3];
	 }
	 else//ǰ�ͺ��
	 {
		wg_data[0]=id[2];
		wg_data[0]=((wg_data[0]<<8)&0xff00)+id[1]; 
		wg_data[0]=((wg_data[0]<<8)&0xffff00)+id[0];
	 }	
	     
    //��12bitż����  
   rcr[0]=0;
   for(i=0;i<12;)  //żУ�� 
   {
       if((wg_data[0]&(1<<23-i))!=0)
       {    rcr[0]+=1;}
       i+=1;
   }  
   //��12bit��У��
   rcr[1]=1;
   for(i=0;i<12;)  //��У�� 
   {
       if((wg_data[0]&(1<<11-i))!=0)
       {    rcr[1]+=1;}
       i+=1;
   }  
   tigg_sum=25;
   tigg_save=0;
   us100_step=0;
   crc_enable=0;//0Ϊ����CRC,other������CRC
   TIM3_Init();
}
//id����Ϊǰ�ߺ��
void Sent_WG34HL(unsigned char *id,unsigned char HorL)
{
   unsigned char i,crc,rbuf[4];
	
	 if(HorL==1)//ǰ�ߺ��
	 {
	 wg_data[0]=id[0];
   wg_data[0]=((wg_data[0]<<8)&0xff00)+id[1]; 
   wg_data[0]=((wg_data[0]<<8)&0xffff00)+id[2];
   wg_data[0]=((wg_data[0]<<8)&0xffffff00)+id[3];
	 }
	 else//ǰ�ͺ��
	 {
		wg_data[0]=id[3];
   wg_data[0]=((wg_data[0]<<8)&0xff00)+id[2]; 
   wg_data[0]=((wg_data[0]<<8)&0xffff00)+id[1];
   wg_data[0]=((wg_data[0]<<8)&0xffffff00)+id[0];
	 }	
   rcr[0]=0;
   for(i=0;i<16;)  //żУ�� 
   {
       if((wg_data[0]&(1<<31-i))!=0)
           rcr[0]+=1;
       i+=1;
   }  
   rcr[1]=1;
   for(i=0;i<16;)  //��У�� 
   {
       if((wg_data[0]&(1<<15-i))!=0)
           rcr[1]+=1;
       i+=1;
   }  
   tigg_sum=33;
   tigg_save=0;
   us100_step=0;
   crc_enable=0;//0Ϊ����CRC,other������CRC
   TIM3_Init();
}