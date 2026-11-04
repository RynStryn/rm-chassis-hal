#ifndef __USART3_H
#define __USART3_H
#include <stdint.h>
#include <stdbool.h>




  /************************** DMA��غ궨�� ********************************************/
  #define USART_CH040_DR_ADDRESS                ((uint32_t)USART3 + 0x04) 

  #define USART_CH040_DMA                       DMA1
  #define USART_CH040_DMAx_CLK                  RCC_AHB1Periph_DMA1
     
  #define USART_CH040_TX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH040_TX_DMA_STREAM             DMA1_Stream3
  #define USART_CH040_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
  #define USART_CH040_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
  #define USART_CH040_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
  #define USART_CH040_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
  #define USART_CH040_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
              
  #define USART_CH040_RX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH040_RX_DMA_STREAM             DMA1_Stream1
  #define USART_CH040_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
  #define USART_CH040_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
  #define USART_CH040_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
  #define USART_CH040_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
  #define USART_CH040_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1
  
  
  
 /*************************************��������С�궨��****************************************************/
  #define CH040_RX_BUFF_SIZE                     100
  
  
  
  #define CH040_DATA_FARMER_LENGHT               82
  #define CH040_FRAMER_HEADER_LENGHT             6 




/***************************�ṹ�嶨��*****************************/
typedef __packed struct
{
    uint8_t     tag;              /* ���ݰ���ǩ��0x91        */
    uint16_t    main_status;      /* ״̬��*/
    int8_t      temperature;      /* ģ��ƽ���¶�*/
    float       pressure;         /* ��ѹ*/
    uint32_t    timestamp;        /* ʱ���             */
    float       acc[3];           /* ���ٶȣ�˳��Ϊ��XYZ��       */
    float       gyr[3];           /* ���ٶȣ�˳��Ϊ��XYZ��      */  
    float       mag[3];           /* ��ǿ�ȣ�˳��Ϊ��XYZ��        */
    float       eul[3];           /* ŷ���ǣ�˳��Ϊroll��pitch��yaw */
    float       quat[4];          /* �ڵ���Ԫ�����ϣ�˳��ΪWXYZ  */
    
    
}imu_data_t;                   //������ԭʼ���ݽṹ��


typedef struct 
{
    float Pitch_Angle;         //ŷ���ǵ�λ deg
    float Yaw_Angle;
    float Roll_Angle;
    
    float Yaw_Multi_Angle;
    
	float Pitch_Gyro_Omega;    //pitch ������ ���ٶ�  deg/s
	float Yaw_Gyro_Omega;      //yaw ������ ���ٶ�  deg/s
	float Roll_Gyro_Omega;     //roll ������ ���ٶ�  deg/s
    
    
	float X_Acc;
	float Y_Acc;
	float Z_Acc;
}CH040DATA_t;                  //CH040���ݽṹ��
/***************************�ṹ�嶨��*****************************/








/******************************��������*********************************/
void USART3_Init(uint32_t baud_rat);
void CH040_Data_Get(imu_data_t* imu_data , CH040DATA_t* CH040DATA);
/******************************��������*********************************/






#endif
