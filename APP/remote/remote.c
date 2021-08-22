#include "remote.h"
#include "control.h"
#include "rc.h"
#include "led.h"
#include "iwdg.h"
#include "rc.h"

/* �������ʱ��û���յ��µ�ң�������ݾ���Ϊ�Ѿ�ʧ�� */
#define REMOTE_LOST_TIME    ((uint32_t)50)   //50ms
#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

/* ��Ŷ�ȡ����ң�������� */
RC_ctrl_t RC_Ctl;
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

portTickType ulRemoteLostTime = 0;

/* ��ȡң����ʧ������ʱ */
uint32_t REMOTE_ulGetLostTime( void )
{
	/* */
	return  ulRemoteLostTime;
}

/* ��λ���� */
void REMOTE_vResetData( void )
{
	/* Channel 0, 1, 2, 3 */
	RC_Ctl.rc.ch[0] = RC_CH_VALUE_OFFSET;
	RC_Ctl.rc.ch[1] = RC_CH_VALUE_OFFSET;
	RC_Ctl.rc.ch[2] = RC_CH_VALUE_OFFSET;
	RC_Ctl.rc.ch[3] = RC_CH_VALUE_OFFSET;

	/* Switch left, right */
	RC_Ctl.rc.s[0]  = RC_SW_MID;
	RC_Ctl.rc.s[1]  = RC_SW_MID;

	/* Mouse axis: X, Y, Z */
	RC_Ctl.mouse.x = 0;
	RC_Ctl.mouse.y = 0;
	RC_Ctl.mouse.z = 0;

	/* Mouse Left, Right Is Press ? */
	RC_Ctl.mouse.press_l = 0;
	RC_Ctl.mouse.press_r = 0;

	/* KeyBoard value */
	RC_Ctl.key.v = 0;
}

/* ˢ��ʧ��ʱ�� */
void REMOTE_vUpdateLostTime( void )
{
	/* �����յ�����ʱ��ˢ��ʧ������ʱ */
	ulRemoteLostTime = xTaskGetTickCount( ) + REMOTE_LOST_TIME;
}

//ң�����ݻ���
bool_t REMOTE_IfDataError( void )
{
	if ( (RC_Ctl.rc.s[0] != RC_SW_UP && RC_Ctl.rc.s[0] != RC_SW_MID && RC_Ctl.rc.s[0] != RC_SW_DOWN)
		|| (RC_Ctl.rc.s[1] != RC_SW_UP && RC_Ctl.rc.s[1] != RC_SW_MID && RC_Ctl.rc.s[1] != RC_SW_DOWN)
		|| (RC_Ctl.rc.ch[0] > RC_CH_VALUE_MAX || RC_Ctl.rc.ch[0] < RC_CH_VALUE_MIN)
		|| (RC_Ctl.rc.ch[1] > RC_CH_VALUE_MAX || RC_Ctl.rc.ch[1] < RC_CH_VALUE_MIN)
		|| (RC_Ctl.rc.ch[2] > RC_CH_VALUE_MAX || RC_Ctl.rc.ch[2] < RC_CH_VALUE_MIN)
		|| (RC_Ctl.rc.ch[3] > RC_CH_VALUE_MAX || RC_Ctl.rc.ch[3] < RC_CH_VALUE_MIN) )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/* ��ȡ���ڻ�������ݣ�����ң����ͨ��Э��ת��Ϊң��������
   ���ջ��������ݵ�Ƶ����ÿ�� 7ms ����һ֡
   Ϊ��ֹ��DMAд�뻺���ͬʱ��ȡ�����е����ݣ����ܵ��¶�ȡ����������Ч����
   ֱ�����жϷ������ж�ȡң�������ݡ�
   ��һ�ַ����Ƕ���໺�棬����Ҫ���³�ʼ��DMA��Ȩ��֮��
   ����ʹ����һ�ַ�����ֱ�Ӽ򵥡�
*/
void REMOTE_vReadData(char *pucRxBuffer )
{
	if(pucRxBuffer == NULL)
	{
	return;
	}
	/* ��������ת��Ϊң�������� */

	/* �ֽ�λ�������ݵĶ�Ӧ��ϵ��
		 �ֽ�    ����
		 ��0��   0000 0000    0��ch0        uint16_t  ʵ����Чλ�� 11 λ
		 ��1��   1111 1000    1��ch1        uint16_t
		 ��2��   2211 1111    2��ch2        uint16_t
		 ��3��   2222 2222    3��ch3        uint16_t
		 ��4��   3333 3332    
		 ��5��   1122 3333    1��s1  2��s2  uint8_t   ʵ����Чλ�� 2 λ
		 ��6��   xxxx xxxx    x��mouse.x    int16_t   ʵ����Чλ�� 16 λ
		 ��7��   xxxx xxxx    
		 ��8��   yyyy yyyy    y��mouse.y    int16_t   ʵ����Чλ�� 16 λ
		 ��9��   yyyy yyyy
		 ��10��  zzzz zzzz    z��mouse.z    int16_t   ʵ����Чλ�� 16 λ
		 ��11��  zzzz zzzz
		 ��12��  llll llll    l��mouse.left   uint8_t   ʵ����Чλ�� 8 λ
		 ��13��  rrrr rrrr    r��mouse.right  uint8_t   ʵ����Чλ�� 8 λ
		 ��14��  kkkk kkkk    k��key          uint16_t  ʵ����Чλ�� 16 λ
		 ��15��  kkkk kkkk    k��key

		 �ܹ�16���ֽڣ���Чλ�� 128 = 11*4 + 2*2 + 16*3 + 8*2 + 16*1         */

	/* Channel 0, 1, 2, 3 */
	RC_Ctl.rc.ch[0] = (  pucRxBuffer[0]       | (pucRxBuffer[1] << 8 ) ) & 0x07ff;
	RC_Ctl.rc.ch[1] = ( (pucRxBuffer[1] >> 3) | (pucRxBuffer[2] << 5 ) ) & 0x07ff;
	RC_Ctl.rc.ch[2] = ( (pucRxBuffer[2] >> 6) | (pucRxBuffer[3] << 2 ) | (pucRxBuffer[4] << 10) ) & 0x07ff;
	RC_Ctl.rc.ch[3] = ( (pucRxBuffer[4] >> 1) | (pucRxBuffer[5] << 7 ) ) & 0x07ff;

	

	/* Switch left, right */
	RC_Ctl.rc.s[0]  = ( (pucRxBuffer[5] >> 4) & 0x000C ) >> 2;
	RC_Ctl.rc.s[1]  = ( (pucRxBuffer[5] >> 4) & 0x0003 );

	/* Mouse axis: X, Y, Z */
	RC_Ctl.mouse.x = pucRxBuffer[6]  | (pucRxBuffer[7 ] << 8);
	RC_Ctl.mouse.y = pucRxBuffer[8]  | (pucRxBuffer[9 ] << 8);
	RC_Ctl.mouse.z = pucRxBuffer[10] | (pucRxBuffer[11] << 8);

	/* Mouse Left, Right Is Press ? */
	RC_Ctl.mouse.press_l = pucRxBuffer[12];
	RC_Ctl.mouse.press_r = pucRxBuffer[13];

	/* KeyBoard value */
	RC_Ctl.key.v = pucRxBuffer[14] | (pucRxBuffer[15] << 8);
	
	/* ���ݳ����������� */
	if( REMOTE_IfDataError() == TRUE )//ʵ��
	{
		led_red_on();
		REMOTE_vResetData();
		SYSTEM_OutCtrlProtect();
	}
	else
	{
		IWDG_Feed();//ι��
	}

}



//�����ж�
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART1);
    }
    else if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART1);
        if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
        {
            //��������DMA
            DMA_Cmd(DMA2_Stream2, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream2);
            DMA_SetCurrDataCounter(DMA2_Stream2, SBUS_RX_BUF_NUM);
            DMA2_Stream2->CR |= DMA_SxCR_CT;
            //��DMA�жϱ�־
            DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA2_Stream2, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                SBUS_TO_RC(SBUS_rx_buf[0], &RC_Ctl);
            }
        }
        else
        {
            //��������DMA
            DMA_Cmd(DMA2_Stream2, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream2);
            DMA_SetCurrDataCounter(DMA2_Stream2, SBUS_RX_BUF_NUM);
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);
            //��DMA�жϱ�־
            DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA2_Stream2, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                SBUS_TO_RC(SBUS_rx_buf[1], &RC_Ctl);
            }

        }
				REMOTE_vUpdateLostTime();
    }
}

static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &
                        0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right

    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
   
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
   
}


void remote_control_init(void)
{
    RC_Init(SBUS_rx_buf[0], SBUS_rx_buf[1], SBUS_RX_BUF_NUM);
}


//����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
const RC_ctrl_t *get_remote_control_point(void)
{
    return &RC_Ctl;
}
