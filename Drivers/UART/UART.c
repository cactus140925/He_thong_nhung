#include "UART.h"

static volatile uint8_t uart1_tx_dma_busy = 0u; //doc trang thai DMA

static void UART1_GPIO_Init(void)
{
    //bat clock GPIOA va AFIO
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;

    //PA9: TX
    GPIOA->CRH &= ~(0xFu << 4); //PA9: 7-4: cau hinh PA9, 0XF: 1111
    GPIOA->CRH |=  (0xBu << 4); //B: 1011 -> CNF:10 (AFIO), Mode:11 (output 50MHz)

    //PA10: RX
    GPIOA->CRH &= ~(0xFu << 8); //PA10: 11-8: cau hinh PA10, 0XF: 1111
    GPIOA->CRH |=  (0x4u << 8); //4: 0100 -> CNF:01 (floating input), Mode:00 (input)
}

void UART1_Init(uint32_t baudrate)
{
    UART1_GPIO_Init();

    //bat clock USART1 thuoc APB2
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    //RESET cac thanh ghi dieu khien: 8bit data, no party, 1 stop bit
    USART1->CR1 = 0u;
    USART1->CR2 = 0u;
    USART1->CR3 = 0u;

    //cau hinh baudrate
    USART1->BRR = (UART1_PCLK_HZ + (baudrate / 2u)) / baudrate; //BRR=24=0X18 -> USART1: 3Mbps

    //bat TX, RX
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
    //bat toan bo UART
    USART1->CR1 |= USART_CR1_UE;
}

void UART1_SendChar(char c)
{
    //cho TX enable: state register
    while ((USART1->SR & USART_SR_TXE) == 0u)
    {
    }

    USART1->DR = (uint8_t)c; //ghi du lieu vao data register
}

void UART1_SendString(const char *str)
{
    while (*str != '\0')
    {
        UART1_SendChar(*str);
        str++;
    }
}

void UART1_SendBufferBlocking(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    for (i = 0u; i < length; i++)
    {
        UART1_SendChar((char)data[i]);
    }
}

uint8_t UART1_DataAvailable(void)
{
    return (USART1->SR & USART_SR_RXNE) ? 1u : 0u;
}

uint8_t UART1_ReadByte(void)
{
    while ((USART1->SR & USART_SR_RXNE) == 0u)
    {
    }

    return (uint8_t)USART1->DR;
}

void UART1_DMA_Init(void)
{
    //bat clock DMA1 thuoc AHB
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    //cau hinh DMA TX CHANNEL4, RX CHANNEL5
    //CCR: Channel Configuration Register
    DMA1_Channel4->CCR = 0u; //tat cau hinh channel4
    //Channel Peripheral Address Register
    DMA1_Channel4->CPAR = (uint32_t)&USART1->DR; //dia chi ngoai vi de gui du lieu toi
    DMA1_Channel4->CCR = DMA_CCR_DIR //huong truyen: RAM->UART
                       | DMA_CCR_MINC //sau khi doc 1 byte tu RAM, dia chi RAM tu tang
                       | DMA_CCR_PL_0 //co 2 bit priority: 01 o muc medium
                       | DMA_CCR_TCIE //Transfer Complete Interrupt Enable: khi DMA truyen xong,toan bo buffer phat ngat
                       | DMA_CCR_TEIE; //Transfer Error Interrupt Enable: neu DMA loi -> phat ngat
    DMA1_Channel5->CCR = 0u;
    DMA1_Channel5->CPAR = (uint32_t)&USART1->DR; //huong UART -> RAM
    DMA1_Channel5->CCR = DMA_CCR_MINC
                       | DMA_CCR_CIRC //Circular Mode: DMA quay vong
                       | DMA_CCR_PL_0;

    //UART phat yeu cau DMA
    //DMAT: DMA Transmitter Enable, cho phep UART tao request DMA khi can truyen
    //DMAR: DMA Receiver Enable, cho phep UART tao request DMA khi nhan du lieu
    USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;

    NVIC_SetPriority(DMA1_Channel4_IRQn, 2u); //dat priority cua ngat o muc 2
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

uint8_t UART1_SendDMA(const uint8_t *data, uint16_t length)
{
    if ((data == 0) || (length == 0u) || uart1_tx_dma_busy)
    {
        return 0u;
    }

    DMA1_Channel4->CCR &= ~DMA_CCR_EN; //tat channel truoc khi cau hinh
    DMA1->IFCR = DMA_IFCR_CGIF4;       //xoa co DMA cu cua channel 4 truoc lan truyen moi

    //dua dia chi buffer vao DMA
    //CMAR: Channel Memory Address Register, dia chi packet
    DMA1_Channel4->CMAR = (uint32_t)data;
    //CNDTR: Channel Number of Data Register, dua so byte can truyen
    DMA1_Channel4->CNDTR = length;

    uart1_tx_dma_busy = 1u;
    //bat DMA
    DMA1_Channel4->CCR |= DMA_CCR_EN;

    return 1u;
}
//ham bat dau DMA
void UART1_StartRxDMA(uint8_t *buffer, uint16_t length)
{
    DMA1_Channel5->CCR &= ~DMA_CCR_EN; //tat DMAA channel 5
    DMA1->IFCR = DMA_IFCR_CGIF5; //xoa co DMA cu

    DMA1_Channel5->CMAR = (uint32_t)buffer; //dia chi packet
    DMA1_Channel5->CNDTR = length; //so byte can truyen

    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

//lay vi tri DMA RX
uint16_t UART1_RxDMA_GetPosition(uint16_t buffer_size)
{
    return (uint16_t)(buffer_size - DMA1_Channel5->CNDTR);
}

uint8_t UART1_TxDMA_Busy(void)
{
    return uart1_tx_dma_busy;
}

//ngat TX: IRQ: Interrupt Service Routine
void DMA1_Channel4_IRQHandler(void)
{
    //kiem tra DMA da chuyen xong chua
    if (DMA1->ISR & DMA_ISR_TCIF4)
    {
        //CTCIF4: Clear Transfer Complete Interrupt Flag Channel 4, xoa co transfer
        DMA1->IFCR = DMA_IFCR_CTCIF4;
        //tat DMA 
        DMA1_Channel4->CCR &= ~DMA_CCR_EN;
        uart1_tx_dma_busy = 0u;
        UART1_TxCompleteCallback();
    }

    //Transfer Error Interrupt Flag Channel 4: kiem tra loi
    if (DMA1->ISR & DMA_ISR_TEIF4)
    {
        DMA1->IFCR = DMA_IFCR_CTEIF4; //xoa co loi
        DMA1_Channel4->CCR &= ~DMA_CCR_EN; //tat DMA
        uart1_tx_dma_busy = 0u;
        UART1_TxErrorCallback(); //goi callback loi
    }
}

__WEAK void UART1_TxCompleteCallback(void)
{
}

__WEAK void UART1_TxErrorCallback(void)
{
}
