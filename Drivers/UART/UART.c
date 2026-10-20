#include "UART.h"

#ifndef UART1_CLEAR_DMA_FLAGS
#define UART1_CLEAR_DMA_FLAGS(flags)    (DMA1->IFCR = (flags))
#endif
/*
 * ============================================================================
 * UART1 DRIVER - CMSIS REGISTER + DMA + INTERRUPT
 * ============================================================================
 */


/* ============================================================================
 * BUFFER NOI BO
 * ============================================================================
 */

/* DMA1 Channel 5 ghi truc tiep vao buffer RX 
 * volatile: CPU luon doc lai gia tri RAM moi nhat do DMA cap nhat.
 */
static volatile uint8_t uart1_rx_dma_buffer[UART1_RX_DMA_BUFFER_SIZE];

/* TX circular queue.
 * main ghi tai tx_head.
 * DMA doc tu tx_tail.
 */
static uint8_t uart1_tx_buffer[UART1_TX_BUFFER_SIZE];

/* ============================================================================
 * TRANG THAI TX
 * ============================================================================
 */

static volatile uint16_t uart1_tx_head = 0u;
static volatile uint16_t uart1_tx_tail = 0u;

/* =1 khi DMA1 Channel 4 dang chay. */
static volatile uint8_t uart1_tx_dma_active = 0u;

/* So byte cua chunk TX DMA hien tai. */
static volatile uint16_t uart1_tx_dma_chunk_len = 0u;

/* =1 khi queue rong va dang cho USART TC. */
static volatile uint8_t uart1_tx_waiting_tc = 0u;

/* ============================================================================
 * TRANG THAI RX DMA
 * ============================================================================
 * Tong so byte da di qua cac moc HT/TC cua DMA RX.
 * Buffer 256 byte:
 *   HT -> +128
 *   TC -> +128
 */
static volatile uint32_t uart1_rx_completed_bytes = 0u;

/* Tong so byte UART1_Read() da tra cho main. */
static uint32_t uart1_rx_read_total = 0u;

/* ============================================================================
 * TRANG THAI CHUNG
 * ============================================================================
 */

static volatile uint8_t uart1_initialized = 0u;

static volatile UART1_Status_t uart1_status =
    UART1_ERROR_NOT_INITIALIZED;

static volatile uint32_t uart1_hw_error_flags =
    UART1_HWERR_NONE;

/* ============================================================================
 * HAM NOI BO - TX QUEUE
 * ============================================================================
 */

static uint16_t UART1_TxNextIndex(uint16_t index)
{
    index++;
    if (index >= UART1_TX_BUFFER_SIZE)
    {
        index = 0u;
    }
    return index;
}

/*
 * Tinh so byte trong cua TX queue.
 * Queue de trong 1 o de phan biet rong/day.
 */
static uint16_t UART1_TxFreeSpace(void)
{
    uint16_t head;
    uint16_t tail;
    uint16_t used;

    head = uart1_tx_head;
    tail = uart1_tx_tail;

    if (head >= tail)
    {
        used = (uint16_t)(head - tail);
    }
    else
    {
        used = (uint16_t)(UART1_TX_BUFFER_SIZE - (tail - head));
    }
    return (uint16_t)(UART1_TX_BUFFER_SIZE - 1u - used);
}

/* ============================================================================
 * GPIO + CLOCK
 * ============================================================================
 * Cau hinh GPIO cho USART1 mapping mac dinh:
 *   PA9  = TX, PA10 = RX
 * Khi cap nhat AFIO->MAPR, driver xoa USART1_REMAP de dua USART1
 * ve PA9/PA10 va dat SWJ_CFG = JTAGDISABLE de van giu SWD tren PA13/PA14.
 * Cac bit remap khac duoc giu nguyen.
 */

static UART1_Status_t UART1_GPIO_Init(void)
{
    /* Bat clock AFIO va GPIOA. */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;

    (void)RCC->APB2ENR;
    /*
     * Dam bao USART1 dung mapping mac dinh:
     *   TX = PA9,RX = PA10
     * SWJ_CFG tren STM32F1 co han che readback, vi vay khi ghi MAPR
     * can ghi lai chinh sach SWD cua project mot cach ro rang.
     * JTAG bi tat nhung SWD van duoc giu de debug/nap code.
     */
    AFIO->MAPR =(AFIO->MAPR &~ 
				(AFIO_MAPR_USART1_REMAP | AFIO_MAPR_SWJ_CFG))
        | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
	
    /* PA9 = USART1_TX
     * CNF9  = 10 -> Alternate Function Push-Pull
     * MODE9 = 11 -> Output 50 MHz => 0xB
     */
    GPIOA->CRH &= ~(0xFUL << 4U);
    GPIOA->CRH |=  (0xBUL << 4U);

    /*
     * PA10 = USART1_RX
     * CNF10  = 10 -> Input pull-up / pull-down
     * MODE10 = 00 -> Input => 0x8
     *
     * ODR10 = 1 -> Pull-up.
     * UART idle o muc HIGH, pull-up giup RX khong bi floating
     * khi USB-TTL chua ket noi.
     */
    GPIOA->CRH &= ~(0xFUL << 8U);
    GPIOA->CRH |=  (0x8UL << 8U);
    GPIOA->ODR |=  (1UL << 10U);
    return UART1_OK;
}

/*
 * Lay PCLK2 thuc te.
 * USART1 nam tren APB2.
 * SystemClock_Config() phai chay truoc UART1_Init().
 */
static uint32_t UART1_GetPCLK2Hz(void)
{
    static const uint8_t apb_div_table[8] =
    {
        1u, 1u, 1u, 1u,
        2u, 4u, 8u, 16u
    };

    uint32_t ppre2;

    /* RCC_CFGR PPRE2 = bit [13:11]. */
    ppre2 = (RCC->CFGR >> 11u) & 0x7u;

    return SystemCoreClock / apb_div_table[ppre2];
}

/* ============================================================================
 * TX DMA - START CHUNK TIEP THEO
 * ============================================================================
 * Goi:
 *   - khi interrupt dang khoa trong UART1_Write(), hoac
 *   - tu DMA1 Channel 4 ISR.
 */

static void UART1_TxStartNextDMA(void)
{
    uint16_t head;
    uint16_t tail;
    uint16_t chunk_len;

    /* Dang co DMA TX -> khong duoc sua CMAR/CNDTR. */
    if (uart1_tx_dma_active != 0u)
    {
        return;
    }

    head = uart1_tx_head;
    tail = uart1_tx_tail;

    /*
     * Queue rong.
     * Cho USART TC de xac nhan byte cuoi da ra het PA9.
     */
    if (head == tail)
    {
        uart1_tx_waiting_tc = 1u;
        USART1->CR1 |= USART_CR1_TCIE;
        return;
    }

     //Co data moi -> khong cho TC cu nua.
    uart1_tx_waiting_tc = 0u;
    USART1->CR1 &= ~USART_CR1_TCIE;

    /*
     * DMA chi truyen duoc vung RAM lien tuc.
     * Neu queue wrap:
     *   truyen tail -> end truoc,
     *   DMA TC se truyen 0 -> head sau.
     */
    if (head > tail)
    {
        chunk_len = (uint16_t)(head - tail);
    }
    else
    {
        chunk_len = (uint16_t)(
            UART1_TX_BUFFER_SIZE - tail
        );
    }

    //Tat channel truoc khi doi CMAR/CNDTR. 
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;

    // Xoa flag cu Channel 4. */
    DMA1->IFCR = DMA_IFCR_CGIF4;
    DMA1_Channel4->CMAR = (uint32_t)&uart1_tx_buffer[tail];
    DMA1_Channel4->CNDTR = chunk_len;
    uart1_tx_dma_chunk_len = chunk_len;
    uart1_tx_dma_active = 1u;

    // Bat DMA TX. 
    DMA1_Channel4->CCR |= DMA_CCR_EN;
}

/* ============================================================================
 * RX DMA - SERVICE HT / TC / TE
 * ============================================================================
 */

static void UART1_RxServicePendingFlags (uint8_t call_error_callback)
{
    uint32_t isr;
    isr = DMA1->ISR;

    // DMA RX Transfer Error.
    if ((isr & DMA_ISR_TEIF5) != 0u)
    {
        /* Clear toan bo flag Channel 5. */
        DMA1->IFCR = DMA_IFCR_CGIF5;
        DMA1_Channel5->CCR &= ~DMA_CCR_EN;
        uart1_status = UART1_ERROR_DMA_RX;
        if (call_error_callback != 0u)
        {
            UART1_ErrorCallback(UART1_ERROR_DMA_RX);
        }

        return;
    }
		
    // Half Transfer: DMA da ghi xong nua dau RX buffer.
    if ((isr & DMA_ISR_HTIF5) != 0u)
    {
        DMA1->IFCR = DMA_IFCR_CHTIF5;
        uart1_rx_completed_bytes += (UART1_RX_DMA_BUFFER_SIZE / 2u);
    }

    /*
     * Transfer Complete:
     * DMA da ghi xong nua sau RX buffer.
     * Circular Mode tu quay ve dau.
     */
    if ((isr & DMA_ISR_TCIF5) != 0u)
    {
        DMA1->IFCR = DMA_IFCR_CTCIF5;
        uart1_rx_completed_bytes += (UART1_RX_DMA_BUFFER_SIZE / 2u);
    }
}

/* ============================================================================
 * INIT
 * ============================================================================
 */

UART1_Status_t UART1_Init(uint32_t baudrate)
{
    uint32_t pclk2_hz;
    uint32_t brr;
    uint64_t wanted_clock;
    uint64_t baud_error;
    volatile uint32_t dummy;
    UART1_Status_t gpio_status;

    /* Baudrate = 0 khong hop le. */
    if (baudrate == 0U)
    {
        uart1_status = UART1_ERROR_PARAM;
        return UART1_ERROR_PARAM;
    }

    /*
     * Neu driver da khoi tao va TX van dang chay,
     * khong reset USART/DMA giua phien truyen.
     */
    if ((uart1_initialized != 0U) &&
        (UART1_IsTxIdle() == 0U))
    {
        return UART1_BUSY;
    }

    /*
     * Tu diem nay co the re-init an toan.
     * Danh dau chua initialized trong luc cau hinh lai peripheral.
     */
    uart1_initialized = 0U;

    /* Tat IRQ module trong luc cau hinh lai peripheral. */
    NVIC_DisableIRQ(DMA1_Channel4_IRQn);
    NVIC_DisableIRQ(DMA1_Channel5_IRQn);
    NVIC_DisableIRQ(USART1_IRQn);

    /* Cau hinh GPIO va kiem tra USART1 khong bi remap. */
    gpio_status = UART1_GPIO_Init();
    if (gpio_status != UART1_OK)
    {
        uart1_status = gpio_status;
        return gpio_status;
    }

    /* Bat clock USART1 va DMA1. */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
    (void)RCC->APB2ENR;
    (void)RCC->AHBENR;

    /*
     * Reset USART1 de dua peripheral ve trang thai sach.
     * Khong anh huong GPIO/SWD.
     */
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

    /* Tat DMA channel truoc khi cau hinh. */
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    /* Reset USART control registers. */
    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;

    /*
     * Cap nhat SystemCoreClock truoc khi tinh PCLK2.
     */
    SystemCoreClockUpdate();
    pclk2_hz = UART1_GetPCLK2Hz();
    if ((pclk2_hz == 0U) ||
        (pclk2_hz > 72000000U))
    {
        uart1_status = UART1_ERROR_CLOCK;
        return UART1_ERROR_CLOCK;
    }

    /*
     * STM32F1 USART oversampling x16:
     * BRR xap xi PCLK2 / baudrate.
     * Cong baudrate / 2 de lam tron gan nhat.
     */
    brr = (pclk2_hz + (baudrate / 2U)) / baudrate;
    if ((brr < 16U) || (brr > 0xFFFFU))
    {
        uart1_status = UART1_ERROR_PARAM;
        return UART1_ERROR_PARAM;
    }

    /*
     * Kiem tra sai so do lam tron BRR.
     * Neu sai so lon hon 2%, xem clock/baudrate khong phu hop.
     */
    wanted_clock = (uint64_t)baudrate * (uint64_t)brr;

    if ((uint64_t)pclk2_hz >= wanted_clock)
    {
        baud_error = (uint64_t)pclk2_hz - wanted_clock;
    }
    else
    {
        baud_error = wanted_clock - (uint64_t)pclk2_hz;
    }

    if ((baud_error * 100ULL) > (wanted_clock * 2ULL))
    {
        uart1_status = UART1_ERROR_CLOCK;
        return UART1_ERROR_CLOCK;
    }
    USART1->BRR = brr;

    /*------------------------------------------------------------------------
     * DMA1 Channel 4 - USART1 TX
     * ------------------------------------------------------------------------
     * DIR  = 1  : Memory -> Peripheral
     * MINC = 1  : tang dia chi RAM
     * PL   = 01 : Medium priority
     * TCIE = 1  : ngat khi xong chunk
     * TEIE = 1  : ngat khi DMA error
     * MSIZE/PSIZE = 00 -> 8 bit
     */
    DMA1_Channel4->CCR = 0U;
    DMA1_Channel4->CPAR = (uintptr_t)&USART1->DR;
    DMA1_Channel4->CMAR = (uintptr_t)uart1_tx_buffer;
    DMA1_Channel4->CNDTR = 0U;
    DMA1_Channel4->CCR = DMA_CCR_DIR
											| DMA_CCR_MINC
											| DMA_CCR_PL_0
											| DMA_CCR_TCIE
											| DMA_CCR_TEIE;
    /*------------------------------------------------------------------------
     * DMA1 Channel 5 - USART1 RX Circular
     * ------------------------------------------------------------------------
     * DIR  = 0  : Peripheral -> Memory
     * MINC = 1
     * CIRC = 1
     * PL   = 01 : Medium priority
     * HTIE = 1
     * TCIE = 1
     * TEIE = 1
     * MSIZE/PSIZE = 00 -> 8 bit
     */
    DMA1_Channel5->CCR = 0U;
    DMA1_Channel5->CPAR = (uintptr_t)&USART1->DR;
    DMA1_Channel5->CMAR = (uintptr_t)uart1_rx_dma_buffer;
    DMA1_Channel5->CNDTR = UART1_RX_DMA_BUFFER_SIZE;
    DMA1_Channel5->CCR = DMA_CCR_MINC
											| DMA_CCR_CIRC
											| DMA_CCR_PL_0
											| DMA_CCR_HTIE
											| DMA_CCR_TCIE
											| DMA_CCR_TEIE;

    /* Xoa cac flag DMA cu cua Channel 4 va Channel 5. */
    UART1_CLEAR_DMA_FLAGS(DMA_IFCR_CGIF5);

    /* USART CR3:
     * DMAT = bat DMA request cho TX
     * DMAR = bat DMA request cho RX
     * EIE  = bat FE/NE/ORE interrupt
     */
    USART1->CR3 = USART_CR3_DMAT
								| USART_CR3_DMAR
								| USART_CR3_EIE;

    /* Reset software state. */
    uart1_tx_head = 0U;
    uart1_tx_tail = 0U;
    uart1_tx_dma_active = 0U;
    uart1_tx_dma_chunk_len = 0U;
    uart1_tx_waiting_tc = 0U;

    uart1_rx_completed_bytes = 0U;
    uart1_rx_read_total = 0U;

    uart1_status = UART1_OK;
    uart1_hw_error_flags = UART1_HWERR_NONE;

    /*
     * Don cac co RX/error cu.
     * STM32F1 clear PE/FE/NE/ORE bang sequence:
     * doc SR -> doc DR.
     */
    dummy = USART1->SR;
    dummy = USART1->DR;
    (void)dummy;

    /* USART CR1:
     * TE   = transmitter enable
     * RE   = receiver enable
     * PEIE = parity error interrupt
     * UE   = USART enable
     * M=0, PCE=0 -> 8 data bit, no parity.
     * CR2 STOP=00 -> 1 stop bit.
     * KHONG bat RXNEIE/TXEIE vi data do DMA xu ly.
     */
    USART1->CR1 = USART_CR1_TE
								| USART_CR1_RE
								| USART_CR1_PEIE
								| USART_CR1_UE;

    /* Bat RX DMA Circular.
     * Tu thoi diem nay USART1 RX se tu dong ghi vao RX buffer.
     */
    __DMB();
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    /* Xoa pending NVIC cu. */
    NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn);
    NVIC_ClearPendingIRQ(USART1_IRQn);

    /* Dat priority theo cau hinh trong UART.h. */
    NVIC_SetPriority(DMA1_Channel4_IRQn,UART1_IRQ_PRIORITY);
    NVIC_SetPriority(DMA1_Channel5_IRQn,UART1_IRQ_PRIORITY);
    NVIC_SetPriority(USART1_IRQn,UART1_IRQ_PRIORITY);

    /* Danh dau module da khoi tao TRUOC khi bat IRQ.
     * Neu IRQ xay ra ngay sau NVIC_EnableIRQ(), ISR se thay state hop le.
     */
    uart1_initialized = 1U;

    /* Bat IRQ. */
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);

    return UART1_OK;
}

/* ============================================================================
 * WRITE
 * ============================================================================
 */

UART1_Status_t UART1_Write(const uint8_t *data,uint16_t length)
{
    uint32_t primask;
    uint16_t free_space;
    uint16_t local_head;
    uint16_t i;

    if (uart1_initialized == 0u)
    {
        uart1_status = UART1_ERROR_NOT_INITIALIZED;
        return UART1_ERROR_NOT_INITIALIZED;
    }

    if ((data == 0) || (length == 0u))
    {
        uart1_status = UART1_ERROR_PARAM;
        return UART1_ERROR_PARAM;
    }

    /* Queue de trong 1 o.
     * Block don lon nhat = size - 1.
     */
    if (length >= UART1_TX_BUFFER_SIZE)
    {
        uart1_status = UART1_ERROR_PARAM;
        return UART1_ERROR_PARAM;
    }

    /* Critical section ngan:
     * tranh DMA ISR doi tx_tail trong luc tinh free space va publish head.
     */
    primask = __get_PRIMASK();
    __disable_irq();

    free_space = UART1_TxFreeSpace();
    if (length > free_space)
    {
        __set_PRIMASK(primask);
        // BUSY la flow-control, khong phai sticky error.
        return UART1_BUSY;
    }

    // Copy TOAN BO block vao queue.
    local_head = uart1_tx_head;

    for (i = 0u; i < length; i++)
    {
        uart1_tx_buffer[local_head] = data[i];
        local_head = UART1_TxNextIndex(local_head);
    }
		//Dam bao data da ghi xong vao queue truoc khi publish head moi.
    __DMB();
    // Publish head sau khi block da copy xong.
    uart1_tx_head = local_head;

    /* Neu DMA ranh -> start ngay.
     * Neu DMA dang chay -> TC ISR se start chunk sau.
     */
    UART1_TxStartNextDMA();
    __set_PRIMASK(primask);

    return UART1_OK;
}

/* ============================================================================
 * READ
 * ============================================================================
 */

uint16_t UART1_Read(
    uint8_t *dst,
    uint16_t capacity
)
{
    uint32_t primask;
    uint32_t completed;
    uint32_t ndtr;
    uint32_t position;
    uint32_t base_position;
    uint32_t write_total;
    uint32_t available;
    uint16_t count;
    uint16_t i;
    uint8_t attempt;

    if (uart1_initialized == 0u)
    {
        uart1_status = UART1_ERROR_NOT_INITIALIZED;
        return 0u;
    }
    if ((dst == 0) || (capacity == 0u))
    {
        uart1_status = UART1_ERROR_PARAM;
        return 0u;
    }

    /* Snapshot nhat quan.
     * DMA phan cung van chay khi CPU tam khoa interrupt.
     */
    primask = __get_PRIMASK();
    __disable_irq();
		
		write_total = uart1_rx_read_total;

    for (attempt = 0U; attempt < 4U; ++attempt)
    {
        UART1_RxServicePendingFlags(0U);
        /* DMA RX da gap Transfer Error. */
        if (uart1_status == UART1_ERROR_DMA_RX)
        {
            break;
        }

        completed = uart1_rx_completed_bytes;
        ndtr = DMA1_Channel5->CNDTR;

        /*
         * CNDTR giam tu BUFFER_SIZE -> 0.
         * position la vi tri DMA hien tai trong mot vong buffer.
         * Phep modulo xu ly ca thoi diem CNDTR tam thoi bang 0
         * tai moc circular reload.
         */
        position = (UART1_RX_DMA_BUFFER_SIZE - ndtr)
										% UART1_RX_DMA_BUFFER_SIZE;

        /* Vi completed tang theo nua buffer,
         * base_position chi nam tai cac moc HT/TC.
         */
        base_position = completed % UART1_RX_DMA_BUFFER_SIZE;

        // So byte DMA da di tiep ke tu moc completed gan nhat.
        available = (position + UART1_RX_DMA_BUFFER_SIZE - base_position)
										% UART1_RX_DMA_BUFFER_SIZE;

        /* Neu flag HT/TC/TE lai xuat hien sau luc service,
         * hoac position cho thay DMA da cross them >= nua buffer,
         * snapshot hien tai khong con nhat quan -> thu lai.
         */
        if ((DMA1->ISR &
             (DMA_ISR_HTIF5 | DMA_ISR_TCIF5 | DMA_ISR_TEIF5)) != 0U)
        {
            continue;
        }
        if (available >= (UART1_RX_DMA_BUFFER_SIZE / 2U))
        {
            continue;
        }
        write_total = completed + available;
        break;
    }

    if (attempt == 4U)
    {
        uart1_status = UART1_ERROR_RX_OVERFLOW;
    }
    __set_PRIMASK(primask);
		
    // Neu DMA RX bi Transfer Error thi khong tra du lieu moi.
    if (uart1_status == UART1_ERROR_DMA_RX)
    {
        return 0U;
    }
    // So byte DMA da ghi nhung main chua doc.
    available = write_total - uart1_rx_read_total;

    /*
     * Neu available > RX buffer size:
     * DMA da quay vong va ghi de data cu.
     */
    if (available > UART1_RX_DMA_BUFFER_SIZE)
    {
        uart1_status = UART1_ERROR_RX_OVERFLOW;
			
        // Du lieu cu da mat. Chi con giu duoc mot buffer moi nhat.
        uart1_rx_read_total = write_total - UART1_RX_DMA_BUFFER_SIZE;
        available = UART1_RX_DMA_BUFFER_SIZE;
    }
    if (available == 0U)
    {
        return 0U;
    }
    if (available > capacity)
    {
        count = capacity;
    }
    else
    {
        count = (uint16_t)available;
    }

    // Dam bao CPU doc RX buffer sau khi da snapshot state DMA.
    __DMB();

    /* Copy data tu DMA circular buffer sang dst cua main. */
    for (i = 0U; i < count; i++)
    {
        uint32_t index;
        index = (uart1_rx_read_total + i) % UART1_RX_DMA_BUFFER_SIZE;
        dst[i] = uart1_rx_dma_buffer[index];
    }
    uart1_rx_read_total += count;
    return count;
}

/* ============================================================================
 * STATUS
 * ============================================================================
 */

uint8_t UART1_IsTxIdle(void)
{
    uint32_t primask;
    uint8_t idle;
    if (uart1_initialized == 0u)
    {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();

    // Idle = queue rong + DMA ranh + USART TC.
    if ((uart1_tx_head == uart1_tx_tail) &&
        (uart1_tx_dma_active == 0u) &&
        ((USART1->SR & USART_SR_TC) != 0u))
    {
        idle = 1u;
    }
    else
    {
        idle = 0u;
    }
		__set_PRIMASK(primask);
    return idle;
}


UART1_Status_t UART1_GetStatus(void)
{
    return uart1_status;
}


uint32_t UART1_GetHwErrorFlags(void)
{
    return uart1_hw_error_flags;
}


void UART1_ClearStatus(void)
{
    uint32_t primask;
    primask = __get_PRIMASK();
    __disable_irq();

    if (uart1_initialized != 0u)
    {
        uart1_status = UART1_OK;
        uart1_hw_error_flags = UART1_HWERR_NONE;
    }
    else
    {
        uart1_status = UART1_ERROR_NOT_INITIALIZED;
    }
		__set_PRIMASK(primask);
}


/* ============================================================================
 * DMA1 CHANNEL 4 IRQ - TX
 * ============================================================================
 */

void UART1_DMA_TX_IRQHandler(void)
{
    uint32_t isr;
		// Bao ve truong hop IRQ bat ngo xay ra truoc khi Init hoan tat.
    if (uart1_initialized == 0U)
    {
        NVIC_DisableIRQ(DMA1_Channel4_IRQn);
        return;
    }
		
    isr = DMA1->ISR;
    /*
     * DMA TX error.
     */
    if ((isr & DMA_ISR_TEIF4) != 0u)
    {
        DMA1->IFCR = DMA_IFCR_CGIF4;
        DMA1_Channel4->CCR &= ~DMA_CCR_EN;

        uart1_tx_dma_active = 0u;
        uart1_tx_dma_chunk_len = 0u;
        uart1_tx_waiting_tc = 0u;

        /*
         * Khong the biet chinh xac bao nhieu byte cua chunk loi
         * da ra ngoai, nen huy phan TX queue con lai de tranh
         * driver tu dong gui lai va gay duplicate.
         */
        uart1_tx_tail = uart1_tx_head;
        USART1->CR1 &= ~USART_CR1_TCIE;
        uart1_status = UART1_ERROR_DMA_TX;
        UART1_ErrorCallback(UART1_ERROR_DMA_TX);
			
        return;
    }

    // DMA da chuyen xong chunk hien tai vao USART1.
    if ((isr & DMA_ISR_TCIF4) != 0u)
    {
        uint16_t new_tail;
        DMA1->IFCR = DMA_IFCR_CTCIF4;
        DMA1_Channel4->CCR &= ~DMA_CCR_EN;
			
        new_tail = (uint16_t)(
                uart1_tx_tail + uart1_tx_dma_chunk_len);

        if (new_tail >= UART1_TX_BUFFER_SIZE)
        {
            new_tail = (uint16_t)(
                    new_tail - UART1_TX_BUFFER_SIZE);
        }
        uart1_tx_tail = new_tail;
        uart1_tx_dma_chunk_len = 0u;
        uart1_tx_dma_active = 0u;

        /* Queue con data -> start chunk tiep.
         * Queue rong -> start helper se bat TCIE.
         */
        UART1_TxStartNextDMA();
    }
}


/* ============================================================================
 * DMA1 CHANNEL 5 IRQ - RX
 * ============================================================================
 */

void UART1_DMA_RX_IRQHandler(void)
{
		// Bao ve truong hop IRQ bat ngo xay ra truoc khi Init hoan tat.
    if (uart1_initialized == 0U)
    {
        NVIC_DisableIRQ(DMA1_Channel5_IRQn);
        return;
    }
    UART1_RxServicePendingFlags(1u);
}


/* ============================================================================
 * USART1 IRQ
 * ============================================================================
 * USART IRQ chi xu ly:
 *   - PE/FE/NE/ORE
 *   - TC cua byte TX cuoi
 */

void UART1_IRQHandler(void)
{
    uint32_t sr;
		// Bao ve truong hop IRQ bat ngo xay ra truoc khi Init hoan tat.
    if (uart1_initialized == 0U)
    {
        NVIC_DisableIRQ(USART1_IRQn);
        return;
    }
    sr = USART1->SR;
	
    /* ------------------------------------------------------------
     * Hardware error
     * ------------------------------------------------------------
     */
    if ((sr & (USART_SR_PE
             | USART_SR_FE
             | USART_SR_NE
             | USART_SR_ORE)) != 0u)
    {
        volatile uint32_t dummy;

        if ((sr & USART_SR_PE) != 0u)
        {
            uart1_hw_error_flags |= UART1_HWERR_PE;
        }

        if ((sr & USART_SR_FE) != 0u)
        {
            uart1_hw_error_flags |= UART1_HWERR_FE;
        }

        if ((sr & USART_SR_NE) != 0u)
        {
            uart1_hw_error_flags |= UART1_HWERR_NE;
        }

        if ((sr & USART_SR_ORE) != 0u)
        {
            uart1_hw_error_flags |= UART1_HWERR_ORE;
        }
        uart1_status = UART1_ERROR_HW;

        /*
         * STM32F1 clear PE/FE/NE/ORE bang sequence:
         * doc SR (da doc o tren) -> doc DR.
         */
        dummy = USART1->DR;
        (void)dummy;

        UART1_ErrorCallback(UART1_ERROR_HW);
    }

    /*
     * ------------------------------------------------------------
     * Transmission Complete
     * ------------------------------------------------------------
     * DMA TC: byte cuoi da vao USART.
     * USART TC: byte cuoi da ra het shift register / PA9.
     */
    if (((sr & USART_SR_TC) != 0u) &&
        ((USART1->CR1 & USART_CR1_TCIE) != 0u))
    {
        /*
         * Tat TC interrupt.
         * Khong RMW USART_SR de clear TC.
         */
        USART1->CR1 &= ~USART_CR1_TCIE;
        uart1_tx_waiting_tc = 0u;

        // Callback chi khi TX thuc su rong.
        if ((uart1_tx_head == uart1_tx_tail) && (uart1_tx_dma_active == 0u))
        {
            UART1_TxCompleteCallback();
        }
    }
}

/* ============================================================================
 * WEAK CALLBACK
 * ============================================================================
 */

__WEAK void UART1_TxCompleteCallback(void)
{   
}

__WEAK void UART1_ErrorCallback(UART1_Status_t error)
{
    (void)error;
}
