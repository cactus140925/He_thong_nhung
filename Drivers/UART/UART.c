#include "UART.h"

/*
 * ============================================================================
 * UART1 DRIVER - CMSIS REGISTER + DMA + INTERRUPT
 * ============================================================================
 *
 * TX:
 *   Main -> UART1_Write()
 *        -> TX circular queue
 *        -> DMA1 Channel 4
 *        -> USART1->DR
 *        -> PA9
 *
 * RX:
 *   PA10 -> USART1->DR
 *        -> DMA1 Channel 5 Circular
 *        -> RX DMA buffer
 *        -> UART1_Read()
 *        -> Main
 *
 * DMA chuyen du lieu.
 * Interrupt chi bao moc hoan tat / loi / TC cua USART.
 * ============================================================================
 */


/* ============================================================================
 * 1. BUFFER NOI BO
 * ============================================================================
 */

/* DMA1 Channel 5 ghi truc tiep vao buffer RX nay. */
static uint8_t uart1_rx_dma_buffer[UART1_RX_DMA_BUFFER_SIZE];

/*
 * TX circular queue.
 *
 * main ghi tai tx_head.
 * DMA doc tu tx_tail.
 */
static uint8_t uart1_tx_buffer[UART1_TX_BUFFER_SIZE];


/* ============================================================================
 * 2. TRANG THAI TX
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
 * 3. TRANG THAI RX DMA
 * ============================================================================
 */

/*
 * Tong so byte da di qua cac moc HT/TC cua DMA RX.
 *
 * Buffer 256 byte:
 *   HT -> +128
 *   TC -> +128
 */
static volatile uint32_t uart1_rx_completed_bytes = 0u;

/* Tong so byte UART1_Read() da tra cho main. */
static uint32_t uart1_rx_read_total = 0u;


/* ============================================================================
 * 4. TRANG THAI CHUNG
 * ============================================================================
 */

static volatile uint8_t uart1_initialized = 0u;

static volatile UART1_Status_t uart1_status =
    UART1_ERROR_NOT_INITIALIZED;

static volatile uint32_t uart1_hw_error_flags =
    UART1_HWERR_NONE;


/* ============================================================================
 * 5. HAM NOI BO - TX QUEUE
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
 *
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
        used = (uint16_t)(
            UART1_TX_BUFFER_SIZE - (tail - head)
        );
    }

    return (uint16_t)(
        UART1_TX_BUFFER_SIZE - 1u - used
    );
}


/* ============================================================================
 * 6. GPIO + CLOCK
 * ============================================================================
 */

static void UART1_GPIO_Init(void)
{
    /* Bat clock AFIO va GPIOA. */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN
                  | RCC_APB2ENR_IOPAEN;

    /*
     * Dam bao USART1 khong remap:
     * TX = PA9
     * RX = PA10
     */
    AFIO->MAPR &= ~AFIO_MAPR_USART1_REMAP;

    /*
     * PA9 = USART1_TX
     * CNF9  = 10 -> Alternate Function Push-Pull
     * MODE9 = 11 -> Output 50 MHz
     * => 0xB
     */
    GPIOA->CRH &= ~(0xFu << 4);
    GPIOA->CRH |=  (0xBu << 4);

    /*
     * PA10 = USART1_RX
     * CNF10  = 01 -> Floating Input
     * MODE10 = 00 -> Input
     * => 0x4
     */
    GPIOA->CRH &= ~(0xFu << 8);
    GPIOA->CRH |=  (0x4u << 8);
}


/*
 * Lay PCLK2 thuc te.
 *
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

    return SystemCoreClock /
           apb_div_table[ppre2];
}


/* ============================================================================
 * 7. TX DMA - START CHUNK TIEP THEO
 * ============================================================================
 *
 * Ham nay chi dung noi bo.
 *
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
     *
     * Cho USART TC de xac nhan byte cuoi da ra het PA9.
     */
    if (head == tail)
    {
        uart1_tx_waiting_tc = 1u;
        USART1->CR1 |= USART_CR1_TCIE;
        return;
    }

    /*
     * Co data moi -> khong cho TC cu nua.
     */
    uart1_tx_waiting_tc = 0u;
    USART1->CR1 &= ~USART_CR1_TCIE;

    /*
     * DMA chi truyen duoc vung RAM lien tuc.
     *
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

    /* Tat channel truoc khi doi CMAR/CNDTR. */
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;

    /* Xoa flag cu Channel 4. */
    DMA1->IFCR = DMA_IFCR_CGIF4;

    DMA1_Channel4->CMAR =
        (uint32_t)&uart1_tx_buffer[tail];

    DMA1_Channel4->CNDTR =
        chunk_len;

    uart1_tx_dma_chunk_len =
        chunk_len;

    uart1_tx_dma_active = 1u;

    /* Bat DMA TX. */
    DMA1_Channel4->CCR |= DMA_CCR_EN;
}


/* ============================================================================
 * 8. RX DMA - SERVICE HT / TC / TE
 * ============================================================================
 */

static void UART1_RxServicePendingFlags(
    uint8_t call_error_callback
)
{
    uint32_t isr;

    isr = DMA1->ISR;

    /*
     * DMA RX Transfer Error.
     */
    if ((isr & DMA_ISR_TEIF5) != 0u)
    {
        /* Clear toan bo flag Channel 5. */
        DMA1->IFCR = DMA_IFCR_CGIF5;

        DMA1_Channel5->CCR &= ~DMA_CCR_EN;

        uart1_status = UART1_ERROR_DMA_RX;

        if (call_error_callback != 0u)
        {
            UART1_ErrorCallback(
                UART1_ERROR_DMA_RX
            );
        }

        return;
    }

    /*
     * Half Transfer:
     * DMA da ghi xong nua dau RX buffer.
     */
    if ((isr & DMA_ISR_HTIF5) != 0u)
    {
        DMA1->IFCR = DMA_IFCR_CHTIF5;

        uart1_rx_completed_bytes +=
            (UART1_RX_DMA_BUFFER_SIZE / 2u);
    }

    /*
     * Transfer Complete:
     * DMA da ghi xong nua sau RX buffer.
     * Circular Mode tu quay ve dau.
     */
    if ((isr & DMA_ISR_TCIF5) != 0u)
    {
        DMA1->IFCR = DMA_IFCR_CTCIF5;

        uart1_rx_completed_bytes +=
            (UART1_RX_DMA_BUFFER_SIZE / 2u);
    }
}


/* ============================================================================
 * 9. INIT
 * ============================================================================
 */

UART1_Status_t UART1_Init(uint32_t baudrate)
{
    uint32_t pclk2_hz;
    uint32_t brr;
    volatile uint32_t dummy;

    uart1_initialized = 0u;

    if (baudrate == 0u)
    {
        uart1_status = UART1_ERROR_PARAM;
        return UART1_ERROR_PARAM;
    }

    /*
     * Tat IRQ module trong luc cau hinh lai peripheral.
     */
    NVIC_DisableIRQ(DMA1_Channel4_IRQn);
    NVIC_DisableIRQ(DMA1_Channel5_IRQn);
    NVIC_DisableIRQ(USART1_IRQn);

    UART1_GPIO_Init();

    /* Bat clock USART1 va DMA1. */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;

    /* Reset USART control register. */
    USART1->CR1 = 0u;
    USART1->CR2 = 0u;
    USART1->CR3 = 0u;

    /* Tat DMA channel truoc khi cau hinh. */
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    /*
     * Tinh baudrate tu PCLK2 thuc te.
     */
    pclk2_hz = UART1_GetPCLK2Hz();

    if (pclk2_hz == 0u)
    {
        uart1_status = UART1_ERROR_CLOCK;
        return UART1_ERROR_CLOCK;
    }

    /*
     * STM32F1 USART oversampling x16:
     * BRR xap xi PCLK / baud.
     */
    brr = (pclk2_hz + (baudrate / 2u))
        / baudrate;

    if ((brr < 16u) || (brr > 0xFFFFu))
    {
        uart1_status = UART1_ERROR_PARAM;
        return UART1_ERROR_PARAM;
    }

    USART1->BRR = brr;

    /*
     * ------------------------------------------------------------
     * DMA1 Channel 4 - USART1 TX
     * ------------------------------------------------------------
     *
     * DIR  = 1  : Memory -> Peripheral
     * MINC = 1  : tang dia chi RAM
     * PL   = 01 : Medium priority
     * TCIE = 1  : ngat khi xong chunk
     * TEIE = 1  : ngat khi DMA error
     *
     * MSIZE/PSIZE = 00 -> 8 bit
     */
    DMA1_Channel4->CCR = 0u;

    DMA1_Channel4->CPAR =
        (uint32_t)&USART1->DR;

    DMA1_Channel4->CMAR =
        (uint32_t)uart1_tx_buffer;

    DMA1_Channel4->CNDTR = 0u;

    DMA1_Channel4->CCR =
          DMA_CCR_DIR
        | DMA_CCR_MINC
        | DMA_CCR_PL_0
        | DMA_CCR_TCIE
        | DMA_CCR_TEIE;

    /*
     * ------------------------------------------------------------
     * DMA1 Channel 5 - USART1 RX Circular
     * ------------------------------------------------------------
     *
     * DIR  = 0  : Peripheral -> Memory
     * MINC = 1
     * CIRC = 1
     * HTIE = 1
     * TCIE = 1
     * TEIE = 1
     */
    DMA1_Channel5->CCR = 0u;

    DMA1_Channel5->CPAR =
        (uint32_t)&USART1->DR;

    DMA1_Channel5->CMAR =
        (uint32_t)uart1_rx_dma_buffer;

    DMA1_Channel5->CNDTR =
        UART1_RX_DMA_BUFFER_SIZE;

    DMA1_Channel5->CCR =
          DMA_CCR_MINC
        | DMA_CCR_CIRC
        | DMA_CCR_PL_0
        | DMA_CCR_HTIE
        | DMA_CCR_TCIE
        | DMA_CCR_TEIE;

    /*
     * Xoa flag DMA cu.
     */
    DMA1->IFCR =
          DMA_IFCR_CGIF4
        | DMA_IFCR_CGIF5;

    /*
     * USART CR3:
     * DMAT = DMA TX request
     * DMAR = DMA RX request
     * EIE  = FE/NE/ORE interrupt
     */
    USART1->CR3 =
          USART_CR3_DMAT
        | USART_CR3_DMAR
        | USART_CR3_EIE;

    /*
     * Reset software state.
     */
    uart1_tx_head = 0u;
    uart1_tx_tail = 0u;
    uart1_tx_dma_active = 0u;
    uart1_tx_dma_chunk_len = 0u;
    uart1_tx_waiting_tc = 0u;

    uart1_rx_completed_bytes = 0u;
    uart1_rx_read_total = 0u;

    uart1_status = UART1_OK;
    uart1_hw_error_flags = UART1_HWERR_NONE;

    /*
     * Don cac co RX/error cu:
     * STM32F1 clear error bang sequence doc SR -> doc DR.
     */
    dummy = USART1->SR;
    dummy = USART1->DR;
    (void)dummy;

    /*
     * USART CR1:
     * TE   = transmitter enable
     * RE   = receiver enable
     * PEIE = parity error interrupt
     * UE   = USART enable
     *
     * M=0, PCE=0 -> 8 data bit, no parity.
     * CR2 STOP=00 -> 1 stop bit.
     *
     * Khong bat RXNEIE/TXEIE vi data do DMA xu ly.
     */
    USART1->CR1 =
          USART_CR1_TE
        | USART_CR1_RE
        | USART_CR1_PEIE
        | USART_CR1_UE;

    /*
     * Bat RX DMA Circular.
     */
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    /*
     * Xoa pending NVIC cu.
     */
    NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn);
    NVIC_ClearPendingIRQ(USART1_IRQn);

    /*
     * Dat priority.
     */
    NVIC_SetPriority(DMA1_Channel4_IRQn, 2u);
    NVIC_SetPriority(DMA1_Channel5_IRQn, 2u);
    NVIC_SetPriority(USART1_IRQn, 2u);

    /*
     * Bat IRQ.
     */
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);

    uart1_initialized = 1u;

    return UART1_OK;
}


/* ============================================================================
 * 10. WRITE
 * ============================================================================
 */

UART1_Status_t UART1_Write(
    const uint8_t *data,
    uint16_t length
)
{
    uint32_t primask;
    uint16_t free_space;
    uint16_t local_head;
    uint16_t i;

    if (uart1_initialized == 0u)
    {
        uart1_status =
            UART1_ERROR_NOT_INITIALIZED;

        return UART1_ERROR_NOT_INITIALIZED;
    }

    if ((data == 0) || (length == 0u))
    {
        uart1_status = UART1_ERROR_PARAM;
        return UART1_ERROR_PARAM;
    }

    /*
     * Queue de trong 1 o.
     * Block don lon nhat = size - 1.
     */
    if (length >= UART1_TX_BUFFER_SIZE)
    {
        uart1_status = UART1_ERROR_PARAM;
        return UART1_ERROR_PARAM;
    }

    /*
     * Critical section ngan:
     * tranh DMA ISR doi tx_tail trong luc tinh free space
     * va publish head.
     */
    primask = __get_PRIMASK();
    __disable_irq();

    free_space = UART1_TxFreeSpace();

    if (length > free_space)
    {
        if (primask == 0u)
        {
            __enable_irq();
        }

        /*
         * BUSY la flow-control, khong phai sticky error.
         */
        return UART1_BUSY;
    }

    /*
     * Copy TOAN BO block vao queue.
     */
    local_head = uart1_tx_head;

    for (i = 0u; i < length; i++)
    {
        uart1_tx_buffer[local_head] =
            data[i];

        local_head =
            UART1_TxNextIndex(local_head);
    }

    /*
     * Publish head sau khi block da copy xong.
     */
    uart1_tx_head = local_head;

    /*
     * Neu DMA ranh -> start ngay.
     * Neu DMA dang chay -> TC ISR se start chunk sau.
     */
    UART1_TxStartNextDMA();

    if (primask == 0u)
    {
        __enable_irq();
    }

    return UART1_OK;
}


/* ============================================================================
 * 11. READ
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

    if (uart1_initialized == 0u)
    {
        uart1_status =
            UART1_ERROR_NOT_INITIALIZED;

        return 0u;
    }

    if ((dst == 0) || (capacity == 0u))
    {
        uart1_status = UART1_ERROR_PARAM;
        return 0u;
    }

    /*
     * Snapshot nhat quan.
     *
     * DMA phan cung van chay khi CPU tam khoa interrupt.
     */
    primask = __get_PRIMASK();
    __disable_irq();

    /*
     * Neu HT/TC flag da set ma ISR chua kip chay,
     * service ngay de counter khong bi cham mot moc.
     */
    UART1_RxServicePendingFlags(0u);

    completed =
        uart1_rx_completed_bytes;

    ndtr =
        DMA1_Channel5->CNDTR;

    /*
     * So byte DMA da ghi trong vong hien tai.
     */
    position =
        UART1_RX_DMA_BUFFER_SIZE - ndtr;

    /*
     * Moc vi tri tuong ung voi completed.
     * Vi counter tang theo nua buffer,
     * base_position thuong la 0 hoac half-buffer.
     */
    base_position =
        completed % UART1_RX_DMA_BUFFER_SIZE;

    /*
     * Neu DMA chua cross them boundary sau luc service flag.
     */
    if (position >= base_position)
    {
        write_total =
            completed +
            (position - base_position);
    }
    else
    {
        /*
         * DMA vua cross boundary trong vai lenh critical section.
         * Cong them phan con lai toi cuoi buffer + position moi.
         */
        write_total =
            completed +
            (UART1_RX_DMA_BUFFER_SIZE - base_position) +
            position;
    }

    if (primask == 0u)
    {
        __enable_irq();
    }

    /*
     * Byte chua tra cho main.
     */
    available =
        write_total - uart1_rx_read_total;

    /*
     * DMA da ghi de data cu.
     */
    if (available > UART1_RX_DMA_BUFFER_SIZE)
    {
        uart1_status =
            UART1_ERROR_RX_OVERFLOW;

        /*
         * Chi con giu duoc 1 buffer moi nhat.
         */
        uart1_rx_read_total =
            write_total -
            UART1_RX_DMA_BUFFER_SIZE;

        available =
            UART1_RX_DMA_BUFFER_SIZE;
    }

    if (available == 0u)
    {
        return 0u;
    }

    if (available > capacity)
    {
        count = capacity;
    }
    else
    {
        count = (uint16_t)available;
    }

    /*
     * Copy data tu DMA circular buffer sang dst cua main.
     */
    for (i = 0u; i < count; i++)
    {
        uint32_t index;

        index =
            (uart1_rx_read_total + i)
            % UART1_RX_DMA_BUFFER_SIZE;

        dst[i] =
            uart1_rx_dma_buffer[index];
    }

    uart1_rx_read_total += count;

    return count;
}


/* ============================================================================
 * 12. STATUS
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

    /*
     * Idle = queue rong + DMA ranh + USART TC.
     */
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

    if (primask == 0u)
    {
        __enable_irq();
    }

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
        uart1_status =
            UART1_OK;

        uart1_hw_error_flags =
            UART1_HWERR_NONE;
    }
    else
    {
        uart1_status =
            UART1_ERROR_NOT_INITIALIZED;
    }

    if (primask == 0u)
    {
        __enable_irq();
    }
}


/* ============================================================================
 * 13. DMA1 CHANNEL 4 IRQ - TX
 * ============================================================================
 */

void UART1_DMA_TX_IRQHandler(void)
{
    uint32_t isr;

    isr = DMA1->ISR;

    /*
     * DMA TX error.
     */
    if ((isr & DMA_ISR_TEIF4) != 0u)
    {
        DMA1->IFCR = DMA_IFCR_CGIF4;

        DMA1_Channel4->CCR &=
            ~DMA_CCR_EN;

        uart1_tx_dma_active = 0u;
        uart1_tx_dma_chunk_len = 0u;
        uart1_tx_waiting_tc = 0u;

        /*
         * Khong the biet chinh xac bao nhieu byte cua chunk loi
         * da ra ngoai, nen huy phan TX queue con lai de tranh
         * driver tu dong gui lai va gay duplicate.
         */
        uart1_tx_tail =
            uart1_tx_head;

        USART1->CR1 &=
            ~USART_CR1_TCIE;

        uart1_status =
            UART1_ERROR_DMA_TX;

        UART1_ErrorCallback(
            UART1_ERROR_DMA_TX
        );

        return;
    }

    /*
     * DMA da chuyen xong chunk hien tai vao USART1.
     */
    if ((isr & DMA_ISR_TCIF4) != 0u)
    {
        uint16_t new_tail;

        DMA1->IFCR =
            DMA_IFCR_CTCIF4;

        DMA1_Channel4->CCR &=
            ~DMA_CCR_EN;

        new_tail =
            (uint16_t)(
                uart1_tx_tail +
                uart1_tx_dma_chunk_len
            );

        if (new_tail >= UART1_TX_BUFFER_SIZE)
        {
            new_tail =
                (uint16_t)(
                    new_tail -
                    UART1_TX_BUFFER_SIZE
                );
        }

        uart1_tx_tail =
            new_tail;

        uart1_tx_dma_chunk_len = 0u;
        uart1_tx_dma_active = 0u;

        /*
         * Queue con data -> start chunk tiep.
         * Queue rong -> start helper se bat TCIE.
         */
        UART1_TxStartNextDMA();
    }
}


/* ============================================================================
 * 14. DMA1 CHANNEL 5 IRQ - RX
 * ============================================================================
 */

void UART1_DMA_RX_IRQHandler(void)
{
    UART1_RxServicePendingFlags(1u);
}


/* ============================================================================
 * 15. USART1 IRQ
 * ============================================================================
 *
 * Khong dung RXNE/TXE de move data.
 * Data do DMA xu ly.
 *
 * USART IRQ chi xu ly:
 *   - PE/FE/NE/ORE
 *   - TC cua byte TX cuoi
 */

void UART1_IRQHandler(void)
{
    uint32_t sr;

    sr = USART1->SR;

    /*
     * ------------------------------------------------------------
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
            uart1_hw_error_flags |=
                UART1_HWERR_PE;
        }

        if ((sr & USART_SR_FE) != 0u)
        {
            uart1_hw_error_flags |=
                UART1_HWERR_FE;
        }

        if ((sr & USART_SR_NE) != 0u)
        {
            uart1_hw_error_flags |=
                UART1_HWERR_NE;
        }

        if ((sr & USART_SR_ORE) != 0u)
        {
            uart1_hw_error_flags |=
                UART1_HWERR_ORE;
        }

        uart1_status =
            UART1_ERROR_HW;

        /*
         * STM32F1 clear PE/FE/NE/ORE bang sequence:
         * doc SR (da doc o tren) -> doc DR.
         */
        dummy = USART1->DR;
        (void)dummy;

        UART1_ErrorCallback(
            UART1_ERROR_HW
        );
    }

    /*
     * ------------------------------------------------------------
     * Transmission Complete
     * ------------------------------------------------------------
     *
     * DMA TC:
     *   byte cuoi da vao USART.
     *
     * USART TC:
     *   byte cuoi da ra het shift register / PA9.
     */
    if (((sr & USART_SR_TC) != 0u) &&
        ((USART1->CR1 & USART_CR1_TCIE) != 0u))
    {
        /*
         * Tat TC interrupt.
         * Khong RMW USART_SR de clear TC.
         */
        USART1->CR1 &=
            ~USART_CR1_TCIE;

        uart1_tx_waiting_tc = 0u;

        /*
         * Callback chi khi TX thuc su rong.
         */
        if ((uart1_tx_head == uart1_tx_tail) &&
            (uart1_tx_dma_active == 0u))
        {
            UART1_TxCompleteCallback();
        }
    }
}


/* ============================================================================
 * 16. WEAK CALLBACK
 * ============================================================================
 */

__WEAK void UART1_TxCompleteCallback(void)
{
    /*
     * Application co the override trong main.c.
     *
     * Vi du:
     *   uart_tx_complete_count++;
     */
}


__WEAK void UART1_ErrorCallback(
    UART1_Status_t error
)
{
    /*
     * Application co the override trong main.c.
     *
     * Vi du:
     *   uart_last_error = error;
     */
    (void)error;
}
