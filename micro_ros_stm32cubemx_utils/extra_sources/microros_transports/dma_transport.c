#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define UART_DMA_BUFFER_SIZE 2048

// RX buffer must be in D2 RAM for DMA1/DMA2 accessibility on STM32H7
uint8_t dma_buffer[UART_DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
// TX bounce buffer to copy data from DTCMRAM/AXI-SRAM to D2 RAM for DMA
static uint8_t tx_bounce_buffer[UART_DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));

static size_t dma_head = 0, dma_tail = 0;

bool cubemx_transport_open(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_DMAStop(uart);
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    if (len > UART_DMA_BUFFER_SIZE) len = UART_DMA_BUFFER_SIZE;
    memcpy(tx_bounce_buffer, buf, len);
    __DSB();

    HAL_StatusTypeDef ret;
    if (uart->gState == HAL_UART_STATE_READY){
        ret = HAL_UART_Transmit_DMA(uart, tx_bounce_buffer, len);
        while (ret == HAL_OK && uart->gState != HAL_UART_STATE_READY){
            osDelay(1);
        }

        return (ret == HAL_OK) ? len : 0;
    }else{
        return 0;
    }
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    int ms_used = 0;
    do
    {
        __disable_irq();
        dma_tail = (UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx)) % UART_DMA_BUFFER_SIZE;
        __enable_irq();

        ms_used++;
        osDelay(1);
    } while (dma_head == dma_tail && ms_used < timeout);
    
    size_t wrote = 0;
    while ((dma_head != dma_tail) && (wrote < len)){
        buf[wrote] = dma_buffer[dma_head];
        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        wrote++;
    }
    
    return wrote;
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM