/**
 * Copyright (c) 2022 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "port_common.h"

#include "timer.h"

#include "core_main.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
#define SEND_TASK_STACK_SIZE 4096
#define SEND_TASK_PRIORITY 10

#define COREMARK_TASK_STACK_SIZE 4096
#define COREMARK_TASK_PRIORITY 8

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (2048 * 2)

/* SPI 1*/
#define SPI1_HANDLE spi1

#define SPI1_PIN_SCK    10
#define SPI1_PIN_TX     11        
#define SPI1_PIN_RX     12       
#define SPI1_PIN_CS     13

/* GPIO */
#define SPI1_STOP_PIN   7           // interrupt pin

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
static uint8_t g_send_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};

/* Semaphore */
static xSemaphoreHandle send_sem;
static xSemaphoreHandle coremark_start_sem;

/* SPI1 DMA */
static uint spi1_dma_tx;
static uint spi1_dma_rx;
static dma_channel_config spi1_dma_channel_config_tx;
static dma_channel_config spi1_dma_channel_config_rx;

static uint32_t send_cnt = 0;

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
//void tcp_task(void *argument);
void send_task(void *argument);
void coremark_task(void *argument);

/* Clock */
static void set_clock_khz(void);

/* GPIO  */
static void gpio_callback(uint gpio, uint32_t events);

/* SPI */
static void spi1_master_init(void);
void spi1_master_write(uint8_t *pBuf, uint16_t len);

/**
 * ----------------------------------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------------------------------
 */
int main()
{
    /* Initialize */
    set_clock_khz();

    stdio_init_all();

    sleep_ms(5000);

    printf("\n\n-- Master start --\r\n");

    spi1_master_init();

    gpio_init(SPI1_STOP_PIN);
    gpio_set_dir(SPI1_STOP_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(SPI1_STOP_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    xTaskCreate(send_task, "SEND_Task", SEND_TASK_STACK_SIZE, NULL, SEND_TASK_PRIORITY, NULL);
    xTaskCreate(coremark_task, "COREMARK_Task", COREMARK_TASK_STACK_SIZE, NULL, COREMARK_TASK_PRIORITY, NULL);

    send_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    coremark_start_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);

    vTaskStartScheduler();

    while (1)
    {
        ;
    }
}
/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
void send_task(void *argument)
{
    uint8_t coremark_start = 0;
    bool input_value;

    printf("Send Task start\r\n");

    memset(g_send_buf, 0, ETHERNET_BUF_MAX_SIZE);

    while(1)
    {
        input_value = gpio_get(SPI1_STOP_PIN);

        if(input_value == 0)
        {
            xSemaphoreTake(send_sem, portMAX_DELAY);
        }

        if(coremark_start == 0)
        {
            xSemaphoreGive(coremark_start_sem);
            coremark_start = 1;
            printf("send cnt : %u\n", send_cnt);
        }

        memset(g_send_buf, 0, ETHERNET_BUF_MAX_SIZE);
        sprintf(g_send_buf, "SPI Channel 1 send : %u\n", send_cnt);

        spi1_master_write(g_send_buf, ETHERNET_BUF_MAX_SIZE);

        send_cnt++;
        if(send_cnt >= 0xFFFFFFFF)
            send_cnt = 0;

        vTaskDelay(10);
    }
}

void coremark_task(void *argument)
{
  xSemaphoreTake(coremark_start_sem, portMAX_DELAY);
  
  coremark_test();
  printf("Test finished\r\n");
  vTaskSuspendAll();

  while(1)
  {
    //vTaskDelay(1000);
  }
}

/* Clock */
static void set_clock_khz(void)
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}

void spi1_master_init(void)
{
    // this example will use SPI1 at 10MHz
    uint baud = spi_init(SPI1_HANDLE, 12000 * 1000);
    printf("baud : %u\n", baud);
    spi_set_slave(SPI1_HANDLE, false);

    gpio_set_function(SPI1_PIN_RX, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_PIN_TX, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_PIN_CS, GPIO_FUNC_SPI);

    bi_decl(bi_4pins_with_func(SPI1_PIN_RX, SPI1_PIN_TX, SPI1_PIN_SCK, SPI1_PIN_CS, GPIO_FUNC_SPI));

    spi1_dma_tx = dma_claim_unused_channel(true);
    spi1_dma_rx = dma_claim_unused_channel(true);

    spi1_dma_channel_config_tx = dma_channel_get_default_config(spi1_dma_tx);
    channel_config_set_transfer_data_size(&spi1_dma_channel_config_tx, DMA_SIZE_8);
    channel_config_set_dreq(&spi1_dma_channel_config_tx, DREQ_SPI1_TX);

    spi1_dma_channel_config_rx = dma_channel_get_default_config(spi1_dma_rx);
    channel_config_set_transfer_data_size(&spi1_dma_channel_config_rx, DMA_SIZE_8);
    channel_config_set_dreq(&spi1_dma_channel_config_rx, DREQ_SPI1_RX);
}

void spi1_master_write(uint8_t *pBuf, uint16_t len)
{
    uint8_t dummy = 0;

    channel_config_set_read_increment(&spi1_dma_channel_config_tx, true);
    channel_config_set_write_increment(&spi1_dma_channel_config_tx, false);

    dma_channel_configure(spi1_dma_tx, &spi1_dma_channel_config_tx,
                        &spi_get_hw(SPI1_HANDLE)->dr, // write address
                        pBuf,                      // read address
                        len,                       // element count (each element is of size transfer_data_size)
                        false);                    // don't start yet

    channel_config_set_read_increment(&spi1_dma_channel_config_rx, false);
    channel_config_set_write_increment(&spi1_dma_channel_config_rx, false);

    dma_channel_configure(
        spi1_dma_rx, &spi1_dma_channel_config_rx,
                        &dummy,                      // write address
                        &spi_get_hw(SPI1_HANDLE)->dr, // read address
                        1,                       // element count (each element is of size transfer_data_size)
                        false);                    // don't start yet

    dma_start_channel_mask((1u << spi1_dma_tx) | (1u << spi1_dma_rx));
    dma_channel_wait_for_finish_blocking(spi1_dma_rx);
}

static void gpio_callback(uint gpio, uint32_t events)
{
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(send_sem, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}