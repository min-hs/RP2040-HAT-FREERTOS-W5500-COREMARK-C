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

#include "wizchip_conf.h"
#include "socket.h"
#include "w5x00_spi.h"
#include "w5x00_gpio_irq.h"

#include "timer.h"

#include "mbedtls/x509_crt.h"
#include "mbedtls/error.h"
#include "mbedtls/ssl.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/compat-1.3.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
#define TCP_TASK_STACK_SIZE 4096
#define TCP_TASK_PRIORITY 10

#define RECV_TASK_STACK_SIZE 4096
#define RECV_TASK_PRIORITY 9

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (2048 * 2)

/* Socket */
#define SOCKET_SSL 2

/* Port */
#define PORT_SSL 443

/* SPI 1*/
#define SPI1_HANDLE spi1

#define SPI1_PIN_SCK 10
#define SPI1_PIN_TX 11       
#define SPI1_PIN_RX 12       
#define SPI1_PIN_CS 13

/* GPIO */
#define SPI1_STOP_PIN   7 

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 0, 100},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 0, 1},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
        .dhcp = NETINFO_STATIC                       // DHCP enable/disable
};

/* SSL */
static uint8_t g_send_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_recv_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_ssl_target_ip[4] = {192, 168, 0, 2};

static mbedtls_ctr_drbg_context g_ctr_drbg;
static mbedtls_ssl_config g_conf;
static mbedtls_ssl_context g_ssl;

/* Semaphore */
static xSemaphoreHandle send_sem = NULL;
static xSemaphoreHandle recv_sem = NULL;

/* Timer  */
static volatile uint32_t g_msec_cnt = 0;

/* SPI1 DMA */
static uint spi1_dma_tx;
static uint spi1_dma_rx;
static dma_channel_config spi1_dma_channel_config_tx;
static dma_channel_config spi1_dma_channel_config_rx;

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
 void tcp_task(void *argument);
void recv_task(void *argument);

void *pvPortCalloc(size_t sNb, size_t sSize);
void pvPortFree(void *vPtr);

/* Clock */
static void set_clock_khz(void);

/* SSL */
static int wizchip_ssl_init(uint8_t *socket_fd);
static int ssl_random_callback(void *p_rng, unsigned char *output, size_t output_len);

/* Timer  */
static void repeating_timer_callback(void);
static time_t millis(void);

/* SPI */
static void spi1_slave_init(void);
static void spi1_slave_read(uint8_t *pBuf, uint16_t len);

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

    printf("\n\n-- Slave start --\r\n");

    mbedtls_platform_set_calloc_free(pvPortCalloc, pvPortFree);

    spi1_slave_init();

    gpio_init(SPI1_STOP_PIN);
    gpio_set_dir(SPI1_STOP_PIN, GPIO_OUT);
    gpio_put(SPI1_STOP_PIN, 0);

    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    //wizchip_gpio_interrupt_initialize(SOCKET_SSL, gpio_callback);
    wizchip_1ms_timer_initialize(repeating_timer_callback);

    xTaskCreate(tcp_task, "TCP_Task", TCP_TASK_STACK_SIZE, NULL, TCP_TASK_PRIORITY, NULL);
    xTaskCreate(recv_task, "RECV_Task", RECV_TASK_STACK_SIZE, NULL, RECV_TASK_PRIORITY, NULL);
    
    send_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    recv_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);

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
void recv_task(void *argument)
{
    printf("recv task start\n");

    while(1)
    {
        spi1_slave_read(g_send_buf, ETHERNET_BUF_MAX_SIZE);

        xSemaphoreGive(send_sem);
    }
}

void tcp_task(void *argument)
{
    const int *list = NULL;
    int retval = 0;
    uint16_t len = 0;
    uint32_t start_ms = 0;

    printf("TCP Task start\r\n");  

    network_initialize(g_net_info);

    /* Get network information */
    print_network_information(g_net_info);

    retval = wizchip_ssl_init(SOCKET_SSL);

    if (retval < 0)
    {
        printf(" SSL initialize failed %d\n", retval);

        while (1)
        {
            vTaskDelay(1000 * 1000);
        }
    }

    /* Get ciphersuite information */
    printf(" Supported ciphersuite lists\n");

    list = mbedtls_ssl_list_ciphersuites();

    while (*list)
    {
        printf(" %-42s\n", mbedtls_ssl_get_ciphersuite_name(*list));

        list++;

        if (!*list)
        {
            break;
        }

        printf(" %s\n", mbedtls_ssl_get_ciphersuite_name(*list));

        list++;
    }

    retval = socket((uint8_t)(g_ssl.p_bio), Sn_MR_TCP, PORT_SSL, SF_TCP_NODELAY);

    if (retval != SOCKET_SSL)
    {
        printf(" Socket failed %d\n", retval);

        while (1)
        {
            vTaskDelay(1000 * 1000);
        }
    }

    start_ms = millis();

    do
    {
        retval = connect((uint8_t)(g_ssl.p_bio), g_ssl_target_ip, PORT_SSL);

        if ((retval == SOCK_OK) || (retval == SOCKERR_TIMEOUT))
        {
            break;
        }

        vTaskDelay(1000);

    } while ((millis() - start_ms) < RECV_TIMEOUT);

    if ((retval != SOCK_OK) || (retval == SOCK_BUSY))
    {
        printf(" Connect failed %d\n", retval);

        while (1)
        {
            vTaskDelay(1000 * 1000);
        }
    }

    printf(" Connected %d\n", retval);

    while ((retval = mbedtls_ssl_handshake(&g_ssl)) != 0)
    {
        if ((retval != MBEDTLS_ERR_SSL_WANT_READ) && (retval != MBEDTLS_ERR_SSL_WANT_WRITE))
        {
            printf(" failed\n  ! mbedtls_ssl_handshake returned -0x%x\n", -retval);

            while (1)
            {
                vTaskDelay(1000 * 1000);
            }
        }
    }

    printf(" ok\n    [ Ciphersuite is %s ]\n", mbedtls_ssl_get_ciphersuite(&g_ssl));

    gpio_put(SPI1_STOP_PIN, 1);

    while (1)
    {
        xSemaphoreTake(send_sem, portMAX_DELAY);
        
        gpio_put(SPI1_STOP_PIN, 0);

        retval = mbedtls_ssl_write(&g_ssl, g_send_buf, strlen(g_send_buf));
        
        gpio_put(SPI1_STOP_PIN, 1);

        if (retval < 0)
        {
            printf(" failed\n  ! mbedtls_ssl_write returned -0x%x\n", -retval);
            continue;
        }
    }
}

void *pvPortCalloc(size_t sNb, size_t sSize)
{
    void *vPtr = NULL;

    if (sSize > 0)
    {
        vPtr = pvPortMalloc(sSize * sNb); // Call FreeRTOS or other standard API

        if (vPtr)
        {
            memset(vPtr, 0, (sSize * sNb)); // Must required
        }
    }

    return vPtr;
}

void pvPortFree(void *vPtr)
{
    if (vPtr)
    {
        vPortFree(vPtr); // Call FreeRTOS or other standard API
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

/* SSL */
static int wizchip_ssl_init(uint8_t *socket_fd)
{
    int retval;

    mbedtls_ctr_drbg_init(&g_ctr_drbg);
    mbedtls_ssl_init(&g_ssl);
    mbedtls_ssl_config_init(&g_conf);

    if ((retval = mbedtls_ssl_config_defaults(&g_conf,
                                              MBEDTLS_SSL_IS_CLIENT,
                                              MBEDTLS_SSL_TRANSPORT_STREAM,
                                              MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
    {
        printf(" failed\n  ! mbedtls_ssl_config_defaults returned %d\n", retval);

        return -1;
    }

    printf(" Socket descriptor %d\n", socket_fd);

    mbedtls_ssl_conf_authmode(&g_conf, MBEDTLS_SSL_VERIFY_NONE);
    mbedtls_ssl_conf_rng(&g_conf, ssl_random_callback, &g_ctr_drbg);
    mbedtls_ssl_conf_endpoint(&g_conf, MBEDTLS_SSL_IS_CLIENT);
    mbedtls_ssl_conf_read_timeout(&g_conf, 1000 * 10);

    if ((retval = mbedtls_ssl_setup(&g_ssl, &g_conf)) != 0)
    {
        printf(" failed\n  ! mbedtls_ssl_setup returned %d\n", retval);

        return -1;
    }

    mbedtls_ssl_set_bio(&g_ssl, socket_fd, send, recv, NULL);

    return 0;
}

static int ssl_random_callback(void *p_rng, unsigned char *output, size_t output_len)
{
    int i;

    if (output_len <= 0)
    {
        return 1;
    }

    for (i = 0; i < output_len; i++)
    {
        *output++ = rand() % 0xff;
    }

    srand(rand());

    return 0;
}

/* Timer */
static void repeating_timer_callback(void)
{
    g_msec_cnt++;
}

static time_t millis(void)
{
    return g_msec_cnt;
}

/* SPI */
void spi1_slave_init(void)
{
    spi_init(SPI1_HANDLE, 0);
    spi_set_slave(SPI1_HANDLE, true);

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

void spi1_slave_read(uint8_t *pBuf, uint16_t len)
{
    uint8_t dummy = 0xFF;

    channel_config_set_read_increment(&spi1_dma_channel_config_tx, false);
    channel_config_set_write_increment(&spi1_dma_channel_config_tx, false);

    dma_channel_configure(spi1_dma_tx, &spi1_dma_channel_config_tx,
                        &dummy,                      // write address
                        &spi_get_hw(SPI_PORT)->dr, // read address
                        1,                       // element count (each element is of size transfer_data_size)
                        false);                    // don't start yet

    channel_config_set_read_increment(&spi1_dma_channel_config_rx, false);
    channel_config_set_write_increment(&spi1_dma_channel_config_rx, true);

    dma_channel_configure(
        spi1_dma_rx, &spi1_dma_channel_config_rx,
                        pBuf,                      // write address
                        &spi_get_hw(SPI1_HANDLE)->dr, // read address
                        len,                       // element count (each element is of size transfer_data_size)
                        false);                    // don't start yet

    dma_start_channel_mask((1u << spi1_dma_tx) | (1u << spi1_dma_rx));
    dma_channel_wait_for_finish_blocking(spi1_dma_rx);
}