/**
 * MIT License
 * 
 * Copyright (c) 2019 Martin Aalien
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "product.h"
#include "timer.h"
#include "nrf_gfx.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include <stdio.h> 
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define GPIO_NUMBER_LED0 13
#define GPIO_NUMBER_LED1 14
#define RX_TIMEOUT 40 /** Time to wait for response  x16us */
#define TRX_PERIOD  45 /** Time from packet sent to Tx enable  x16us */

#define DATABASE 0x20001000 /** Base address for measurement database */
#define DATA_SIZE 128 
#define NUM_BINS 128 
#define NUMBER_OF_MEASUREMENTS NUM_BINS*10

#define BLE2M

#define CYAN 0x07FF /** Background color for ili9341 LCD display */

static uint8_t test_frame[255] = {0x00, 0x04, 0xFF, 0xC1, 0xFB, 0xE8};

static uint32_t tx_pkt_counter = 0;
static uint32_t radio_freq = 78;

static uint32_t timeout;
static uint32_t telp;
static uint32_t rx_pkt_counter = 0;
static uint32_t rx_pkt_counter_crcok = 0;
static uint32_t rx_timeouts = 0;
static uint32_t rx_ignored = 0;
static uint8_t rx_test_frame[256];

static uint32_t database[DATA_SIZE] __attribute__((section(".ARM.__at_DATABASE")));
static uint32_t dbptr=0;
static uint32_t bincnt[NUM_BINS];

static uint32_t highper=0;
static uint32_t txcntw=0;

extern const nrf_gfx_font_desc_t orkney_8ptFontInfo;
extern const nrf_lcd_t nrf_lcd_ili9341;

static const nrf_gfx_font_desc_t * p_font = &orkney_8ptFontInfo;
static const nrf_lcd_t * p_lcd = &nrf_lcd_ili9341;

static char distributionNumbers[5];
static char buf[5];
static int numClock;

/**
 * @brief Initializes the radio
 */
void nrf_radio_init(void)
{
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) | 
                        (RADIO_SHORTS_DISABLED_TXEN_Enabled << RADIO_SHORTS_DISABLED_TXEN_Pos);
                    
    NRF_RADIO->TIFS = 210;
            

#if defined(BLE2M)
    NRF_RADIO->MODE = 4 << RADIO_MODE_MODE_Pos;
    
    uint32_t aa_address = 0x71764129;
    NRF_RADIO->BASE0 = aa_address << 8;
    NRF_RADIO->PREFIX0 = (0xffffff00 | aa_address >> 24);        

    NRF_RADIO->TXADDRESS = 0;
    NRF_RADIO->RXADDRESSES = 1;
    NRF_RADIO->DATAWHITEIV = 39;        
    NRF_RADIO->PCNF0 = 0x01000108;
        
    NRF_RADIO->PCNF1 = 0x000300FF; /* sw:turn off whitening */
    NRF_RADIO->CRCPOLY = 0x65B;
    NRF_RADIO->CRCINIT = 0x555555;
    NRF_RADIO->CRCCNF = 0x103;
#endif

    NRF_RADIO->FREQUENCY = (RADIO_FREQUENCY_MAP_Default << RADIO_FREQUENCY_MAP_Pos)  +
                         ((radio_freq << RADIO_FREQUENCY_FREQUENCY_Pos) & RADIO_FREQUENCY_FREQUENCY_Msk);

    NRF_RADIO->PACKETPTR = (uint32_t)test_frame;
    NRF_RADIO->EVENTS_DISABLED = 0;

    NRF_RADIO->TXPOWER=0x0;
}

/**
 * @brief Setting up LEDs used for inication packet received or sent
 */
void setup_leds()
{
    NRF_GPIO->PIN_CNF[GPIO_NUMBER_LED0] = \
    ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | \
    (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) | \
    (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | \
    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | \
    (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));

    NRF_GPIO->PIN_CNF[GPIO_NUMBER_LED1] = \
    ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | \
    (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) | \
    (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | \
    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | \
    (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
}

/**
 * @brief Setting up ppi for capturing time of flight
 */
void nrf_ppi_config (void)
{
    NRF_PPI->CH[6].TEP = (uint32_t)(&NRF_TIMER0->TASKS_CAPTURE[0]);
    NRF_PPI->CH[6].EEP = (uint32_t)(&NRF_RADIO->EVENTS_ADDRESS); 

    NRF_PPI->CH[7].TEP = (uint32_t)(&NRF_TIMER0->TASKS_START);
    NRF_PPI->CH[7].EEP = (uint32_t)(&NRF_RADIO->EVENTS_END);
 
    NRF_PPI->CHENSET =  (1 << 6) | (1 << 7);

    NRF_TIMER0->TASKS_STOP = 1;
    NRF_TIMER0->TASKS_CLEAR = 1;
}


/**
 * @brief Initializes the ili9341 LCD display
 */
static void gfx_initialization(void)
{
    APP_ERROR_CHECK(nrf_gfx_init(p_lcd));
}

/**
 * @brief Clears the screen and sets the background color
 */
static void screen_clear(void)
{
    nrf_gfx_screen_fill(p_lcd, CYAN);
}

/**
 * @brief Calculates and returns distance in meters
 * 
 * @return Distance [m]
 */
float calc_dist()
{
    float val = 0;
    int sum = 0;
    for(int i = 0; i < NUM_BINS; i++)
    {
        val += database[i]*(i+1);
        sum += database[i];
    }
    val = val/sum;
    //val = 0.5*18.737*val;
    return val;
}

int main(void)
{
    uint32_t tempval, tempval1;
    int i, j, binNum;
    
    nrf_ppi_config();
    setup_leds();
        
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    /* Puts zeros into bincnt */
    memset(bincnt, 0, sizeof bincnt);

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
    }

    /* Configure the timer with prescaler 0, counts every 1 cycle of timer clock (16MHz) */
    timer0_capture_init(0); 

    nrf_radio_init();
    NRF_CLOCK->TASKS_HFCLKSTART = 1; /* Start HFCLK */

    tx_pkt_counter = 0;

    /* Initialize ili9341 LCD screen */
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    gfx_initialization();

    /* Setting start writing position on the dispay */
    nrf_gfx_point_t text_start = NRF_GFX_POINT(5, nrf_gfx_height_get(p_lcd) - 250);
    
    screen_clear();

    while (true)
    { 
        while (dbptr < NUMBER_OF_MEASUREMENTS)
        {
            NRF_RADIO->PACKETPTR = (uint32_t) test_frame; /* Switch to tx buffer */
            NRF_RADIO->TASKS_RXEN = 0x0;
                
            NRF_GPIO->OUTSET = 1 << GPIO_NUMBER_LED0; /* Rx LED Off */
            NRF_GPIO->OUTCLR = 1 << GPIO_NUMBER_LED1; /* Tx LED On */

            /* Copy the tx packet counter into the payload */
            test_frame[2]=(tx_pkt_counter & 0x0000FF00) >> 8;
            test_frame[3]=(tx_pkt_counter & 0x000000FF);
            
            /* Start Tx */
            NRF_RADIO->TASKS_TXEN = 0x1;
                
            /* Wait for transmision to begin */
            while (NRF_RADIO->EVENTS_READY == 0)
            {
            }
            
            NRF_RADIO->EVENTS_READY = 0;
                
            /* Packet is sent */
            while (NRF_RADIO->EVENTS_END == 0)
            {
            }
            NRF_RADIO->EVENTS_END = 0;      
            
            /* Disable radio */
            while (NRF_RADIO->EVENTS_DISABLED == 0)
            {
            }
            NRF_RADIO->EVENTS_DISABLED = 0;
            
            tx_pkt_counter++;
            txcntw++;
            
            if(txcntw > 50)
            {
                txcntw = 0;
                if(rx_timeouts > 10)
                    highper = 1;
                else
                    highper = 0;
                rx_timeouts = 0;
            }
            
            /** 
             * Packet sent, switch to Rx asap 
             * Note: there is a small delay inserted on the B side to avoid race here
             */
            NRF_GPIO->OUTSET = 1 << GPIO_NUMBER_LED1; /* Tx LED Off */
            NRF_RADIO->TASKS_TXEN = 0x0;
            NRF_RADIO->TASKS_RXEN = 0x1;
            NRF_RADIO->PACKETPTR = (uint32_t) rx_test_frame; /* Switch to rx buffer*/
            
            /* Wait for response or timeout */
            timeout=0; 
            while ((NRF_RADIO->EVENTS_DISABLED == 0) && (timeout<2048))
            { 
                    timeout++;
            }

            NRF_TIMER0->TASKS_STOP = 1;
            NRF_TIMER0->TASKS_CLEAR = 1;

            /* Now, did we time out? */
            if(timeout>=2048)
            {
                /* Timeout, stop radio manually */
                NRF_RADIO->TASKS_STOP = 1;
                NRF_RADIO->TASKS_DISABLE = 1;
                while(NRF_RADIO->EVENTS_DISABLED == 0);
                rx_timeouts++;
            }
            else
            {
                /* Packet received */
                if(highper < 1)
                    NRF_GPIO->OUTCLR = 1 << GPIO_NUMBER_LED0;  /* Rx LED On */
                
                rx_pkt_counter++;

                if(NRF_RADIO->CRCSTATUS>0)
                {
                    /**
                     * Process the received packet
                     * Check the sequence number in the received packet against our tx packet counter
                     */
                    rx_pkt_counter_crcok++;
                    tempval = ((rx_test_frame[2] << 8) + (rx_test_frame[3]));
                    tempval1 = tx_pkt_counter-1;

                    if(tempval != (tempval1&0x0000FFFF))
                        rx_ignored++;
                    else
                    {
                        /* Packet is good, update stats */
                        
                        telp = NRF_TIMER0->CC[0];  
                        binNum = telp - 4030; /* Magic number to trim away dwell time in device B, etc */
                        
                        if((binNum >= 0) && (binNum < NUM_BINS))
                                bincnt[binNum]++;
                        dbptr++;
                    }
                }
            }
            NRF_RADIO->EVENTS_DISABLED = 0;
        }
        
        NRF_RADIO->TASKS_STOP = 1;
        NRF_RADIO->TASKS_DISABLE = 1;
        while(NRF_RADIO->EVENTS_DISABLED == 0);

        /* Loading measurements in to database */
        for(j = 0; j < NUM_BINS; j++)
        {
            database[j] = bincnt[j];
            bincnt[j] = 0;
        }
        dbptr = 0;
        
        /* Distribution plot */
        nrf_gfx_point_t const * p_point = &text_start;
        nrf_lcd_t const * p_instance = p_lcd;
        uint16_t x = p_point->x;
        uint16_t y = p_point->y;

        gcvt(calc_dist(), 5, buf);
        
        numClock = (int) calc_dist();

        screen_clear();
        
        /* Print number of clock cycles */
        for(i = 0; i < 5; i++)
        {
            write_character(p_instance, p_font, (uint8_t) buf[i], &x, y - 40, 0);
        }

        // memset(buf, 0, sizeof buf);

        // sprintf(buf, "%d", (int) rx_timeouts);
        
        // for(i = 0; i < 5; i++)
        // {
        //     write_character(p_instance, p_font, (uint8_t) buf[i], &x, y, 0);
        // }

        // rx_timeouts = 0;

        /* Print distribution from database */
        for(i = numClock - 5; i < numClock + 6; i++)
        {  
            x = p_point->x;
            y = p_point->y;
            
            memset(distributionNumbers, 0, sizeof distributionNumbers);
            sprintf(distributionNumbers, "%lu", database[i]);
            
            for(j = 0; j < 5; j++)
            {
                if(distributionNumbers[j] != 0)
                {
                    write_character(p_instance, p_font, (uint8_t) distributionNumbers[j], &x, y + 20*(i - numClock + 5), 0);
                }
            }
        }

    }
    NRF_GPIO->OUTSET = 1 << GPIO_NUMBER_LED1; /* Tx LED Off */
}


void HardFault_Handler(void)
{
    while (true)
    {
    }
}

void MemoryManagement_Handler(void)
{
    while (true)
    {
    }
}

void BusFault_Handler(void)
{
    while (true)
    {
    }
}

void UsageFault_Handler(void)
{
    while (true)
    {
    }
}