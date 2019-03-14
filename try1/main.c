/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
*
* @defgroup nrf_dev_radio_rx_example_main main.c
* @{
* @ingroup nrf_dev_radio_rx_example
* @brief Radio Receiver example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO peripheral.
*
*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "nrf52840.h"
#include "nrf52840_bitfields.h"
#include "nrf_error.h"

#define USE_RX_IRQ
//#define USE_ED



char data[100];
static uint32_t position=0;
static uint32_t                   packet;              /**< Packet to transmit. */

void EnableRadioIRQ()
{
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);
  NRF_RADIO->INTENSET=RADIO_INTENSET_FRAMESTART_Msk 
    | RADIO_INTENSET_PAYLOAD_Msk
    | RADIO_INTENSET_END_Msk;
  NRF_RADIO->INTENCLR = 0x00;
}


/**@brief Function for initialization oscillators.
 */
void clock_initialization()
{
  
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    /* Start low frequency crystal oscillator for app_timer(used by bsp)*/
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}


void dostuff()
{
  if (position == 100)
  {
    position = 0;
  }
  else
  {
    
    position = position + 1;
    
  }
  data[position]=packet;
}

/**@brief Radio interrupt handler.
 */
void RADIO_IRQHandler(void)
{
  uint32_t result = 0;
  
  
  if (NRF_RADIO->EVENTS_FRAMESTART)// ==1UL)
  {
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NRF_RADIO->EVENTS_FRAMESTART = 0;
    dostuff();
    
    
  }
  else if (NRF_RADIO->EVENTS_END)
  {
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NRF_RADIO->EVENTS_END= 0;
    //process packet
  }
  else if (NRF_RADIO->EVENTS_PAYLOAD)
  {
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NRF_RADIO->EVENTS_PAYLOAD= 0;
    //process packet
  }
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  EnableRadioIRQ();
  //result = packet;
  
  
  
  
}

/**@brief Function for reading packet.
 */
uint32_t read_packet()
{
    uint32_t result = 0;
    NRF_RADIO->TASKS_RXEN = 1UL; // Enable Radio RX
    while (!NRF_RADIO->EVENTS_READY)
    {
      //do nothing
    }
    
    NRF_RADIO->EVENTS_END = 0U;
    // Start listening and wait for address received event
    
    NRF_RADIO->TASKS_START=1UL;//Start Radio Rx
    //NRF_RADIO->EVENTS_READY = 0U;
    // Enable radio and wait for ready
    //NRF_RADIO->TASKS_RXEN = 1U;


    

    // Wait for end of packet or buttons state changed
    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }

    //if (NRF_RADIO->CRCSTATUS == 1U)
    {
        result = packet;
    }
    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        // wait
    }
    return result;
}



/**@brief Function for reading packet.
 */
void read_packet2()
{
    uint32_t result = 0;
 
#ifndef USE_ED
    // Wait for end of packet or buttons state changed
    while (NRF_RADIO->EVENTS_FRAMESTART == 0U )
    {
        
    }
#endif

#ifdef USE_ED
    while (NRF_RADIO->EVENTS_EDEND != 1 && NRF_RADIO->EDSAMPLE >20UL )
    {
        //NRF_RADIO->TASKS_EDSTART = 1UL;
        
        NRF_RADIO->TASKS_EDSTART = 1UL;
        
    }
    NRF_RADIO->TASKS_EDSTOP;
#endif    
    
    while (NRF_RADIO->EVENTS_PHYEND == 0U)
    {
          dostuff();
    }
    
      NRF_RADIO->EVENTS_END = 0U;
    NRF_RADIO->EVENTS_FRAMESTART = 0U;
    // Start listening and wait for address received event
    
    NRF_RADIO->TASKS_START=1UL;//Start Radio Rx


    
}

/**
 * @brief Function for configuring the radio to operate in ShockBurst compatible mode.
 *
 * To configure the application running on nRF24L series devices:
 *
 * @verbatim
 * uint8_t tx_address[5] = { 0xC0, 0x01, 0x23, 0x45, 0x67 };
 * hal_nrf_set_rf_channel(7);
 * hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);
 * hal_nrf_set_address(HAL_NRF_TX, tx_address);
 * hal_nrf_set_address(HAL_NRF_PIPE0, tx_address);
 * hal_nrf_open_pipe(0, false);
 * hal_nrf_set_datarate(HAL_NRF_1MBPS);
 * hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);
 * hal_nrf_setup_dynamic_payload(0xFF);
 * hal_nrf_enable_dynamic_payload(false);
 * @endverbatim
 *
 * When transmitting packets with hal_nrf_write_tx_payload(const uint8_t *tx_pload, uint8_t length),
 * match the length with PACKET_STATIC_LENGTH.
 * hal_nrf_write_tx_payload(payload, PACKET_STATIC_LENGTH);
 *
*/
void radio_configure()
{
    // Radio config
    
    NRF_RADIO->MODE      = RADIO_MODE_MODE_Ieee802154_250Kbit;//(RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->FREQUENCY = 5UL;  // Frequency 2400MHz + FREQUENCY (MHz)
    
    
    NRF_RADIO->CRCCNF=0x202;
    NRF_RADIO->CRCPOLY= 0x11021;
    NRF_RADIO->CRCINIT=0UL;
    
    NRF_RADIO->TASKS_RXEN = 1UL; // Enable Radio RX
    while (!NRF_RADIO->EVENTS_READY)
    {
      //do nothing
    }
    
    //NRF_RADIO->SFD = 0x41;
    
#ifdef USE_ED
    NRF_RADIO->TASKS_EDSTART=1UL;
#endif

#ifdef USE_RX_IRQ    
       
    
    //NRF_RADIO->EVENTS_READY = 0U;
    
    //NVIC_SetPriority(RADIO_IRQn, 1);
    
    EnableRadioIRQ();
       
#endif
    
    
    NRF_RADIO->PACKETPTR = (uint32_t)&packet;
    NRF_RADIO->TASKS_START=1UL;//Start Radio Rx
    
  
    
    
}

/**
 * @brief Function for application main entry.
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
    uint32_t err_code = NRF_SUCCESS;

    clock_initialization();
   
    // Set radio configuration parameters
    
    //NRF_RADIO->FREQUENCY = 15UL;  // Frequency bin 15, 2425MHz
    
    
    radio_configure();
    



    while (true)
    
    
    
    {
        //uint32_t received = read_packet();
#ifndef USE_RX_IRQ
      read_packet2();
#endif

    }

}

/**
 *@}
 **/
