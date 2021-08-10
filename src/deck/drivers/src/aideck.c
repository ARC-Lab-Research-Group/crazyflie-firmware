/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * aideck.c - Deck driver for the AIdeck
 */
#define DEBUG_MODULE "AIDECK"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "debug.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"
#include "system.h"

#include "aideck.h"
#include "stabilizer_types.h"
#include "aideck_uart_dma.h"
#include "statsCnt.h"

#if defined CBF_TYPE_POS || defined CBF_TYPE_EUL
#ifdef CBF_ITERS
static u_it_t u_struct;
#else
static u_t u_struct;
#endif // CBF_ITERS
u_t *u;
static CBFPacket pk_rx; // Packet for receiving via UART
static CBFPacket pk_tx; // Packet to send via UART
static uint8_t aideck_ready_flag; // Set to 1 whenever a CBFPacket is received via UART
static uint8_t missed_cycles; // Keeps track of the number of cbf_qpdata that have been discarded
static cbf_qpdata_comp_t data_comp; // Compressed CBF-QP Data
volatile uint8_t dma_flag = 0;
static statsCntRateLogger_t counterCBF; // Rate logger
#endif //CBF_TYPE
static bool isInit = false;

//Uncomment when NINA printout read is desired from console
//#define DEBUG_NINA_PRINT

#ifdef DEBUG_NINA_PRINT
static void NinaTask(void *param)
{
    systemWaitStart();
    vTaskDelay(M2T(1000));
    DEBUG_PRINT("Starting reading out NINA debugging messages:\n");
    vTaskDelay(M2T(2000));

    // Pull the reset button to get a clean read out of the data
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    vTaskDelay(10);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

    // Read out the byte the NINA sends and immediately send it to the console.
    uint8_t byte;
    while (1)
    {
        if (uart2GetDataWithDefaultTimeout(&byte) == true)
        {
            consolePutchar(byte);
        }
    }
}
#endif

#ifdef AI_CBF_DEBUG
static uint8_t hx_code[2];
// Transform a raw (0-16) number
// to the equivalent ASCII char in
// HEX code
uint8_t offset_hex(uint8_t raw){
  if(raw < 10)
    return(raw + 48);
  else
    return(raw + 87);
}
// Allow DEBUG_PRINT to display HEX code
// Convert 1 byte to 2 Chars representing
// the HEX code for the byte
static void hex_to_char(uint8_t hex){
  hx_code[0] = offset_hex(hex/16);
  hx_code[1] = offset_hex(hex%16);
}
#endif

#if defined CBF_TYPE_POS || defined CBF_TYPE_EUL
// Update u with a stop command in case of error
static void force_stop_u(void){
  u->T = 0;
#ifdef CBF_TYPE_EUL
  u->p = 0;
  u->q = 0;
  u->r = 0;
#elif CBF_TYPE_POS
  u->phi = 0;
  u->theta = 0;
  u->psi = 0;
#endif
}
#endif

#ifdef AI_CBF_DEBUG
// DEBUG PRINT the u_t struct
static void print_u(void){
  DEBUG_PRINT("u.T = %.4f\n",(double)u->T);
#ifdef CBF_TYPE_EUL
  DEBUG_PRINT("u.p = %.4f\n",(double)u->p);
  DEBUG_PRINT("u.q = %.4f\n",(double)u->q);
  DEBUG_PRINT("u.r = %.4f\n",(double)u->r);
#elif CBF_TYPE_POS
  DEBUG_PRINT("u.phi = %.4f\n",(double)u->phi);
  DEBUG_PRINT("u.theta = %.4f\n",(double)u->theta);
  DEBUG_PRINT("u.psi = %.4f\n",(double)u->psi);
#endif
#ifdef CBF_ITERS
  DEBUG_PRINT("iters = %d\n",u_struct.iters);
#endif
  DEBUG_PRINT("Missed Cycles = %d\n\n", missed_cycles);
}
#endif

#if defined CBF_TYPE_POS || defined CBF_TYPE_EUL
// Update the u struct from received data and clear pk_rx
static uint8_t unpack(void){
  if(pk_rx.header!='V'){ return 1; // Check healthy pk
    memset(pk_rx.raw, 0, sizeof(CBFPacket)); // Clear packet for new data
    return 1;
  }
  STATS_CNT_RATE_EVENT(&counterCBF); // Packet received, update rate
  aideck_ready_flag = 1; // AI Deck is ready for more data
  memcpy(&u_struct, pk_rx.data, sizeof(u_struct)); // Extract data from packet
  memset(pk_rx.raw, 0, sizeof(CBFPacket)); // Clear packet for new data
  return 0;
}
#endif



// AI Deck task to listen for UART traffic from the AI Deck
static void Gap8Task(void *param) {
  systemWaitStart();
  vTaskDelay(M2T(1000));

  // Pull the reset button to get a clean read out of the data
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);
  vTaskDelay(10);
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

  // Receive data in a loop
  while (1){
#if defined CBF_TYPE_POS || defined CBF_TYPE_EUL
    vTaskDelay(M2T(1));
    if(dma_flag){
      dma_flag = 0; // Clear the flag
      if(unpack()){ // process CBFPacket
        USART_DMA_ResetCounter(sizeof(CBFPacket), pk_rx.raw); // Sync with AI Deck
        aideck_ready_flag = 1; // AI Deck is ready for more data
      }
#ifdef AI_CBF_DEBUG
      print_u(); // Show debug info
#endif
    }
#else // CBF_TYPE
    vTaskDelay(M2T(100)); // Longer delay if not in CBF mode
#endif // CBF_TYPE
  }
}

static void aideckInit(DeckInfo *info){
  if (isInit)
      return;

  // Initialize task for the GAP8
  xTaskCreate(Gap8Task, AI_DECK_GAP_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
              AI_DECK_TASK_PRI, NULL);

#if defined CBF_TYPE_EUL || defined CBF_TYPE_POS
#ifdef CBF_ITERS
  u = &(u_struct.u);
#else
  u = &(u_struct);
#endif
  // Start DMA Rx and configure USART Tx Rx
  USART_DMA_Start(115200, pk_rx.raw, sizeof(CBFPacket));
  // Start a rate logger for 100 Hz
  STATS_CNT_RATE_INIT(&counterCBF, 10);
  // The AI Deck is ready for UART Data
  aideck_ready_flag = 1;
#endif

#ifdef DEBUG_NINA_PRINT
  // Initialize the UART for the NINA
  uart2Init(115200);
  // Initialize task for the NINA
  xTaskCreate(NinaTask, AI_DECK_NINA_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);
#endif

  isInit = true;
}

static bool aideckTest(){
    return true;
}

#if defined CBF_TYPE_EUL || defined CBF_TYPE_POS
// Send the CBF-QP Parametric data via UART1
void aideck_send_cbf_data(const cbf_qpdata_t *data){
  if(aideck_ready_flag){ // Is the AI Deck ready to receive data?
    // Compress data
#ifdef CBF_TYPE_EUL
    data_comp.phi = (int16_t)(data->phi*1000.0f);
    data_comp.theta = (int16_t)(data->theta*1000.0f);
    data_comp.u.T = (int16_t)(data->u.T*1000.0f);
    data_comp.u.p = (int16_t)(data->u.p*1000.0f);
    data_comp.u.q = (int16_t)(data->u.q*1000.0f);
    data_comp.u.r = (int16_t)(data->u.r*1000.0f);
#elif CBF_TYPE_POS
    data_comp.x = (int16_t)(data->x*1000.0f);
    data_comp.y = (int16_t)(data->y*1000.0f);
    data_comp.z = (int16_t)(data->z*1000.0f);
    data_comp.x_dot = (int16_t)(data->x_dot*1000.0f);
    data_comp.y_dot = (int16_t)(data->y_dot*1000.0f);
    data_comp.z_dot = (int16_t)(data->z_dot*1000.0f);
    data_comp.u.T = (int16_t)(data->u.T*1000.0f);
    data_comp.u.phi = (int16_t)(data->u.phi*1000.0f);
    data_comp.u.theta = (int16_t)(data->u.theta*1000.0f);
    data_comp.u.psi = (int16_t)(data->u.psi*1000.0f);
#endif
    // Pack data
    cbf_pack(sizeof(cbf_qpdata_comp_t), (uint8_t *)&data_comp);
#ifdef AI_CBF_DEBUG
#undef DEBUG_FMT
#define DEBUG_FMT(fmt) fmt
    DEBUG_PRINT("TX %c",pk_tx.header);
    for (int j=0;j<sizeof(CBFPacket)-1;j++){
      hex_to_char(pk_tx.data[j]);
      DEBUG_PRINT("%c%c",hx_code[0],hx_code[1]);
    }
    DEBUG_PRINT("\n");
#undef DEBUG_FMT
#define DEBUG_FMT(fmt) DEBUG_MODULE ": " fmt
#endif
    // Send packet
    USART_DMA_Send(sizeof(CBFPacket), pk_tx.raw);
    memset(&pk_tx, 0, sizeof(CBFPacket)); // Clear packet after sending
    // AI Deck is processing the data
    aideck_ready_flag = 0;
    missed_cycles = 0; // Reset cycles
  }
  else{
    missed_cycles++; // Missed this cycle
#ifdef AI_CBF_DEBUG
    //DEBUG_PRINT("Missed Cycles = %d\n",missed_cycles);
#endif
    if(missed_cycles > 200){ // Don't miss more than X cycles
      DEBUG_PRINT("Too many missed cycles\n");
      force_stop_u();
      aideck_ready_flag = 1; // Force ready
    }
  }
}
#endif

#if defined CBF_TYPE_POS || defined CBF_TYPE_EUL
// Pack data into CBFPacket pk_tx
CBFPacket *cbf_pack(const uint8_t size, uint8_t *data){
  // Check data size
  if (size > MAX_CBFPACKET_DATA_SIZE){
    pk_tx.header = 0;
    DEBUG_PRINT("ERROR Size %d too large for CBFPacket (%d)\n",size,MAX_CBFPACKET_DATA_SIZE);
    return NULL;
  }

  // Populate header with 'V' char
  pk_tx.header = 'V'; // 0d86 0x56 0b01010110
  // Populate data array
  memcpy(pk_tx.data, data, size);

  return &pk_tx;
}
#endif

// Give controller the CBF-QP Solution
void aideck_get_safe_u(float *u_control){
#ifdef CBF_TYPE_EUL
  u_control[0] = u->T;
  u_control[1] = u->p;
  u_control[2] = u->q;
  u_control[3] = u->r;
#elif CBF_TYPE_POS
  u_control[0] = u->T;
  u_control[1] = u->phi;
  u_control[2] = u->theta;
  u_control[3] = u->psi;
#endif
}


// IRQ DMA
#if defined CBF_TYPE_POS || defined CBF_TYPE_EUL
void __attribute__((used)) DMA1_Stream1_IRQHandler(void){
  DMA_ClearFlag(DMA1_Stream1, UART3_RX_DMA_ALL_FLAGS);
  dma_flag = 1;
}
#endif


static const DeckDriver aideck_deck = {
    .vid = 0xBC,
    .pid = 0x12,
    .name = "bcAI",

    .usedPeriph = DECK_USING_UART1 | DECK_USING_UART2, // GAP | NINA
    .usedGpio = DECK_USING_RX1 | DECK_USING_TX1 | DECK_USING_RX2 | DECK_USING_TX2,

    .init = aideckInit,
    .test = aideckTest,
};



LOG_GROUP_START(aideck)
#if defined CBF_TYPE_POS || defined CBF_TYPE_EUL
STATS_CNT_RATE_LOG_ADD(rateCBF, &counterCBF)
LOG_ADD(LOG_UINT8, missed_cycles, &missed_cycles)
#ifdef CBF_ITERS
LOG_ADD(LOG_UINT16, iters, &(u_struct.iters))
#endif
#endif
LOG_GROUP_STOP(aideck)

/** @addtogroup deck
*/
PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [AI deck](%https://store.bitcraze.io/collections/decks/products/ai-deck-1-1) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAIDeck, &isInit)

PARAM_GROUP_STOP(deck)

DECK_DRIVER(aideck_deck);
