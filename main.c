/***************************************************************************//**
 * @file
 * @brief Simple LED Blink Demo for SLSTK3401A
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "bsp.h"

volatile uint32_t msTicks; /* counts 1ms timeTicks */

void Delay(uint32_t dlyTicks);

/***************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 ******************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

/***************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 ******************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}

uint16_t CRC_fnCalcDown_u16(uint8_t  *vF_pu8StartAddress, uint8_t  *vF_pu8EndAddress)

{

   uint16_t u16Ret = 0x0000u;

   uint8_t *pu8Data;



   for (pu8Data = vF_pu8EndAddress; pu8Data > vF_pu8StartAddress; pu8Data--)

   {

       u16Ret = (u16Ret >> 8) | (u16Ret << 8);

       u16Ret ^= *pu8Data;

       u16Ret ^= (u16Ret & 0xff) >> 4;

       u16Ret ^= u16Ret << 12;

       u16Ret ^= (u16Ret & 0xff) << 5;

   }



   return u16Ret;

}


uint16_t u16CheckCRC;

uint16_t u16CheckCRC1;
/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_STK_DEFAULT;
  CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_STK_DEFAULT;

  /* Chip errata */
  CHIP_Init();



  u16CheckCRC = CRC_fnCalcDown_u16((uint32_t *) 0x0FE081B1, (uint32_t *) 0x0FE085af);
  u16CheckCRC1 = DEVINFO->CAL & 0xFFFF;

  /* Init DCDC regulator and HFXO with kit specific parameters */
  EMU_DCDCInit(&dcdcInit);
  CMU_HFXOInit(&hfxoInit);

  /* Switch HFCLK to HFXO and disable HFRCO */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
  CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) {
    while (1) ;
  }

  /* Initialize LED driver */
  BSP_LedsInit();
  BSP_LedSet(0);
  BSP_LedClear(1);

  /* Infinite blink loop */
  while (1) {
    BSP_LedToggle(0);
    BSP_LedToggle(1);
    Delay(1000);
  }
}
