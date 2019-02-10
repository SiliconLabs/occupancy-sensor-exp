/***************************************************************************//**
 * @file
 * @brief Main demonstration file for OCCUPANCY-EXP-EB
 * @version 1.0.0
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "em_device.h"
#include "em_chip.h"
#include "em_assert.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_pcnt.h"

#include "display.h"
#include "textdisplay.h"
#include "bspconfig.h"
#include "retargettextdisplay.h"

#include "pirdrv.h"
#include "ambient_light.h"
#include "occupancy_exp_main.h"

static volatile bool startTest = false;             // Start selected energy mode test.
static uint32_t shownDemoMode = demoModeOccupancy;  // The demo selected by button
/*******************************************************************************
 *************************   FUNCTION PROTOTYPES   *****************************
 ******************************************************************************/
static void gpioSetup(void);
static void pcntInit(void);

static void LCD_Init(DISPLAY_Device_t *displayDevice);
static Demo_Mode_TypeDef LCD_SelectDemo(void);
static void LCD_UpdateMenu(uint8_t selectedDemoLine);
static void LCD_TearDown(DISPLAY_Device_t *displayDevice);

/*******************************************************************************
 ***************************   LOCAL FUNCTIONS   *******************************
 ******************************************************************************/
/**
 * @brief Main function for demonstrating the OCCUPANCY-EXP-EB
 */
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  /* Set up for low power operation. */
  EMU_EM23Init_TypeDef em2Init = EMU_EM23INIT_DEFAULT;
  em2Init.vScaleEM23Voltage = emuVScaleEM23_LowPower;
  em2Init.em23VregFullEn = false;
  EMU_EM23Init(&em2Init);

  EMU_EM01Init_TypeDef em0Init = EMU_EM01INIT_DEFAULT;
  em0Init.vScaleEM01LowPowerVoltageEnable = true;
  EMU_EM01Init(&em0Init);

  gpioSetup();

  DISPLAY_Device_t lcd;
  LCD_Init(&lcd);
  Demo_Mode_TypeDef demo = LCD_SelectDemo();
  if (demo == demoModeOccupancyLowPower || demo == demoModeAmbientLightLowPower) {
    LCD_TearDown(&lcd);
  }

  switch (shownDemoMode) {
    case demoModeOccupancy:
      PIR_Main(false);
      break;
    case demoModeOccupancyLowPower:
      PIR_Main(true);
      break;
    case demoModeAmbientLight:
      ALS_Main(false);
      break;
    case demoModeAmbientLightLowPower:
      ALS_Main(true);
      break;
    default:
      EFM_ASSERT(false);
      break;
  }

  /* Infinite loop */
  while (1) {
  }
}

void LCD_Init(DISPLAY_Device_t *displayDevice)
{
  /* Select LFRCO as clock source for LFA (PCNT) */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);

  /* Set PCNT to generate an interrupt every second. */
  pcntInit();

  DISPLAY_Init();
  if (DISPLAY_DeviceGet(0, displayDevice) != DISPLAY_EMSTATUS_OK) {
    EFM_ASSERT(false);
  }

  // Re-target stdio to the display.
  if (TEXTDISPLAY_EMSTATUS_OK != RETARGET_TextDisplayInit()) {
    // Text display initialization failed.
    EFM_ASSERT(false);
  }
}

Demo_Mode_TypeDef LCD_SelectDemo(void)
{
  Demo_Mode_TypeDef selectedDemoMode = demoModeOccupancy;
  LCD_UpdateMenu(selectedDemoMode);

  /* Allow demo selection until user presses PB0 to start the test. */
  while (!startTest) {
    if (shownDemoMode != selectedDemoMode) {
      selectedDemoMode = shownDemoMode;
      LCD_UpdateMenu(selectedDemoMode);
    }
  }
  return shownDemoMode;
}

/**
 * @brief
 *  Updates the LCD for demo selection cursor.
 *
 * @param[in] selectedDemo
 *  Current demo selected by the user.
 *
 */
void LCD_UpdateMenu(uint8_t selectedDemoLine)
{
  printf(CLEAR_SCREEN);
  printf(MENU_INDENT HEADER_STRING);
  printf(MENU_INDENT PIR_STRING "\r\n");
  printf(MENU_INDENT ALS_STRING "\r\n");
  printf(MENU_INDENT PIR_LOW_PWR_STRING "\r\n");
  printf(MENU_INDENT ALS_LOW_PWR_STRING "\r\n");
  printf("\r\n" MENU_INDENT DOC_STRING);

  /* Place menu cursor. */
  printf(TEXTDISPLAY_ESC_SEQ_CURSOR_HOME_VT100);
  for (uint8_t i = 0; i < selectedDemoLine + DEMO_LINE_OFFSET; i++) {
    printf("\r\n");
  }
  printf(CURSOR_STRING);
}

/**
 * @brief Turns off LCD and related peripherals to avoid affecting low current measurements.
 */
void LCD_TearDown(DISPLAY_Device_t *displayDevice)
{
  /* Get ready to start the demo. Turn off everything that is not needed. */
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);
  NVIC_DisableIRQ(GPIO_ODD_IRQn);
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeDisabled, 1);
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeDisabled, 1);

  CMU_ClockEnable(cmuClock_RTCC, false);

  PCNT_Enable(PCNT0, pcntModeDisable);
  CMU_ClockEnable(cmuClock_PCNT0, false);
  NVIC_DisableIRQ(RTCC_IRQn);
  NVIC_DisableIRQ(PCNT0_IRQn);
  displayDevice->pDisplayPowerOn(displayDevice, false);
}

/***************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons.
 ******************************************************************************/
static void gpioSetup(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(LDO_SHDN_B_PORT, LDO_SHDN_B_PIN, gpioModePushPull, 0);  /* Disable the LDO. */
  GPIO_PinModeSet(ADC_P_PORT, ADC_P_PIN, gpioModeDisabled, 0); // ADC_P
  GPIO_PinModeSet(ADC_N_PORT, ADC_N_PIN, gpioModeDisabled, 0); // ADC_N
  GPIO_PinModeSet(MOTION_B_PORT, MOTION_B_PIN, gpioModePushPull, 1);      /* Disable the EXP side LED. */

  GPIO_PinModeSet(SENSOR_SCL_PORT, SENSOR_SCL_PIN, gpioModeWiredAndPullUp, 1);
  GPIO_PinModeSet(SENSOR_SDA_PORT, SENSOR_SDA_PIN, gpioModeWiredAndPullUp, 1);
  GPIO_PinModeSet(SENSOR_INT_PORT, SENSOR_INT_PIN, gpioModeWiredAndPullUp, 1);

  // Demo selection buttons
  // Configure PB0 as input and enable interrupt.
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, true, true);

  // Configure PB1 as input and enable interrupt.
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/***************************************************************************//**
 * @brief Unified GPIO Interrupt handler (pushbuttons).
 *        PB0 Starts selected test.
 *        PB1 Cycles through the available tests.
 ******************************************************************************/
void GPIO_Unified_IRQ(void)
{
  // Get and clear all pending GPIO interrupts.
  uint32_t interruptMask = GPIO_IntGet();
  GPIO_IntClear(interruptMask);

  if (!startTest) {
    /* Demo menu selection interface. */
    if (interruptMask & (1 << BSP_GPIO_PB0_PIN)) {
      // BTN0: Start test.
      startTest = true;
    }

    if (interruptMask & (1 << BSP_GPIO_PB1_PIN)) {
      // BTN1: cycle through tests.
      shownDemoMode = (Demo_Mode_TypeDef) (((int32_t) shownDemoMode + 1) % (int32_t) NUM_DEMOS);
    }
  } else if (shownDemoMode == demoModeAmbientLight || shownDemoMode == demoModeAmbientLightLowPower) {
    if (interruptMask & (1 << SENSOR_INT_PIN)) {
      sensorInt = true;
    }
  }
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler for even pins.
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler for odd pins.
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

/***************************************************************************//**
 * @brief   Set up PCNT to generate an interrupt every second.
 *          There is already a timebase from the RTC since we have to toggle
 *          the display inversion pin regularly. We can use that same signal
 *          to keep a one-second timebase in the +LCD modes, so we can update
 *          the spinner.
 ******************************************************************************/
static void pcntInit(void)
{
  PCNT_Init_TypeDef pcntInit = PCNT_INIT_DEFAULT;

  // Enable PCNT clock.
  CMU_ClockEnable(cmuClock_PCNT0, true);
  // Set up the PCNT to count RTC_PULSE_FREQUENCY pulses -> one second.
  pcntInit.mode = pcntModeOvsSingle;
  pcntInit.top = RTC_PULSE_FREQUENCY;
  pcntInit.s1CntDir = false;
  // The PRS channel used depends on the configuration and which pin the
  // LCD inversion toggle is connected to. So use the generic define here.
  pcntInit.s0PRS = (PCNT_PRSSel_TypeDef) LCD_AUTO_TOGGLE_PRS_CH;

  PCNT_Init(PCNT0, &pcntInit);

  // Select PRS as the input for the PCNT.
  PCNT_PRSInputEnable(PCNT0, pcntPRSInputS0, true);

  // Enable PCNT interrupt every second.
  NVIC_EnableIRQ(PCNT0_IRQn);
  PCNT_IntEnable(PCNT0, PCNT_IF_OF);
}

/***************************************************************************//**
 * @brief   This interrupt is triggered at every second by the PCNT.
 ******************************************************************************/
void PCNT0_IRQHandler(void)
{
  PCNT_IntClear(PCNT0, PCNT_IF_OF);
}
