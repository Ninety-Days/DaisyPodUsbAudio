/* usbd_irq.c — NVIC vector -> HAL_PCD_IRQHandler routing for OTG_FS.
 * Trivial glue calling STMicroelectronics' public HAL_PCD_IRQHandler API
 * (declared in their HAL, licensed BSD-3-Clause per STM32CubeH7's
 * LICENSE.md); this file itself is original and MIT-licensed (see LICENSE
 * at repository root). */
#include "stm32h7xx_hal.h"
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* One handler, not three. The dedicated EP1 IN/OUT interrupt lines only
   fire when the core is configured with use_dedicated_ep1, and usbd_conf.c
   sets that to DISABLE -- every endpoint event arrives on OTG_FS_IRQn.
   Handlers for the other two used to sit here and could never run. */
void OTG_FS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS); }
