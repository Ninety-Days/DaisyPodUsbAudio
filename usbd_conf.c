/**
  ******************************************************************************
  * @file    usbd_conf.c
  * @brief   USB Device board support / PCD glue layer for Daisy Seed UAC1
  *
  * @attention
  * Copyright (c) 2019 STMicroelectronics. All rights reserved.
  *
  * PROVENANCE: this file reached the project as ST -> libDaisy -> a
  * community firmware archive -> here. The immediate upstream is
  * libDaisy's src/usbd/usbd_conf.c (Electro-Smith, MIT, Copyright (c)
  * 2019 Electrosmith), not ST directly. Both attributions apply; where
  * they conflict, SLA0044 governs. See THIRD_PARTY_NOTICES.md.
  *
  * Originally derived from STMicroelectronics' STM32Cube USB Device Library
  * board-support template (Copyright (c) STMicroelectronics), which is
  * licensed under the Ultimate Liberty license SLA0044 -- NOT the MIT
  * license this repository's own original files use. See LICENSE-SLA0044.txt
  * (or http://www.st.com/SLA0044) for the full text. Per SLA0044 section 4,
  * this code (modified or not) must be used and executed solely on or in
  * combination with a processing unit manufactured by or for
  * STMicroelectronics -- true here, as it targets the STM32H750 on the
  * Daisy Seed. Per section 5, it may not be placed under Open Source Terms
  * (which SLA0044 defines as including MIT/BSD/Apache-style licenses); the
  * MIT LICENSE file at the root of this repository therefore does NOT apply
  * to this file. That scoping is deliberate and required -- extending MIT
  * over ST's files would breach SLA0044 section 5. See README.md's
  * "Licensing" section and THIRD_PARTY_NOTICES.md for the full file-by-file
  * breakdown.
  *
  * Changes from libDaisy CDC baseline:
  *  1. Sof_enable = ENABLE      — kept on, but NOT load-bearing; see the
  *                                  note at the Init.Sof_enable line below
  *  2. TX FIFO 1 resized        — audio IN, 0xB0 words (704 B); was sized
  *                                for 196-byte (16-bit) packets, now covers
  *                                294-byte (24-bit) packets with reduced
  *                                but still safe headroom -- see the FIFO
  *                                table in HAL_PCD_MspInit for the exact
  *                                current math (was 0x80, then 0xC0)
  *  3. TX FIFO 2 added          — MIDI Bulk IN EP 0x82: 0x10 words (64 B)
  *                                fits exactly one FS bulk max packet
  *  4. vbus_sensing_enable      — left ENABLE (Daisy has VBUS sense on PA9)
  *  5. HS init kept as stub     — Daisy Seed FS connector only
  *
  * FIFO budget (see the table and derivation in HAL_PCD_MspInit below):
  *   Rx FIFO  0x140 = 320 words  shared OUT FIFO (ISO OUT + Bulk OUT + EP0)
  *   TX0      0x40 =  64 words   EP0 control
  *   TX1      0xB0 = 176 words   EP1 IN audio ISO (was 0xC0; freed 16 words)
  *   TX2      0x10 =  16 words   EP2 IN MIDI Bulk (new)
  *   Total  = 576 words
  *
  * There is NO 320-word ceiling on this part. That figure is the STM32F4 FS
  * core's budget; USB_OTG_FS on the H750 is USB2_OTG_FS at 0x40080000, the
  * second full OTG block, speed-limited rather than RAM-limited. The core
  * reports its own synthesised FIFO depth in GHWCFG3[31:16]; read there
  * with a debugger, it is comfortably above this allocation. Do not
  * re-derive the old 320 figure from this comment's history.
  ******************************************************************************
  */

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_compat.h"   /* USBD_LL_XFER_SIZE_T, see header */
#include "usbd_audio.h"  /* USBD_AUDIO_HandleTypeDef, AUDIO_IN_EP -- the resume fix in HAL_PCD_ResumeCallback needs both */

PCD_HandleTypeDef hpcd_USB_OTG_FS;
/* No hpcd_USB_OTG_HS: main.cpp only ever calls USBD_Init(..., DEVICE_FS),
 * so the OTG_HS-pin device role was unreachable code. Removed along with
 * its descriptor set in usbd_desc.c. */

void Error_Handler(void) { while(1) {} }

USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);

/* -------------------------------------------------------------------------- */
/* MSP Init / DeInit                                                           */
/* -------------------------------------------------------------------------- */

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcdHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(pcdHandle->Instance == USB_OTG_FS)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA11 = DM, PA12 = DP, PA9 = VBUS */
        GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_11;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* VBUS sense (input, no pull) */
        GPIO_InitStruct.Pin  = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

        /* Priority 0, equal to SAI DMA (DMA1_Stream0/1, priority 0 by
         * default -- libDaisy's per/sai.cpp never calls
         * HAL_NVIC_SetPriority for it, so it sits at the CMSIS reset-state
         * priority, which is 0 = highest). Previously this was priority 2,
         * i.e. LOWER priority (more preemptable) than SAI DMA, despite the
         * comment above claiming the opposite intent ("so audio IRQs
         * aren't delayed") -- the code and the comment disagreed, and the
         * code is what actually ran.
         *
         * That ordering meant SAI's DMA-complete interrupt (firing every
         * ~1ms from AudioCallback) could preempt a USB ISOC transaction
         * ISR mid-execution -- including while Audio_PlaybackData /
         * Audio_PeriodicTC were partway through updating pb_ring/rec_ring
         * state (pb_write increment, memcpy into the ring, etc.). USB
         * isochronous transfers are hard-bound to the host's 1ms SOF
         * frame; if the ISR that services them gets held up by a
         * preempting SAI interrupt, that's a direct path to the kind of
         * irregular, variable-length artifact we measured (the tone2.aifc
         * repeat-block analysis: a clean 2ms period, i.e. every OTHER 1ms
         * service window losing its slot).
         *
         * Matching SAI's priority exactly (0), rather than trying to go
         * "higher" than 0 (impossible -- 0 is the max on this NVIC config)
         * or lowering SAI's priority (which would mean patching libDaisy
         * itself, and risks AudioCallback missing its own 1ms DMA budget),
         * means NEITHER interrupt can preempt the other. At equal preempt
         * priority the NVIC does not nest them; whichever fires first
         * runs to completion, and the other waits its turn. That removes
         * the specific "SAI cuts USB off mid-update" failure mode without
         * risking a symmetric "USB now starves SAI" one, since both
         * handlers are short (bounded ring-buffer operations, not
         * unbounded work). Sub-priority 0 for all three USB IRQs on this
         * line keeps their relative ordering stable among themselves. */
        /* Only OTG_FS_IRQn. The dedicated EP1 IN/OUT lines were enabled here
           as well, which was harmless but meaningless: Init.use_dedicated_ep1
           is DISABLE below, so the core never asserts them and usbd_irq.c had
           two handlers that could not run. */
        HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcdHandle)
{
    if(pcdHandle->Instance == USB_OTG_FS)
    {
        __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_9);

        HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    }
}

/* -------------------------------------------------------------------------- */
/* PCD → USB Device Library callbacks                                         */
/* -------------------------------------------------------------------------- */

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum,
                          hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum,
                         hpcd->IN_ep[epnum].xfer_buff);
}

/* SOF is essential for the audio class — it drives the periodic transfer
   callback that feeds the isochronous IN endpoint with new audio data. */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_SpeedTypeDef speed = USBD_SPEED_FULL;
    if(hpcd->Init.speed == PCD_SPEED_HIGH)       speed = USBD_SPEED_HIGH;
    else if(hpcd->Init.speed == PCD_SPEED_FULL)  speed = USBD_SPEED_FULL;
    else                                          Error_Handler();
    USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, speed);
    USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
    __HAL_PCD_GATE_PHYCLOCK(hpcd);
    if(hpcd->Init.low_power_enable)
    {
        SCB->SCR |= (uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
    }
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);

    /* Re-arm the IN pipe after a suspend/resume cycle.
     *
     * BUG THIS FIXES: HAL_PCD_SuspendCallback gates the PHY clock while
     * an isochronous IN transmit may be in flight. That transmit's
     * completion is lost -- it never fires the DataIn callback that
     * would normally arm the next transmit. USBD_LL_Resume only restores
     * clock/PHY state; it does not know this class needs its transmit
     * chain restarted.
     *
     * Symptom without the fix: after ANY USB suspend (which macOS can
     * trigger during normal operation), the DataIn->Transmit->DataIn
     * chain stops permanently. alt_setting_in still reads 1,
     * SET_INTERFACE still succeeds, the audio callback keeps running --
     * every host-visible signal looks healthy except that audio silently
     * stops, and only a full replug recovered it.
     *
     * The extra transmit is idempotent: if the pipe was not actually
     * stuck, the host discards it as a duplicate/early frame. */
    USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef *)hpcd->pData;
    if(pdev != NULL) {
        USBD_AUDIO_HandleTypeDef *haudio =
            (USBD_AUDIO_HandleTypeDef *)pdev->pClassData;
        if(haudio != NULL && haudio->in_streaming) {
            /* last_tx_size declared in usbd_audio.h; not static in
             * usbd_audio.c precisely so this can read it. */
            USBD_LL_Transmit(pdev, AUDIO_IN_EP,
                             &haudio->in_buf[haudio->in_wr_ptr], last_tx_size);
        }
    }
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
}

/* -------------------------------------------------------------------------- */
/* USB Device Library → PCD (LL driver interface)                             */
/* -------------------------------------------------------------------------- */

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    if(pdev->id == DEVICE_FS)
    {
        hpcd_USB_OTG_FS.pData = pdev;
        pdev->pData           = &hpcd_USB_OTG_FS;

        hpcd_USB_OTG_FS.Instance                     = USB_OTG_FS;
        hpcd_USB_OTG_FS.Init.dev_endpoints           = 9;
        hpcd_USB_OTG_FS.Init.speed                   = PCD_SPEED_FULL;
        hpcd_USB_OTG_FS.Init.dma_enable              = DISABLE;
        hpcd_USB_OTG_FS.Init.phy_itface              = PCD_PHY_EMBEDDED;

        /* SOF is enabled, but the comment that used to sit here was wrong,
           and is corrected rather than acted on.

           It claimed USBD_AUDIO_SOF() drives the periodic audio feed and
           that streaming will not start without SOF. Neither is true:
           USBD_AUDIO_SOF() is an empty stub, and the IN stream is driven
           entirely by USBD_AUDIO_DataIn, which re-arms itself from each
           transfer completion. Isochronous frame parity is set by
           HAL_PCD_EP_Transmit from the DSTS frame counter, not from this
           interrupt. Disabling SOF would therefore change nothing
           functionally.

           It is left ENABLED anyway, deliberately:
             - The cost is noise. One interrupt per 1 ms frame costs a few
               hundred cycles against a 400,000-cycle budget at
               400 MHz, i.e. around 0.1%. Not a lever on CPU load, and
               not worth revalidating a configuration that is field-proven.
             - An explicit-feedback endpoint, if one is ever added, needs
               SOF. Turning it off now would only have to be undone then.

           Do not "optimise" this away on the strength of the old comment's
           premise. */
        hpcd_USB_OTG_FS.Init.Sof_enable              = ENABLE;

        hpcd_USB_OTG_FS.Init.low_power_enable        = DISABLE;
        hpcd_USB_OTG_FS.Init.lpm_enable              = DISABLE;
        hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
        hpcd_USB_OTG_FS.Init.vbus_sensing_enable     = ENABLE;
        hpcd_USB_OTG_FS.Init.use_dedicated_ep1       = DISABLE;

        if(HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) { Error_Handler(); }

        /* FIFO allocation (32-bit words):
           ┌──────────┬──────────┬──────────┬──────────────────────────────────┐
           │ Region   │ Words    │ Bytes    │ Rationale                        │
           ├──────────┼──────────┼──────────┼──────────────────────────────────┤
           │ Rx FIFO  │0x140=320 │ 1280 B   │ Shared OUT (ISO OUT + Bulk OUT   │
           │          │          │          │ + EP0). ST's rule asks 170w for  │
           │          │          │          │ a 294-byte ISO OUT packet; this  │
           │          │          │          │ is headroom over that floor, not │
           │          │          │          │ a derived figure. Costs nothing  │
           │          │          │          │ -- see the note below before     │
           │          │          │          │ trimming it ✓                    │
           │ TX0 FIFO │ 0x40= 64 │ 256 B    │ EP0 control transfers            │
           │ TX1 FIFO │ 0xB0=176 │ 704 B    │ EP1 IN audio (ISO IN 0x81).     │
           │          │          │          │ 704 B / 294 B ≈ 2.4-packet      │
           │          │          │          │ headroom (was ≈3.6 at 196 B --   │
           │          │          │          │ same word count, smaller margin  │
           │          │          │          │ now that packets grew; still     │
           │          │          │          │ well above the single-packet     │
           │          │          │          │ floor of ⌈294/4⌉+1=75w) ✓        │
           │          │          │          │ Reduced 16w from 0xC0 for TX2    │
           │ TX2 FIFO │ 0x10= 16 │  64 B    │ EP2 IN MIDI Bulk (0x82).        │
           │          │          │          │ FS bulk max = 64 B = 16w ✓       │
           ├──────────┼──────────┼──────────┼──────────────────────────────────┤
           │ Total    │ 576w     │ 2304 B   │ TX2 funded by −16w from TX1.     │
           │          │          │          │ Well inside the core's           │
           │          │          │          │ synthesised depth. There is no   │
           │          │          │          │ 320-word ceiling on this part.   │
           └──────────┴──────────┴──────────┴──────────────────────────────────┘
           OUT endpoints (ISO OUT 0x01, Bulk OUT 0x02) use Rx FIFO only.
           No TX FIFO is needed for OUT direction endpoints.

           None of the word allocations below changed for the 24-bit move --
           checked against the new 294-byte packet size above and all four
           regions remain comfortably above their minimums, just with less
           spare margin than at 196 bytes. If a future change grows the
           packet size again (e.g. a sample-rate bump), re-check this table
           before assuming these same word counts still hold.
        */
        /* RxFIFO raised 0x80 -> 0xC0 -> 0x140.
         *
         * ST's sizing rule for an isochronous OUT endpoint is
         *   13 + 2*((largest packet)/4 + 1) + 2*(number of OUT EPs) + 1
         * -- the doubled packet term is what lets the core accept the
         * next ISO packet while the previous one is still draining to
         * the AHB.
         *
         * OUT ENDPOINT COUNT: 3 for this device -- EP0 control (EP0
         * RECEIVES, so it counts), ISO OUT 0x01, Bulk OUT 0x02. So the
         * endpoint term is 2*3 = 6.
         *
         * At 294 bytes (49 frames x 2ch x 3B):
         *   13 + 2*(74+1) + 2*3 + 1 = 170 words.
         *
         * NOT 176. An earlier version of this comment wrote 2*6 = 12
         * there, which is the literal from ST's own worked example --
         * and that example sizes SIX OUT endpoints (five OUT plus the
         * control endpoint). Copied without recounting for this device
         * it double-counts. A figure of 174 also circulated at one point
         * and is likewise wrong. All three variants sit under 192, so the
         * chosen value is unaffected; the derivation is what was wrong.
         *
         * 0x140 is headroom over that floor, not a figure the arithmetic
         * produced. It costs nothing: 0x140 + 0x40 + 0xB0 + 0x10 = 576 of
         * the 1024 words this core has, and no endpoint here wants the
         * rest. It is also the value every passing measurement was taken
         * on, which is the main reason not to trim it back.
         *
         * The history, since the number would otherwise look arbitrary.
         * Three vendor control requests issued back to back under playback
         * intermittently killed the control pipe -- measured at ~1.25% per
         * poll, spread evenly through a window rather than accumulating.
         * It was never traced to the transport, and not for want of
         * looking: watchdog kicks, packet-size deviations and under/over-
         * runs stayed at zero through every failure, instrumentation on
         * USBD_CtlError showed this device never stalled EP0, and counters
         * on both incomplete-isochronous callbacks never moved. Whatever
         * broke the pipe left no trace in anything the core reports.
         *
         * Raising the Rx FIFO was tried against it and never isolated:
         * single windows pass about two times in three by luck at that
         * rate, so the early results that seemed to show an effect had no
         * power behind them. What did resolve it was spacing the three
         * reads apart in read_transport_diagnostics.py -- 0 failures in
         * 600 polls against 7.5 expected. Normal playback never produced
         * the failure at all; only that diagnostic burst did.
         *
         * So: do not read this value as a fix for anything, and do not
         * lower it expecting a measurement to back the smaller number.
         *
         * One correction for anyone reading this against familiar
         * numbers: the 320-word budget is the STM32F4 FS core's and does
         * not apply to this part. */
        HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x140);
        HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
        HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0xB0);
        HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 0x10);
    }

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    return USBD_Get_USB_Status(HAL_PCD_DeInit(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    return USBD_Get_USB_Status(HAL_PCD_Start(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    return USBD_Get_USB_Status(HAL_PCD_Stop(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev,
                                   uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type));
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_Close(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_Flush(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_SetStall(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_ClrStall(pdev->pData, ep_addr));
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
    if((ep_addr & 0x80) == 0x80)
        return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
    else
        return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    return USBD_Get_USB_Status(HAL_PCD_SetAddress(pdev->pData, dev_addr));
}

/* The 4th parameter widened from uint16_t to uint32_t in ST's composite
 * rework. USBD_LL_XFER_SIZE_T (usbd_compat.h) picks the right one; these
 * two assertions check that pick against the prototype ST actually
 * declared, so a wrong guess produces this message instead of a
 * "conflicting types" wall. HAL_PCD_EP_Transmit/Receive take uint32_t in
 * both eras, so widening the parameter never truncates. */
_Static_assert(_Generic(&USBD_LL_Transmit,
    USBD_StatusTypeDef (*)(USBD_HandleTypeDef *, uint8_t, uint8_t *, USBD_LL_XFER_SIZE_T) : 1,
    default : 0),
    "USBD_LL_XFER_SIZE_T does not match this ST middleware's USBD_LL_Transmit prototype. "
    "Rebuild with C_DEFS+=-DUSBD_LL_XFER_SIZE_T=uint16_t (or =uint32_t).");

_Static_assert(_Generic(&USBD_LL_PrepareReceive,
    USBD_StatusTypeDef (*)(USBD_HandleTypeDef *, uint8_t, uint8_t *, USBD_LL_XFER_SIZE_T) : 1,
    default : 0),
    "USBD_LL_XFER_SIZE_T does not match this ST middleware's USBD_LL_PrepareReceive prototype. "
    "Rebuild with C_DEFS+=-DUSBD_LL_XFER_SIZE_T=uint16_t (or =uint32_t).");

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev,
                                     uint8_t ep_addr, uint8_t *pbuf,
                                     USBD_LL_XFER_SIZE_T size)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size));
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev,
                                           uint8_t ep_addr, uint8_t *pbuf,
                                           USBD_LL_XFER_SIZE_T size)
{
    return USBD_Get_USB_Status(HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size));
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay)
{
    HAL_Delay(Delay);
}

USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
    switch(hal_status)
    {
        case HAL_OK:      return USBD_OK;
        case HAL_ERROR:   return USBD_FAIL;
        case HAL_BUSY:    return USBD_BUSY;
        case HAL_TIMEOUT: return USBD_FAIL;
        default:          return USBD_FAIL;
    }
}
