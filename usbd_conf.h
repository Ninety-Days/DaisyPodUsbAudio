/**
  ******************************************************************************
  * @file    usbd_conf.h
  * @brief   USB Device configuration for Daisy Seed UAC1 Bidirectional Audio
  *
  * @attention
  * Copyright (c) 2019 STMicroelectronics. All rights reserved.
  *
  * PROVENANCE: the immediate upstream for this file is libDaisy
  * (src/usbd/usbd_conf.h, Electro-Smith, MIT, Copyright (c) 2019
  * Electrosmith), which redistributes the STMicroelectronics original
  * below. Both attributions apply; where they conflict, SLA0044 governs.
  *
  * This header configures STMicroelectronics' STM32Cube USB Device Library
  * (Copyright (c) STMicroelectronics), licensed under the Ultimate Liberty
  * license SLA0044 -- NOT the MIT license this repository's own original
  * files use. Per SLA0044 section 5, ST USB Device Library code may not be
  * placed under Open Source Terms; the MIT LICENSE file at this
  * repository's root does NOT apply to this file. See LICENSE-SLA0044.txt
  * and THIRD_PARTY_NOTICES.md.
  *
  * WHY THIS FILE EXISTS LOCALLY
  *
  * For a while it did not, and #include "usbd_conf.h" silently resolved to
  * libDaisy's own copy, which is tuned for CDC serial logging rather than
  * for this device: USBD_MAX_NUM_INTERFACES is 1U there against the 4U
  * this device needs, and USBD_DEBUG_LEVEL is 3U against 0U here.
  *
  * A project-local copy fixes that for everything this project compiles
  * and includes it -- but only for those files. Two cases it does not
  * reach:
  *
  *   - usbd_core.c is already compiled inside libdaisy.a, against
  *     libDaisy's header. A local header cannot recompile a prebuilt
  *     library.
  *   - usbd_ctlreq.c is compiled here, but the Makefile passes
  *     -I../../libDaisy/src/usbd, so its own #include "usbd_conf.h" still
  *     resolves to libDaisy's copy rather than this one. That shadowing is
  *     why the wIndex bound check in that file keeps a hardcoded 4U --
  *     see docs/PITFALLS.md before touching it.
  *
  * Neither case is currently load-bearing: this project's own Setup()
  * handles SET_INTERFACE/GET_INTERFACE for all four interfaces directly
  * (manual wIndex dispatch on a single pClassData pointer, not ST's newer
  * per-interface pClassDataCmsit[] array), so the generic core-level path
  * these constants would guard is not exercised for this device's
  * requests. The device enumerated and ran correctly against libDaisy's
  * copy before this file was restored; keeping it local is hygiene for
  * our own compiled code, not a fix for an observed failure.
  ******************************************************************************
  */
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

/* -------------------------------------------------------------------------- */
/* USB Device configuration                                                    */
/* -------------------------------------------------------------------------- */

/* This device declares 4 interfaces: AudioControl(0) + MIC AudioStreaming(1)
   + SPK AudioStreaming(2) + MIDIStreaming(3) -- see USB_AUDIO_CONFIG_DESC_SIZ's
   bNumInterfaces in usbd_audio.c. Was 2U (input-only, pre-duplex) in the
   original version of this file, then 3U during a diagnostic build with
   MIDI temporarily removed; libDaisy's own copy that silently stood in for
   it at one point has 1U (its own CDC use case) -- none of those match the
   current descriptor. usbd_audio.h #undef's and re-defines this same macro
   to stay in sync if it's ever bumped again there instead of here -- keep
   both in agreement. */
/* WARNING -- this value does NOT reach usbd_ctlreq.c.
 *
 * The Makefile passes -I../../libDaisy/src/usbd, and libDaisy ships its own
 * usbd_conf.h there with USBD_MAX_NUM_INTERFACES = 1U. Since usbd_ctlreq.c
 * includes "usbd_core.h" (which lives in libDaisy's Middlewares tree), the
 * nested #include "usbd_conf.h" resolves against the -I list -- and hits
 * libDaisy's copy, not this file. This directory is not in -I at all.
 *
 * Consequence: USBD_StdItfReq's interface-number bound would silently
 * become 1U, so every interface request above 1 gets STALLed and this
 * composite device (AudioControl + AudioStreaming IN + AudioStreaming OUT
 * + MIDIStreaming) fails to enumerate at all -- no error, no partial
 * function, just an absent device.
 *
 * That is why the wIndex bound check in usbd_ctlreq.c keeps a hardcoded
 * 4U instead of this
 * macro. It is the one intentional deviation from ST-verbatim in that
 * file. Do not "restore" the macro -- it has been tried and it breaks
 * enumeration outright. */
#define USBD_MAX_NUM_INTERFACES     4U

#define USBD_MAX_NUM_CONFIGURATION  1U
#define USBD_MAX_STR_DESC_SIZ       512U
/* Has no effect: usbd_ctlreq.c is compiled against libDaisy's copy of this
   header, not this one, and that copy sets the macro to 1U. Kept at 0U so
   this file states what the project intends. What actually keeps the
   unknown-string path safe is the USBD_AUDIO_GetUsrStrDesc stub in
   usbd_audio.c -- see PITFALLS section 2. */
#define USBD_SUPPORT_USER_STRING    0U

/* Set to 0 — we have no USB serial port when running as audio device,
   so printf goes nowhere and USBD_DEBUG_LEVEL > 0 will loop on printf
   with undefined behaviour. Doesn't matter for this project's own code
   (nothing here calls USBD_UsrLog/ErrLog/DbgLog), but matters if this
   header is ever the one actually compiled against ST's own core/ctlreq
   sources rather than libDaisy's prebuilt library. */
#define USBD_DEBUG_LEVEL            0U

#define USBD_LPM_ENABLED            0U
#define USBD_SELF_POWERED           1U


/* Device ID constant — must match the DEVICE_FS check in usbd_conf.c.
 * DEVICE_HS is gone: main.cpp only ever calls
 * USBD_Init(..., DEVICE_FS), so the OTG_HS-pin device role was
 * unreachable code and has been removed along with its descriptor set. */
#define DEVICE_FS   0

/* -------------------------------------------------------------------------- */
/* Memory management macros                                                    */
/* -------------------------------------------------------------------------- */
/* Real heap, but called exactly once: USBD_AUDIO_Init allocates the class
   handle at USBD_RegisterClass time and nothing frees or reallocates it
   while the device runs. No fragmentation is possible from this. Alignment
   is not guaranteed, which is why the ring and endpoint buffers in
   usbd_audio.c are file-scope __ALIGN_BEGIN arrays rather than handle
   members. */
#define USBD_malloc         malloc
#define USBD_free           free
#define USBD_memset         memset
#define USBD_memcpy         memcpy
#define USBD_Delay          HAL_Delay

/* -------------------------------------------------------------------------- */
/* Debug macros — all disabled (USBD_DEBUG_LEVEL == 0)                        */
/* -------------------------------------------------------------------------- */
#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF__H__ */
