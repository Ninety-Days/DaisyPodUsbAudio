/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @brief   USB Device descriptors for Daisy Seed UAC1 Bidirectional
  *
  * @attention
  * PROVENANCE: descriptor-callback layout and the IntToUnicode /
  * Get_SerialNum helpers follow ST's STM32CubeF4 AUDIO_Standalone
  * example; the file reached this project via libDaisy and a community
  * firmware archive. See THIRD_PARTY_NOTICES.md for the full chain.
  *
  * Originally derived from STMicroelectronics' STM32Cube USB Device Library
  * descriptor template (Copyright (c) STMicroelectronics), licensed under
  * the Ultimate Liberty license SLA0044 -- NOT the MIT license this
  * repository's own original files use. See LICENSE-SLA0044.txt (or
  * http://www.st.com/SLA0044) for the full text. Per SLA0044 section 4,
  * this code must be used and executed solely on a processing unit
  * manufactured by or for STMicroelectronics -- true here (STM32H750 on the
  * Daisy Seed). Per section 5, it may not be placed under Open Source Terms;
  * the MIT LICENSE file at this repository's root does NOT apply to this
  * file. THIRD_PARTY_NOTICES.md carries the file-by-file breakdown.
  *
  * Note on the PID: it was deliberately moved off an earlier value when
  * this descriptor set grew from input-only to three interfaces. Windows
  * caches the PID-to-driver mapping, so reusing the old PID makes it apply
  * stale input-only settings and the SPK OUT stream never opens. Changing
  * the PID forces re-enumeration and correct driver binding on all hosts.
  ******************************************************************************
  */

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

/* -------------------------------------------------------------------------- */
/* Device identification                                                       */
/* -------------------------------------------------------------------------- */
#define USBD_VID                    0x0483     /* ST's VID, acceptable for development builds */
#define USBD_LANGID_STRING          1033       /* English */
#define USBD_MANUFACTURER_STRING    "DIY"      /* not an official Electro-Smith product — runs on Daisy Pod hardware, custom community firmware */

/* Internal FS connector — the only USB device role this firmware builds.
   The OTG_HS-pin ("USB HS") descriptor set that used to live in this file
   was removed: main.cpp only ever calls USBD_Init(..., DEVICE_FS), so the
   whole HS callback layer was unreachable code that also misleadingly
   implied high-speed support. */
#define USBD_PID_FS                 0x5743           /* was 0x5741 (input-only) */
#define USBD_PRODUCT_STRING_FS      "DaisyPodUsbAudio"
#define USBD_CONFIGURATION_STRING_FS "Audio Config"
#define USBD_INTERFACE_STRING_FS    "Audio Streaming"


/* DEVICE_ID1/2/3 and USB_SIZ_STRING_SERIAL come from usbd_desc.h, included
   above. They used to be repeated here as well; identical redefinition is
   legal C and compiled silently, which is exactly why the duplication could
   sit unnoticed -- and why editing one copy would have had no effect. */

/* -------------------------------------------------------------------------- */
/* Descriptor callback prototypes                                              */
/* -------------------------------------------------------------------------- */
uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);



/* -------------------------------------------------------------------------- */
/* Descriptor tables                                                           */
/* -------------------------------------------------------------------------- */
USBD_DescriptorsTypeDef DaisyPodUsbAudio_FS_Desc = {
    USBD_FS_DeviceDescriptor,
    USBD_FS_LangIDStrDescriptor,
    USBD_FS_ManufacturerStrDescriptor,
    USBD_FS_ProductStrDescriptor,
    USBD_FS_SerialStrDescriptor,
    USBD_FS_ConfigStrDescriptor,
    USBD_FS_InterfaceStrDescriptor,
};


/* -------------------------------------------------------------------------- */
/* Working buffers                                                             */
/* -------------------------------------------------------------------------- */
#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
    0x12,                       /* bLength */
    USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
    0x10, 0x01,                 /* bcdUSB = 1.10 (FS-only device; declaring 2.00
                                   makes Windows request a Device Qualifier and
                                   Other-Speed Configuration this FS device has
                                   no meaningful answer for -- a known "device
                                   visible but Code 10" cause on usbaudio.sys.
                                   macOS ignores that path, so this is safe both
                                   ways.) */
    0x00,                       /* bDeviceClass (defined at interface level) */
    0x00,                       /* bDeviceSubClass */
    0x00,                       /* bDeviceProtocol */
    USB_MAX_EP0_SIZE,           /* bMaxPacketSize0 */
    LOBYTE(USBD_VID),           /* idVendor */
    HIBYTE(USBD_VID),
    LOBYTE(USBD_PID_FS),        /* idProduct */
    HIBYTE(USBD_PID_FS),
    0x00, 0x02,                 /* bcdDevice = 2.00 */
    USBD_IDX_MFC_STR,           /* iManufacturer */
    USBD_IDX_PRODUCT_STR,       /* iProduct */
    USBD_IDX_SERIAL_STR,        /* iSerialNumber */
    USBD_MAX_NUM_CONFIGURATION  /* bNumConfigurations */
};

__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING),
    HIBYTE(USBD_LANGID_STRING)
};

__ALIGN_BEGIN uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
    USB_SIZ_STRING_SERIAL,
    USB_DESC_TYPE_STRING,
};

__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

/* -------------------------------------------------------------------------- */
/* Helper                                                                      */
/* -------------------------------------------------------------------------- */
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
    for(uint8_t idx = 0; idx < len; idx++)
    {
        uint8_t nib = (value >> 28) & 0x0F;
        pbuf[2 * idx]     = (nib < 10) ? ('0' + nib) : ('A' + nib - 10);
        pbuf[2 * idx + 1] = 0;
        value <<= 4;
    }
}

static void Get_SerialNum(void)
{
    uint32_t deviceserial0 = *(uint32_t *)DEVICE_ID1;
    uint32_t deviceserial1 = *(uint32_t *)DEVICE_ID2;
    uint32_t deviceserial2 = *(uint32_t *)DEVICE_ID3;
    deviceserial0 += deviceserial2;
    if(deviceserial0 != 0)
    {
        IntToUnicode(deviceserial0, &USBD_StringSerial[2],  8);
        IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
    }
}

/* -------------------------------------------------------------------------- */
/* FS descriptor callbacks                                                     */
/* -------------------------------------------------------------------------- */
uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_FS_DeviceDesc);
    return USBD_FS_DeviceDesc;
}

uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_LangIDDesc);
    return USBD_LangIDDesc;
}

uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = USB_SIZ_STRING_SERIAL;
    Get_SerialNum();
    return USBD_StringSerial;
}

uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
    return USBD_StrDesc;
}

uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
    return USBD_StrDesc;
}

