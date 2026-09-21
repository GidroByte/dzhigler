// USB дескрипторы dzhigler: CDC (консоль) + мышь + клавиатура.
//
// Производное от examples/device/hid_composite из TinyUSB:
// Copyright (c) 2019 Ha Thach (tinyusb.org), MIT. См. LICENSE.

#include <string.h>
#include "tusb.h"
#include "usb_descriptors.h"

#ifndef CFG_TUD_HID_EP_BUFSIZE
#define CFG_TUD_HID_EP_BUFSIZE 64
#endif

//---------------- Device descriptor ----------------
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0xCafe,
    .idProduct          = 0x4001,
    .bcdDevice          = 0x0110,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *)&desc_device;
}

/*---------------- HID report descriptors ------------------- */
// Мышь: 5 кнопок + 3 бита паддинга = 1 байт, затем X, Y, Wheel.
uint8_t const desc_hid_report[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (1)
    0x29, 0x05,        //     Usage Maximum (5)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x05,        //     Report Count (5)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x03,        //     Report Size (3)
    0x81, 0x01,        //     Input (Const,Array,Abs) - паддинг
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0xC0,              //   End Collection
    0xC0               // End Collection
};

// Клавиатура: стандартный boot-протокол, 8 байт.
uint8_t const desc_hid_keyboard_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD()
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
  switch (instance) {
    case HID_INSTANCE_MOUSE:    return desc_hid_report;
    case HID_INSTANCE_KEYBOARD: return desc_hid_keyboard_report;
    default:                    return NULL;
  }
}

/*---------------- Configuration descriptor ----------------- */
#define EPNUM_CDC_NOTIF    0x81
#define EPNUM_CDC_DATA_OUT 0x02
#define EPNUM_CDC_DATA_IN  0x82
#define EPNUM_HID_MOUSE    0x83
#define EPNUM_HID_KEYBOARD 0x84

#define CONFIG_TOTAL_LEN   (TUD_CONFIG_DESC_LEN + \
                            TUD_CDC_DESC_LEN + \
                            (TUD_HID_DESC_LEN * 2))

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    /* CDC */
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4,
                       EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_DATA_OUT, EPNUM_CDC_DATA_IN, 64),

    /* HID: мышь */
    TUD_HID_DESCRIPTOR(ITF_NUM_HID_MOUSE, 0, HID_ITF_PROTOCOL_MOUSE,
                       sizeof(desc_hid_report), EPNUM_HID_MOUSE,
                       CFG_TUD_HID_EP_BUFSIZE, 5),

    /* HID: клавиатура */
    TUD_HID_DESCRIPTOR(ITF_NUM_HID_KEYBOARD, 0, HID_ITF_PROTOCOL_KEYBOARD,
                       sizeof(desc_hid_keyboard_report), EPNUM_HID_KEYBOARD,
                       CFG_TUD_HID_EP_BUFSIZE, 5)
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
  return desc_configuration;
}

/*---------------- String descriptors ------------------------ */
char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},   // 0: English (0x0409)
    "dzhigler",                   // 1: Manufacturer
    "Mouse+KB+CDC",               // 2: Product
    "0001",                       // 3: Serial
    "Debug Console",              // 4: CDC Interface
};

static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;
  uint8_t chr_count;

  if (index == 0) {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  } else {
    if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
    const char *str = string_desc_arr[index];
    chr_count = (uint8_t)strlen(str);
    size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1;
    if (chr_count > max_count) chr_count = (uint8_t)max_count;
    for (uint8_t i = 0; i < chr_count; i++) _desc_str[1 + i] = str[i];
  }

  _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}
