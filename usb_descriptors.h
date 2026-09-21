#ifndef USB_DESCRIPTORS_H_
#define USB_DESCRIPTORS_H_

#include "tusb.h"

#define HID_INSTANCE_MOUSE    0
#define HID_INSTANCE_KEYBOARD 1

typedef struct TU_ATTR_PACKED {
  uint8_t buttons;
  int8_t  x;
  int8_t  y;
  int8_t  wheel;
} dzh_mouse_report_t;

typedef struct TU_ATTR_PACKED {
  uint8_t modifier;
  uint8_t reserved;
  uint8_t keycode[6];
} dzh_kbd_report_t;

enum {
  ITF_NUM_CDC          = 0,  /* коммуникационный интерфейс CDC */
  ITF_NUM_CDC_DATA     = 1,  /* данные CDC */
  ITF_NUM_HID_MOUSE    = 2,  /* HID мышь */
  ITF_NUM_HID_KEYBOARD = 3,  /* HID клавиатура */
  ITF_NUM_TOTAL        = 4
};

#endif /* USB_DESCRIPTORS_H_ */
