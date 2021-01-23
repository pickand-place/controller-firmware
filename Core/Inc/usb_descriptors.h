/*
 * usb_descriptors.h
 *
 *  Created on: Jan 1, 2021
 *      Author: carl
 */

#ifndef SRC_USB_DESCRIPTORS_H_
#define SRC_USB_DESCRIPTORS_H_

enum
{
  VENDOR_REQUEST_WEBUSB = 1,
  VENDOR_REQUEST_MICROSOFT = 2
};

extern uint8_t const desc_ms_os_20[];

#endif /* SRC_USB_DESCRIPTORS_H_ */
