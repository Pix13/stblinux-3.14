/*
 * Copyright (C) 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef _USB_BRCM_COMMON_INIT_H
#define _USB_BRCM_COMMON_INIT_H

void brcm_usb_common_ctrl_init(uintptr_t ctrl, int ioc, int ipp, int xhci);
void brcm_usb_common_ctrl_xhci_soft_reset(uintptr_t ctrl, int on_off);

#endif /* _USB_BRCM_COMMON_INIT_H */
