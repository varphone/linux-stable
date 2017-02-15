/*******************************************************
 *
 *  PenMount HID TouchScreen Driver
 *
 *  Copyright (c) 2014 PenMount Touch Solutions <penmount@seed.net.tw>
 *
 *******************************************************/

/*******************************************************
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *******************************************************/

////////////////////////////////////////////////////////
#include <linux/module.h>
#include <linux/version.h>
#include <linux/hid.h>
#include <linux/input.h>
////////////////////////////////////////////////////////
#ifndef USB_VENDOR_ID_PENMOUNT
#define USB_VENDOR_ID_PENMOUNT          0x14E1
#endif

#ifndef USB_DEVICE_ID_PENMOUNT_5000
#define USB_DEVICE_ID_PENMOUNT_5000     0x5000
#endif

#ifndef USB_DEVICE_ID_PENMOUNT_6000
#define USB_DEVICE_ID_PENMOUNT_6000     0x6000
#endif
////////////////////////////////////////////////////////
static
int penmount_input_mapping(struct hid_device *pHidDevice,
		struct hid_input *pHidInput, struct hid_field *pHidField,
		struct hid_usage *pHidUsage, unsigned long **bit, int *max) {
	struct input_dev *pInputDev = pHidInput->input;

	if (!pInputDev)
		return 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
	set_bit(INPUT_PROP_DIRECT, pInputDev->propbit);
#endif

	switch (pHidUsage->hid & HID_USAGE_PAGE) {
	case HID_UP_GENDESK:
		switch (pHidUsage->hid) {
		case HID_GD_X:
			hid_map_usage(pHidInput, pHidUsage, bit, max, EV_ABS, ABS_X);
			return 1;
		case HID_GD_Y:
			hid_map_usage(pHidInput, pHidUsage, bit, max, EV_ABS, ABS_Y);
			return 1;
		}
		break;
	case HID_UP_BUTTON:
		hid_map_usage(pHidInput, pHidUsage, bit, max, EV_KEY, BTN_TOUCH);
		return 1;
		break;
	}

	return 0;
}
//------------------------------------------------------
static
int penmount_probe(struct hid_device *pHidDevice,
		const struct hid_device_id *pHidDevID) {
	int ret = 0;

	ret = hid_parse(pHidDevice);
	if (ret)
		return ret;

	ret = hid_hw_start(pHidDevice, HID_CONNECT_DEFAULT);
	if (ret)
		return ret;

	return ret;
}
//------------------------------------------------------
static
void penmount_remove(struct hid_device *pHidDevice) {
	if (!pHidDevice)
		return;

	hid_hw_stop(pHidDevice);

	return;
}
////////////////////////////////////////////////////////
static const struct hid_device_id PENMOUNT_DEVICEID[] = {
{
	HID_USB_DEVICE(USB_VENDOR_ID_PENMOUNT, USB_DEVICE_ID_PENMOUNT_5000) }, {
	HID_USB_DEVICE(USB_VENDOR_ID_PENMOUNT, USB_DEVICE_ID_PENMOUNT_6000) },
	{ }
};
//------------------------------------------------------
static const struct hid_usage_id PENMOUNT_USAGETABLE[] = {
	{ HID_ANY_ID, HID_ANY_ID, HID_ANY_ID },
	{ HID_ANY_ID - 1, HID_ANY_ID - 1, HID_ANY_ID - 1 }
};
////////////////////////////////////////////////////////
static
struct hid_driver penmount_DRIVER =
{
	.name = "hid-penmount" ,
	.id_table = PENMOUNT_DEVICEID ,
	.probe = penmount_probe ,
	.remove = penmount_remove ,
	.input_mapping = penmount_input_mapping ,
	.usage_table = PENMOUNT_USAGETABLE ,
};
//------------------------------------------------------
static
int __init penmount_init ( void )
{
	return hid_register_driver ( &penmount_DRIVER );
}
//------------------------------------------------------
static
void __exit penmount_exit ( void )
{
	hid_unregister_driver ( &penmount_DRIVER );
	return;
}
//------------------------------------------------------
module_init (penmount_init);
module_exit (penmount_exit);
////////////////////////////////////////////////////////
MODULE_AUTHOR("PenMount Touch Solutions <penmount@seed.net.tw>");
MODULE_DESCRIPTION("PenMount HID TouchScreen Driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(hid, PENMOUNT_DEVICEID);
////////////////////////////////////////////////////////

