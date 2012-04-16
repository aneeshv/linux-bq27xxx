/**
 * otg-eh-test.c - OTG & Embedded Host Test Support Driver
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author: Felipe Balbi <balbi@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/usb.h>

#include <linux/usb/ch11.h>
#include <linux/usb/ch9.h>

#define USB_IF_TEST_VID			0x1a0a

#define USB_IF_TEST_SE0_NAK			0x0101
#define USB_IF_TEST_J				0x0102
#define USB_IF_TEST_K				0x0103
#define USB_IF_TEST_PACKET			0x0104
#define USB_IF_HS_HOST_PORT_SUSPEND_RESUME	0x0106
#define USB_IF_SINGLE_STEP_GET_DEV_DESC		0x0107
#define USB_IF_SINGLE_STEP_GET_DEV_DESC_DATA	0x0108

#define USB_IF_PROTOCOL_OTG_ELECTRICAL_TEST	0x0200

static int usb_set_hub_port_test(struct usb_device *udev, int test)
{
	struct usb_device		*hub = udev->parent;
	int				port = udev->portnum;

	return usb_control_msg(hub, usb_sndctrlpipe(hub, 0),
			USB_REQ_SET_FEATURE, USB_RT_PORT, USB_PORT_FEAT_TEST,
			(test << 8) | port, NULL, 0, 1000);
}

static int otg_eh_probe(struct usb_interface *intf,
		const struct usb_device_id *id)
{
	struct usb_device_descriptor	desc;
	struct usb_device		*udev;
	int				ret;

	udev = interface_to_usbdev(intf);

	usb_enable_autosuspend(udev);
	usb_autopm_get_interface(intf);

	switch (id->idProduct) {
	case USB_IF_TEST_SE0_NAK:
		/*
		 * Upon enumerating VID 0x1A0A/PID 0x0101, the host’s
		 * downstream port shall enter a high-speed receive mode as
		 * described in Section 7.1.20 [USB2.0] and drives an SE0 until
		 * the controller is reset.
		 */
		usb_set_hub_port_test(udev, TEST_SE0_NAK);
		break;
	case USB_IF_TEST_J:
		/*
		 * Upon enumerating VID 0x1A0A/PID 0x0102, the host’s
		 * downstream port shall enter a high-speed J state as
		 * described in Section 7.1.20 of [USB2.0] until the host
		 * controller is reset.
		 */
		usb_set_hub_port_test(udev, TEST_J);
		break;
	case USB_IF_TEST_K:
		/*
		 * Upon enumerating VID 0x1A0A/PID 0x0103, the host’s
		 * downstream port shall enter a high-speed K state as
		 * described in Section 7.1.20 of [USB2.0] until the host
		 * controller is reset.
		 */
		usb_set_hub_port_test(udev, TEST_K);
		break;
	case USB_IF_TEST_PACKET:
		/*
		 * Upon enumerating VID 0x1A0A/PID 0x0104, the host shall begin
		 * sending test packets as described in Section 7.1.20 of
		 * [USB2.0] until the host controller is reset.
		 */
		usb_set_hub_port_test(udev, TEST_PACKET);
		break;
	case USB_IF_HS_HOST_PORT_SUSPEND_RESUME:
		/*
		 * Upon enumerating VID:0x1A0A/PID 0x0106, the host shall
		 * continue sending SOFs for 15 seconds, then suspend the
		 * downstream port under test per Section 7.1.7.6.1 of
		 * [USB2.0]. After 15 seconds has elapsed, the host shall issue
		 * a ResumeK state on the bus, then continue sending SOFs.
		 */
		msleep(15000);
		usb_autopm_put_interface(intf);
		msleep(15000);
		usb_autopm_get_interface(intf);
		break;
	case USB_IF_SINGLE_STEP_GET_DEV_DESC:
		/*
		 * When the host discovers a device with VID:0x1A0A/PID 0x0107,
		 * the following steps are executed by the host and the device.
		 *
		 * 1. The host enumerates the test device, reads VID:0x1A0A/PID
		 * 0x0107, then completes its enumeration procedure.
		 *
		 * 2. The host issues SOFs for 15 seconds allowing the test
		 * engineer to raise the scope trigger just above the SOF
		 * voltage level.
		 *
		 * 3. The host sends a complete GetDescriptor(Device) transfer
		 *
		 * 4. The device ACKs the request, triggering the scope. (Note:
		 * SOFs continue.)
		 */
		msleep(15000);
		ret = usb_get_descriptor(udev, USB_DT_DEVICE, 0, &desc,
				USB_DT_DEVICE_SIZE);
		if (ret < 0)
			dev_err(&intf->dev, "failed to get device descriptor\n");

		break;
	case USB_IF_SINGLE_STEP_GET_DEV_DESC_DATA:
		/*
		 * When the host discovers a device with VID:0x1A0A/PID 0x0108,
		 * the following steps are executed by the host and the device.
		 *
		 * 1. The host enumerates the test device and reads
		 * VID:0x1A0A/PID 0x0108, then completes its enumeration
		 * procedure
		 *
		 * 2. After enumerating the device, the host sends
		 * GetDescriptor(Device)
		 *
		 * 3. The device ACKs the request
		 *
		 * 4. The host issues SOFs for 15 seconds allowing the test
		 * engineer to raise the scope trigger just above the SOF
		 * voltage level
		 *
		 * 5. The host sends an IN packet
		 *
		 * 6. The device sends data in response to the IN packet,
		 * triggering the scope
		 *
		 * 7. The host sends an ACK in response to the data. (Note:
		 * SOFs may follow the IN transaction).
		 */
		ret = usb_get_descriptor(udev, USB_DT_DEVICE, 0, &desc,
				USB_DT_DEVICE_SIZE);
		if (ret < 0)
			dev_err(&intf->dev, "failed to get device descriptor\n");
		msleep(15000);
		break;
	case USB_IF_PROTOCOL_OTG_ELECTRICAL_TEST:
		if (udev->bus->otg_port == udev->portnum) {
			/* We're OTG-A */
			ret = usb_driver_set_configuration(udev, 1);
			if (ret < 0)
				dev_err(&intf->dev, "unable to set configuration #1\n");
		} else if (udev->bus->is_b_host) {
			/* We're OTG-B acting as Host */
			ret = usb_driver_set_configuration(udev, -1);
			if (ret < 0)
				dev_err(&intf->dev, "unable to unconfigure device\n");

			/* forcefully suspend */
			usb_autopm_put_interface(intf);
		}

		break;
	default:
		dev_err(&intf->dev, "Unsupported device\n");
	}

	usb_disable_autosuspend(udev);

	return 0;
}

static void otg_eh_disconnect(struct usb_interface *intf)
{
	struct usb_device		*udev;

	udev = interface_to_usbdev(intf);

	usb_reset_device(udev);
}

static const struct usb_device_id otg_eh_id_table[] __devinitconst = {
	{ USB_DEVICE(USB_IF_TEST_VID, USB_IF_TEST_SE0_NAK), },
	{ USB_DEVICE(USB_IF_TEST_VID, USB_IF_TEST_J), },
	{ USB_DEVICE(USB_IF_TEST_VID, USB_IF_TEST_K), },
	{ USB_DEVICE(USB_IF_TEST_VID, USB_IF_TEST_PACKET), },
	{ USB_DEVICE(USB_IF_TEST_VID, USB_IF_HS_HOST_PORT_SUSPEND_RESUME), },
	{ USB_DEVICE(USB_IF_TEST_VID, USB_IF_SINGLE_STEP_GET_DEV_DESC), },
	{ USB_DEVICE(USB_IF_TEST_VID, USB_IF_SINGLE_STEP_GET_DEV_DESC_DATA), },
	{ USB_DEVICE(USB_IF_TEST_VID, USB_IF_PROTOCOL_OTG_ELECTRICAL_TEST), },

	{ } /* Terminating Entry */
};
MODULE_DEVICE_TABLE(usb, otg_eh_id_table);

static struct usb_driver otg_eh_driver = {
	.name		= "otg-eh-test",
	.probe		= otg_eh_probe,
	.disconnect	= otg_eh_disconnect,
	.id_table	= otg_eh_id_table,
	.supports_autosuspend = true,
};

static int __init usb_otg_ehtest_init(void)
{
	return usb_register(&otg_eh_driver);
}
module_init(usb_otg_ehtest_init);

static void __exit usb_otg_ehtest_exit(void)
{
	usb_deregister(&otg_eh_driver);
}
module_exit(usb_otg_ehtest_exit);

MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("USB OTG & EH Test Driver");
