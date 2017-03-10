/*
 * MOTU MicroBook II driver
 *
 *   Copyright (C) 2014 Mario Kicherer (dev@kicherer.org)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 */

#include <linux/version.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/bitmap.h>
#include <linux/delay.h>

#include "microbookii.h"
#include "midi.h"
#include "audio.h"
#include "control.h"

static struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x07FD, 0x0004) },
	{ },
};

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;

static DEFINE_MUTEX(devices_mutex);
DECLARE_BITMAP(devices_used, SNDRV_CARDS);
static struct usb_driver microbookii_driver;

#ifdef CONFIG_SND_DEBUG
void microbookii_dump_buffer(const char *prefix, const char *buf, int len)
{
	print_hex_dump(KERN_DEBUG, prefix,
			DUMP_PREFIX_NONE, 16, 1,
			buf, len, false);
}
#else
void microbookii_dump_buffer(const char *prefix, const char *buf, int len) {
	print_hex_dump(KERN_ALERT, prefix,
			DUMP_PREFIX_NONE, 16, 1,
			buf, len, false);
}
#endif

static void microbookii_disconnect(struct usb_interface *interface)
{
	struct microbookii *mbii = usb_get_intfdata(interface);

	if (!mbii)
		return;

	mutex_lock(&devices_mutex);
	
	/* make sure that userspace cannot create new requests */
	snd_card_disconnect(mbii->card);

	microbookii_free_control(mbii);
	microbookii_free_audio(mbii);

	if (mbii->intf) {
		usb_set_intfdata(mbii->intf, NULL);
		mbii->intf = NULL;
	}

	clear_bit(mbii->card_index, devices_used);

	snd_card_free_when_closed(mbii->card);

	mutex_unlock(&devices_mutex);
}

static int microbookii_probe(struct usb_interface *interface,
				const struct usb_device_id *usb_id)
{
	struct snd_card *card;
	struct microbookii *mbii;
	unsigned int card_index;
	char usb_path[32];
	int err;
	

	mutex_lock(&devices_mutex);

	for (card_index = 0; card_index < SNDRV_CARDS; ++card_index)
		if (!test_bit(card_index, devices_used))
			break;

	if (card_index >= SNDRV_CARDS) {
		mutex_unlock(&devices_mutex);
		return -ENOENT;
	}

	#if LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0)
	err = snd_card_create(index[card_index], id[card_index], THIS_MODULE,
			sizeof(*mbii), &card);
	#else
	err = snd_card_new(&interface->dev, index[card_index], id[card_index],
			THIS_MODULE, sizeof(*mbii), &card);
	#endif
	if (err < 0) {
		mutex_unlock(&devices_mutex);
		return err;
	}

	mbii = card->private_data;
	mbii->dev = interface_to_usbdev(interface);
	mbii->card = card;
	mbii->card_index = card_index;
	mbii->intf = interface;

	snd_card_set_dev(card, &interface->dev);

	strncpy(card->driver, "snd-microbookii", sizeof(card->driver));
	strncpy(card->shortname, DEVICENAME, sizeof(card->shortname));
	usb_make_path(mbii->dev, usb_path, sizeof(usb_path));
	snprintf(mbii->card->longname, sizeof(mbii->card->longname),
			"MOTU " DEVICENAME " at %s",
			usb_path);
			
	/* set alternate configuration */
	err = usb_set_interface(mbii->dev, 0, 1);
	if (err < 0)
		goto probe_error;

	err = microbookii_init_audio(mbii);
	if (err < 0) {
		goto probe_error;
	}
	
	err = microbookii_init_control(mbii);
	if (err < 0) {
		microbookii_free_audio(mbii);
		goto probe_error;
	}

	err = snd_card_register(card);
	if (err < 0) {
		microbookii_free_audio(mbii);
		microbookii_free_control(mbii);
		goto probe_error;
	}
	
	usb_set_intfdata(interface, mbii);
	set_bit(card_index, devices_used);

	mutex_unlock(&devices_mutex);
	return 0;

probe_error:
	dev_info(&mbii->dev->dev, PREFIX "error during probing");

	microbookii_disconnect(interface);
	mutex_unlock(&devices_mutex);

	return err;
}

static struct usb_driver microbookii_driver = {
	.name =		"snd-microbookii",
	.probe =	microbookii_probe,
	.disconnect =	microbookii_disconnect,
	.id_table =	id_table,
};

module_usb_driver(microbookii_driver);

MODULE_DEVICE_TABLE(usb, id_table);
MODULE_AUTHOR("Manuel Reinhardt, manuel.jr16@gmail.com");
MODULE_DESCRIPTION("MOTU MicroBook II driver");
MODULE_LICENSE("GPL");
