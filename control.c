/*
 * Behringer BCD2000 driver
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

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <linux/string.h>

#include "microbookii.h"
#include "control.h"

static void microbookii_interrupt_in_urb_handler(struct urb *usb_urb) {
	struct microbookii *mbii = usb_urb->context;
	struct microbookii_message *msg = &mbii->control.msg;
	
	if (usb_urb->status < 0) {
		dev_err(&mbii->dev->dev, PREFIX "URB rcv Error: %i", usb_urb->status);
		return;
	}
	
	msg->len = usb_urb->actual_length;
	complete_all(&msg->responded);
}

static void microbookii_interrupt_out_urb_handler(struct urb *usb_urb) {
	int err = 0;
	struct usb_host_endpoint *ep;
	unsigned int rcv_pipe = 0;

	struct microbookii_message *msg = usb_urb->context;
	struct microbookii *mbii = msg->mbii;
	
	if (usb_urb->status < 0) {
		dev_err(&mbii->dev->dev, PREFIX "URB Error: %i", usb_urb->status);
		return;
	}
	
	msg->len = 0;
	memset(msg->buffer, 0, MSG_BUF_LEN);
	
	rcv_pipe = usb_rcvintpipe(mbii->dev, 0x82);
	ep = usb_pipe_endpoint(mbii->dev, rcv_pipe);
	usb_fill_int_urb(msg->urb, mbii->dev, rcv_pipe, msg->buffer, 
			MSG_BUF_LEN, microbookii_interrupt_in_urb_handler, 
			mbii, ep->desc.bInterval);
	err = usb_submit_urb(msg->urb, GFP_ATOMIC);
	if (err < 0) {
		dev_err(&mbii->dev->dev, PREFIX "Error: rcv: usb_submit_urb returned %i", err);
	}
}

int microbookii_control_communicate(struct microbookii *mbii, struct microbookii_message *msg) {
	int ret = 0;
	unsigned int snd_pipe = 0;
	struct usb_host_endpoint *ep;
	struct microbookii_control *control = &mbii->control;

	snd_pipe = usb_sndintpipe(mbii->dev, 0x01);
	ep = usb_pipe_endpoint(mbii->dev, snd_pipe);
	
	msg->mbii = mbii;
	msg->message_num = control->message_counter;
	((unsigned char *)msg->buffer)[0] = (unsigned char)control->message_counter;
	control->message_counter += 1;
	
	microbookii_dump_buffer(PREFIX "snd: ", msg->buffer, msg->len);
	
	usb_fill_int_urb(msg->urb, mbii->dev, snd_pipe, msg->buffer, 
			msg->len, microbookii_interrupt_out_urb_handler, 
			msg, ep->desc.bInterval);
			
	ret = usb_submit_urb(msg->urb, GFP_KERNEL);
	if (ret < 0) {
		dev_err(&mbii->dev->dev, PREFIX "Could not send message: usb_submit_urb returned %i", ret);
		return ret;
	}
	
	ret = (int)wait_for_completion_timeout(&msg->responded, msecs_to_jiffies(100));
	if (ret == 0) {
		// timed out
		return -EBUSY;
	}
	
	if (msg->len <= 0) {
		// no answer
		return -EINVAL;
	}
	
	microbookii_dump_buffer(PREFIX "rcv: ", msg->buffer, msg->len);
	
	if (((unsigned char *)msg->buffer)[0] != msg->message_num) {
		dev_err(&mbii->dev->dev, PREFIX "Received wrong message");
		return -EINVAL;
	}
	
	return 0;
}

int microbookii_poll_device_ready(struct microbookii *mbii) {
	int err = 0;
	unsigned char last_byte = 0;
	struct microbookii_message *msg;

	unsigned char msg_is_ready[] = {0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x18};
	int snd_len = sizeof(msg_is_ready);

	while (!completion_done(&mbii->control.device_setup)) {
		dev_err(&mbii->dev->dev, PREFIX "They see me polling...");
		msg = microbookii_message_get(mbii);
		memcpy(msg->buffer, msg_is_ready, snd_len);
		msg->len = snd_len;
		err = microbookii_control_communicate(mbii, msg);
		if (err < 0) {
			dev_err(&mbii->dev->dev, PREFIX "Error while polling...");
			microbookii_message_put(msg);
			return err;
		}
		last_byte = *((unsigned char *)msg->buffer + msg->len - 1);
		if (last_byte == 1) {
			complete_all(&mbii->control.device_setup);
			dev_err(&mbii->dev->dev, PREFIX "Device ready!");
			microbookii_message_put(msg);
			break;
		}
		microbookii_message_put(msg);
		
		msleep(100);
	}
	
	return 0;
}

int microbookii_set_rate(struct microbookii *mbii, unsigned int rate) {
	int err = 0;
	struct microbookii_message *msg;
	static unsigned int msg_len = 12;
	
	if (mbii->control.current_rate == rate) {
		return 0;
	}
	
	msg = microbookii_message_get(mbii);
	memset(msg->buffer, 0, msg_len);
	msg->len = msg_len;
	
	((unsigned char *)msg->buffer)[6] = 0x0b;
	((unsigned char *)msg->buffer)[7] = 0x14;
	
	switch(rate) {
		case 44100:
			((unsigned char *)msg->buffer)[11] = 0x02;
			break;
		case 48000:
			((unsigned char *)msg->buffer)[11] = 0x00;
			break;
		case 96000:
			((unsigned char *)msg->buffer)[6] = 0x01;
			break;
		default:
			/* unsupported rate */
			dev_err(&mbii->dev->dev, PREFIX "Error setting rate (unsupported rate)");
			microbookii_message_put(msg);
			return -EINVAL;
	}
	
	err = microbookii_control_communicate(mbii, msg);
	if (err < 0) {
		dev_err(&mbii->dev->dev, PREFIX "Error setting rate...");
		microbookii_message_put(msg);
		return err;
	}
	
	microbookii_message_put(msg);
	
	mbii->control.current_rate = rate;
	// Device needs to resetup
	reinit_completion(&mbii->control.device_setup);
	return 0;
}

struct microbookii_message *microbookii_message_get(struct microbookii *mbii) {
	mutex_lock(&mbii->control.message_lock);
	return &mbii->control.msg;
}

void microbookii_message_put(struct microbookii_message *msg) {
	msg->len = 0;
	memset(msg->buffer, 0, MSG_BUF_LEN);
	reinit_completion(&msg->responded);
	mutex_unlock(&msg->mbii->control.message_lock);
}

int microbookii_init_message(struct microbookii_message *msg) {
	msg->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!msg->urb) {
		return -ENOMEM;
	}
	msg->buffer = kzalloc(MSG_BUF_LEN, GFP_KERNEL);
	if (!msg->buffer) {
		usb_free_urb(msg->urb);
		return -ENOMEM;
	}
	msg->len = 0;
	init_completion(&msg->responded);
	msg->message_num = 0;
	return 0;
}

void microbookii_del_message(struct microbookii_message *msg) {
	if (!completion_done(&msg->responded)) {
		msg->len = 0;
		complete_all(&msg->responded);
	}
	usb_kill_urb(msg->urb);
	usb_free_urb(msg->urb);
	
	kfree(msg->buffer);
}

int microbookii_init_control(struct microbookii *mbii)
{
	int ret = 0;
	
	mutex_init(&mbii->control.message_lock);

	mbii->control.message_counter = 1;
	mbii->control.current_rate = 0;
	init_completion(&mbii->control.device_setup);
	
	ret = microbookii_init_message(&mbii->control.msg);
	if (ret < 0) {
		return ret;
	}
	
	return 0;
}

void microbookii_free_control(struct microbookii *mbii)
{
	if (!completion_done(&mbii->control.device_setup)) {
		complete_all(&mbii->control.device_setup);
	}
	
	microbookii_del_message(&mbii->control.msg);
}
