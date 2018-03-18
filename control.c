/*
 * MOTU Microbook II driver
 *
 *   Copyright (C) 2014 Mario Kicherer (dev@kicherer.org)
 *   Copyright (C) 2017 Manuel Reinhardt (manuel.jr16@gmail.com)
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
#include <sound/tlv.h>
#include <linux/string.h>

#include "microbookii.h"
#include "control.h"

#define MASTER_VOL 0x00000c1e
#define PHONES_VOL 0x00000c1f

#define FLAG_MASTER_VOL_CHANGED 0x04
#define FLAG_PHONES_VOL_CHANGED 0x08

static SNDRV_CTL_TLVD_DECLARE_DB_LINEAR(db_scale_master, -9000, 0);

static void microbookii_interrupt_in_urb_handler(struct urb *usb_urb)
{
	struct microbookii_message *msg = usb_urb->context;
	struct microbookii *mbii = msg->mbii;

	if (usb_urb->status < 0) {
		dev_err(&mbii->dev->dev, PREFIX "URB rcv Error: %i",
			usb_urb->status);
		return;
	}

	msg->len = usb_urb->actual_length;
	complete_all(&msg->responded);
}

static void microbookii_interrupt_out_urb_handler(struct urb *usb_urb)
{
	int err = 0;
	struct usb_host_endpoint *ep;
	unsigned int rcv_pipe = 0;

	struct microbookii_message *msg = usb_urb->context;
	struct microbookii *mbii = msg->mbii;

	if (usb_urb->status < 0) {
		dev_err(&mbii->dev->dev, PREFIX "URB Error: %i",
			usb_urb->status);
		return;
	}

	msg->len = 0;
	memset(msg->buffer, 0, MSG_BUF_LEN);

	rcv_pipe = usb_rcvintpipe(mbii->dev, 0x82);
	ep = usb_pipe_endpoint(mbii->dev, rcv_pipe);
	usb_fill_int_urb(msg->urb, mbii->dev, rcv_pipe, msg->buffer,
			 MSG_BUF_LEN, microbookii_interrupt_in_urb_handler, msg,
			 ep->desc.bInterval);
	err = usb_submit_urb(msg->urb, GFP_ATOMIC);
	if (err < 0) {
		dev_err(&mbii->dev->dev,
			PREFIX "Error: rcv: usb_submit_urb returned %i", err);
	}
}

int microbookii_control_communicate(struct microbookii_message *msg)
{
	int ret = 0;
	unsigned int snd_pipe = 0;
	struct usb_host_endpoint *ep;
	struct microbookii *mbii = msg->mbii;
	struct microbookii_control *control = &mbii->control;

	if (mbii == NULL || control == NULL) {
		printk(KERN_ALERT PREFIX
		       "Internal Error (microbookii_control_communicate: mbii is NULL).");
		return -EINVAL;
	}

	snd_pipe = usb_sndintpipe(mbii->dev, 0x01);
	ep = usb_pipe_endpoint(mbii->dev, snd_pipe);

	msg->message_num = control->message_counter;
	msg->buffer[0] = (unsigned char)control->message_counter;
	control->message_counter += 1;

	microbookii_dump_buffer(PREFIX "snd: ", msg->buffer, msg->len);

	usb_fill_int_urb(msg->urb, mbii->dev, snd_pipe, msg->buffer, msg->len,
			 microbookii_interrupt_out_urb_handler, msg,
			 ep->desc.bInterval);

	ret = usb_submit_urb(msg->urb, GFP_KERNEL);
	if (ret < 0) {
		dev_err(&mbii->dev->dev,
			PREFIX
			"Could not send message: usb_submit_urb returned %i",
			ret);
		return ret;
	}

	ret = (int)wait_for_completion_timeout(&msg->responded,
					       msecs_to_jiffies(100));
	if (ret == 0) {
		// timed out
		return -EBUSY;
	}

	if (msg->len <= 0) {
		// no answer
		return -EINVAL;
	}

	microbookii_dump_buffer(PREFIX "rcv: ", msg->buffer, msg->len);

	if (msg->buffer[0] != msg->message_num) {
		dev_err(&mbii->dev->dev, PREFIX
			"Error communicating with device (message number mismatch).");
		return -EINVAL;
	}

	return 0;
}

static void __microbookii_msg_communicate(struct work_struct *work)
{
	int err;
	struct microbookii_message *msg =
		container_of(work, struct microbookii_message, work);

	err = microbookii_control_communicate(msg);
	if (err < 0) {
		dev_err(&msg->mbii->dev->dev,
			PREFIX "Communication Error Code: %i", err);
	}

	if (msg->callback != NULL) {
		msg->callback(msg);
	}
}

int microbookii_message_send(struct microbookii_message *msg)
{
	int ret;
	ret = microbookii_message_send_async(msg, NULL);
	if (ret < 0) {
		return ret;
	}
	flush_workqueue(msg->mbii->control.message_queue);
	return 0;
}

int microbookii_message_send_async(struct microbookii_message *msg,
				   message_callback_t callback)
{
	msg->callback = callback;
	queue_work(msg->mbii->control.message_queue, &msg->work);
	return 0;
}

void microbookii_poll_callback(struct microbookii_message *msg)
{
	unsigned char last_byte;
	unsigned char change_flags;
	struct microbookii *mbii = msg->mbii;

	last_byte = msg->buffer[msg->len - 1];
	if (last_byte == 1
	    && !completion_done(&msg->mbii->control.device_setup)) {
		complete_all(&msg->mbii->control.device_setup);
		dev_info(&msg->mbii->dev->dev,
			 PREFIX "Device ready for playback/capture.");
	}
	change_flags = msg->buffer[msg->len - 2];
	if ((change_flags & FLAG_MASTER_VOL_CHANGED)
	    == FLAG_MASTER_VOL_CHANGED) {
		snd_ctl_notify(mbii->card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &mbii->control.master_volume->id);
	}
	if ((change_flags & FLAG_PHONES_VOL_CHANGED)
	    == FLAG_PHONES_VOL_CHANGED) {
		snd_ctl_notify(mbii->card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &mbii->control.phones_volume->id);
	}

	microbookii_message_free(msg);
}

void microbookii_poll(struct microbookii *mbii)
{
	int err = 0;
	struct microbookii_message *msg;

	unsigned char msg_poll[] = {0x00, 0x04, 0x00, 0x00,
				    0x00, 0x00, 0x0B, 0x18};
	int snd_len = sizeof(msg_poll);

	msg = microbookii_message_alloc(mbii);
	if (msg == NULL) {
		dev_err(&mbii->dev->dev,
			PREFIX "Error (could not allocate message)");
		return;
	}
	memcpy(msg->buffer, msg_poll, snd_len);
	msg->len = snd_len;
	err = microbookii_message_send_async(msg, microbookii_poll_callback);
	if (err < 0) {
		dev_err(&mbii->dev->dev,
			PREFIX "Error communicating with device.");
		microbookii_message_free(msg);
	}
}

enum hrtimer_restart poll_timer_callback(struct hrtimer *timer)
{
	ktime_t ktime;
	struct microbookii_control *control =
		container_of(timer, struct microbookii_control, poll_timer);
	microbookii_poll(control->mbii);

	ktime = ktime_set(0, MICROBOOKII_POll_MS * NSEC_PER_MSEC);

	hrtimer_forward_now(timer, ktime);
	return HRTIMER_RESTART;
}

int microbookii_wait_device_ready(struct microbookii *mbii)
{
	return wait_for_completion_timeout(&mbii->control.device_setup,
					   msecs_to_jiffies(1000));
}

int microbookii_set_rate(struct microbookii *mbii, unsigned int rate)
{
	int err = 0;
	struct microbookii_message *msg;
	static unsigned int msg_len = 12;

	if (mbii->control.current_rate == rate) {
		return 0;
	}

	msg = microbookii_message_alloc(mbii);
	memset(msg->buffer, 0, msg_len);
	msg->len = msg_len;

	msg->buffer[6] = 0x0b;
	msg->buffer[7] = 0x14;

	switch (rate) {
	case 44100:
		msg->buffer[11] = 0x02;
		break;
	case 48000:
		msg->buffer[11] = 0x00;
		break;
	case 96000:
		msg->buffer[11] = 0x01;
		break;
	default:
		/* unsupported rate */
		dev_err(&mbii->dev->dev,
			PREFIX "Error setting rate (unsupported rate)");
		microbookii_message_free(msg);
		return -EINVAL;
	}

	err = microbookii_message_send(msg);
	if (err < 0) {
		dev_err(&mbii->dev->dev, PREFIX "Error setting rate...");
		microbookii_message_free(msg);
		return err;
	}

	microbookii_message_free(msg);

	mbii->control.current_rate = rate;
	// Device needs to resetup
	reinit_completion(&mbii->control.device_setup);
	return 0;
}

static int microbookii_init_message(struct microbookii_message *msg)
{
	msg->urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!msg->urb) {
		return -ENOMEM;
	}
	msg->len = 0;
	init_completion(&msg->responded);
	msg->message_num = 0;

	INIT_WORK(&msg->work, __microbookii_msg_communicate);

	msg->callback = NULL;

	return 0;
}

static void microbookii_del_message(struct microbookii_message *msg)
{
	if (!completion_done(&msg->responded)) {
		msg->len = 0;
		complete_all(&msg->responded);
	}
	usb_kill_urb(msg->urb);
	usb_free_urb(msg->urb);
}

struct microbookii_message *microbookii_message_alloc(struct microbookii *mbii)
{
	struct microbookii_message *msg;

	msg = kzalloc(sizeof(struct microbookii_message), GFP_ATOMIC);
	if (!msg) {
		return NULL;
	}

	if (microbookii_init_message(msg) != 0) {
		kzfree(msg);
		return NULL;
	}

	msg->mbii = mbii;

	return msg;
}

void microbookii_message_free(struct microbookii_message *msg)
{
	microbookii_del_message(msg);
	kzfree(msg);
}

static int microbookii_control_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0x20000000;
	return 0;
}

static int microbookii_control_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct microbookii *mbii = snd_kcontrol_chip(kcontrol);
	int err = 0;
	struct microbookii_message *msg;

	static unsigned int msg_len = 8;

	msg = microbookii_message_alloc(mbii);
	memset(msg->buffer, 0, msg_len);
	msg->len = msg_len;

	msg->buffer[1] = 0x04;
	*((int32_t *)&msg->buffer[4]) = kcontrol->private_value;

	err = microbookii_message_send(msg);
	if (err < 0) {
		dev_err(&mbii->dev->dev, PREFIX "Error getting control value.");
		microbookii_message_free(msg);
		return err;
	}

	WARN_ON(msg->len != 12);
	if (msg->len >= 12) {
		ucontrol->value.integer.value[0] =
			be32_to_cpup((int32_t *)&msg->buffer[8]);
	}

	microbookii_message_free(msg);

	return 0;
}

static int microbookii_control_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct microbookii *mbii = snd_kcontrol_chip(kcontrol);
	int err = 0;
	struct microbookii_message *msg;

	static unsigned int msg_len = 12;

	msg = microbookii_message_alloc(mbii);
	memset(msg->buffer, 0, msg_len);
	msg->len = msg_len;

	*((int32_t *)&msg->buffer[4]) = kcontrol->private_value;
	*((int32_t *)&msg->buffer[8]) =
		cpu_to_be32(ucontrol->value.integer.value[0]);

	err = microbookii_message_send(msg);
	if (err < 0) {
		dev_err(&mbii->dev->dev, PREFIX "Error getting control value.");
		microbookii_message_free(msg);
		return err;
	}

	WARN_ON(msg->len != 8);

	microbookii_message_free(msg);

	return 1;
}


int __microbookii_add_control(struct microbookii *mbii, const char *name,
			      const int identifier)
{
	struct snd_kcontrol *kcontrol;
	int ret = 0;

	struct snd_kcontrol_new microbookii_kcontrol = {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = name,
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE
			  | SNDRV_CTL_ELEM_ACCESS_TLV_READ,
		.private_value = cpu_to_be32(identifier),
		.info = microbookii_control_info,
		.get = microbookii_control_get,
		.put = microbookii_control_put,
		.tlv.p = db_scale_master};

	kcontrol = snd_ctl_new1(&microbookii_kcontrol, mbii);
	if (!kcontrol) {
		return -ENOMEM;
	}

	ret = snd_ctl_add(mbii->card, kcontrol);
	if (ret < 0) {
		return ret;
	}

	if (identifier == MASTER_VOL) {
		mbii->control.master_volume = kcontrol;
	}
	if (identifier == PHONES_VOL) {
		mbii->control.phones_volume = kcontrol;
	}

	return 0;
}

#define microbookii_add_control(mbii, name, ident)                             \
	{                                                                      \
		int err = 0;                                                   \
		err = __microbookii_add_control(mbii, name, ident);            \
		if (err < 0) {                                                 \
			return err;                                            \
		}                                                              \
	}

int microbookii_add_controls(struct microbookii *mbii)
{
	microbookii_add_control(mbii, "Master Playback Volume", MASTER_VOL);
	microbookii_add_control(mbii, "Phone Playback Volume", PHONES_VOL);

	return 0;
}

int microbookii_init_control(struct microbookii *mbii)
{
	int ret = 0;
	ktime_t ktime;

	mbii->control.mbii = mbii;

	mbii->control.message_counter = 1;
	mbii->control.current_rate = 0;
	init_completion(&mbii->control.device_setup);


	ret = microbookii_add_controls(mbii);
	if (ret < 0) {
		return ret;
	}

	mbii->control.message_queue =
		alloc_ordered_workqueue("microbookii_comm", 0);
	if (mbii->control.message_queue == NULL) {
		return -EPERM;
	}

	// Setup a timer to fire every 0.1 seconds to poll the device.
	ktime = ktime_set(0, MICROBOOKII_POll_MS * NSEC_PER_MSEC);
	hrtimer_init(&mbii->control.poll_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	mbii->control.poll_timer.function = poll_timer_callback;
	hrtimer_start(&mbii->control.poll_timer, ktime, HRTIMER_MODE_REL);

	return 0;
}

void microbookii_free_control(struct microbookii *mbii)
{
	if (!completion_done(&mbii->control.device_setup)) {
		complete_all(&mbii->control.device_setup);
	}

	hrtimer_cancel(&mbii->control.poll_timer);

	destroy_workqueue(mbii->control.message_queue);
}
