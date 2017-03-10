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

#include "microbookii.h"
#include "midi.h"

/*
 * For details regarding the usable MIDI commands, please see the official
 * manual: http://www.behringer.com/EN/Products/BCD2000.aspx#softwareContent
 */

/*
 * Some bytes of the init sequence are always the same, some only vary by a
 * small amount and some values look random.
 *
 * After sending the init sequence, the device also returns data via the
 * INTERRUPT IN endpoint which we simply ignore currently as its purpose is
 * unknown.
 */
static unsigned char microbookii_init_sequence[] = {
	0x07, 0x00, 0x00, 0x00, /* always the same */
	0x78, 0x48, 0x1c, 0x81, /* random, the last byte is either 0x81 or 0xff */
	0xc4, 0x00, 0x00, 0x00, /* always the same */

	0x5e,					/* Group A: always the same */
	0x53,					/*          observed 0x23, 0x43, 0x53, 0x83, 0xd3 */
	0x4a,					/*          looks random */
	0xf7,					/*          varies within a value of +- 4 */

	0x18, 0xfa, 0x11, 0xff, /* Group B (repeats two times) */
	0x6c, 0xf3, 0x90, 0xff, /* repeating values, last byte is always 0xff */
	0x00, 0x00, 0x00, 0x00, /* always the same */
	0x01, 0x00, 0x00, 0x00, /* always the same */

	0x18, 0xfa, 0x11, 0xff, /* Group B again */
	0x14, 0x00, 0x00, 0x00, /* always the same */
	0x00, 0x00, 0x00, 0x00, /* always the same */
	0xf2, 0x34, 0x4a, 0xf7, /* similar to group A */

	0x18, 0xfa, 0x11, 0xff  /* Group B again */
};

static unsigned char device_cmd_prefix[] = MIDI_CMD_PREFIX_INIT;

static int microbookii_midi_input_open(struct snd_rawmidi_substream *substream)
{
	return 0;
}

static int microbookii_midi_input_close(struct snd_rawmidi_substream *substream)
{
	return 0;
}

/* (de)register midi substream from client */
static void microbookii_midi_input_trigger(struct snd_rawmidi_substream *substream,
						int up)
{
	struct microbookii *mbii = substream->rmidi->private_data;
	mbii->midi.receive_substream = up ? substream : NULL;
}

static void microbookii_midi_handle_input(struct microbookii *mbii,
				const unsigned char *buf, unsigned int buf_len)
{
	unsigned int payload_length, tocopy;
	struct snd_rawmidi_substream *receive_substream;

	receive_substream = ACCESS_ONCE(mbii->midi.receive_substream);
	if (!receive_substream)
		return;

	microbookii_dump_buffer(PREFIX "received from device: ", buf, buf_len);

	if (buf_len < 2)
		return;

	payload_length = buf[0];

	/* ignore packets without payload */
	if (payload_length == 0)
		return;

	tocopy = min(payload_length, buf_len-1);

	microbookii_dump_buffer(PREFIX "sending to userspace: ",
					&buf[1], tocopy);

	snd_rawmidi_receive(receive_substream,
					&buf[1], tocopy);
}

static void microbookii_midi_send(struct microbookii *mbii)
{
	int len, ret;
	struct snd_rawmidi_substream *send_substream;

	BUILD_BUG_ON(sizeof(device_cmd_prefix) >= MIDI_URB_BUFSIZE);

	send_substream = ACCESS_ONCE(mbii->midi.send_substream);
	if (!send_substream)
		return;

	/* copy command prefix bytes */
	memcpy(mbii->midi.out_buffer, device_cmd_prefix,
			sizeof(device_cmd_prefix));

	/*
	 * get MIDI packet and leave space for command prefix
	 * and payload length
	 */
	len = snd_rawmidi_transmit(send_substream,
							   mbii->midi.out_buffer + 3, MIDI_URB_BUFSIZE - 3);

	if (len < 0)
		dev_err(&mbii->dev->dev, "%s: snd_rawmidi_transmit error %d\n",
				__func__, len);

	if (len <= 0)
		return;

	/* set payload length */
	mbii->midi.out_buffer[2] = len;
	mbii->midi.out_urb->transfer_buffer_length = MIDI_URB_BUFSIZE;

	microbookii_dump_buffer(PREFIX "sending to device: ",
						mbii->midi.out_buffer, len+3);

	/* send packet to the BCD2000 */
	ret = usb_submit_urb(mbii->midi.out_urb, GFP_ATOMIC);
	if (ret < 0)
		dev_err(&mbii->dev->dev, PREFIX
			"%s (%p): usb_submit_urb() failed, ret=%d, len=%d\n",
			__func__, send_substream, ret, len);
	else
		mbii->midi.out_active = 1;
}

static int microbookii_midi_output_open(struct snd_rawmidi_substream *substream)
{
	return 0;
}

static int microbookii_midi_output_close(struct snd_rawmidi_substream *substream)
{
	struct microbookii *mbii = substream->rmidi->private_data;

	if (mbii->midi.out_active) {
		usb_kill_urb(mbii->midi.out_urb);
		mbii->midi.out_active = 0;
	}

	return 0;
}

/* (de)register midi substream from client */
static void microbookii_midi_output_trigger(struct snd_rawmidi_substream *substream,
						int up)
{
	struct microbookii *mbii = substream->rmidi->private_data;

	if (up) {
		mbii->midi.send_substream = substream;
		/* check if there is data userspace wants to send */
		if (!mbii->midi.out_active)
			microbookii_midi_send(mbii);
	} else {
		mbii->midi.send_substream = NULL;
	}
}

static void microbookii_output_complete(struct urb *urb)
{
	struct microbookii *mbii = urb->context;

	mbii->midi.out_active = 0;

	if (urb->status)
		dev_warn(&urb->dev->dev,
			PREFIX "output urb->status: %d\n", urb->status);

	if (urb->status == -ESHUTDOWN)
		return;

	/* check if there is more data userspace wants to send */
	microbookii_midi_send(mbii);
}

static void microbookii_input_complete(struct urb *urb)
{
	int ret;
	struct microbookii *mbii = urb->context;

	if (urb->status)
		dev_warn(&urb->dev->dev,
			PREFIX "input urb->status: %i\n", urb->status);

	if (!mbii || urb->status == -ESHUTDOWN)
		return;

	if (urb->actual_length > 0)
		microbookii_midi_handle_input(mbii, urb->transfer_buffer,
					urb->actual_length);

	/* return URB to device */
	ret = usb_submit_urb(mbii->midi.in_urb, GFP_ATOMIC);
	if (ret < 0)
		dev_err(&mbii->dev->dev, PREFIX
			"%s: usb_submit_urb() failed, ret=%d\n",
			__func__, ret);
}

static struct snd_rawmidi_ops microbookii_midi_output = {
	.open =    microbookii_midi_output_open,
	.close =   microbookii_midi_output_close,
	.trigger = microbookii_midi_output_trigger,
};

static struct snd_rawmidi_ops microbookii_midi_input = {
	.open =    microbookii_midi_input_open,
	.close =   microbookii_midi_input_close,
	.trigger = microbookii_midi_input_trigger,
};

int microbookii_init_midi(struct microbookii *mbii)
{
	int ret;
	struct snd_rawmidi *rmidi;
	struct microbookii_midi *midi;

	ret = snd_rawmidi_new(mbii->card, mbii->card->shortname, 0,
					1, /* output */
					1, /* input */
					&rmidi);

	if (ret < 0)
		return ret;

	strlcpy(rmidi->name, mbii->card->shortname, sizeof(rmidi->name));

	rmidi->info_flags = SNDRV_RAWMIDI_INFO_DUPLEX;
	rmidi->private_data = mbii;

	rmidi->info_flags |= SNDRV_RAWMIDI_INFO_OUTPUT;
	snd_rawmidi_set_ops(rmidi, SNDRV_RAWMIDI_STREAM_OUTPUT,
					&microbookii_midi_output);

	rmidi->info_flags |= SNDRV_RAWMIDI_INFO_INPUT;
	snd_rawmidi_set_ops(rmidi, SNDRV_RAWMIDI_STREAM_INPUT,
					&microbookii_midi_input);

	midi = &mbii->midi;
	midi->rmidi = rmidi;

	midi->in_urb = usb_alloc_urb(0, GFP_KERNEL);
	midi->out_urb = usb_alloc_urb(0, GFP_KERNEL);

	if (!midi->in_urb || !midi->out_urb) {
		dev_err(&mbii->dev->dev, PREFIX "usb_alloc_urb failed\n");
		return -ENOMEM;
	}

	usb_fill_int_urb(midi->in_urb, mbii->dev,
				usb_rcvintpipe(mbii->dev, 0x81),
				midi->in_buffer, MIDI_URB_BUFSIZE,
				microbookii_input_complete, mbii, 1);

	usb_fill_int_urb(midi->out_urb, mbii->dev,
				usb_sndintpipe(mbii->dev, 0x1),
				midi->out_buffer, MIDI_URB_BUFSIZE,
				microbookii_output_complete, mbii, 1);

	init_usb_anchor(&midi->anchor);
	usb_anchor_urb(midi->out_urb, &midi->anchor);
	usb_anchor_urb(midi->in_urb, &midi->anchor);

	/* copy init sequence into buffer */
	memcpy(midi->out_buffer, microbookii_init_sequence, 52);
	midi->out_urb->transfer_buffer_length = 52;

	/* submit sequence */
	ret = usb_submit_urb(midi->out_urb, GFP_KERNEL);
	if (ret < 0)
		dev_err(&mbii->dev->dev, PREFIX
			"%s: usb_submit_urb() out failed, ret=%d: ",
			__func__, ret);
	else
		midi->out_active = 1;

	/* pass URB to device to enable button and controller events */
	ret = usb_submit_urb(midi->in_urb, GFP_KERNEL);
	if (ret < 0)
		dev_err(&mbii->dev->dev, PREFIX
			"%s: usb_submit_urb() in failed, ret=%d: ",
			__func__, ret);

	/* ensure initialization is finished */
	usb_wait_anchor_empty_timeout(&midi->anchor, 1000);

	return 0;
}

void microbookii_free_midi(struct microbookii *mbii)
{
	/* usb_kill_urb not necessary, urb is aborted automatically */
	usb_free_urb(mbii->midi.out_urb);
	usb_free_urb(mbii->midi.in_urb);
}
