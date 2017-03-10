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
#include <linux/usb.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "audio.h"
#include "control.h"
#include "microbookii.h"

static struct snd_pcm_hardware microbookii_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_BATCH |
			SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats	= SNDRV_PCM_FMTBIT_S24_3BE,
	/*
	.rates		= SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000,
	.rate_min	= 44100,
	.rate_max	= 96000,
	*/
	.rates		= SNDRV_PCM_RATE_48000,
	.rate_min	= 48000,
	.rate_max	= 48000,
	.channels_min	= 8,
	.channels_max	= 8,
	.buffer_bytes_max = ALSA_BUFFER_SIZE,
	.period_bytes_min = BYTES_PER_PERIOD,
	.period_bytes_max = ALSA_BUFFER_SIZE,
	.periods_min	= 1,
	.periods_max	= PERIODS_MAX,
};

enum {
	STREAM_DISABLED, /* no pcm streaming */
	STREAM_STARTING, /* pcm streaming requested, waiting to become ready */
	STREAM_RUNNING, /* pcm streaming running */
	STREAM_STOPPING
};

/* copy the audio frames from the URB packets into the ALSA buffer */
static void microbookii_pcm_capture(struct microbookii_substream *sub, struct microbookii_urb *urb)
{
	int i, frame, frame_count, bytes_per_frame;
	void *src, *dest, *dest_end;
	struct microbookii_pcm *rt;
	struct snd_pcm_runtime *alsa_rt;

	rt = snd_pcm_substream_chip(sub->instance);
	alsa_rt = sub->instance->runtime;

	dest = (alsa_rt->dma_area + sub->dma_off);
	dest_end = alsa_rt->dma_area +
				frames_to_bytes(alsa_rt, alsa_rt->buffer_size);

	bytes_per_frame = alsa_rt->frame_bits / 8;
	src = urb->buffer;

	for (i = 0; i < USB_N_PACKETS_PER_URB; i++) {
		frame_count = urb->packets[i].actual_length / bytes_per_frame;

		for (frame = 0; frame < frame_count; frame++) {
			memcpy(dest, src, bytes_per_frame);

			dest += bytes_per_frame;
			src += bytes_per_frame;
			sub->dma_off += bytes_per_frame;
			sub->period_off += bytes_per_frame;

			if (dest >= dest_end) {
				sub->dma_off = 0;
				dest = alsa_rt->dma_area;
			}
		}

		/* if packet was not full, make src point to data of next packet */
		src += urb->packets[i].length - urb->packets[i].actual_length;
	}
}

/* handle incoming URB with captured data */
static void microbookii_pcm_in_urb_handler(struct urb *usb_urb)
{
	struct microbookii_urb *mbii_urb = usb_urb->context;
	struct microbookii_pcm *pcm = &mbii_urb->mbii->pcm;
	struct microbookii_substream *stream = mbii_urb->stream;
	unsigned long flags;
	int ret, k, period_bytes;
	struct usb_iso_packet_descriptor *packet;

	if (pcm->panic || stream->state == STREAM_STOPPING)
		return;

	if (unlikely(usb_urb->status == -ENOENT ||		/* unlinked */
				usb_urb->status == -ENODEV ||		/* device removed */
				usb_urb->status == -ECONNRESET ||	/* unlinked */
				usb_urb->status == -ESHUTDOWN))		/* device disabled */
	{
		goto out_fail;
	}

	if (stream->state == STREAM_STARTING) {
		stream->wait_cond = true;
		wake_up(&stream->wait_queue);
	}

	if (stream->active) {
		spin_lock_irqsave(&stream->lock, flags);

		/* copy captured data into ALSA buffer */
		microbookii_pcm_capture(stream, mbii_urb);

		period_bytes = snd_pcm_lib_period_bytes(stream->instance);

		/* do we have enough data for one period? */
		if (stream->period_off > period_bytes) {
			stream->period_off %= period_bytes;

			spin_unlock_irqrestore(&stream->lock, flags);

			/* call this only once even if multiple periods are ready */
			snd_pcm_period_elapsed(stream->instance);

			memset(mbii_urb->buffer, 0, USB_BUFFER_SIZE);
		} else {
			spin_unlock_irqrestore(&stream->lock, flags);
			memset(mbii_urb->buffer, 0, USB_BUFFER_SIZE);
		}
	} else {
		memset(mbii_urb->buffer, 0, USB_BUFFER_SIZE);
	}

	/* reset URB data */
	for (k = 0; k < USB_N_PACKETS_PER_URB; k++) {
		packet = &mbii_urb->packets[k];
		packet->offset = k * USB_PACKET_OFFSET;
		packet->length = USB_PACKET_SIZE;
		packet->actual_length = 0;
		packet->status = 0;
	}
	mbii_urb->instance.number_of_packets = USB_N_PACKETS_PER_URB;

	/* send the URB back to the BCD2000 */
	ret = usb_submit_urb(&mbii_urb->instance, GFP_ATOMIC);
	if (ret < 0)
		goto out_fail;

	return;

out_fail:
	dev_info(&mbii_urb->mbii->dev->dev, PREFIX "error in in_urb handler");
	pcm->panic = true;
}

/* copy audio frame from ALSA buffer into the URB packet */
static void microbookii_pcm_playback(struct microbookii_substream *sub, struct microbookii_urb *urb)
{
	int i, frame, frame_count, bytes_per_frame;
	void *src, *src_end, *dest;
	struct microbookii_pcm *rt;
	struct snd_pcm_runtime *alsa_rt;

	rt = snd_pcm_substream_chip(sub->instance);
	alsa_rt = sub->instance->runtime;

	src = (alsa_rt->dma_area + sub->dma_off);
	src_end = alsa_rt->dma_area +
				frames_to_bytes(alsa_rt, alsa_rt->buffer_size);

	bytes_per_frame = alsa_rt->frame_bits / 8;
	dest = urb->buffer;

	for (i = 0; i < USB_N_PACKETS_PER_URB; i++) {
		frame_count = urb->packets[i].length / bytes_per_frame;

		for (frame = 0; frame < frame_count; frame++) {
			memcpy(dest, src, bytes_per_frame);

			src += bytes_per_frame;
			dest += bytes_per_frame;
			sub->dma_off += bytes_per_frame;
			sub->period_off += bytes_per_frame;

			if (src >= src_end) {
				sub->dma_off = 0;
				src = alsa_rt->dma_area;
			}
		}
	}
}

/* refill empty URB that comes back from the BCD2000 */
static void microbookii_pcm_out_urb_handler(struct urb *usb_urb)
{
	struct microbookii_urb *mbii_urb = usb_urb->context;
	struct microbookii_pcm *pcm = &mbii_urb->mbii->pcm;
	struct microbookii_substream *stream = mbii_urb->stream;
	unsigned long flags;
	int ret, k, period_bytes;
	struct usb_iso_packet_descriptor *packet;


	if (pcm->panic || stream->state == STREAM_STOPPING)
		return;

	if (unlikely(usb_urb->status == -ENOENT ||		/* unlinked */
		usb_urb->status == -ENODEV ||		/* device removed */
		usb_urb->status == -ECONNRESET ||	/* unlinked */
		usb_urb->status == -ESHUTDOWN))		/* device disabled */
	{
		goto out_fail;
	}

	if (stream->state == STREAM_STARTING) {
		stream->wait_cond = true;
		wake_up(&stream->wait_queue);
	}

	if (stream->active) {
		spin_lock_irqsave(&stream->lock, flags);

		memset(mbii_urb->buffer, 0, USB_BUFFER_SIZE);

		/* fill URB with data from ALSA */
		microbookii_pcm_playback(stream, mbii_urb);

		period_bytes = snd_pcm_lib_period_bytes(stream->instance);

		/* check if a complete period was written into the URB */
		if (stream->period_off > period_bytes) {
			stream->period_off %= period_bytes;

			spin_unlock_irqrestore(&stream->lock, flags);

			snd_pcm_period_elapsed(stream->instance);
		} else {
			spin_unlock_irqrestore(&stream->lock, flags);
		}

		for (k = 0; k < USB_N_PACKETS_PER_URB; k++) {
			packet = &mbii_urb->packets[k];
			packet->offset = k * USB_PACKET_OFFSET;
			packet->length = USB_PACKET_SIZE;
			packet->actual_length = 0;
			packet->status = 0;
		}
		mbii_urb->instance.number_of_packets = USB_N_PACKETS_PER_URB;

		ret = usb_submit_urb(&mbii_urb->instance, GFP_ATOMIC);
		if (ret < 0)
			goto out_fail;
	}

	return;

out_fail:
	dev_info(&mbii_urb->mbii->dev->dev, PREFIX "error in out_urb handler");
	pcm->panic = true;
}

static void microbookii_pcm_stream_stop(struct microbookii_pcm *pcm, struct microbookii_substream *stream)
{
	int i;

	if (stream->state != STREAM_DISABLED) {
		stream->state = STREAM_STOPPING;

		for (i = 0; i < USB_N_URBS; i++) {
			usb_kill_urb(&stream->urbs[i].instance);
		}

		stream->state = STREAM_DISABLED;
	}
}

static int microbookii_substream_open(struct snd_pcm_substream *substream)
{
	struct microbookii_substream *stream = NULL;
	struct microbookii_pcm *pcm = snd_pcm_substream_chip(substream);

	substream->runtime->hw = pcm->pcm_info;

	if (pcm->panic)
		return -EPIPE;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		stream = &pcm->playback;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		stream = &pcm->capture;

	if (!stream) {
		dev_err(&pcm->mbii->dev->dev, PREFIX "invalid stream type\n");
		return -EINVAL;
	}

	mutex_lock(&stream->mutex);
	stream->instance = substream;
	stream->active = false;
	mutex_unlock(&stream->mutex);

	return 0;
}

static int microbookii_substream_close(struct snd_pcm_substream *substream)
{
	unsigned long flags;
	struct microbookii_pcm *pcm = snd_pcm_substream_chip(substream);
	struct microbookii_substream *stream = NULL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		stream = &pcm->playback;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		stream = &pcm->capture;

	if (pcm->panic)
		return 0;

	mutex_lock(&stream->mutex);
	if (stream) {
		microbookii_pcm_stream_stop(pcm, stream);

		spin_lock_irqsave(&stream->lock, flags);
		stream->instance = NULL;
		stream->active = false;
		spin_unlock_irqrestore(&stream->lock, flags);
	}
	mutex_unlock(&stream->mutex);

	return 0;
}

static int microbookii_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_alloc_vmalloc_buffer(substream,
					params_buffer_bytes(hw_params));
}

static int microbookii_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int microbookii_pcm_stream_start(struct microbookii_pcm *pcm, struct microbookii_substream *stream)
{
	int ret;
	int i, k;
	struct usb_iso_packet_descriptor *packet;

	if (stream->state == STREAM_DISABLED) {
		/* reset panic state when starting a new stream */
		pcm->panic = false;

		stream->state = STREAM_STARTING;

		/* initialize data of each URB */
		for (i = 0; i < USB_N_URBS; i++) {
			for (k = 0; k < USB_N_PACKETS_PER_URB; k++) {
				packet = &stream->urbs[i].packets[k];
				packet->offset = k * USB_PACKET_OFFSET;
				packet->length = USB_PACKET_SIZE;
				packet->actual_length = 0;
				packet->status = 0;
			}

			/* immediately send data with the first audio out URB */
			if (stream->instance == SNDRV_PCM_STREAM_PLAYBACK) {
				microbookii_pcm_playback(stream, &stream->urbs[i]);
			}

			ret = usb_submit_urb(&stream->urbs[i].instance, GFP_ATOMIC);
			if (ret) {
				microbookii_pcm_stream_stop(pcm, stream);
				dev_err(&pcm->mbii->dev->dev, PREFIX
												"could not submit urb, Error: %i\n", ret);
				return ret;
			}
		}

		/* wait for first out urb to return (sent in in urb handler) */
		wait_event_timeout(stream->wait_queue,
						   stream->wait_cond, HZ);
		if (stream->wait_cond) {
			dev_dbg(&pcm->mbii->dev->dev, PREFIX
					"%s: stream is running wakeup event\n", __func__);
			stream->state = STREAM_RUNNING;
		} else {
			microbookii_pcm_stream_stop(pcm, stream);
			return -EIO;
		}
	}
	return 0;
}

static int microbookii_pcm_prepare(struct snd_pcm_substream *substream)
{
	int ret;
	struct microbookii_pcm *pcm = snd_pcm_substream_chip(substream);
	struct microbookii_substream *stream = NULL;
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		stream = &pcm->playback;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		stream = &pcm->capture;

	if (pcm->panic)
		return -EPIPE;

	if (!stream)
		return -ENODEV;
		
	ret = microbookii_set_rate(pcm->mbii, runtime->rate);
	if (ret < 0) {
		dev_err(&pcm->mbii->dev->dev, PREFIX
					"could not set rate\n");
		return ret;
	}
	ret = microbookii_poll_device_ready(pcm->mbii);
	if (ret < 0) {
		dev_err(&pcm->mbii->dev->dev, PREFIX
					"could not check if device ready\n");
		return ret;
	}
	
	mutex_lock(&stream->mutex);
	stream->dma_off = 0;
	stream->period_off = 0;

	if (stream->state == STREAM_DISABLED) {
		ret = microbookii_pcm_stream_start(pcm, stream);
		if (ret) {
			mutex_unlock(&stream->mutex);
			dev_err(&pcm->mbii->dev->dev, PREFIX
					"could not start pcm stream\n");
			return ret;
		}
	}
	mutex_unlock(&stream->mutex);
	return 0;
}

static int microbookii_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct microbookii_pcm *pcm = snd_pcm_substream_chip(substream);
	struct microbookii_substream *stream = NULL;
	unsigned long flags;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		stream = &pcm->playback;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		stream = &pcm->capture;

	if (pcm->panic)
		return -EPIPE;

	if (!stream)
		return -ENODEV;

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			spin_lock_irqsave(&stream->lock, flags);
			stream->active = true;
			spin_unlock_irqrestore(&stream->lock, flags);

			return 0;

		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			spin_lock_irqsave(&stream->lock, flags);
			stream->active = false;
			spin_unlock_irqrestore(&stream->lock, flags);

			return 0;
		default:
			return -EINVAL;
	}
}

static snd_pcm_uframes_t
microbookii_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct microbookii_pcm *pcm = snd_pcm_substream_chip(substream);
	unsigned long flags;
	snd_pcm_uframes_t ret;
	struct microbookii_substream *stream = NULL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		stream = &pcm->playback;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		stream = &pcm->capture;

	if (pcm->panic || !stream)
		return SNDRV_PCM_POS_XRUN;

	spin_lock_irqsave(&stream->lock, flags);
	/* return the number of the last written period in the ALSA ring buffer */
	ret = bytes_to_frames(stream->instance->runtime, stream->dma_off);
	spin_unlock_irqrestore(&stream->lock, flags);

	return ret;
}

static struct snd_pcm_ops microbookii_ops = {
	.open = microbookii_substream_open,
	.close = microbookii_substream_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = microbookii_pcm_hw_params,
	.hw_free = microbookii_pcm_hw_free,
	.prepare = microbookii_pcm_prepare,
	.trigger = microbookii_pcm_trigger,
	.pointer = microbookii_pcm_pointer,
	.page = snd_pcm_lib_get_vmalloc_page,
	.mmap = snd_pcm_lib_mmap_vmalloc,
};

static int microbookii_pcm_init_urb(struct microbookii_urb *urb,
							struct microbookii *mbii,
							char in, unsigned int ep,
							void (*handler)(struct urb *))
{
	urb->mbii = mbii;
	usb_init_urb(&urb->instance);

	urb->buffer = kzalloc(USB_BUFFER_SIZE, GFP_KERNEL);
	if (!urb->buffer)
		return -ENOMEM;

	urb->instance.transfer_buffer = urb->buffer;
	urb->instance.transfer_buffer_length = USB_BUFFER_SIZE;
	urb->instance.dev = mbii->dev;
	urb->instance.pipe = in ? usb_rcvisocpipe(mbii->dev, ep)
							: usb_sndisocpipe(mbii->dev, ep);
	urb->instance.interval = 1;
	urb->instance.complete = handler;
	urb->instance.context = urb;
	urb->instance.number_of_packets = USB_N_PACKETS_PER_URB;

	return 0;
}

static void microbookii_pcm_destroy(struct microbookii *mbii)
{
	int i;

	for (i = 0; i < USB_N_URBS; i++) {
		kfree(mbii->pcm.playback.urbs[i].buffer);
		kfree(mbii->pcm.capture.urbs[i].buffer);
	}
}

static void microbookii_pcm_free(struct snd_pcm *pcm)
{
	struct microbookii_pcm *mbii_pcm = pcm->private_data;
	if (mbii_pcm)
		microbookii_pcm_destroy(mbii_pcm->mbii);
}

int microbookii_init_stream(struct microbookii *mbii,struct microbookii_substream *stream, bool in)
{
	int i, ret;

	stream->state = STREAM_DISABLED;

	init_waitqueue_head(&stream->wait_queue);
	mutex_init(&stream->mutex);

	for (i=0; i<USB_N_URBS; i++) {
		ret = microbookii_pcm_init_urb(&stream->urbs[i], mbii, in, in? 0x84 : 0x3,
							 in? microbookii_pcm_in_urb_handler : microbookii_pcm_out_urb_handler);
		if (ret) {
			dev_err(&mbii->dev->dev, PREFIX
					"%s: urb init failed, ret=%d: ",
					__func__, ret);
			return ret;
		}
		stream->urbs[i].stream = stream;
	}

	return 0;
}

int microbookii_init_audio(struct microbookii *mbii)
{
	int ret;
	struct microbookii_pcm * pcm;

	pcm = &mbii->pcm;
	pcm->mbii = mbii;

	spin_lock_init(&pcm->playback.lock);
	spin_lock_init(&pcm->capture.lock);

	microbookii_init_stream(mbii, &pcm->playback, 0);
	/* microbookii_init_stream(mbii, &pcm->capture, 1); */

	// 1 playback stream 0 capture streams for now
	ret = snd_pcm_new(mbii->card, DEVICENAME, 0, 1, 0, &pcm->instance);
	if (ret < 0) {
		dev_err(&mbii->dev->dev, PREFIX
			"%s: snd_pcm_new() failed, ret=%d: ",
			__func__, ret);
		return ret;
	}
	pcm->instance->private_data = pcm;
	pcm->instance->private_free = microbookii_pcm_free;

	strlcpy(pcm->instance->name, DEVICENAME, sizeof(pcm->instance->name));

	memcpy(&pcm->pcm_info, &microbookii_pcm_hardware,
		sizeof(microbookii_pcm_hardware));

	snd_pcm_set_ops(pcm->instance, SNDRV_PCM_STREAM_PLAYBACK, &microbookii_ops);
	/* snd_pcm_set_ops(pcm->instance, SNDRV_PCM_STREAM_CAPTURE, &microbookii_ops); */

	return 0;
}

void microbookii_free_audio(struct microbookii *mbii)
{
}
