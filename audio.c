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
#include <linux/usb.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "audio.h"
#include "control.h"
#include "microbookii.h"

#define DEBUG 1


static struct snd_pcm_hardware microbookii_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID
		| SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BATCH
		| SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S24_3BE,
	/*
	.rates		= SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
	SNDRV_PCM_RATE_96000, .rate_min	= 44100, .rate_max	= 96000,
	*/
	.rates = SNDRV_PCM_RATE_96000,
	.rate_min = 96000,
	.rate_max = 96000,
	.buffer_bytes_max = 1024 * 1024,
	.period_bytes_min = 64,
	.period_bytes_max = 512 * 1024,
	.periods_min = 2,
	.periods_max = 1024,
};

enum { STREAM_DISABLED, /* no pcm streaming */
       STREAM_STARTING, /* pcm streaming requested, waiting to become ready */
       STREAM_RUNNING,  /* pcm streaming running */
       STREAM_STOPPING };


/* return the estimated delay based on USB frame counters */
snd_pcm_uframes_t
microbookii_usb_pcm_delay(struct microbookii_substream *stream,
			  unsigned int rate)
{
	int current_frame_number;
	int frame_diff;
	int est_delay;
	struct microbookii_pcm *pcm;

	if (!stream->instance) {
		return -1;
	}
	pcm = snd_pcm_substream_chip(stream->instance);

	if (stream->last_delay == 0)
		return 0; /* short path */

	current_frame_number = usb_get_current_frame_number(pcm->mbii->dev);
	/*
	 * HCD implementations use different widths, use lower 8 bits.
	 * The delay will be managed up to 256ms, which is more than
	 * enough
	 */
	frame_diff = (current_frame_number - stream->last_usb_frame) & 0xff;

	/* Approximation based on number of samples per USB frame (ms),
	   some truncation for 44.1 but the estimate is good enough */
	est_delay = frame_diff * rate / 1000;
	if (stream->instance->stream == SNDRV_PCM_STREAM_PLAYBACK)
		est_delay = stream->last_delay - est_delay;
	else
		est_delay = stream->last_delay + est_delay;

	if (est_delay < 0)
		est_delay = 0;
	return est_delay;
}

/* copy the audio frames from the URB packets into the ALSA buffer */
static void microbookii_pcm_capture(struct microbookii_substream *sub,
				    struct microbookii_urb *urb)
{
	int i, frame, frame_count, bytes_per_frame;
	void *src, *dest, *dest_end;
	struct microbookii_pcm *rt;
	struct snd_pcm_runtime *alsa_rt;

	rt = snd_pcm_substream_chip(sub->instance);
	alsa_rt = sub->instance->runtime;

	dest = (alsa_rt->dma_area + sub->dma_off);
	dest_end = alsa_rt->dma_area
		   + frames_to_bytes(alsa_rt, alsa_rt->buffer_size);

	bytes_per_frame = alsa_rt->frame_bits / 8;
	src = urb->instance.transfer_buffer;

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

		/* if packet was not full, make src point to data of next packet
		 */
		src += urb->packets[i].length - urb->packets[i].actual_length;
	}
}

/* Copy audio frames from ALSA buffer into the URB packet. Returns the number of
 * frames written to the transfer buffer. */
static int microbookii_pcm_playback(struct microbookii_substream *sub,
				    struct microbookii_urb *urb)
{
	int i, frame, frame_count, bytes_per_frame, total_frames = 0,
						    total_bytes;
	void *src, *src_end, *dest;
	struct microbookii_pcm *rt;
	struct snd_pcm_runtime *alsa_rt;

	rt = snd_pcm_substream_chip(sub->instance);
	alsa_rt = sub->instance->runtime;

	src = (alsa_rt->dma_area + sub->dma_off);
	src_end = alsa_rt->dma_area
		  + frames_to_bytes(alsa_rt, alsa_rt->buffer_size);

	dest = urb->instance.transfer_buffer;

	bytes_per_frame = frames_to_bytes(alsa_rt, 1);

	for (i = 0; i < USB_N_PACKETS_PER_URB; i++) {
		frame_count = bytes_to_frames(alsa_rt, urb->packets[i].length);
		total_frames += frame_count;

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

	total_bytes = frames_to_bytes(alsa_rt, total_frames);
	urb->instance.transfer_buffer_length = total_bytes;

	return total_frames;
}

static void microbookii_prepare_playback_urb(struct microbookii_urb *urb)
{
	unsigned long flags;
	unsigned int period_bytes;
	int frames;

	struct microbookii_substream *stream = urb->stream;
	struct microbookii *mbii = urb->mbii;
	struct snd_pcm_runtime *runtime = stream->instance->runtime;

	spin_lock_irqsave(&stream->lock, flags);

	memset(urb->instance.transfer_buffer, 0,
	       stream->max_packet_size * USB_N_PACKETS_PER_URB);

	/* fill URB with data from ALSA */
	frames = microbookii_pcm_playback(stream, urb);

	/* update delay with exact number of samples queued */
	runtime->delay = stream->last_delay;
	runtime->delay += frames;
	stream->last_delay = runtime->delay;

	/* realign last_frame_number */
	stream->last_usb_frame = usb_get_current_frame_number(mbii->dev);
	stream->last_usb_frame &= 0xFF; /* keep 8 LSBs */

	if (stream->trigger_tstamp_pending_update) {
		/* this is the first actual URB submitted,
		 * update trigger timestamp to reflect actual start time
		 */
		snd_pcm_gettime(runtime, &runtime->trigger_tstamp);
		stream->trigger_tstamp_pending_update = false;
	}

	/* check if a complete period was written into the URB */
	period_bytes = snd_pcm_lib_period_bytes(stream->instance);
	if (stream->period_off > period_bytes) {
		stream->period_off %= period_bytes;

		spin_unlock_irqrestore(&stream->lock, flags);

		snd_pcm_period_elapsed(stream->instance);
	} else {
		spin_unlock_irqrestore(&stream->lock, flags);
	}
}

/*
 * Send output urbs that have been prepared previously. URBs are dequeued
 * from ep->ready_playback_urbs and in case there there aren't any available
 * or there are no packets that have been prepared, this function does
 * nothing.
 *
 * The reason why the functionality of sending and preparing URBs is separated
 * is that host controllers don't guarantee the order in which they return
 * inbound and outbound packets to their submitters.
 *
 * This function is only used for implicit feedback endpoints. For endpoints
 * driven by dedicated sync endpoints, URBs are immediately re-submitted
 * from their completion handler.
 */
static void queue_pending_output_urbs(struct microbookii_substream *stream)
{
	struct microbookii_pcm *pcm = snd_pcm_substream_chip(stream->instance);
	struct microbookii *mbii = pcm->mbii;
	if (stream->instance->stream != SNDRV_PCM_STREAM_PLAYBACK) {
		dev_err(&mbii->dev->dev,
			"Called queue_pending_output_urbs for capture substream.");
		return;
	}

	while (stream->state == STREAM_RUNNING) {
		unsigned long flags;
		struct microbookii_usb_packet_info *packet;
		struct microbookii_urb *urb = NULL;
		int err, i, offset;

		spin_lock_irqsave(&stream->lock, flags);
		if (stream->next_packet_read_pos
		    != stream->next_packet_write_pos) {
			packet = stream->next_packet
				 + stream->next_packet_read_pos;
			stream->next_packet_read_pos++;
			stream->next_packet_read_pos %= USB_N_URBS;

			/* take URB out of FIFO */
			if (!list_empty(&stream->ready_playback_urbs))
				urb = list_first_entry(
					&stream->ready_playback_urbs,
					struct microbookii_urb, ready_list);
		}
		spin_unlock_irqrestore(&stream->lock, flags);

		if (urb == NULL)
			return;

		list_del_init(&urb->ready_list);

		/* copy over the length information */
		offset = 0;
		for (i = 0; i < packet->packets; i++) {
			urb->packets[i].offset = offset;
			urb->packets[i].length = packet->packet_size[i];
			urb->packets[i].actual_length = 0;
			urb->packets[i].status = 0;

			offset += packet->packet_size[i];
		}

		/* fill URB with data from ALSA */
		microbookii_prepare_playback_urb(urb);

		err = usb_submit_urb(&urb->instance, GFP_ATOMIC);
		if (err < 0)
			dev_err(&mbii->dev->dev,
				"Unable to submit urb: %d (urb %p)\n", err,
				&urb->instance);
		else
			set_bit(urb->index, &stream->active_mask);
	}
}

/**
 * microbookii_handle_sync_urb: parse an USB sync packet
 *
 * @ep: the endpoint to handle the packet
 * @sender: the sending endpoint
 * @urb: the received packet
 *
 * This function is called from the context of an endpoint that received
 * the packet and is used to let another endpoint object handle the payload.
 */
void microbookii_handle_sync_urb(struct microbookii_pcm *pcm,
				 const struct urb *usb_urb)
{
	unsigned long flags;

	/*
	 * In case the endpoint is operating in implicit feedback mode, prepare
	 * a new outbound URB that has the same layout as the received packet
	 * and add it to the list of pending urbs. queue_pending_output_urbs()
	 * will take care of them later.
	 */

	/* implicit feedback */
	int i, bytes = 0;
	struct microbookii_urb *urb;
	struct microbookii_usb_packet_info *out_packet;

	urb = usb_urb->context;

	/* Count overall packet size */
	for (i = 0; i < USB_N_PACKETS_PER_URB; i++)
		if (usb_urb->iso_frame_desc[i].status == 0)
			bytes += usb_urb->iso_frame_desc[i].actual_length;

	spin_lock_irqsave(&pcm->playback.lock, flags);
	out_packet =
		pcm->playback.next_packet + pcm->playback.next_packet_write_pos;

	/*
	 * Iterate through the inbound packet and prepare the lengths
	 * for the output packet. The OUT packet we are about to send
	 * will have the same amount of payload bytes per stride as the
	 * IN packet we just received. Since the actual size is scaled
	 * by the stride, use the sender stride to calculate the length
	 * in case the number of channels differ between the implicitly
	 * fed-back endpoint and the synchronizing endpoint.
	 */

	out_packet->packets = USB_N_PACKETS_PER_URB;
	for (i = 0; i < USB_N_PACKETS_PER_URB; i++) {
		if (usb_urb->iso_frame_desc[i].status == 0)
			out_packet->packet_size[i] =
				usb_urb->iso_frame_desc[i].actual_length / 6
				* 8;
		else
			out_packet->packet_size[i] = 0;
	}

	pcm->playback.next_packet_write_pos++;
	pcm->playback.next_packet_write_pos %= USB_N_URBS;
	spin_unlock_irqrestore(&pcm->playback.lock, flags);
	queue_pending_output_urbs(&pcm->playback);
}

/* handle URB callback */
static void microbookii_pcm_urb_handler(struct urb *usb_urb)
{
	struct microbookii_urb *mbii_urb = usb_urb->context;
	struct microbookii_pcm *pcm = &mbii_urb->mbii->pcm;
	struct microbookii_substream *stream = mbii_urb->stream;
	struct snd_pcm_runtime *runtime = NULL;
	unsigned long flags;
	int ret, k, period_bytes, est_delay, current_frame_number, processed;
	struct usb_iso_packet_descriptor *packet;

	if (pcm->panic || stream->state == STREAM_STOPPING) {
		return;
	}

	if (unlikely(usb_urb->status == -ENOENT ||     /* unlinked */
		     usb_urb->status == -ENODEV ||     /* device removed */
		     usb_urb->status == -ECONNRESET || /* unlinked */
		     usb_urb->status == -ESHUTDOWN))   /* device disabled */
	{
		goto out_fail;
	}

	if (stream->state == STREAM_STARTING) {
		stream->wait_cond = true;
		wake_up(&stream->wait_queue);
	}

	if (usb_pipeout(usb_urb->pipe)) {
		runtime = stream->instance->runtime;
		processed = bytes_to_frames(runtime,
					    usb_urb->transfer_buffer_length);

		spin_lock_irqsave(&stream->lock, flags);

		if (stream->last_delay != 0 && processed != 0) {
			est_delay = microbookii_usb_pcm_delay(stream,
							      runtime->rate);
			/* update delay with exact number of samples played */
			if (processed > stream->last_delay)
				stream->last_delay = 0;
			else
				stream->last_delay -= processed;
			runtime->delay = stream->last_delay;

			/*
			 * Report when delay estimate is off by more than 2ms.
			 * The error should be lower than 2ms since the estimate
			 * relies on two reads of a counter updated every ms.
			 */
			if (abs(est_delay - stream->last_delay) * 1000
			    > runtime->rate * 2)
				dev_err_ratelimited(
					&pcm->mbii->dev->dev,
					"delay: estimated %d, actual %d\n",
					est_delay, stream->last_delay);

			if (!stream->active) {
				/* update last_frame_number for delay counting
				 * here since prepare_playback_urb won't be
				 * called during pause
				 */
				stream->last_usb_frame =
					usb_get_current_frame_number(
						pcm->mbii->dev)
					& 0xff;
			}
		}


		list_add_tail(&mbii_urb->ready_list,
			      &stream->ready_playback_urbs);
		spin_unlock_irqrestore(&stream->lock, flags);

		clear_bit(mbii_urb->index, &stream->active_mask);

		queue_pending_output_urbs(stream);
	} else {
		if (pcm->playback.active) {
			microbookii_handle_sync_urb(pcm, usb_urb);
		}
		if (stream->active) {
			runtime = stream->instance->runtime;

			current_frame_number =
				usb_get_current_frame_number(pcm->mbii->dev);

			spin_lock_irqsave(&stream->lock, flags);

			/* copy captured data into ALSA buffer */
			microbookii_pcm_capture(stream, mbii_urb);

			period_bytes =
				snd_pcm_lib_period_bytes(stream->instance);

			/* capture delay is by construction limited to one URB,
			 * reset delays here
			 */
			runtime->delay = stream->last_delay = 0;

			/* realign last_frame_number */
			stream->last_usb_frame = current_frame_number;
			stream->last_usb_frame &= 0xFF; /* keep 8 LSBs */

			/* do we have enough data for one period? */
			if (stream->period_off > period_bytes) {
				stream->period_off %= period_bytes;

				spin_unlock_irqrestore(&stream->lock, flags);

				/* call this only once even if multiple periods
				 * are ready */
				snd_pcm_period_elapsed(stream->instance);
			} else {
				spin_unlock_irqrestore(&stream->lock, flags);
			}
		}
		memset(mbii_urb->instance.transfer_buffer, 0,
		       stream->max_packet_size * USB_N_PACKETS_PER_URB);

		/* reset URB data */
		for (k = 0; k < USB_N_PACKETS_PER_URB; k++) {
			packet = &mbii_urb->packets[k];
			packet->offset = k * stream->max_packet_size;
			packet->length = stream->max_packet_size;
			packet->actual_length = 0;
			packet->status = 0;
		}
		mbii_urb->instance.number_of_packets = USB_N_PACKETS_PER_URB;

		/* send the URB back to the MicroBook */
		ret = usb_submit_urb(&mbii_urb->instance, GFP_ATOMIC);
		if (ret < 0)
			goto out_fail;
	}
	return;

out_fail:
	dev_dbg(&mbii_urb->mbii->dev->dev, PREFIX "Device not available.");
	pcm->panic = true;
}


static void microbookii_pcm_stream_stop(struct microbookii_pcm *pcm,
					struct microbookii_substream *stream)
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

	if (pcm->panic)
		return -EPIPE;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		stream = &pcm->playback;
		substream->runtime->hw = pcm->pcm_playback_info;
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		stream = &pcm->capture;
		substream->runtime->hw = pcm->pcm_capture_info;
	}

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
	struct microbookii_substream *sync_stream = NULL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		stream = &pcm->playback;
		sync_stream = &pcm->capture;
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* only stop capture stream when playback stream is disabled, so
		feedback sync works */
		if (pcm->playback.state == STREAM_DISABLED) {
			stream = &pcm->capture;
		}
	}

	if (pcm->panic)
		return 0;


	if (stream) {
		mutex_lock(&stream->mutex);
		microbookii_pcm_stream_stop(pcm, stream);

		spin_lock_irqsave(&stream->lock, flags);
		stream->instance = NULL;
		stream->active = false;
		spin_unlock_irqrestore(&stream->lock, flags);
		mutex_unlock(&stream->mutex);
	}

	/* When sync stream has no instance alsa is not using it as capture
	stream right now so we stop it now. */
	if (sync_stream && !sync_stream->instance) {
		mutex_lock(&sync_stream->mutex);
		microbookii_pcm_stream_stop(pcm, sync_stream);

		spin_lock_irqsave(&sync_stream->lock, flags);
		sync_stream->instance = NULL;
		sync_stream->active = false;
		spin_unlock_irqrestore(&sync_stream->lock, flags);
		mutex_unlock(&sync_stream->mutex);
	}

	return 0;
}

static int
microbookii_substream_urb_buffer_alloc(struct microbookii_substream *stream,
				       unsigned int buffer_size)
{
	int i;
	struct microbookii_urb *urb;

	for (i = 0; i < USB_N_URBS; i++) {
		urb = &stream->urbs[i];

		if (urb->instance.transfer_buffer_length == buffer_size)
			continue;

		if (urb->instance.transfer_buffer_length > 0) {
			kfree(urb->instance.transfer_buffer);
			urb->instance.transfer_buffer_length = 0;
		}

		urb->instance.transfer_buffer =
			kzalloc(buffer_size, GFP_ATOMIC);
		if (!urb->instance.transfer_buffer)
			return -ENOMEM;

		urb->instance.transfer_buffer_length = buffer_size;
	}
	return 0;
}

static int microbookii_pcm_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *hw_params)
{
	struct microbookii_pcm *pcm = snd_pcm_substream_chip(substream);
	struct microbookii_substream *stream = NULL;
	struct microbookii_substream *sync_stream = NULL;

	unsigned int urb_buffer_size;
	unsigned int max_frames_per_packet = USB_MAX_FRAMES_PER_PACKET;
	unsigned int bytes_per_frame = 3;
	int err;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		stream = &pcm->playback;
		sync_stream = &pcm->capture;
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		stream = &pcm->capture;
	}

	if (params_rate(hw_params) >= 88200) {
		max_frames_per_packet *= 2;
	}
	stream->max_packet_size = params_channels(hw_params)
				  * max_frames_per_packet * bytes_per_frame;
	urb_buffer_size = stream->max_packet_size * USB_N_PACKETS_PER_URB;

	err = microbookii_substream_urb_buffer_alloc(stream, urb_buffer_size);
	if (err < 0)
		return err;

	if (sync_stream != NULL) {
		sync_stream->max_packet_size =
			6 * max_frames_per_packet * bytes_per_frame;
		urb_buffer_size =
			sync_stream->max_packet_size * USB_N_PACKETS_PER_URB;
		err = microbookii_substream_urb_buffer_alloc(sync_stream,
							     urb_buffer_size);
		if (err < 0)
			return err;
	}

	return snd_pcm_lib_alloc_vmalloc_buffer(substream,
						params_buffer_bytes(hw_params));
}

static int microbookii_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int microbookii_pcm_stream_start(struct microbookii_pcm *pcm,
					struct microbookii_substream *stream)
{
	int ret;
	int i, k;
	struct usb_iso_packet_descriptor *packet;

	if (!stream)
		return -ENODEV;

	if (stream->state == STREAM_DISABLED) {
		/* reset panic state when starting a new stream */
		pcm->panic = false;

		stream->state = STREAM_STARTING;

		INIT_LIST_HEAD(&stream->ready_playback_urbs);

		/* initialize data of each URB */
		for (i = 0; i < USB_N_URBS; i++) {
			/* immediately send data with the first audio out URB */
			if (stream->instance
			    && stream->instance->stream
				       == SNDRV_PCM_STREAM_PLAYBACK) {
				list_add_tail(&stream->urbs[i].ready_list,
					      &stream->ready_playback_urbs);
			} else {
				for (k = 0; k < USB_N_PACKETS_PER_URB; k++) {
					packet = &stream->urbs[i].packets[k];
					packet->offset =
						k * stream->max_packet_size;
					packet->length =
						stream->max_packet_size;
					packet->actual_length = 0;
					packet->status = 0;
				}

				ret = usb_submit_urb(&stream->urbs[i].instance,
						     GFP_ATOMIC);
				if (ret) {
					microbookii_pcm_stream_stop(pcm,
								    stream);
					dev_err(&pcm->mbii->dev->dev,
						PREFIX
						"could not submit urb, Error: %i\n",
						ret);
					return ret;
				}
			}
		}

		stream->state = STREAM_RUNNING;
	}
	return 0;
}

static int microbookii_stream_prepare(struct microbookii_pcm *pcm,
				      struct microbookii_substream *stream)
{
	int err;

	mutex_lock(&stream->mutex);
	stream->dma_off = 0;
	stream->period_off = 0;
	stream->last_delay = 0;
	stream->last_usb_frame = 0;

	if (stream->state == STREAM_DISABLED) {
		err = microbookii_pcm_stream_start(pcm, stream);
		if (err) {
			mutex_unlock(&stream->mutex);
			return err;
		}
	}
	mutex_unlock(&stream->mutex);

	return 0;
}

static int microbookii_pcm_prepare(struct snd_pcm_substream *substream)
{
	int ret;
	struct microbookii_pcm *pcm = snd_pcm_substream_chip(substream);
	struct microbookii_substream *stream = NULL;
	struct microbookii_substream *sync_stream = NULL;
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		stream = &pcm->playback;
		sync_stream = &pcm->capture;
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		stream = &pcm->capture;
	}

	if (pcm->panic)
		return -EPIPE;

	if (!stream)
		return -ENODEV;

	ret = microbookii_set_rate(pcm->mbii, runtime->rate);
	if (ret < 0) {
		dev_err(&pcm->mbii->dev->dev, PREFIX "could not set rate\n");
		return ret;
	}
	ret = microbookii_wait_device_ready(pcm->mbii);
	if (ret < 0) {
		dev_err(&pcm->mbii->dev->dev,
			PREFIX "could not check if device ready\n");
		return ret;
	}

	substream->runtime->delay = 0;

	ret = microbookii_stream_prepare(pcm, stream);
	if (ret < 0) {
		dev_err(&pcm->mbii->dev->dev,
			PREFIX "could not start pcm stream\n");
		return ret;
	}

	if (sync_stream != NULL) {
		ret = microbookii_stream_prepare(pcm, sync_stream);
		if (ret < 0) {
			dev_err(&pcm->mbii->dev->dev,
				PREFIX "could not start pcm sync stream\n");
			return ret;
		}
	}

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
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			stream->trigger_tstamp_pending_update = true;
		}
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

	if (pcm->panic || !stream || !stream->instance)
		return SNDRV_PCM_POS_XRUN;

	spin_lock_irqsave(&stream->lock, flags);
	/* return the number of the last written period in the ALSA ring buffer
	 */
	ret = bytes_to_frames(stream->instance->runtime, stream->dma_off);

	substream->runtime->delay =
		microbookii_usb_pcm_delay(stream, substream->runtime->rate);

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
				    struct microbookii *mbii, char in,
				    unsigned int ep,
				    void (*handler)(struct urb *))
{
	urb->mbii = mbii;
	usb_init_urb(&urb->instance);

	urb->instance.transfer_buffer = NULL;
	urb->instance.transfer_buffer_length = 0;
	urb->instance.dev = mbii->dev;
	urb->instance.pipe = in ? usb_rcvisocpipe(mbii->dev, ep)
				: usb_sndisocpipe(mbii->dev, ep);
	urb->instance.interval = 1;
	urb->instance.complete = handler;
	urb->instance.context = urb;
	urb->instance.number_of_packets = USB_N_PACKETS_PER_URB;

	INIT_LIST_HEAD(&urb->ready_list);

	return 0;
}

static void microbookii_pcm_destroy(struct microbookii *mbii)
{
	int i;
	struct urb *urb;

	for (i = 0; i < USB_N_URBS; i++) {
		urb = &mbii->pcm.playback.urbs[i].instance;
		if (urb->transfer_buffer_length > 0) {
			kfree(urb->transfer_buffer);
			urb->transfer_buffer_length = 0;
		}

		urb = &mbii->pcm.capture.urbs[i].instance;
		if (urb->transfer_buffer_length > 0) {
			kfree(urb->transfer_buffer);
			urb->transfer_buffer_length = 0;
		}
	}
}

static void microbookii_pcm_free(struct snd_pcm *pcm)
{
	struct microbookii_pcm *mbii_pcm = pcm->private_data;
	if (mbii_pcm)
		microbookii_pcm_destroy(mbii_pcm->mbii);
}

int microbookii_init_stream(struct microbookii *mbii,
			    struct microbookii_substream *stream, bool in)
{
	int i, ret;

	stream->state = STREAM_DISABLED;

	init_waitqueue_head(&stream->wait_queue);
	mutex_init(&stream->mutex);

	INIT_LIST_HEAD(&stream->ready_playback_urbs);

	for (i = 0; i < USB_N_URBS; i++) {
		ret = microbookii_pcm_init_urb(&stream->urbs[i], mbii, in,
					       in ? 0x84 : 0x3,
					       microbookii_pcm_urb_handler);
		if (ret) {
			dev_err(&mbii->dev->dev,
				PREFIX "%s: urb init failed, ret=%d: ",
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
	struct microbookii_pcm *pcm;

	pcm = &mbii->pcm;
	pcm->mbii = mbii;

	spin_lock_init(&pcm->playback.lock);
	spin_lock_init(&pcm->capture.lock);

	microbookii_init_stream(mbii, &pcm->playback, 0);
	microbookii_init_stream(mbii, &pcm->capture, 1);

	ret = snd_pcm_new(mbii->card, DEVICENAME, 0, 1, 1, &pcm->instance);
	if (ret < 0) {
		dev_err(&mbii->dev->dev,
			PREFIX "%s: snd_pcm_new() failed, ret=%d: ", __func__,
			ret);
		return ret;
	}
	pcm->instance->private_data = pcm;
	pcm->instance->private_free = microbookii_pcm_free;

	strlcpy(pcm->instance->name, DEVICENAME, sizeof(pcm->instance->name));

	memcpy(&pcm->pcm_playback_info, &microbookii_pcm_hardware,
	       sizeof(microbookii_pcm_hardware));
	memcpy(&pcm->pcm_capture_info, &microbookii_pcm_hardware,
	       sizeof(microbookii_pcm_hardware));

	pcm->pcm_playback_info.channels_min = 8;
	pcm->pcm_playback_info.channels_max = 8;
	pcm->pcm_capture_info.channels_min = 6;
	pcm->pcm_capture_info.channels_max = 6;

	snd_pcm_set_ops(pcm->instance, SNDRV_PCM_STREAM_CAPTURE,
			&microbookii_ops);
	snd_pcm_set_ops(pcm->instance, SNDRV_PCM_STREAM_PLAYBACK,
			&microbookii_ops);

	return 0;
}

void microbookii_free_audio(struct microbookii *mbii)
{
}
