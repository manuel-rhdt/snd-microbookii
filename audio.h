#ifndef AUDIO_H
#define AUDIO_H

#include <sound/pcm.h>

#define USB_N_URBS 4
#define USB_N_PACKETS_PER_URB 10
#define USB_PACKET_OFFSET 144
#define USB_PACKET_SIZE 144
#define USB_BUFFER_SIZE (USB_PACKET_SIZE * USB_N_PACKETS_PER_URB)

#define BYTES_PER_PERIOD (24 * 16)
#define PERIODS_MAX 128
#define ALSA_BUFFER_SIZE (BYTES_PER_PERIOD * PERIODS_MAX)

struct microbookii;

struct microbookii_urb {
	int index;

	struct microbookii *mbii;
	struct microbookii_substream *stream;

	/* BEGIN DO NOT SEPARATE */
	struct urb instance;
	struct usb_iso_packet_descriptor packets[USB_N_PACKETS_PER_URB];
	/* END DO NOT SEPARATE */
	
	int packet_size[USB_N_PACKETS_PER_URB]; /* size of packets for next submission */
	struct list_head ready_list;
};

struct microbookii_substream {
	struct snd_pcm_substream *instance;

	u8 state;
	bool active;
	snd_pcm_uframes_t dma_off; /* current position in alsa dma_area */
	snd_pcm_uframes_t period_off; /* current position in current period */

	struct microbookii_urb urbs[USB_N_URBS];
	struct list_head ready_playback_urbs;
	long unsigned int active_mask;
	
	struct microbookii_usb_packet_info {
		uint32_t packet_size[USB_N_PACKETS_PER_URB];
		int packets;
	} next_packet[USB_N_URBS];
	int next_packet_read_pos, next_packet_write_pos;

	spinlock_t lock;
	struct mutex mutex;
	wait_queue_head_t wait_queue;
	bool wait_cond;
};

struct microbookii_pcm {
	struct microbookii *mbii;

	struct snd_pcm *instance;
	struct snd_pcm_hardware pcm_info;

	struct microbookii_substream capture;
	struct microbookii_substream playback;
	
	bool panic; /* if set driver won't do anymore pcm on device */
};

int microbookii_init_audio(struct microbookii *mbii);
void microbookii_free_audio(struct microbookii *mbii);

#endif
