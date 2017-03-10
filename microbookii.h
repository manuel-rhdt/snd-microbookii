#ifndef MICROBOOKII_H
#define MICROBOOKII_H

#include <linux/usb.h>
#include <linux/usb/audio.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/timer.h>
#include <linux/completion.h>

#define DEVICENAME "MicrobookII"
#define PREFIX "snd-microbookii: "

#include "audio.h"
#include "control.h"

struct microbookii {
	struct usb_device *dev;
	struct snd_card *card;
	struct usb_interface *intf;
	int card_index;

	struct microbookii_control control;
	struct microbookii_pcm pcm;
};

void microbookii_dump_buffer(const char *prefix, const char *buf, int len);

#endif
