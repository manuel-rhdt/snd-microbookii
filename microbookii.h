#ifndef BCD2000_H
#define BCD2000_H

#include <linux/usb.h>
#include <linux/usb/audio.h>
#include <sound/core.h>
#include <sound/initval.h>

#define DEVICENAME "MicrobookII"
#define PREFIX "snd-microbookii: "

#include "audio.h"
#include "control.h"
#include "midi.h"

struct microbookii {
	struct usb_device *dev;
	struct snd_card *card;
	struct usb_interface *intf;
	int card_index;

	struct microbookii_midi midi;
	struct microbookii_pcm pcm;
	struct microbookii_control control;
};

void microbookii_dump_buffer(const char *prefix, const char *buf, int len);

#endif
