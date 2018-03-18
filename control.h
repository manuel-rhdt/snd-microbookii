#ifndef __MICROBOOKII_CONTROL_H
#define __MICROBOOKII_CONTROL_H

#define MSG_BUF_LEN 64
#define MICROBOOKII_POll_MS 100

#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include <sound/control.h>

struct microbookii;
struct microbookii_message;

typedef void (*message_callback_t)(struct microbookii_message *msg);

struct microbookii_message {
	struct microbookii *mbii;
	struct urb *urb;
	unsigned char message_num;
	int len;
	struct completion responded;
	struct work_struct work;
	message_callback_t callback;
	unsigned char buffer[MSG_BUF_LEN];
};

struct microbookii_control {
	struct microbookii *mbii;

	unsigned char message_counter;
	struct workqueue_struct *message_queue;

        struct hrtimer poll_timer;

	struct completion device_setup;
	unsigned int current_rate;

        struct snd_kcontrol *master_volume;
        struct snd_kcontrol *phones_volume;
};

int microbookii_init_control(struct microbookii *mbii);
void microbookii_free_control(struct microbookii *mbii);

struct microbookii_message *microbookii_message_alloc(struct microbookii *mbii);
void microbookii_message_free(struct microbookii_message *msg);

int microbookii_message_send(struct microbookii_message *msg);
int microbookii_message_send_async(struct microbookii_message *msg,
				   message_callback_t callback);

int microbookii_wait_device_ready(struct microbookii *mbii);
int microbookii_set_rate(struct microbookii *mbii, unsigned int rate);

#endif	/* __MICROBOOKII_CONTROL_H */
