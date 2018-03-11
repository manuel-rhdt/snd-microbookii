#ifndef CONTROL_H
#define CONTROL_H

#define MSG_BUF_LEN 64

#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/completion.h>

struct microbookii;

struct microbookii_message {
	struct microbookii *mbii;
	struct urb *urb;
	unsigned char message_num;
	char *buffer;
	int len;
	struct completion responded;
};

struct microbookii_control {
	struct microbookii *mbii;

	unsigned char message_counter;
	struct mutex message_lock;
	struct microbookii_message msg;
	
	struct completion device_setup;
	unsigned int current_rate;
};

int microbookii_init_control(struct microbookii *mbii);

void microbookii_free_control(struct microbookii *mbii);

struct microbookii_message *microbookii_message_get(struct microbookii *mbii);
void microbookii_message_put(struct microbookii_message *msg);
int microbookii_message_send(struct microbookii *mbii, struct microbookii_message *msg);

int microbookii_poll_device_ready(struct microbookii *mbii);
int microbookii_set_rate(struct microbookii *mbii, unsigned int rate);

#endif
