#ifndef CONTROL_H
#define CONTROL_H

struct microbookii;

struct microbookii_control {
	struct microbookii *bcd2k;

	bool phono_mic_switch;
};

int microbookii_init_control(struct microbookii *bcd2k);
void microbookii_free_control(struct microbookii *bcd2k);

#endif