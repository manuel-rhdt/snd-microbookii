obj-m := snd-microbookii.o
snd-microbookii-objs := audio.o microbookii.o control.o midi.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
