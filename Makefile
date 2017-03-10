obj-m := snd-microbookii.o
snd-microbookii-objs := audio.o control.o microbookii.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
