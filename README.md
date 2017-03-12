snd-microbookii
===============

Linux driver for the Motu Microbook II.

Usage:

* Either assure that kernel sources are available under:
  <tt>/lib/modules/$(shell uname -r)/build</tt>
  and execute "make" in the snd-microbookii folder, _or_
  use DKMS to build the kernel (see [Ubuntu DKMS](https://help.ubuntu.com/community/DKMS)).
* Either directly load the module using ```insmod snd-microbookii.ko``` or use ```modprobe snd-microbookii```  
  if the module has been installed into the kernel's module tree.

If it doesn't work:

* Make sure you have all build tools, e.g., gcc and make, and the kernel headers installed.
  For Ubuntu: ```apt-get install build-essential linux-headers```
