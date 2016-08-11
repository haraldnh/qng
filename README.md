# Linux driver for ComScire QNG device

The ComScire Quantum Noise Generator is a random number generator from
back in the late '90s.  It attaches to the parallel port, and produces
around 2kB of random data per second.

While not all that fast, it is one of the highest quality random
sources that have been available.  There is a slight bias giving
marginally more 1s than 0s, but I know of no other patterns found.

It may be neccessary to remove the lp kernel module before loading
this one, it requires exclusive access to the parallel port.  When
loaded, random numbers can be read from /dev/qng0.
