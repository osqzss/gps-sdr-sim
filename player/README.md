## Building

Modified to build on Linux.

```
$ make all
```
Will build all players if dependencies are met.

### Dependencies - bladeRF
#### libbladeRF
```
$ git clone https://github.com/Nuand/bladeRF.git
$ cd bladeRF
$ dpkg-buildpackage -b
```
Or Nuand has some build/install instructions including an Ubuntu PPA
at https://github.com/Nuand/bladeRF/wiki/Getting-Started:-Linux

#### Build

```
$ make bladeplayer
```

### Dependecies - hackRF
#### libhackrf

```
> git clone https://github.com/mossmann/hackrf.git
> mkdir hackrf/host/build
> cd hackrf/host/build
> cmake ..
> make
> sudo make install
> sudo ldconfig
```
Build instructions https://github.com/mossmann/hackrf/tree/master/host

#### Build

```
> make hackplayer
```

### Dependecies - lime

LimeSuite https://github.com/myriadrf/LimeSuite

Build instructions http://wiki.myriadrf.org/Lime_Suite

:exclamation: Build not tested.

### Dependecies - ADALM-Pluto
#### libiio

Use the latest version from Github.
```
$ git clone https://github.com/analogdevicesinc/libiio.git
$ cd libiio
$ cmake ./
$ make all
$ sudo make install
```
[How to build it in detail.](https://wiki.analog.com/resources/tools-software/linux-software/libiio)

#### libad9361

Use of the latest Github version mandatory.
```
$ git clone https://github.com/analogdevicesinc/libad9361-iio.git
$ cd libad9361-iio
$ cmake ./
$ make all
$ sudo make install
```

#### Build

```
$ make plutoplayer
```