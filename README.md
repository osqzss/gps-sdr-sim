# Cumstomized GPS-SDR-SIM For Kolmostar Users

This repo is forked from [osqzss/gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim) and customized for Kolmostar users' usage. It takes GPS navigation data as input and generate GPS baseband data streams in Kolmostar's 4-bits format.

## Quick Start For Kolmostar Users

In this section you'll see how a Kolmostar user shold use this tool step by step.

### Download Navigation Data

You can download and uncompress navigation data from NASA's ftp by executing the following command. Please change '2018' to the year you want and '001' to day of year.

```shell
wget ftp://cddis.gsfc.nasa.gov/pub/gps/data/daily/2018/brdc/brdc0010.18n.Z
zcat brdc0010.18n.Z > brdc0010.18n
```

### Build Code

To build this project, please run

```shell
gcc gpssim.c -lm -O3 -o gps-sdr-sim
```

### Generate Signal

To use this tool, please run

```shell
./gps-sdr-sim -e brdc0010.18n  -l 39.98,116.35,50 -o data_capture_interval_0__.bin -d 30 -s 4092000 -b 4
```

Explanation for arguments:

- -e: RINEX navigation file for GPS ephemerides (required)
- -l: Lattitude, longitude and height for the location where your want to simulate.
- -o: Output file path. Use format 'data_capture_interval_xx__.bin' and file name so that is can be recognized by simple-scan.
- -d: Duration of simulation in seconds. As the output file grows fast, we strongly recommend you not to choose a number that is too large.
- -s: Frequency(Hz) of signal. 1M in simple-scan is not 1024*1024 or 1000000, as usual, but 1023000 actually. Thus we have two valid values for simple-scan's gps_4M_32M_DataManager: 4092000 for 4M and 32736000 for 32M.
- -b: I/Q data format. We have four valid options '1', '4', '8', '16'. Option '4', which is the default value, stands for Kolmo format(2bits I and 2 bits Q).

To learn more about all the arguments, please run

```shell
./gps-sdr-sim -h
```

### Simple-Scan Output

The output file 'data_capture_interval_0__.bin' can now be recoginized by simple-scan. Personal suggestion is to change simple-scan's parameter 'dbhz_thresh' to 40, since the noise's signal strength can easily reach 38dbHz or higher.

## Signal Convert

Signal representations for I/Q data are different between GPS-SDR-SIM and simple-scan. GPS-SDR-SIM uses 12-bits unsigned number while simple-scan uses 2bits unsigned number. Therefore we can get the  reflection rules to convert signal from GPS-SDR-SIM to simple-scan

- 0xFFF~0xC00 => 01
- 0xBFF~0x800 => 00
- 0x7FF~0x400 => 11
- 0x3FF~0x000 => 10

====================================================

# Original Readme

GPS-SDR-SIM generates GPS baseband signal data streams, which can be converted 
to RF using software-defined radio (SDR) platforms, such as 
[ADALM-Pluto](https://wiki.analog.com/university/tools/pluto), [bladeRF](http://nuand.com/), [HackRF](https://github.com/mossmann/hackrf/wiki), and [USRP](http://www.ettus.com/).

### Windows build instructions

1. Start Visual Studio.
2. Create an empty project for a console application.
3. On the Solution Explorer at right, add "gpssim.c" and "getopt.c" to the Souce Files folder.
4. Select "Release" in Solution Configurations drop-down list.
5. Build the solution.

### Building with GCC

```
$ gcc gpssim.c -lm -O3 -o gps-sdr-sim
```

### Generating the GPS signal file

A user-defined trajectory can be specified in either a CSV file, which contains 
the Earth-centered Earth-fixed (ECEF) user positions, or an NMEA GGA stream.
The sampling rate of the user motion has to be 10Hz.
The user is also able to assign a static location directly through the command line.

The user specifies the GPS satellite constellation through a GPS broadcast 
ephemeris file. The daily GPS broadcast ephemeris file (brdc) is a merge of the
individual site navigation files into one. The archive for the daily file is:

[ftp://cddis.gsfc.nasa.gov/gnss/data/daily/](ftp://cddis.gsfc.nasa.gov/gnss/data/daily/)

These files are then used to generate the simulated pseudorange and
Doppler for the GPS satellites in view. This simulated range data is 
then used to generate the digitized I/Q samples for the GPS signal.

The bladeRF and ADALM-Pluto command line interface requires I/Q pairs stored as signed 
16-bit integers, while the hackrf_transfer and gps-sdr-sim-uhd.py
support signed bytes.

HackRF, bladeRF and ADALM-Pluto require 2.6 MHz sample rate, while the USRP2 requires
2.5 MHz (an even integral decimator of 100 MHz).

The simulation start time can be specified if the corresponding set of ephemerides
is available. Otherwise the first time of ephemeris in the RINEX navigation file
is selected.

The maximum simulation duration time is defined by USER_MOTION_SIZE to 
prevent the output file from getting too large.

The output file size can be reduced by using "-b 1" option to store 
four 1-bit I/Q samples into a single byte. 
You can use [bladeplayer](https://github.com/osqzss/gps-sdr-sim/tree/master/player)
for bladeRF to playback the compressed file.

```
Usage: gps-sdr-sim [options]
Options:
  -e <gps_nav>     RINEX navigation file for GPS ephemerides (required)
  -u <user_motion> User motion file (dynamic mode)
  -g <nmea_gga>    NMEA GGA stream (dynamic mode)
  -c <location>    ECEF X,Y,Z in meters (static mode) e.g. 3967283.15,1022538.18,4872414.48
  -l <location>    Lat,Lon,Hgt (static mode) e.g. 30.286502,120.032669,100
  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss
  -T <date,time>   Overwrite TOC and TOE to scenario start time
  -d <duration>    Duration [sec] (dynamic mode max: 300 static mode max: 86400)
  -o <output>      I/Q sampling data file (default: gpssim.bin ; use - for stdout)
  -s <frequency>   Sampling frequency [Hz] (default: 2600000)
  -b <iq_bits>     I/Q data format [1/4/8/16] (default: 4 which is Kolmo format)
  -i               Disable ionospheric delay for spacecraft scenario
  -v               Show details about simulated channels
```

The user motion can be specified in either dynamic or static mode:

```
> gps-sdr-sim -e brdc3540.14n -u circle.csv
```

```
> gps-sdr-sim -e brdc3540.14n -g triumphv3.txt
```

```
> gps-sdr-sim -e brdc3540.14n -l 30.286502,120.032669,100
```

### Transmitting the samples

The TX port of a particular SDR platform is connected to the GPS receiver 
under test through a DC block and a fixed 50-60dB attenuator.

#### BladeRF:

The simulated GPS signal file, named "gpssim.bin", can be loaded
into the bladeRF for playback as shown below:

```
set frequency 1575.42M
set samplerate 2.6M
set bandwidth 2.5M
set txvga1 -25
cal lms
cal dc tx
tx config file=gpssim.bin format=bin
tx start
```

You can also execute these commands via the `bladeRF-cli` script option as below:
```
> bladeRF-cli -s bladerf.script
```

#### HackRF:

```
> hackrf_transfer -t gpssim.bin -f 1575420000 -s 2600000 -a 1 -x 0
```

#### UHD supported devices (tested with USRP2 only):

```
> gps-sdr-sim-uhd.py -t gpssim.bin -s 2500000 -x 0
```

#### LimeSDR (in case of 1 Msps 1-bit file, to get full BaseBand dynamic and low RF power):

```
> limeplayer -s 1000000 -b 1 -d 2047 -g 0.1 < ../circle.1b.1M.bin
```

#### ADALM-Pluto (PlutoSDR):

The ADALM-Pluto device is expected to have its network interface up and running and is accessible
via "pluto.local" by default.

Default settings:
```
> plutoplayer -t gpssim.bin
```
Set TX attenuation:
```
> plutoplayer -t gpssim.bin -a -30.0
```
Default -20.0dB. Applicable range 0.0dB to -80.0dB in 0.25dB steps.

Set RF bandwidth:
```
> plutoplayer -t gpssim.bin -b 3.0
```
Default 3.0MHz. Applicable range 1.0MHz to 5.0MHz.

### License

Copyright &copy; 2015-2018 Takuji Ebinuma  
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php).
