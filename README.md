# GPS-SDR-SIM

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

### Using bigger user motion files

In order to use user motion files with more than 30000 samples (at 10Hz), the `USER_MOTION_SIZE`
variable can be set to the maximum time of the user motion file in seconds. It is advisable to do
this using make so gps-sdr-bin can update the size when needed. e.g:

```
$ make USER_MOTION_SIZE=4000
```

This variable can also be set when compiling directly with GCC:

```
$ gcc gpssim.c -lm -O3 -o gps-sdr-sim -DUSER_MOTION_SIZE=4000
```

### Generating the GPS signal file

A user-defined trajectory can be specified in either a CSV file, which contains 
the Earth-centered Earth-fixed (ECEF) user positions, or an NMEA GGA stream.
The sampling rate of the user motion has to be 10Hz.
The user is also able to assign a static location directly through the command line.

The user specifies the GPS satellite constellation through a GPS broadcast 
ephemeris file. The daily GPS broadcast ephemeris file (brdc) is a merge of the
individual site navigation files into one. The archive for the daily file can 
be downloaded from: https://cddis.nasa.gov/archive/gnss/data/daily/. Access 
to this site requires registration, which is free.

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
  -u <user_motion> User motion file in ECEF x, y, z format (dynamic mode)
  -x <user_motion> User motion file in lat, lon, height format (dynamic mode)
  -g <nmea_gga>    NMEA GGA stream (dynamic mode)
  -c <location>    ECEF X,Y,Z in meters (static mode) e.g. 3967283.15,1022538.18,4872414.48
  -l <location>    Lat,Lon,Hgt (static mode) e.g. 30.286502,120.032669,100
  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss
  -T <date,time>   Overwrite TOC and TOE to scenario start time
  -d <duration>    Duration [sec] (dynamic mode max: 300 static mode max: 86400)
  -o <output>      I/Q sampling data file (default: gpssim.bin ; use - for stdout)
  -s <frequency>   Sampling frequency [Hz] (default: 2600000)
  -b <iq_bits>     I/Q data format [1/8/16] (default: 16)
  -i               Disable ionospheric delay for spacecraft scenario
  -v               Show details about simulated channels
```

The user motion can be specified in either dynamic or static mode:

```
> gps-sdr-sim -e brdc3540.14n -u circle.csv
```

```
> gps-sdr-sim -e brdc3540.14n -x circle_llh.csv
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

You can also use `tx_samples_from_file` tool included in the UHD examples:
```
> tx_samples_from_file --file gpssim.bin --type short --rate 2500000 --freq 1575420000 --gain 0
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

Copyright &copy; 2015-2022 Takuji Ebinuma  
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php).
