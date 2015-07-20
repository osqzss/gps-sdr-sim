# GPS-SDR-SIM

GPS-SDR-SIM generates GPS baseband signal data streams, which can be converted 
to RF using software-defined radio (SDR) platforms, such as 
[bladeRF](http://nuand.com/), [HackRF](https://github.com/mossmann/hackrf/wiki), and USRP.

### Windows build instructions

1. Start Visual Studio.
2. Create an empty project for a console application.
3. On the Solution Explorer at right, add "gpssim.c" and "getopt.c" to the Souce Files folder.
4. Select "Release" in Solution Configurations drop-down list.
5. Open the Property Pages dialog box and expand the Configuration Properties.
6. Expand the C/C++ node and select the Language property page.
7. Enable the OpenMP Support (/openmp).
8. Build the solution.

### Building with GCC

```
$ gcc gpssim.c -lm -fopenmp -o gps-sdr-sim
```

### Generating the GPS signal file

A user-defined trajectory can be specified in a CSV file, which contains 
the Earth-centered Earth-fixed (ECEF) user positions at 10Hz.
The user is also able to assign a static location directly through the command line.

The user specifies the GPS satellite constellation through a GPS broadcast 
ephemeris file. The daily GPS broadcast ephemers file (brdc) is a merge of the
indiviual site navigation files into one. The archive for the daily file is:

[ftp://cddis.gsfc.nasa.gov/gnss/data/daily/](ftp://cddis.gsfc.nasa.gov/gnss/data/daily/)

These files are then used to generate the simulated pseudorange and
Doppler for the GPS satellites in view. This simulated range data is 
then used to generate the digitized I/Q samples for the GPS signal.

The bladeRF command line interface requires I/Q pairs stored as signed 
16-bit integers, while the hackrf_transfer supports signed bytes.

```
Usage: gps-sdr-sim [options]
Options:
  -e <gps_nav>     RINEX navigation file for GPS ephemerides (required)
  -u <user_motion> User motion file (dynamic mode)
  -l <location>    Lat,Lon,Hgt (static mode) e.g. 30.286502,120.032669,100
  -o <output>      I/Q sampling data file (default: gpssim.bin)
  -s <frequency>   Sampling frequency [Hz] (default: 2600000)
  -b <iq_bits>     I/Q data format [8/16] (default: 8)
```

The user motion can be specified in either dynamic or static mode:

```
> gps-sdr-sim -e brdc3540.14n -u circle.csv -b 16
```

```
> gps-sdr-sim -e brdc3540.14n -l 30.286502,120.032669,100 -b 16
```

### Transmitting the samples

The TX port of a particular SDR platform is connected to the GPS receiver 
under test through a DC block and a fixed 50-60dB attenuator.

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

Also, you can execute those commandline below as `bladeRF-cli` script:

```
> bladeRF-cli -s bladerf.script
```

For the HackRF:

```
> hackrf_transfer -t gpssim.bin -f 1575420000 -s 2600000 -a 1 -x 0
```

### License

Copyright &copy; 2015 Takuji Ebinuma  
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php).
