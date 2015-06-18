# GPS-SDR-SIM

GPS-SDR-SIM generates GPS baseband signal data streams, which can be converted 
to RF using software-defined radio (SDR) platforms, such as 
[bladeRF](http://nuand.com/), HackRF, and USRP.

### Windows build instructions

1. Start Visual Studio.
2. Create an empty project for a console application.
3. On the Solution Explorer at right, add "gpssim.c" to the Souce Files folder.
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

A user-defined trajectory can be defined in a CSV file, which contains 
the Earth-centered Earth-fixed (ECEF) user positions at 10Hz.

The user specifies the GPS satellite constellation through a GPS broadcast 
ephemeris file. The daily GPS broadcast ephemers file (brdc) is a merge of the
indiviual site navigation files into one. The archive for the daily file is:

[ftp://cddis.gsfc.nasa.gov/gnss/data/daily/](ftp://cddis.gsfc.nasa.gov/gnss/data/daily/)

These files are then used to generate the simulated pseudorante and
Doppler for the GPS satellites in view. This simulated range data is 
then used to generate the digitized I/Q samples for the GPS signal.
For example;

```
> gps-sdr-sim brdc3540.14n circle.csv gpssim.bin
```

### Transmitting the samples with bladeRF

The TX port of the bladeRF is connected to the GPS receiver under
test through a DC block and a fixed 50dB attenuator.

The simulated GPS signal file, named "gpssim.bin", can be loaded
into bladeRF for playback as shown below:

```
set frequency 1575.42M
set samplerate 4M
set bandwidth 2.5M
set txvga1 -25
cal lms
cal dc tx
tx config file=gpssim.bin format=bin
tx start
```

### License

Copyright &copy; 2015 Takuji Ebinuma  
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php).
