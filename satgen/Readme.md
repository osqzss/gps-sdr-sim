# NMEA2UM

NMEA2UM generates the ECEF user motion file for GPS-SDR-SIM from
NMEA GGA data simulated by the free GPS NMEA simulation software
from LabSat:

[http://www.labsat.co.uk/index.php/en/free-gps-nmea-simulator-software](http://www.labsat.co.uk/index.php/en/free-gps-nmea-simulator-software)

### Usage

```
nmea2um <nmea_gga> <user_motion>
```

### Instructions

1. Sketch out a route in Google Earth.
2. Save the path as a KML file.
3. Load the KML file in SatGen.
4. Set the output rate at 10Hz and generate an NMEA file.
5. Convert the NMEA data to the user motion CSV format.
