# DistanceBounding-SITL-Simulation

This repository provides the new and modified files for adding ADSB-based 
DistanceBounding to the ArduPilot and Software-in-the-Loop environments.

## Research ##
The Automatic Dependent Surveillance-Broadcast
(ADS-B) protocol is being adopted for use in unmanned aerial vehicles
(UAVs) as the primary source of information for emerging
multi-UAV collision avoidance algorithms. The lack of security
features in ADS-B leaves any processes dependent upon the information
vulnerable to a variety of threats from compromised and
dishonest UAVs. This could result in substantial losses or damage
to properties. This research proposes a new distance-bounding
scheme for verifying the distance and flight trajectory in the
ADS-B broadcast data from surrounding UAVs. The proposed
scheme enables UAVs or ground stations to identify fraudulent
UAVs and avoid collisions. The scheme was implemented and
tested in the Ardupilot SITL (Software In The Loop) simulator
to verify its ability to detect fraudulent UAVs. The experiments
showed that the scheme achieved the desired accuracy in both
flight trajectory measurement and attack detection.

## Adding Files ##
- SIM_ADSB.cpp and SIM_ADSB.h must be added to the following directory: ~/ardupilot/libraries/SITL
- AP_ADSB directory must be replaced in the following directory: ~/ardupilot/libraries
- AP_DistanceBounding directory must be added in the following directory: ~/ardupilot/libraries
  - *Note: Update the absolute logging path in AP_DistanceBounding.h

## Building ##
- In the ardupilot directory use $ ./waf copter


## Developer Information ##

- Github repository: <https://github.com/ArduPilot/ardupilot>
- Main developer wiki: <http://dev.ardupilot.org>


## License ##

The ArduPilot and this project are licensed under the GNU General Public
License, version 3.

- [Overview of license](http://dev.ardupilot.com/wiki/license-gplv3)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)
