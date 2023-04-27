# CISII_dvrk

## Description

This project aims to build a DaVinci-Assisted Continuum Robot Navigation and Manipulation system with a tendon-driven continuum robot end, based on the sawIntuitiveResearchKit repository available at https://github.com/jhu-dvrk/sawIntuitiveResearchKit. The goal is to improve the reachability and dexterity of the DaVinci robot arm.

## Acknowledgment

This project is derived from the sawIntuitiveResearchKit:

* Original Repository: https://github.com/jhu-dvrk/sawIntuitiveResearchKit
* Documentation: http://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki
* License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
* JHU-LCSR software: http://jhu-lcsr.github.io/software/

I would like to extend my special thanks to Anton Deguet, an expert in dVRK from Johns Hopkins University. Anton has maintained the dvrk repository and provided invaluable assistance to my team during the development of this project.

## Modifications

We have prototyped a dVRK-catheter and implemented navigation and manipulation of a continuum robot end on dVRK. More details can be found in `continuum-robot` folder.

We also implemented NDI Aurora EM tracking system on ROS for dVRK control. More details can be found in `NDI-emtracking` folder.


## Dependencies

* da Vinci Research Kit hardware: http://research.intusurg.com/dvrkwiki
* Linux only, ROS 1.0 (tested with ROS Noetic ): http://wiki.ros.org/noetic/Installation/Ubuntu
* cisst libraries: https://github.com/jhu-cisst/cisst
* sawRobotIO1394: http://github.com/jhu-saw/sawRobotIO1394
* sawControllers: http://github.com/jhu-saw/sawControllers
* NDI Aurora EM tracking system: https://www.ndigital.com/electromagnetic-tracking-technology/aurora/

## Installation

Detailed instructions on how to install and set up the modified project, including any additional requirements or dependencies, can be found in https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/FirstSteps.

## License

This project is licensed under the same terms as the original sawIntuitiveResearchKit repository. See [LICENSE](LICENSE) for more information.