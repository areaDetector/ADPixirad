ADPixirad Releases
==================

The latest untagged master branch can be obtained at
https://github.com/areaDetector/ADPixirad.

Tagged source code releases from R2-0 onward can be obtained at 
https://github.com/areaDetector/ADPixirad/releases.

Tagged prebuilt binaries from R2-0 onward can be obtained at
http://cars.uchicago.edu/software/pub/ADPixirad.

The versions of EPICS base, asyn, and other synApps modules used for each release can be obtained from 
the EXAMPLE_RELEASE_PATHS.local, EXAMPLE_RELEASE_LIBS.local, and EXAMPLE_RELEASE_PRODS.local
files respectively, in the configure/ directory of the appropriate release of the 
[top-level areaDetector](https://github.com/areaDetector/areaDetector) repository.


Release Notes
=============
R2-2 (XXX-May-2018)
* Added support for the PIXI-III chip
  - New CountMode record to select Normal, NPI and NPISUM count modes.
  - New HitThreshold record to control the VtHit value.
  - Added new pixiradAutoCal iocsh command to set the 7 calibration values unique to each PIII detector.
* Changed AUTOCAL so that it does not read back the calibration images.
  These were not useful, added complexity, and could be confusing to the user.
* TODO before releasing R2-2  
  - Test that older PIXI-II chip models still work

R2-1 (16-April-2015)
----
* Changes for compatibility with ADCore R2-2.


R2-0 (4-April-2014)
----
* New driver added in this release.
