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
  - NPI and NPISUM count modes.
  - NOTE: These count modes need to be tested before R2-2 is released
* R2-2 needs to be tested on older PIXI-II chip models to make sure it still works
* Documentation needs to be updated for new CountMode parameter
* Need to document that 1 Color Low is now the counts between Thresh1 and Thresh2

R2-1 (16-April-2015)
----
* Changes for compatibility with ADCore R2-2.


R2-0 (4-April-2014)
----
* New driver added in this release.
