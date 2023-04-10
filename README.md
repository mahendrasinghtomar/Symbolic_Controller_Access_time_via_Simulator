# Symbolic_Controller_Access_time_via_Simulator
Time to access symbolic control input via [Scarab](https://github.com/kofyou/scarab) (a microarchitecture simulator)



Thanks to [Mingsheng Xu](https://kofyou.github.io) for CMakeLists.txt which is used to run start/stop API of DynamoRio.

* For a 3 dimensional kinematic vehicle model, symbolic controller for reachability designed using [SCOTSv0.2](https://github.com/mahendrasinghtomar/SCOTSv0.2_Copy). 
* SCOTSv0.2 saves symbolic controller, in the form of a lookup table, in a '.scs' file. To get a controller of size greater than 1 GB, multiple adjacent scenarios with randomised location of obstacles are created via [vehicle.cc](). The number of scenarios is controlled by ...
* ...
