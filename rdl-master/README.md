![build status](https://gitlab.com/jlack/rdl/badges/master/pipeline.svg?ignore_skipped=true)
![coverage report](https://gitlab.com/jlack/rdl/badges/master/coverage.svg?ignore_skipped=true)

# [RDL API Documentation](https://jlack.gitlab.io/rdl/doxygen)
# [Code Coverage](https://jlack.gitlab.io/rdl/coverage)

# Disclaimer

RDL is a dynamics library that is a derivative of Rigid Body Dynamics Library(RBDL) written, owned, and copyrighted by Martin Felis. I, Jordan Lack, do not 
wish to misrepresent any of the works contained within this project that were not originally created by me as my own in any way.

The original work from which RDL is based upon can be found [here](https://github.com/rbdl/rbdl).

## RDL - Robot Dynamics Library

RDL is a c++ library derived from Rigid Body Dynamics Library(RBDL) for computing dynamics and kinematics of rigid body systems, however, kinematics 
in RDL are handled differently than in the original work of RBDL. In RDL, a number of spatial objects(SpatialForce, SpatialMotion, SpatialInertia, FramePoint, FrameVector, etc) are 
provided that, when used, provide runtime checks that ensure rules on the frames these objects are expressed in are obeyed.

## Licensing

The library is published under the very permissive zlib free software
license which should allow you to use the software wherever you need. 
See the LICENSE file for the detailed language and permissions of the 
zlib license. The License file for the original work from which RDL 
is a derivative can be found in the file RBDL_LICENSE.

## Credits

First and foremost I would like to give due credit to Martin Felis, the creator of RBDL of which this work is a derivative. 
Additionally, RBDL is a fantastic library, and should you find that RDL doesn't meet your needs you may consider giving RBDL a try. It may better fit your needs/application. 

I would also like to acknowledge the folks at the [IHMC Robotics Lab](http://robots.ihmc.us/). Their dynamics libary(which is written in Java) provides runtime frame checks, and after 
working with them and seeing first hand how it enables very quick development of control algorithms and helps prevent a lot of bugs I knew the world of c++ needed something similar. If you are interested in writing code for robots and Java is your thing then definitely check out IHMC's open source stuff [here](https://github.com/ihmcrobotics). 
