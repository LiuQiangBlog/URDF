/** \file Mainpage.h
 * \mainpage Mainpage
 * \image html logo/rdl_logo.png
 *
 * This is the documentation of RDL, the Robot Dynamics Library, a derivative work of <a href="https://rbdl.bitbucket.io/?">Rigid Body Dynamics Library</a>. The
 * library contains code for both forward and inverse dynamics for kinematic chains and branched models, but it's principle feature is the way kinematics
 * are handled so that runtime checks are made that ensure rules involving reference frames are obeyed. Additionally, the process of changing
 * the reference frame in which a geometric object is expressed in is simple and clear.

 * One of the key goals has been to keep the API and overall functionality as close to the
 * original RBDL as possible(at least as of the day RDL diverged from RBDL), but to only modify the
 * way kinematics were handled such that it was more obvious in what reference frame geometric objects
 * are expressed.
 *
 * The original work, RBDL, was developed by <a
 * href="mailto:martin.felis@iwr.uni-heidelberg.de">Martin Felis
 * <martin.felis@iwr.uni-heidelberg.de></a> at the research group <a
 * href="http://orb.iwr.uni-heidelberg.de/">Optimization in Robotics and
 * Biomechanics (ORB)</a> of the <a
 * href="http://www.iwr.uni-heidelberg.de"> Interdisciplinary Center for
 * Scientific Computing (IWR)</a> at <a
 * href="http://www.uni-heidelberg.de">Heidelberg University</a>.

 * The modifications were made by <a href="mailto:jlack1987@gmail.com">Jordan Lack
 * <jlack1987@gmail.com></a>
 * The modifications were inspired by the dynamics library created by <a href="http://robots.ihmc.us/">Jerry Pratt and the IHMC Robotics Group</a>.
 * Fortunately for those of you who want to work with robots and love Java, they have open sourced much of their software, and you may
 * find it <a href="https://github.com/ihmcrobotics/ihmc-open-robotics-software">here</a>.
 * As is true with RBDL, The code is heavily inspired by the pseudo code of the book "Rigid Body Dynamics
 * Algorithms" of <a href="http://royfeatherstone.org" target="_parent">Roy
 * Featherstone</a>. Another useful book from which inspiration was drawn is "Port-Based Modeling And Control For Efficient
 * Bipedal Walking Robots" by <a href="https://sites.google.com/site/vincentduindam/" target="_parent">Vincent Duindam</a>.
 *
 * \section ModuleOverview API reference separated by functional modules
 *
 * \li \subpage modeling_page
 * \li \subpage reference_frame_page
 * \li \subpage joint_description
 * \li \subpage kinematics_page
 * \li \subpage dynamics_page
 * \li \subpage contacts_page
 * \li \subpage utils_page
 * \li \subpage urdfreader_page
 *
 *
 * \section Examples Examples
 *
 * For some simple examples, see the \subpage examples_page
 *
 * \section Licensing Licensing
 *
 * The original libary, RBDL, as well as the modifications are
 * published under the very permissive zlib free software
 * license which should allow you to use the software wherever you need.
 * The full license text can be found in the LICENCE file in the root
 * directory of the RDL repo.
 */
