# relaxed_ik
RelaxedIK Solver

Welcome to RelaxedIK! RelaxedIK is an inverse kinematics (IK) solver designed for robot platforms such that the conversion
between Cartesian end-effector pose goals (such as "move the robot's right arm end-effector to position X, while maintaining an end-effector
orientation Y") to Joint-Space (i.e., the robot's rotation values for each joint degree-of-freedom at a particular time-point) is
done both ACCURATELY and FEASIBLY.  By this, we mean that RelaxedIK attempts to find the closest possible solution to the
desired end-effector pose goals without exhibiting negative effects such as self-collisions, environment collisions,
kinematic-singularities, or joint-space discontinuities.

To start using the solver, follow the step-by-step instructions in the file start_here.py (in the root directory)

If anything with the solver is not working as expected, or if you have any feedback, feel free to let us know! (rakita@cs.wisc.edu)
We are actively supporting and extending this code, so we are interested to hear about how the solver is being used and any positive or negative experiences in using it.


Dependencies: 

KDL urdf parser - 
  sudo apt-get install ros-[your ros distro]-urdfdom-py
  sudo apt-get install ros-[your ros distro]-kdl-parser-py
  sudo apt-get install ros-[your ros distro]-kdl-conversions
  
FCL collision library - 
  (install from source)
  https://github.com/BerkeleyAutomation/python-fcl
  
Boost - 
  https://www.boost.org/doc/libs/1_67_0/more/getting_started/unix-variants.html
  NOTE: RelaxedIK uses boost python and boost numpy to speed up parts of the solver.  To benefit from these speed gains, make 
   sure these are properly configured on your system (the solver will try to find your boost lib files in /usr/local/lib, which    is the default install location if configured correctly.  If boost is not on your system, the solver will still try to run      with a base python implementation.  However, even if this works, the solver will be considerably slower.
   
Keras - 
   https://github.com/keras-team/keras
   NOTE: for best collision-avoidance deep learning performance (both during learning as well as during the feed-forward pass
   at run-time), make sure keras is running on your GPU if your system supports that.
   
