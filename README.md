# relaxed_ik
<b> RelaxedIK Solver </b>

Welcome to RelaxedIK! RelaxedIK is an inverse kinematics (IK) solver designed for robot platforms such that the conversion
between Cartesian end-effector pose goals (such as "move the robot's right arm end-effector to position X, while maintaining an end-effector
orientation Y") to Joint-Space (i.e., the robot's rotation values for each joint degree-of-freedom at a particular time-point) is
done both ACCURATELY and FEASIBLY.  By this, we mean that RelaxedIK attempts to find the closest possible solution to the
desired end-effector pose goals without exhibiting negative effects such as self-collisions, environment collisions,
kinematic-singularities, or joint-space discontinuities.

To start using the solver, please follow the step-by-step instructions in the file start_here.py (in the root directory)

If anything with the solver is not working as expected, or if you have any feedback, feel free to let us know! (rakita@cs.wisc.edu)
We are actively supporting and extending this code, so we are interested to hear about how the solver is being used and any positive or negative experiences in using it.

<b> Dependencies </b>

kdl urdf parser:
<div> >> sudo apt-get install ros-[your ros distro]-urdfdom-py </div>
<div> >> sudo apt-get install ros-[your ros distro]-kdl-parser-py </div>
<div> >> sudo apt-get install ros-[your ros distro]-kdl-conversions </div> 

<br>

fcl collision library:
https://github.com/BerkeleyAutomation/python-fcl

boost: https://www.boost.org/doc/libs/1_67_0/more/getting_started/unix-variants.html
The boost c++ libraries are used to interface between c++ and python code in the solver.  The solver will look for boost library files in the directory /usr/local/lib/ (the default install directory); if the library files are not found, the solver will try to move on anyway using the default python implementation, though performance will be slower. (UPDATE: Boost implementations are not turned on in the current version, but these will be included in the next RelaxedIK update after some testing).


scikit learn:
http://scikit-learn.org/stable/index.html



