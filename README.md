# relaxed_ik

<b> RelaxedIK Solver </b>

Welcome to RelaxedIK! This solver implements the methods discussed in our paper <i> RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion </i> (http://www.roboticsproceedings.org/rss14/p43.html)

Video of presentation at RSS 2018 (RelaxedIK part starts around 12:00) :
https://youtu.be/bih5e9MHc88?t=737

Video explaining relaxedIK
https://youtu.be/AhsQFJzB8WQ

RelaxedIK is an inverse kinematics (IK) solver designed for robot platforms such that the conversion
between Cartesian end-effector pose goals (such as "move the robot's right arm end-effector to position X, while maintaining an end-effector
orientation Y") to Joint-Space (i.e., the robot's rotation values for each joint degree-of-freedom at a particular time-point) is
done both ACCURATELY and FEASIBLY.  By this, we mean that RelaxedIK attempts to find the closest possible solution to the
desired end-effector pose goals without exhibiting negative effects such as self-collisions, environment collisions,
kinematic-singularities, or joint-space discontinuities.

To start using the solver, please follow the step-by-step instructions in the file start_here.py (in the root directory)

If anything with the solver is not working as expected, or if you have any feedback, feel free to let us know! (email: rakita@cs.wisc.edu, website: http://pages.cs.wisc.edu/~rakita)
We are actively supporting and extending this code, so we are interested to hear about how the solver is being used and any positive or negative experiences in using it.

<b> Citation </b>

If you use our solver, please cite our RSS paper RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion
http://www.roboticsproceedings.org/rss14/p43.html

<pre>
@INPROCEEDINGS{Rakita-RSS-18, 
    AUTHOR    = {Daniel Rakita AND Bilge Mutlu AND Michael Gleicher}, 
    TITLE     = {{RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion}}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2018}, 
    ADDRESS   = {Pittsburgh, Pennsylvania}, 
    MONTH     = {June}, 
    DOI       = {10.15607/RSS.2018.XIV.043} 
} 
</pre>

If you use our solver for a robot teleoperation interface, also consider citing our prior work that shows the effectiveness of RelaxedIK in this setting:


A Motion Retargeting Method for Effective Mimicry-based Teleoperation of Robot Arms
https://dl.acm.org/citation.cfm?id=3020254
<pre>
@inproceedings{rakita2017motion,
  title={A motion retargeting method for effective mimicry-based teleoperation of robot arms},
  author={Rakita, Daniel and Mutlu, Bilge and Gleicher, Michael},
  booktitle={Proceedings of the 2017 ACM/IEEE International Conference on Human-Robot Interaction},
  pages={361--370},
  year={2017},
  organization={ACM}
}
</pre>


An Autonomous Dynamic Camera Method for Effective Remote Teleoperation
https://dl.acm.org/citation.cfm?id=3171221.3171279
<pre>
@inproceedings{rakita2018autonomous,
  title={An autonomous dynamic camera method for effective remote teleoperation},
  author={Rakita, Daniel and Mutlu, Bilge and Gleicher, Michael},
  booktitle={Proceedings of the 2018 ACM/IEEE International Conference on Human-Robot Interaction},
  pages={325--333},
  year={2018},
  organization={ACM}
}

</pre>

<b> Dependencies </b>

kdl urdf parser:
<div> >> sudo apt-get install ros-[your ros distro]-urdfdom-py </div>
<div> >> sudo apt-get install ros-[your ros distro]-kdl-parser-py </div>
<div> >> sudo apt-get install ros-[your ros distro]-kdl-conversions </div> 

<br>

fcl collision library:
https://github.com/BerkeleyAutomation/python-fcl

<!--
boost: https://www.boost.org/doc/libs/1_67_0/more/getting_started/unix-variants.html
The boost c++ libraries are used to interface between c++ and python code in the solver.  The solver will look for boost library files in the directory /usr/local/lib/ (the default install directory); if the library files are not found, the solver will try to move on anyway using the default python implementation, though performance will be slower. (UPDATE: Boost implementations are not turned on in the current version, but these will be included in the next RelaxedIK update after some testing).
-->


scikit learn:
http://scikit-learn.org/stable/index.html


<b> Tutorial </b>

For full setup and usage details, please refer to start_here.py in the src directory.

<b> Coming Soon </b>

RelaxedIK has been rewritten in Julia to substantially boost performance. If you would like to try out this version of RelaxedIK before it is pushed to the main branch, feel free to try it out as part of a beta testing phase.  To use it, please clone the "dev" branch using the following commands: 

<pre> git clone -b dev https://github.com/uwgraphics/relaxed_ik.git </pre>

install the proper dependenices, and follow the new set of instructions in the start_here.py file.  

If you have feedback on your experience using the new version of the solver (positive or negative), please email me at rakita@cs.wisc.edu .  The code will be moved to the main branch after this testing phase.  Thanks!

====================================================================================================================
Development update 10/10/19

The RelaxedIK solver has presented two main options over the past six months: (1) For a stable, reliable, yet relatively slow version of the solver, use the python implementation on the main branch; or (2) for a less stable, less reliable, yet much faster version of the solver, use the Julia implementation on the dev branch. My goal was always to iterate on the Julia version to become just as stable and reliable as its Python counterpart, and eventually unite these two camps into one on the main branch with the Julia version as the centerpiece.

However, there are a few idiosyncrasies that come along with the Julia programming language that have precluded those plans. First, the Julia version of relaxedIK takes at least a minute to do its JIT compilation and start from scratch each time. This is unacceptable for a stable release of a solver that should be convenient and lightweight to use. Plus, it makes development on top of the base solver incredibly tedious, since each little change requires this full JIT compilation. While the people developing the Julia programming language seem to be well aware of this issue for large projects developed in the language, and they may have a sufficient solution for this problem down the road, just waiting around and hoping this gets fixed does not seem like a good option. Second, while ros is somewhat supported in julia through RobotOS.jl, this library leads to mysterious errors too often to be the foundation of a stable release. With both of these issues in mind, I'd rather be proactive and just turn the page away from Julia and plan future development elsewhere.

So, over the next couple of months, I will be re-implementing relaxedIK in a fully statically compiled language (still TBD). Given recent testing, I anticipate the new solver will be even faster than the Julia version, without the JIT compilation hassle and stability issues. Everything will remain ROS compatible. The Julia and python versions will remain in the new release for backwards compatibility, but I anticipate new features will just be officially added and supported in the new language.

Along with the new language, I plan to add in a few other features as part of a "major" new release, such as better support for fast, standard IK solver options. The tools that make relaxedIK actually afford incredibly fast standard IK solver options (i.e., "standard" meaning just hitting end-effector pose goals accurately, without the motion continuity, collision avoidance, and other feasibility considerations included in relaxedIK). We've used these "standard" IK options in our prior work and, in our testing, it is faster and more reliable than other numerical IK solvers we have tried. I plan to make these features more publicly accessible and well documented.

If you have any comments or questions on any of this, feel free to use the iusse thread titled "Development update" to foster conversation. Or, if you prefer, feel free to email me directly at rakita@cs.wisc.edu



<b> Change Log </b>

Version 1.1 (8/16/18): added support for prismatic and fixed joints


