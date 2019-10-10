# relaxed_ik

<b> RelaxedIK Solver </b>

DEVELOPMENT BRANCH

<b> NOTE: If you have any problems setting up the solver with your robot, let me know!  I've set up many robots from kinova, UR, Kuka, Rethink, Rainbow, ABB, etc. in the solver, so chances are I could quickly ease the setup process for you if you encounter issues.  </b>

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
<pre> sudo apt-get install ros-[your ros distro]-urdfdom-py </pre>
<pre> sudo apt-get install ros-[your ros distro]-kdl-parser-py </pre>
<pre> sudo apt-get install ros-[your ros distro]-kdl-conversions </pre> 

readchar:
<pre> sudo pip install readchar </pre>

fcl collision library:
https://github.com/BerkeleyAutomation/python-fcl
<pre> sudo pip install python-fcl </pre>

scikit learn:
http://scikit-learn.org/stable/index.html
<pre> sudo pip install scikit-learn </pre>

scipy:
<pre> sudo pip install scipy </pre>

Pyyaml:
<pre> sudo pip install PyYaml </pre>

Lastly, update your version of numpy:
<pre> sudo pip install --upgrade numpy </pre>

<br>

To use the Julia version of the solver (which is the recommended option), you will first need to install Julia.
https://julialang.org/

The solver was written in Julia 1.0.2, though any more recent 1.X.X version should suffice.

To install Julia, follow these simple, yet intentionally verbose, steps:
1. Go to the downloads page here: https://julialang.org/downloads/
2. Scroll down to the "Current Stable Release" section, find the "Generic Linux Binaries for [x86/ARM]" row that is correct for you (if you're unsure, it's most likely x86), then click either 32-bit or 64-bit depending on your OS configuration.  A prompt will come up asking where you want to save the download.  Anywhere convenient (like the desktop) is fine.
3. Right click the downloaded folder, and click "extract here"
4. Move the extracted folder to a permanent location of your choice.  I usually just keep it in my "home" folder.
5. Update your Path variable such that your system can find and start up Julia.  There are a couple of ways to do this; my preferred way is to update the /etc/environment file.  To do this, open the /etc/environment file in your favorite text editor (vim, emacs, etc) with sudo privilege.  I use vim, so for me the command would read "sudo vim /etc/environment".  You'll probably get a prompt to enter your password.  Next, use the text editor to append ":/[your path to your julia bin folder]" to the path.  For example, at the time of writing this, my system path line ends with ":/home/rakita/julia-1.1.0/bin".  Save the file and exit.
6. Restart your computer.  And you're done!  To make sure this worked, open up a terminal and type the command "julia".  A Julia environment should initialize.

If you plan to extend any of the Julia code, we recommend using the Juno IDE (not required)

Once Julia is installed, initialize a Julia environment using the following command:
<pre> julia </pre>

then run the following commands to install Julia dependencies for RelaxedIK:
<pre> using Pkg </pre>
<pre> Pkg.add(["YAML", "BenchmarkTools", "ForwardDiff", "Calculus", "ReverseDiff", "StaticArrays", "Rotations", "Flux", "BSON", "NLopt", "Knet", "Random", "RobotOS", "Distributions", "PyCall", "Dates", "LinearAlgebra", "Zygote", "Distances"]) </pre>

<b> NOTE: Knet version 1.2.5 appears to cause an error during preprocessing.  I will look into permanently addressing this issue by updating RelaxedIK to be compatible with this new version or potentially switching to Flux.  In the meantime, please revert to Knet version 1.2.4.  </b>

in the same Julia session, configure PyCall within Julia by running the following commands:
<pre> ENV["PYTHON"] = "/usr/bin/python2.7" </pre>
<pre> Pkg.build("PyCall") </pre>

<b> Tutorial </b>

For full setup and usage details, please refer to start_here.py in the src directory.  Prior to starting the setup process, please ensure that you have installed all dependencies listed above.

<b> Usage Notes </b>

If you are using the Julia version of the RelaxedIK solver, note that it currently takes a little while (~30 seconds) for the solver to start.  Julia code is VERY fast once it starts up, but it takes a bit of overhead time to do its JIT compilation.

Unfortunately, there's little we can do at this point to eliminate this JIT compilation time the first time RelaxedIK starts up (though, we are hoping compilation times get better in general in future versions of the Julia programming language!).  However, to alleviate this issue in the short term, we have provided a way to "reset" the solver on subsequent runs, such that you will not have to kill the program, restart it, and wait another 30 seconds for it to compile every time.

To perform this reset, first start the following ROS node using this command:
<pre> rosrun relaxed_ik reset_and_quit.py </pre>

Then, make sure that the terminal that that command was entered into has focus (i.e., it has been clicked), and type the character 'r'.  This will reset the robot to its initial configuration.  If you would like to quit instead of resetting, type the character 'q'.

<b> Coming Soon </b>

RelaxedIK has been rewritten in Julia to substantially boost performance.  This version of the solver is currently in a beta test on this branch.  If you have feedback based on your experience using this version of the solver (positive or negative), please email me at rakita@cs.wisc.edu.  Thanks!

==================================================================================================================== Development update 10/10/19

The RelaxedIK solver has presented two main options over the past six months: (1) For a stable, reliable, yet relatively slow version of the solver, use the python implementation on the main branch; or (2) for a less stable, less reliable, yet much faster version of the solver, use the Julia implementation on the dev branch. My goal was always to iterate on the Julia version to become just as stable and reliable as its Python counterpart, and eventually unite these two camps into one on the main branch with the Julia version as the centerpiece.

However, there are a few idiosyncrasies that come along with the Julia programming language that have precluded those plans. First, the Julia version of relaxedIK takes at least a minute to do its JIT compilation and start from scratch each time. This is unacceptable for a stable release of a solver that should be convenient and lightweight to use. Plus, it makes development on top of the base solver incredibly tedious, since each little change requires this full JIT compilation. While the people developing the Julia programming language seem to be well aware of this issue for large projects developed in the language, and they may have a sufficient solution for this problem down the road, just waiting around and hoping this gets fixed does not seem like a good option. Second, while ros is somewhat supported in julia through RobotOS.jl, this library leads to mysterious errors too often to be the foundation of a stable release. With both of these issues in mind, I'd rather be proactive and just turn the page away from Julia and plan future development elsewhere.

So, over the next couple of months, I will be re-implementing relaxedIK in a fully statically compiled language (still TBD). Given recent testing, I anticipate the new solver will be even faster than the Julia version, without the JIT compilation hassle and stability issues. Everything will remain ROS compatible. The Julia and python versions will remain in the new release for backwards compatibility, but I anticipate new features will just be officially added and supported in the new language.

Along with the new language, I plan to add in a few other features as part of a "major" new release, such as better support for fast, standard IK solver options. The tools that make relaxedIK actually afford incredibly fast standard IK solver options (i.e., "standard" meaning just hitting end-effector pose goals accurately, without the motion continuity, collision avoidance, and other feasibility considerations included in relaxedIK). We've used these "standard" IK options in our prior work and, in our testing, it is faster and more reliable than other numerical IK solvers we have tried. I plan to make these features more publicly accessible and well documented.

If you have any comments or questions on any of this, feel free to use the iusse thread titled "Development update" to foster conversation. Or, if you prefer, feel free to email me directly at rakita@cs.wisc.edu

<b> Change Log </b>

Version 1.1 (8/16/18): added support for prismatic and fixed joints


