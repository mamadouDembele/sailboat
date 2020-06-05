 <h1>Package simulation</h1>

The simulation folder consists of several .cpp files describing algorithms.

<h2>Line following algorithm</h2>

The algorithm used is taken from Luc Jaulin and Fabrice Le Bars
paper on sailboat: *A simple controller for line following of sailboats*.

A simulation on ROS and the use of the rviz tool allows us to visualize the figures below.

<img src="figure/suivi_ligne.png" alt="suivi_ligne" style="display:inline-block;" width="450" height="340"/> <img src="figure/cap_au_pres.png" alt="suivi_ligne" style="display:inline-block;" width="450" height="340"/>

<div id="centre">
<div align="center"><h3>(a)  The boat follows the line AB shown in blue. The red lines represent the strip to never leave. (b) In this figure, we can see the boat going upwind. It performs a series of tacks called beating</h3></div>


<h2>Station keeping algoritm</h2>

Retention consists of sailing the boat around a certain area. An area represented by a circle. Many algorithms have been implemented to solve this problem. 
The idea of the first algorithm is to make the boat do a cycloid. This cycloid will be composed of two circles connected by two straight lines. To make the boat follow a circle, a vector field has been used. The figure below shows the shape of the cycloid and the vector field.


<div id="centre">
<div align="center"><img src="figure/sk_figure.png" alt="sk_figure"  width="550" height="340" /></div>
-----------------------------

The next 3 figures show the boat performing the holding mission with different wind speed values. The purpose of these figures is to justify the robustness of the control law.

<img src="figure/stat_figure_1.png" alt="stat_figure_1" style="display:inline-block;" width="300" height="340"/> <img src="figure/stat_keeping_2.png" alt="stat_k_2" style="display:inline-block;" width="300" height="340"/> <img src="figure/stat_keeping_3.png" alt="stat_figure_1" style="display:inline-block;" width="300" height="340"/>
 
 <div id="centre">
<div align="center"><h3>From left to right, we observe the station keeping challenge for the different speed values v=1, v=3, v=5</h3></div>
 
 
<h2>Station keeping algoritm and avoidance</h2>

For this situation two algorithms have been developed. The first one consists in making a triangle around a physical buoy while remaining a circle of well defined radius. As in the previous case, I varied the wind speed to test the efficiency of the algorithm.The figures below show these limits from a slightly high wind speed.

<img src="figure/triangle1.png" alt="triangle1" style="display:inline-block;" width="300" height="340"/> <img src="figure/triangle2.png" alt="triangle2" style="display:inline-block;" width="300" height="340"/> <img src="figure/triangle3.png" alt="triangle3" style="display:inline-block;" width="300" height="340"/>



<h3>On the figures, the red arrow indicates the direction of the wind, the small white dots indicate the trajectory of the boat after several turns, The big dot in the middle is the physical buoy. </h3>
------------------------------
The second algorithm is that the boat can make a square. For this same problem the results are rather satisfactory than the first one. The boat manages to stay in the circle with the different speed values.

<img src="figure/quadri1.png" alt="quadri1" style="display:inline-block;" width="300" height="340"/> <img src="figure/quadri2.png" alt="quadri2" style="display:inline-block;" width="300" height="340"/> <img src="figure/quadri3.png" alt="quadri3" style="display:inline-block;" width="300" height="340"/>

<div id="centre">
<div align="center"><h3>station keeping for different value of the speed</h3></div>

--------------------------------
<h1>Package mission</h1>
The other main objective of the internship is to be able to use previously developed algorithms in order to be able to carry out certain missions. These missions are challenges of the WRSC competition (World Robotic Sailing Championships).

See for more information https://www.roboticsailing.org/2019/rules/ 

<h2>Fleet race</h2>

For this mission the boat starts from a start line, passes three virtual buoy and then crosses the finish line. To validate a buoy, the boat must be within a radius of 5m around the buoy.The figure below illustrates the principle of the mission.

<div id="centre">
<div align="center"><img src="figure/fleetrace.png" alt="fleet"  width="550" height="340" /></div>

<div id="centre">
<div align="center"><h3>fleet race</h3></div>

<h2> Station keeping and avoidance </h2>
This mission is a combination of two of the algorithms described above. The first one is to stay in the zone for a certain time without constraint, then to leave this zone and go to another zone but this time to stay there while avoiding collisions with the physical buoy.

<div id="centre">
<div align="center"><img src="figure/mission2.png" alt="mission2"  width="550" height="340" /></div>

<div id="centre">
<div align="center"><h3>station keeping and avoidance</h3></div>

<h2> Collaborative area scanning</h2>
For this challenge to effectively analyze a terrain. 
Four boats will be deployed for this task. Every boat is to scan an area as soon as possible and during the mission, every boat can receive the trajectory of the other boats and those scanned by the other boats will not be considered as a useful area to scan. That means the four boats are to scan an area together and the more you scan new areas the better.

<div id="centre">
<div align="center"><img src="figure/mission3.png" alt="mission3"  width="550" height="340" /></div>

<div id="centre">
<div align="center"><h3>Area scanning</h3></div>

<h2> Hide and Seek </h2>
This challenge is a mission between two sailing ships. Each boat has to sail between two circles of 5m radius that have been associated with it, this is the "hide" part. The other part called "seek" consists in detecting an AprilTag of its partner.
<div id="centre">
<div align="center"><img src="figure/mission4.png" alt="mission4"  width="550" height="340" /></div>

<div id="centre">
<div align="center"><h3>Hide and seek</h3></div>

