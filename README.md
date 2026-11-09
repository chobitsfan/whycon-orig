https://uloz.to/file/zF5w5Tw4WbGz/test-avi

The latest version of the system is [here](https://github.com/jiriUlr/whycon-ros). 

## WhyCon

### A precise, efficient and low-cost localization system 

_WhyCon_ is a version of a vision-based localization system that can be used with low-cost web cameras, and achieves millimiter precision with very high performance.
The system is capable of efficient real-time detection and precise position estimation of several circular markers in a video stream. 
It can be used both off-line, as a source of ground-truth for robotics experiments, or on-line as a component of robotic systems that require real-time, precise position estimation.
_WhyCon_ is meant as an alternative to widely used and expensive localization systems. It is fully open-source.
_WhyCon-orig_ is WhyCon's original, minimalistic version that was supposed to be ROS and openCV independent.


| WhyCon example application (video)  | Scenario description |
| ------ | ----------- |
|[![WhyCon applications](https://raw.githubusercontent.com/wiki/gestom/WhyCon/pics/whycon.png)](https://www.youtube.com/watch?v=KgKrN8_EmUA"AAAA")|-precise docking to a charging station (EU project STRANDS),<br/> -fitness evaluation for self-evolving robots (EU proj. SYMBRION),<br/>-relative localization of UAV-UGV formations (CZ-USA project COLOS),<br/>-energy source localization in (EU proj REPLICATOR),<br/>-robotic swarm localization (EU proj HAZCEPT).|

The _WhyCon_ system was developed as a joint project between the University of Buenos Aires, Czech Technical University and University of Lincoln, UK.
The main contributors were [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao), [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao), [Peter Lightbody](https://scholar.google.com/citations?user=tBUM-8oAAAAJ&hl=en)  and [Jan Faigl](https://scholar.google.co.uk/citations?user=-finD_sAAAAJ&hl=en). Each of these contributors maintains a slightly different version of WhyCon. Later contributions were made by [Jiri Ulrich](https://scholar.google.com/citations?user=vMtZ5FcAAAAJ&hl=cs&oi=ao) and Kristi Zampachu.

| WhyCon version  | Application | Main features | Maintainer|
| --------------- | ----------- | ------ | ----- |
| [WhyCon/WhyCode](https://github.com/jiriUlr/whycon-ros) | general | **actively maintained**, ROS | [Jiri Ulrich](https://scholar.google.com/citations?user=vMtZ5FcAAAAJ&hl=cs&oi=ao) |
| [WhyCon-orig](../../) | general | 2D, 3D, ROS, lightweight, autocalibration | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao)|
| [WhyCon-ROS](https://github.com/lrse/whycon) | general | 2D, ROS | [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao) |
| [SwarmCon](https://github.com/gestom/CosPhi/tree/master/Localization) | μ-swarms | 2D, individual IDs, autocalibration | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) |
| [Caspa-WhyCon](http://robotics.fel.cvut.cz/faigl/caspa/) | UAVs | embedded, open HW-SW solution | [Jan Faigl](https://scholar.google.co.uk/citations?user=-finD_sAAAAJ&hl=en) |
| [Social-card](https://github.com/strands-project/strands_social/tree/hydro-devel/social_card_reader) | HRI | ROS, allows to command a robot | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) |

#### Where is it described ?

<i>WhyCon</i> was first presented on International Conference on Advanced Robotics 2013 [[2](#references)], later in the Journal of Intelligent and Robotics Systems [[1](#references)] and finally at the Workshop on Open Source Aerial Robotics during the International Conference on Intelligent Robotic Systems, 2015 [[3](#references)]. Its early version was also presented at the International Conference of Robotics and Automation, 2013 [[4](#references)]. An extension of the system, which used a necklace code to add ID's to the tags, achieved a best paper award at the SAC 2017 conference [[5](#references)].
If you decide to use this software for your research, please cite <i>WhyCon</i> using the one of the references [provided](#references) ~~in this [bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/WhyCon.bib) file~~.

-----

### Setting up WhyCon 

#### Prepare prerequisities

1. Install ROS 2 (Humble or newer) and source it, e.g. <b>source /opt/ros/humble/setup.bash</b>. Only <i>rclcpp</i>, <i>sensor_msgs</i>, <i>geometry_msgs</i> and <i>std_srvs</i> are used.
1. Install git and cmake <b>sudo apt install git cmake build-essential</b>.
1. Start a ROS 2 camera driver that publishes <b>mono8</b> images and check the topic with <b>ros2 topic hz /camera/image_raw</b>.

#### Compile, run and test

1. Download the software from GitHub <b>git clone https://github.com/gestom/whycon-orig.git</b> and go to the <b>src</b> directory.
1. Adjust the camera resolution in the <i>main/whycon.cpp</i> - images of a different size are ignored.
1. Compile the software with plain cmake (in a shell where ROS 2 is sourced): <b>mkdir build && cd build && cmake .. && make</b>. The binary is placed in the <b>bin</b> directory.
1. Download, resize and print one circular <a href="etc/test.pdf">pattern</a> - you have the pattern also in the <b>whycon-orig/etc/test.pdf</b> file.
1. Try a test run - you need to run the binary in the <b>bin</b> directory. Type <b>./whycon /camera/image_raw 1</b>, where /camera/image_raw is the mono8 image topic and 1 tells the system to track one pattern.
1. The positions are published as <i>geometry_msgs/PoseArray</i> on the <b>whycon/markers</b> topic (<b>ros2 topic echo /whycon/markers</b>). They are in metres relative to the camera, with x forward (along the optical axis), y left and z up; the header (including frame_id) is copied from the image and the orientation is not estimated.

#### Setting up the coordinate system

1. Calibrate your camera with OpenCV (e.g. the OpenCV <i>calibration</i> sample or <b>ros2 run camera_calibration cameracalibrator</b>) and save the resulting YAML file (with <i>camera_matrix</i> and <i>distortion_coefficients</i>) as <b>etc/camera_calibration.yaml</b>.
1. If you have resized the markers (their default size is 122mm), then adjust their diameter in the <b>main/whycon.cpp</b> file.
1. Print additional four circular <a href="etc/test.pdf">markers</a> and place to the corners of your (reclangular) operational space.
1. Modify the dimensions of the operation space in the <b>main/whycon.cpp</b> and recompile - the system will now assume that the four markers are at positions [0,0],[fieldLength,0], [0,fieldWidth],[fieldLength,fieldWidth].
1. Position and fixate your camera so that it has all four circles in it's field of view.
1. Go to <b>bin</b> directory and run  <b>./whycon /camera/image_raw Y</b>, where Y is the number of patterns you want to track, i.e. Y=NxM+4.
1. Once all the patterns are found, call the autocalibration service <b>ros2 service call /whycon/autocalibrate std_srvs/srv/Trigger</b> and the four outermost patterns will be used to calculate the coordinate system. It is saved to <b>etc/default.cal</b> and loaded on restart.

#### To postprocess recorded data

1. To create a log of the robot positions, simply create an <b>output</b> folder at the directory where you run the <b>whycon</b>.
1. Record the image topic with <b>ros2 bag record</b> and play it back with <b>ros2 bag play</b> while <b>whycon</b> runs.
1. Logs might be large - to prevent saving logs, run the system with <b>nolog</b> argument, e.g. <b>./whycon /camera/image_raw 1 nolog</b>.

#### Some additional remarks

2. At this point, you can start experimenting with the syste by adding whatever features you might think useful.
3. We have tried to comment the code so an experienced programmer should be able to alter the system accordingly. However, if you have any questions regarding the code, feel free to contact [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) or [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao)
4. If you use this localization system for your research, please don't forget to cite at least one relevant paper from ~~these [bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/WhyCon.bib)~~ [records](#references).
5. Have fun!
</ol>

<hr>

### Dependencies

1. <b>ROS 2</b> (<i>rclcpp</i>, <i>sensor_msgs</i>, <i>geometry_msgs</i>, <i>std_srvs</i>) for the image input and position output.
2. <b>cmake</b> to build the software.

### References
1. J. Ulrich, A. Alsayed et al.: **[Towards fast fiducial marker with full 6 DOF pose estimation](https://dl.acm.org/doi/abs/10.1145/3477314.3507043)**. Symposium on Applied Computing, 2022. [[bibtex](https://gist.github.com/jiriUlr/7d333e90c43e6b41c79e5150c7a59267)].
1. J. Ulrich: **[Fiducial marker-based multiple camera localisation system](https://dspace.cvut.cz/bitstream/handle/10467/101526/F3-DP-2022-Ulrich-Jiri-main.pdf?sequence=-1&isAllowed=y)**. Master's thesis. Czech Technical University in Prague, 2022. [[bibtex](https://gist.github.com/jiriUlr/e8d53c7edd6b14c824e67e60596a489f)].
3. K. Zampachu: **[Visual analysis of beehive queen behaviour](https://dspace.cvut.cz/bitstream/handle/10467/101048/F3-BP-2022-Zampachu-Kristi-main.pdf?sequence=-1&isAllowed=y)**. Bachelor's thesis. Czech Technical University in Prague, 2022. [[bibtex](https://gist.github.com/jiriUlr/eb08ee4b183c615e312ab2db767e9b18)].
1. J. Ulrich: **[Fiducial Marker Detection for Vision-Based Mobile Robot Localisation](https://dspace.cvut.cz/bitstream/handle/10467/89879/F3-BP-2020-Ulrich-Jiri-main.pdf?sequence=-1&isAllowed=y)**. Bachelor's thesis. Czech Technical University in Prague, 2020. [[bibtex](https://gist.github.com/jiriUlr/348d42b7a1cdd08b94953adedc50c5d7)].
5. T. Krajník, M. Nitsche et al.: <b>[A Practical Multirobot Localization System.](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_JINT_whycon.pdf)</b> Journal of Intelligent and Robotic Systems (JINT), 2014. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_JINT_whycon.bib)].
6. T. Krajník, M. Nitsche et al.: <b>[External localization system for mobile robotics.](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2013_icar_whycon.pdf)</b> International Conference on Advanced Robotics (ICAR), 2013. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2013_icar_whycon.bib)].
7. M. Nitsche, T. Krajník et al.: <b>[WhyCon: An Efficent, Marker-based Localization System.](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_irososar_whycon.pdf)</b> IROS Workshop on Open Source Aerial Robotics, 2015. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_irososar_whycon.bib)].
8. J. Faigl, T. Krajník et al.: <b>[Low-cost embedded system for relative localization in robotic swarms.](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6630694)</b> International Conference on Robotics and Automation (ICRA), 2013. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2013_icra_whycon.bib)].
9. P. Lightbody, T. Krajník et al.: <b>[A versatile high-performance visual fiducial marker detection system with scalable identity encoding.](http://eprints.lincoln.ac.uk/25828/1/4d0bd9e8a3b3b5ad6ca2d56c1438fbbc.pdf)</b>Symposium on Applied Computing, 2017.[[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2017_sac_whycon.bib)].

### Acknowledgements

The development of this work is currently supported by the EU FET Open programme under grant agreement No.964492 project _RoboRoyale_.
The development of this work was supported by the Czech Science Foundation project 17-27006Y _STRoLL_.
In the past, the work was supported by EU within its Seventh Framework Programme project ICT-600623 _STRANDS_.
The Czech Republic and Argentina have given support through projects 7AMB12AR022, ARC/11/11 and 13-18316P.
We sincerely acknowledge [Jean Pierre Moreau](http://jean-pierre.moreau.pagesperso-orange.fr/infos.html) for his excellent libraries for numerical analysis that we use in our project. 
