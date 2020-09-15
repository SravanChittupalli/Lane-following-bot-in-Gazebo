# Lane-following-bot-in-Gazebo

<!--
*** Thanks for checking out this README Template. If you have a suggestion that would
*** make this better, please fork the repo and create a pull request or simply open
*** an issue with the tag "enhancement".
*** Thanks again! Now go create something AMAZING! :D
-->





<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
<!--[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url] -->
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<!-- <br />
<p align="center">
  <a href="https://github.com/othneildrew/Best-README-Template">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

  <h3 align="center">Best-README-Template</h3>

  <p align="center">
    An awesome README template to jumpstart your projects!
    <br />
    <a href="https://github.com/othneildrew/Best-README-Template"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/othneildrew/Best-README-Template">View Demo</a>
    ·
    <a href="https://github.com/othneildrew/Best-README-Template/issues">Report Bug</a>
    ·
    <a href="https://github.com/othneildrew/Best-README-Template/issues">Request Feature</a>
  </p>
</p>
-->


<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
  * [Built With](#built-with)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
* [Usage](#usage)
* [Future Work](#Futurework)
* [License](#license)
* [Acknowledgements](#acknowledgements)



<!-- ABOUT THE PROJECT -->
## About The Project

[![output_screenshot][product-screenshot]](assets/screenshot.png)

This project was made just out of pure interest in Self-Driving Cars. I've always wondered how cars follow lanes. During my 1st year of engineering I've worked with line following bots and thought why everyone uses a line sensing array and not a camera to follow the line. Using that motivation I've learnt about line detection and algorithms used to follow the line. Later in second year I've learnt ROS and Gazebo then I thought why not apply the line following to detect lanes and make the bot follow the lanes using a control loop like PID.

Few algorithms used for Lane Detection:
* [Image Blurring](https://www.projectrhea.org/rhea/index.php/Applications_of_Convolution:_Simple_Image_Blurring) : To remove noise
* [Canny Edge Detection](http://fourier.eng.hmc.edu/e161/lectures/canny/node1.html) : To detect edges
* [Hough Transform](https://homepages.inf.ed.ac.uk/rbf/HIPR2/hough.htm) : To detect lines from edges.

The math behind all the algorithms is really good and is not that tough. I would recommend everyone to implement those algorithms from scratch in python. I've directly used openCV library here. I will write a blog post on these basic topics and you can find them in [Blogs](#blogs) section

A list of commonly used resources that I find helpful are listed in the acknowledgements.

### Built With

* [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation)
* [Gazebo](http://gazebosim.org/)
* [opencv-python 4.2.0.34 ](https://pypi.org/project/opencv-python/4.2.0.34/)
* [RVIZ](http://wiki.ros.org/rviz)


<!-- GETTING STARTED -->
## Getting Started
[![output_screenshot][product-screenshot]](assets/output.mkv)

Using this repo is extremely easy if the [prerequisites](#prerequisites) are fulfilled.

### Prerequisites

* [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation)
* [Gazebo](http://gazebosim.org/)
* [opencv-python 4.2.0.34 ](https://pypi.org/project/opencv-python/4.2.0.34/)

### Installation

1. Enter into your ROS workspace
```
cd catkin_ws/src
```
2. Clone the repo
```
git clone https://github.com/SravanChittupalli/Lane-following-bot-in-Gazebo
```
3. Then go to your workspace directory and run `catkin_make`
4. Start ROS core using command: `roscore`
5. Open a new terminal and use `roslaunch my2wr gazebo.launch`
6. Open another terminal and run `roslaunch gazebo_ros empty_world.launch`
7. Open a new terminal and cd into `ros_ws/lane_follow/scripts`
8. Enter `python main.py`



<!-- USAGE EXAMPLES -->
## Usage

This repo can act as our starting point in Self-Driving Car journey. This repo contains the URDF of a bot this bot can be spawned in any environment. The bot is equiped with a USB camera and has differential drive. You can perform different tasks like:
- Advanced Lane Detection
- Lane detection using Neural Networks
- Behavioural Cloning 

This repo can act as a base so that you dont have to waste time building up a bot.



<!-- To-Do -->
## Future work

I am learning behavioural cloning. The next target is to feed in images of the road and the bot decides if it wants to turn, stop, acceplerate.


<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.


<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements
* [Amazon AWS Racetrack](https://github.com/aws-robotics/aws-robomaker-racetrack-world)
* [Image Blurring](https://www.projectrhea.org/rhea/index.php/Applications_of_Convolution:_Simple_Image_Blurring)
* [Lane Detection](https://www.youtube.com/watch?v=yvfI4p6Wyvk)
* [The Construct YouTube channel](https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q)
* [Canny Edge Detection](http://fourier.eng.hmc.edu/e161/lectures/canny/node1.html)
* [Hough Transform](https://homepages.inf.ed.ac.uk/rbf/HIPR2/hough.htm) 





<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=flat-square
[contributors-url]: https://github.com/SravanChittupalli/Lane-following-bot-in-Gazebo/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=flat-square
[forks-url]: https://github.com/SravanChittupalli/Lane-following-bot-in-Gazebo/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=flat-square
[stars-url]: https://github.com/SravanChittupalli/Lane-following-bot-in-Gazebo/stargazers
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=flat-square
[license-url]: https://github.com/SravanChittupalli/Lane-following-bot-in-Gazebo/blob/master/LICENSE
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/sravan-chittupalli-a3777b16a/
[product-screenshot]: assets/screenshot.png
