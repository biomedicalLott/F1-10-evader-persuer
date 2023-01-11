# F1-10-evader-persuer
A simple mulit-car evader-pursuer setup in  F1/10 simulator   
F1/10 is a piece of simulation software in ROS Melodic for Ubuntu. It makes implementing robotic movement much more straight forward. In this project I impemented an evader-pursuer situation. 

The evader's job is to do anything it can to avoid crashing into the wall and the pursuer robot. It has a very very simple pathfinding where it will turn and move so long as it doesn't believe it will crash and if it comes close to crashing it will stop. 

The pursuer will always chase after the evader with its goal ultimately to crash into the evader car. It will also make an effort to not crash into any in a similar way. 

Both robots use a 360 degree LIDAR sensor to "see" the environment. With distance/time logic to determine risk of collision. 
