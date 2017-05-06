# CompRobo Final Project: Neato Soccer, Extended Edition
For this project, our ostensible goal is to program a robot (a neato vacuum cleaner) to recognize a soccer ball visually, approach it, and kick it. The real purpose is to develop our own learning goals around object recognition via a neural network and computer vision. The project is largely split along those lines.

## Setup
```
git clone https://github.com/cebeery/extended_neato_soccer
cd extended_neato_soccer
sudo pip install -r requirements.txt
chmod u+x <relative path to directory>/scripts/neato_soccer.py
```

## Usage
Connect to a robot that publishes camera images to /camera/image_raw and is controlled via /cmd_vel. In particular, control commands for linear motion are the linear x component of a Twist msg and commands for rotation motion are the angular z  component of a Twist msg. Then use

`rosrun extended_neato_soccer neato_soccer.py`

from the command line to run our code.

## The Story 
For information about the project as well as our steps and stumbles along the way, check out our wiki pages. 


