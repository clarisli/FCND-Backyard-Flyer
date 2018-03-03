# Backyard Flyer Project

The goals/steps of this project are following:
* Set up a state machine using event-driven programming to autonomously flying a drone in Unity Simulator. 
* Manually control the the drone to fly a 10 meter box at a 3 meter altitude
* Autonomous control the drone to perform the same task
* Use the logs to plot the trajectory of the drone and analyze the performance of the task.

## Setup
* Download the [simulator](https://github.com/udacity/FCND-Simulator/releases).
* Setup [Python environment](https://github.com/udacity/FCND-Term1-Starter-Kit)
* Run the task autonomously by `python backyard_flyer.py`
* To log data while flying, open the simulator first then run `python drone.py`. The logs will be saved to "TLog-manual.txt". 
* Refer to the [UdaciDrone API](https://udacity.github.io/udacidrone/) if needed


[//]: # (Image References)

[image1]: ./images/plots-auto.png "Autonomous Trajectory"
[image2]: ./images/plots-manual.png "Manual Trajectory"

## The Approach
The task was to command the drone to fly a 10 meter box at a 3 meter altitude. 

My first attempt was to manually control the drone performing the task. 

Here's the trajectory plotted from the telemetry logs:

![alt text][image2]

I got the drone fly at an altitude of 3m, but failed to fly a square, landed at a wrong location, and the distance flied was too far. It was not an easy task, because I had to pay attention to many things at the same time, such as altitude, headings, location..etc. 

I thought this is where a flight computer can become handy. I then implemented a state machine in `backyard_flyer.py` to automously fly the box. 

Here's the trajectory plotted from the logs:

![alt text][image1]

The drone succesfully performed the task - flied a 10 meter box at a 3 meter altitude.

## Future Work
* Port the code to a real drone
