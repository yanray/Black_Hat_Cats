# Milestone 1

### Team Member: Hadi Alzayer, Alberto Lopez Delgado, Yanrui Wang , Ian Switzer

## Objective :

Learn how to use infrared reflectnce Line Sensor, and successfully make our robot follow the line and figure eight !! Besides that, we even make more challegens to our amazing robots. See below, you will get all answers. 

## Sensor Set-up 


## Line Following

Before we test Line Following, we test our IR sensor to make sure that it could distinguish the white tape and black tape correctly. We print the sensor output on the Serial Monitor, even test it on our map. After 100% sure to get the right value, we test line following. 

Our thought is to use two sensors at first. See diagram below. 

![](Line_Analysis)

At first, we make our robot move forward. If the left sensor detects the white tape, we will turn robot move right. Same theory, if the right sensor detects the white tape, we will turn robot move right. In this way, we could make our robot follow the line. However, it is not stable. 

Then, we make a new iead. use three sensors. The third sensor is in the middle of our robor's bottom. What we will do is make the third sensor detects the line all the time. If the middle sensor detects the white tape, we know that our robot is almost in the middle. So in this situation, if left sensor detects the white tape, we will move it slightly right. So does the right sensor detect. **However**, if the middle sensor could not detect the white tape, we will make a bigger turn. In this adjustment, our robot is more stable than before. 
See the video below. 

## Firgure Eight Travel


## Breakthrough and new try

