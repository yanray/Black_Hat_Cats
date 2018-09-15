# Milestone 1

## Objectives

* Implement IR sensors to detect lines with Arduino
* Implement line following algorithm 
* Implement “grid shapes” algorithm 

## Line Detection

We used the provided QRE1113 IR sensors with digital output. These sensors work by us first sending a digital pulse to the output. We then set that same pin to be an input, and time how long it takes to drop back down to logical zero. This is directly proportional to the amount of light the sensor is receiving. We then sample this into an array and convert that data into a one or zero, depending on a threshold. 

``` c++
long readQD(int pin) { //read from IR sensor
  pinMode( pin, OUTPUT );
  digitalWrite( pin, HIGH );
  delayMicroseconds(10);
  pinMode(pin, INPUT);

  long time = micros();

  while (digitalRead(pin) == HIGH && ((micros() - time) < 3000));
  return (micros() - time);
}

void read_line() {
  for (int i = 0; i < 5; i++) {
	line_sens_data_time[i] = readQD(line_sens_pin[i]);
  }
}

void conv_to_digi() {
  for (int i = 0; i < 5; i++) {
	line_sens_data_digi[i] = (line_sens_data_time[i] < threshold);
  }
}
```

In terms of sensor placement, we chose to place 3 sensors in the front of our robot (one in the center, and two on the sides separated at a distance slightly wider than the tape), aligned with the axis of rotation of the servomotors. This allows us to turn in place around the center senor, and makes the line tracing algorithm far more robust. 

![alt text](sensors.jpg)


## Line Following

Before we test Line Following, we test our IR sensor to make sure that it could distinguish the white tape and black tape correctly. We print the sensor output on the Serial Monitor, even test it on our map. After 100% sure to get the right value, we test line following. 

Our thought is to use two sensors at first. See diagram below. 

![](Line_Analysis)

At first, we make our robot move forward. If the left sensor detects the white tape, we will turn robot move right. Same theory, if the right sensor detects the white tape, we will turn robot move right. In this way, we could make our robot follow the line. However, it is not stable. 

Then, we make a new iead. use three sensors. The third sensor is in the middle of our robor's bottom. What we will do is make the third sensor detects the line all the time. If the middle sensor detects the white tape, we know that our robot is almost in the middle. So in this situation, if left sensor detects the white tape, we will move it slightly right. So does the right sensor detect. **However**, if the middle sensor could not detect the white tape, we will make a bigger turn. In this adjustment, our robot is more stable than before. 
See the video below. 

## Firgure Eight Travel


## Breakthrough and new try

