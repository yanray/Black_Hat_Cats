[Return Home](https://yanray.github.io/Black_Hat_Cats/)

# Lab 1: Microcontroller

### Objective
Learn basics of Arduino io by controlling LEDs and motors
Assemble basic body for a robot and use the Arduino to move in a basic square

### Teams
**subteam 1**: Hadi Alzayer, Alberto Lopez Delgado

**subteam 2**: Yanrui Wang , Ian Switzer

## Step 1: Pre-lab work

[Click here](https://www.arduino.cc/en/Main/Software) to download Arduino IDE.

After install Arduino IDE, you will see this.

![](images/Lab_1/20180905_210630.jpg)

## Step 2: Internal LED Blink

**Firstly**, Open Ardunio IDE -> File -> Examples -> 1.Basics -> Blink

**Then**, Click **Checkmark** to compile code -> Click **right arrow** to upload your code to Arduino

**Luckily**, The Internal LED blinks, see the video below.

***Warning!!***  Make sure your Arduino IDE is connected to right COM port. Click Tools -> Serial Port list to see.

<iframe width="560" height="315" src="https://www.youtube.com/embed/iQ6RLm8GsXc" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

[Coding for Step1]

``` c++
int led_pin = LED_BUILTIN;
void setup() {
  pinMode(led_pin, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(led_pin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(led_pin, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
```

## Step 3: External LED Blink

After Internal LED Blink, Let us Blink an External LED ! 

Pick up a LED and a Resistor. [Need Picture of LED and Resistor here !!!!]

**Then**, Connect LED and Resistor on breadboard, use a different Pin and modify code to blink Internal LED. 

**Great!**, The External LED blinks, see the video below.

***Warning!!***  Make sure your LED is connected to a Resistor at least 300ohm. Unless you want to blow up your Arduino. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/0pE7rAklaJY" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Step 4: Read the potentiometer 

After External LED Blink, Let us Read Analog value of Potentiometer ! 

Pick up a Potentiometer and a Resistor. [Need Picture of LED and Resistor here !!!!]

**Then**, Connect Potentiometer and Resistor on breadboard, use the Analog Pin A0 and write code to read the value. Remember click **Serial monitor** to see the value. (on the top right)
**Cool !**, Get the value of Potentiometer, see the video below.

***Warning!!***  Make sure your Potentiometer is connected to a Resistor at least 300ohm. Unless you want to blow up your Arduino. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/qJ6spCfECX0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Step 5: Analog Output (PWM)

In this step, we connect **Step 3** and **Step 4** to read the potentiometer and put the output to External LED. 
***Note *** You could learn PWM [Click here](https://cei-lab.github.io/ece3400-2017/tutorials/PWM/PWM.html).

To use PWM as an input, you can use analogWrite to an ‘analog’ like output:
```c++
analogWrite(output_pin, value);
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/FzOQpxXkZDM" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Step 6: Parallax Servos 
Unlike a regular servo that is usually limited to 180 degrees and control the angle, a continuous servo (like the one used for this lab) can be controlled with direction and speed.
Servo

To run a servo, you can use Arduino’s servo library and run this code:

```c++
#include <Servo.h>
int servo_pin = 5;
Servo myServo;

void setup() {
    myServo.attach(servo_pin);
}
void loop() {
  move_motor(myServo, 180);
}
void move_motor(Servo motor, int value){
	int new_value=value;
	motor.write(new_value);
}

```
<iframe width="560" height="315" src="https://www.youtube.com/embed/gK3PRNAZcIo" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Step 7: Assemble Impressive Robot 

To assemble the robot, we looked at previous designs and decided the cleanest design with the parts available. We chose a laser-cut plexiglass with a breadboard glued to it, to facilitate the connections with the Arduino.

![alt text](parts.jpg)
 
Next, we attached the servo mounts and the servos themselves. We tried using the 3D-printed battery holders, but they were too large for the battery and didn’t provide the sturdiness we expected. In turn, we temporarily used a zip-tie.
 
![alt text](servos.jpg) 

We assembled the wheels to the servos with no issues. However, the ball bearing was too short, so we had to use the plastic extension shown in the picture. We are going to 3D-print a new ball bearing, because the extension causes undesirable wiggling that could lead to instability and malfunctions later in the project.
 
We next mounted the Arduino (partially on top of the breadboard) and used 2 plastic extensions, nuts and bolts to provide the desired stability, as shown below.

![alt text](final_prototype.jpg)


## Step 8: Make the Robot Go !
Now using what we have done to run the motors, we can use it to control two motors to move in a square. Here is the code for it: 
``` c++
 #include <Servo.h>
int MAX_READING = 1023;
int analog_pin = A0;
int led_pin = 11;
int leftServo_pin = 5;
int rightServo_pin = 6;
Servo leftServo;
Servo rightServo;
void start_now() {
  Serial.begin(9600);
  pinMode(analog_pin, INPUT);
  pinMode(led_pin, OUTPUT);
  leftServo.attach(leftServo_pin);
  rightServo.attach(rightServo_pin); //check pin
}
void setup() {
  start_now();
}
void loop() {
  //int value_analogRead = analogRead(analog_pin);
  //Serial.print(value_analogRead*5.0/MAX_READING);
  Serial.println(" Volts");
  //output_to_led(value_analogRead);
  int motor_value = 180;
  move_motor(leftServo, 180);
  move_motor(rightServo, 180);
  delay(635);
move_forward();
  delay(1000);
}
void output_to_led(int value) {
  int new_value=map(value,0,MAX_READING,0,255);
  analogWrite(led_pin,new_value);
}
void move_motor(Servo motor, int value){
	int new_value=value;
	motor.write(new_value);
}
void move_forward(){
 	move_motor(leftServo, 0);
  	move_motor(rightServo,180);
}
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/J6l1jtC9Hv0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Extra step: 

The modification I added since we had extra time was a basic microphone circuit. This was done before realizing this is very similar to the next lab. Regardless, I guess we are ahead of the game.
[AUDIO CIRCUIT FILE]
The way the circuit works is as follows. The signal starts at the microphone. One leg of the microphone is pulled up to 5 volts through a resistor, the other is set to ground. When sound hits the microphone the microphone alternates the voltage on its pulled up leg as a direct representation of the sound. I then isolate the AC component  by passing it through a capacitor. This signal then goes to the noninverting side of an LM358 op amp. In this case we use a 25k resistor and a 50 ohm resistor to amplify by a factor of 501.
[waveform picture]
If we left it like this we would have a detectable signal. Unfortunately this signal would vary between about +-2.5 volts, so we add a dc offset through two 1k resistors to the noninverting input of the op amp. I also added a capacitor in line with the 50 ohms resistor to make it so the dc offset is not amplified. I then fed this signal to the analog input of the Arduino Uno. This signal is sampled at a rate of 9.5kHz until 128 samples are collected. The way I have it currently setup it can easily detect a signal anywhere between 100hz and 4.25khz. I had to “detach” the servos while sampling or else the PWM interrupts would interfere with sampling.
[VIDEO OF IT WOKRING]
From there frequency of the whistle is used to control whether the robot is moving straight, turning, or stopped, depending on how high the frequency is.

<iframe width="560" height="315" src="https://www.youtube.com/embed/ufud2p73alA" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>







