# Lab 1: Microcontroller

[Home](https://yanray.github.io/Black_Hat_Cats/)

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

Here is the code for blinking the internal LED, to use it with an external LED, replace LED_BUILTIN with the desired output pin.

''' c++
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
'''
[We need a place to put code and need some explanation for code !!! please]

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

<iframe width="560" height="315" src="https://www.youtube.com/embed/FzOQpxXkZDM" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Step 6: Parallax Servos 

<iframe width="560" height="315" src="https://www.youtube.com/embed/gK3PRNAZcIo" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Step 7: Assemble Impressive Robot 

## Step 8: Make the Robot Go !

<iframe width="560" height="315" src="https://www.youtube.com/embed/J6l1jtC9Hv0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

## Extra step: 

The modification I added since we had extra time was a basic microphone circuit. This was done before realizing this is very similar to the next lab. Regardless, I guess we are ahead of the game.


[AUDIO CIRCUIT FILE]
The way the circuit works is as follows. The signal starts at the microphone. One leg of the microphone is pulled up to 5 volts through a resistor, the other is set to ground. When sound hits the microphone the microphone alternates the voltage on its pulled up leg as a direct representation of the sound. I then isolate the AC component  by passing it through a capacitor. This signal then goes to the noninverting side of an LM358 op amp. In this case we use a 25k resistor and a 50 ohm resistor to amplify by a factor of 501.

[waveform picture]
If we left it like this we would have a detectable signal. Unfortunately this signal would vary between about +-2.5 volts, so we add a dc offset through two 1k resistors to the noninverting input of the op amp. I also added a capacitor in line with the 50 ohms resistor to make it so the dc offset is not amplified. I then fed this signal to the analog input of the Arduino Uno. This signal is sampled at a rate of 9.5kHz until 128 samples are collected. The way I have it currently setup it can easily detect a signal anywhere between 400hz and 4.25khz. I had to “detach” the servos while sampling or else the PWM interrupts would interfere with sampling.

[VIDEO OF IT WOKRING]
From there frequency of the whistle is used to control whether the robot is moving straight, turning, or stopped, depending on how high the frequency is.








