# PID-Motor-Controller
Abstract

		DC motors are used in industry extensively due to their high reliability, low cost, simple control of speed and position, low energy consumption and their compatibility with digital systems. There are different methods for controlling the speed of DC motors, mainly armature voltage or field control. In this paper, the speed of a separately excited DC motor is controlled by means of self-tuning PID method in which the KP, KI and KD values are constant, and are determined for a specific speed, in a self-tuning PID, KP, KI and KD values are varied with the speed variations. In this paper, two distinct systems have been suggested for the control of DC motor. The output is examined and compared using the error and derivative error, or error and integrated error. 

 
Table of Contents
	Page

	INTRODUCTION	1

	PROCEDURE	1

	      Designing of the PID Motor Controller	 1

Program of the PID Motor Controller							 4

      	Constructing the PID Motor Controller 						 9

Testing the PID Motor Controller							10

	ERRORS	12

	CONCLUSIONS	13

	REFERENCES	14




























List of Figures

Figure	Page

	1	PID Schematic	4

	2	PID Motor Circuit	5

 	3     	Graph of a PID System Behavior	6

	4     	Graph of a Stopped PID System Behavior	7



 
INTRODUCTION

This PID speed controlling motor demonstrates how to implement a PID based DC motor controller using Raspberry Pi. The PID motor controller program is developed using Python. It consists of a GUI, RPM measuring, error detector, PID controller functions, and PWM generation function. The PID based closed-loop DC motor control system is one that determines a difference in the required speed and actual motor speed and creates a correction control signal to remove the error. 

PROCEDURE

	The closed loop control system incorporates hardware circuits for the PID controller circuit, Feedback Circuit, User Interface Unit, Error Detector, and process control signal generator. The student integrated all these functions using the Raspberry Pi 3 using the GPIO pins to gather the input of the feedback signals, and then include the output to the actual speed of the motor. The GPIO are implemented to monitor and control the process of the motor. The logic for the PID controller was implemented on Raspberry Pi 3 by using Python.


Designing the PID Motor Controller

	The PID controller output is derived using the algebraic sum of a proportional, integral, and derivative controller. Where the Proportional controller improves the transient response of the system. The proportional controller adds the product of the proportional gain, Kp, and error value to the process control output. In this case, the system response is directly proportional to the gain hence the error decreases with the increase in proportional gain. The correction is proportional to the amount of error (1). 
𝑒(𝑡) = Set Speed – Current Speed				(1)
Motor Speed = 𝐾𝑝 ∗ 𝑒(𝑡)					(2)
𝐾𝑝= proportional gain and is the problem dependent.		(3)
𝑒(𝑡) = Error						(4)
The Integral controller is always used with the proportional controller to reduce the steady-state error. The steady state error is the difference between the process output and steady state. The Integral output is proportional to both the magnitude and duration of the error. It accumulates the past error terms until the output value becomes equal to the desired value. The system response is inversely proportional to the integral gain. When a significant error occurs in the system output, the I-controller changes the output rapidly. As the error decreases the rate of change of I-controller output decreases, resulting in damping of the system output. Once the error value becomes zero, the I-controller holds the output value which eliminates the error. If the Integral controller is used alone, the time it takes to settle response becomes higher (2). Consider an altimeter that takes a reading every t, seconds. If the sum of all the errors adds up to zero, then integral control would not correct it. Now, let us look at a second situation where every time a reading is taken, the altitude is two feet. Then the integral control would integrate over the magnitude of the readings times the numbers of time it was read and apply a correction to two feet. Integral considers the time of the error before applying a correction. 
𝑒(𝑡) = Set Speed – Current Speed				(5)
𝑒  =  𝑑𝑡 • 𝐾𝑖 Motor Speed		           		(6)
The Integral can be calculated by approximation: divide the region under the curve into rectangles. In conclusion, the integral is the sum of the areas of these rectangles. The D controller increases the system stability and reduces the overshoot. The derivative controller output is proportional (multiplying the derivative gain KD to the rate of change of error over time. The following formula shows the output of the D controller. The Derivative Control Equations are as follows: 
𝑒(𝑡) = Set Position – Current Position				(7)
Motor Speed = 𝐾𝑑  𝑑𝑡𝑑 𝑒(t)				(8)
The PID controller output depends on the input signal, feedback reference signal, and PID gain values.  There are four types of error compensation systems based on P, I, and D controllers. The Raspberry Pi produces the Pulse Width Modulation, PWM, signal based on the user inputs and the feedback signal. A driver circuit amplifies the controller output and drives the DC motor depending on the PWM signal.  A speed measurement sensor is used to encode the motor speed and produce the feedback signal in the form of a digital or analog signal. The feedback signal is fed to the Raspberry Pi to repeat the process to maintain the system in an equilibrium state. With these given values, ideal values can then be calculated within the PID speed controlling motor (3). After these calculations have been made the student would then develop the schematic of the theorized PID motor controller as seen in the figure below.
 
PID Schematic [1].
	 

Program of the PID Motor Controller

 	This python program reads output of the sensors and adjusts the speed of the motors accordingly. The program was written as a combination of the proportional, integral and derivative control. The program is as follows:
###################################
# PID Controller Project
#
#
#
###################################

import RPi.GPIO as GPIO
import matlab and plt
import time

# # # # # #  GPIO Setup # # # # # #

GPIO.setmode(GPIO.BOARD)                    # Uses physical pin locations
GPIO.setup(23, GPIO.OUT)                    # PWM pulse
GPIO.output(23, 0)

# # # # # # # # # # # # # # # # # #

SET_RPM = 7800                             # SET RPM value
FEEDBACK = 0.0
previous_time = 0.0
previous_error = 0.0
Integral = 0.0
D_CYCLE = 10
Kp = 22.0641                                      # Proportional controller Gain (0 to 100)
Ki = 74.6668                                      # Integral controller Gain (0 to 100)
Kd = 1.63                                     # Derivative controller Gain (0 to 100)

# # # # # # # # # # # # # # # # # #

GatePulse = GPIO.PWM(23, 100)

error = int(SET_RPM) - FEEDBACK             # Differnce between expected RPM and run RPM

if previous_time == 0:

    previous_time = time.time()

    current_time = time.time()
    delta_time = current_time - previous_time
    delta_error = error - previous_error

    Pout = (Kp/10 * error)

    Integral += (error * delta_time)

if Integral>10:

    Integral=10

if Integral<-10:

    Integral=-10
    Iout=((Ki/10) * Integral)
    Derivative = (delta_error/delta_time)
    previous_time = current_time
    previous_error = error
    Dout=((Kd/1000 )* Derivative)
    output = Pout + Iout + Dout             # PID controller output
    if ((output>D_CYCLE)&(D_CYCLE<90)):
        D_CYCLE+=1
    if ((output<D_CYCLE)&(D_CYCLE>10)):
        D_CYCLE-=1
# plot results
plt.figure()
plt.subplot(2,1,1)
plt.plot(t,u)
plt.legend([r'$T_c$'])
plt.ylabel('MV')
plt.subplot(2,1,2)
plt.plot(t,y)
plt.plot(t,yp)
plt.legend([r'$T_{meas}$',r'$T_{pred}$'])
plt.ylabel('CV')
plt.xlabel('Time')
plt.savefig('sysid.png')

# # # # # # # # # # # # # # # #

Constructing of the PID Motor Controller

 The Raspberry Pi produces the Pulse Width Modulation (PWM) signal based on the user inputs and the feedback signal. A driver circuit amplifies the controller output and drives the DC motor depending on the PWM signal.  A speed measurement sensor is used to encode the motor speed and produce the feedback signal in the form of a digital or analog signal. The feedback signal is fed to the Raspberry Pi to repeat the process to maintain the system in an equilibrium state. PID based DC motor controller is designed to control the DC motor at a constant speed. RPi.GPIO.22 is configured for the feedback input from RPM sensor.
RPi.GPIO.23 is configured as an output. It provides PWM pulses for DC motor driver. The student first connected the Raspberry Pi display and Input-Output pins. Then a connection of the Power adapter to power up the raspberry pi, and then connect to the Raspberry Pi 3 to an ethernet cable to communicate to the student’s laptop for output. Once these steps have been taken, the student would download and run the DC motor controller code once the student secure-shelled into the internet connected Raspberry Pi, the student would be able to turn the potentiometer to be able to increase the speed of the motor with the calculated PID calculations within the PID python code. Below is a figure of the circuit may appear:
 

PID Motor Circuit [2].  




Testing the PID Motor Controller

	Once the circuit is constructed, tests can begin to insure how the PID speed controller motor circuit functions and behaves while comparing theoretical results from the designing procedure. Connecting the circuit to the proper materials insure proper feedback. The student would begin by testing the motor coming and the measurements. Before testing the student had determined the PID to have a overshoot percentage of 10% and a four second settling time. In the laboratory the student realized the system had maintained the 10% overshoot, the system had a much more reduced settling time of .8 seconds. After the code was complete the graph below shows the simulation of testing the PID motor system, along with the characteristics of the system. 
 

Graph of a PID System Behavior [3].


	The next experiment when testing in the laboratory, the student tested to see how the motor would act with a force producing a slower speed and in return correcting the motor from the error correcting feedback. Below is a close up of the settling time from experimenting of stopping the DC motor. The motor is seen having a quicker settling time than calculated of a settling time of 4 seconds. 
 
Graph of a Stopped PID System Behavior [4].

ERRORS

	During the Project, errors have occurred to have the PID motor have an overshoot of around 10% while also having a settling time of .8 seconds. While the student had calculated he values to insure a 10% overshoot and a settling time of 4 seconds. Some reasons to contribute to the settling time is miscalculating values of the PID of the proportional, integral, and derivative values. Another miscalculation could include tuning the PID system before construction of the circuit. These values are both key values in designing and could deviate the output. Another error could have occurred with the inner circuitry within the motor was not none and could have added to why the motor did not behave as calculated. 









CONCLUSION

	 The objective of this project was to gain knowledge in the use of the designing process of a PID system. Which will also aid the student in future courses and in careers in the electrical engineering field. The proportional, integral, and derivative control modes each fulfill a unique function. 
































REFERENCES

	
[1]		“Fritzing.” Fritzing Fritzing, Fritzing, May 2018, fritzing.org/home/.
[2]		“Introduction: PID Controller Design.”  MATLAB and Simulink - Introduction: PID Controller Design, 
www.ctms.engin.umich.edu/CTMS/index.php?example=Introduction§ion=ControlPID
[3]		“PID Theory Explained.” PID Theory Explained - National Instruments, www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html 
[4]	





