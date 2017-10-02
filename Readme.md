# MATLAB Simulated Autonomous Mobile Robot (AMR) #

This project provides a basic AMR simulated in MATLAB. The various components are:

* **Platform** : 4 Wheeled Mobile Robot; Rear wheel drive
* **Controller** : Nonlinear Back Stepping or Neural Network Controller
* **Sensor** : LIDAR
* **Navigation Algoritm** : Potential Field Navigation or Wall Follow or Bug 2
* **Localization Algorithm** : Not Implemented (Particle Filter Localization planned)


This project was completed as a requirement to MST class EE5380.

Author: Singh, Siddharth & Sengupta, Ayush

## How to run ##

Download the code and run using MATLAB.

_**Requirement**_   
This code was written in MATLAB 2015a. The code should also work with GNU Octave but is not tested.

## Explanation ##

### Controller ###
#### Nonlinear Back Stepping ####

In back stepping control, the control torque(𝜏) is a function of velocity given by:

```
  𝜏 = 𝐵^−1 * 𝑀(−𝐾𝑒𝑐 + 𝑉̇𝑐+ ℎ𝑐 + 𝑓)

  Where,
    𝐵 = Input Transformation Matrix
    𝑀 = Inertia Matrix
    𝐾 = Control Gain
    𝑒𝑐 = Velocity Tracking Error
    𝑓 = Dynamical Quantities

 ```

Our objective is to select a control torque 𝜏(t) which motivates the velocity for a desired behavior.

Learn more about [Backstepping](https://en.wikipedia.org/wiki/Backstepping)

#### Neural Network Control ####

The concept of neural network control comes in when traditional control mechanism cannot correctly estimate the torque (𝜏). The drawback with neural network is that we need to train it before it can be used. For the project we are using the trace from Nonlinear Backstepping using bug2 on simple maps as initial trainer. This design supports online training which means the it keeps learning during run-time.

The implementation here is a discrete time implementation where the weight update law is given by:

```
For Hidden Layer Weights,
    𝑊1(𝑘 + 1) = 𝑊1(𝑘) − 𝛼1*𝜙1*(𝑘)[ 𝑦1(𝑘) + 𝐵1*𝑘𝑣*𝑟(𝑘)]^𝑇
Where,
    𝑊1 = Hidden layer weight
    𝜙1(𝑘) = Activation function
    𝛼1=learning rate, 𝐵1=constant matrix
    𝑦1(𝑘) = Output of previous layer
    𝑘𝑣 = proportionality constant
    𝑟(𝑘) = filter tracking error

For Output Layer:
    W2(𝑘 + 1) = 𝑊2(𝑘) − 𝛼2*𝜙2(𝑘)*𝑟(𝑘)^𝑇
Where,
    𝑊2 = Output layer weights
    𝜙2(𝑘) = Activation function
    𝛼2 = learning rate
```
Learn more about [Discrete-time Neural Network](http://www.dlsi.ua.es/~mlf/nnafmc/pbook/node21.html)

### Sensor ###

The sensor is a simulated LIDAR sensor.

Learn more about [LIDAR](https://en.wikipedia.org/wiki/Lidar)

## Results ##

Read the original paper containing results [here](https://drive.google.com/file/d/0B7b-pBapfUqkNEF1VWczM2J2Zk0/view?usp=sharing)
