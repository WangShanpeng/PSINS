# Precise Strapdown Inertial Navigation System (PSINS) Toolbox for MATLAB

## 1. Preface

Precise Strapdown Inertial Navigation System (PSINS) toolbox for MATLAB is an open source program package, primarily developed for inertial-grade or higher grade inertial navigation system simulation and data processing. PSINS toolbox includes strapdown inertial sensor (gyro & accelerometer) sampling simulation, initial self-alignment simulation, pure SINS navigation algorithm simulation, SINS/DR & SINS/GPS integrated navigation simulation and many other useful routes, which are all implemented by a bunch of powerful library functions. The PSINS library functions are well modularized and organized, then they are easy to understand and master. Surely, PSINS toolbox has the capability to processing real SIMU and GPS sampling data with a little or even no modification. On the basis of this PSINS toolbox, users can quickly and conveniently set up an inertial navigation solution to achieve their specific purpose.


## 3. System Requirements

When developing this toolbox, the author’s PC setting is: 

Microsoft Windows 7 (SP1) + MATLAB 8.2.0 (R2013b) + CPU 2.1GHz + RAM 2.0GB.

## 4. Quick Start

1. Copy the PSINS toolbox root folder `psins\`, including all subfolders and files, to your computer.
2. Run `psins\psinsinit.m` to initialize PSINS environment.
3. Run `psins\demos\test_SINS_trj.m` to generate a moving trajectory.
4. Run `psins\demos\test_SINS_GPS.m` to demonstrate SINS/GPS integrated navigation.
5. There are many demo examples in `psins\demos`, such as coning & sculling motion demonstration, initial alignment, pure inertial navigation and POS data fusion, etc.
6. Try to do some modification and put your exercise file under `psins\mytest`. Enjoy yourself and may you find something helpful!
