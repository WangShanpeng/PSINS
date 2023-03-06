# Precise Strapdown Inertial Navigation System (PSINS) Toolbox for MATLAB

## 1. Preface

Precise Strapdown Inertial Navigation System (PSINS) toolbox for MATLAB is an open source program package, primarily developed for inertial-grade or higher grade inertial navigation system simulation and data processing. PSINS toolbox includes strapdown inertial sensor (gyro & accelerometer) sampling simulation, initial self-alignment simulation, pure SINS navigation algorithm simulation, SINS/DR & SINS/GPS integrated navigation simulation and many other useful routes, which are all implemented by a bunch of powerful library functions. The PSINS library functions are well modularized and organized, then they are easy to understand and master. Surely, PSINS toolbox has the capability to processing real SIMU and GPS sampling data with a little or even no modification. On the basis of this PSINS toolbox, users can quickly and conveniently set up an inertial navigation solution to achieve their specific purpose.

## 2. License

The PSINS toolbox is distributed under the BSD 2-clause license

(see http://opensource.org/licenses/BSD-2-Clause). Users are permitted to download, copy, modify and redistribute this toolbox freely as long as they comply with the following license.

Copyright (c) 2009-2015, Gongmin Yan, All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

## QQ Group

468195931

[jq.qq.com/?_wv=1027&k=G7HPNTQU](https://jq.qq.com/?_wv=1027&k=G7HPNTQU)

[![ppVUaXd.png](https://s1.ax1x.com/2023/03/06/ppVUaXd.png)](https://imgse.com/i/ppVUaXd)