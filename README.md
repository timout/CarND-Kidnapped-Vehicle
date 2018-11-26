# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Running the Code

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

## Results
### Performance
The particle filter met the requirements, which were 1 meter in error for x and y translations, 0.05 rad in error for yaw, and 100 seconds of runtime for the particle filter. Error below is cumulative mean weighted error.
  
**Number of particles: 100**

**Runtime: 56.32 seconds**

| Estim |  Error  |
| ----- | ------- |
|   x   | 0.117 | (m)
|   y   | 0.103 | (m)
|  yaw  | 0.004 | (rad)
