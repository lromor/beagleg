# Experiments

## New planner

We want to digest a set of enqueued curve primitives and, based on the motor constraints
generate a smoothed trajectory and an optimal motion profile.

``` sh
cat ../testdata/square-moves-direct.gcode | ./planner-experiment
```

For now it just spits the enqueued primitives as json lines.

Next:

- [ ] Spit a new queue of primitives containing the planned starting and final speed.
- [ ] Write a plotting utility that spits:
  * different between original and smoothed curve
  * motors acceleration
  * curvature
  

# Planner algorithm overview

The planning algorithm can be split in the following steps macro steps.
The planner takes as input a trajectory (T) and spits a planned trajectory (PT)
in terms of a sequence of cubic-bezier (CB) curves.

0. (COMPRESSION?): Operation offloaded as preprocessor of the gcode.
1. (SMOOTHING): Digest the original trajectory and spit a sequence of cubic bezier segments.
2. (SPEED-PROFILE PLANNING): Compute a speed profile for the generated smooth trajectory that satisfy the provided constraints.
  
  
## COMPRESSION
The sequence of gcode primitives define a trajectory. As first step, we try to build a more uniform (in terms of primitives length) trajectory. 
We can accomplish this by fitting the original trajectory with a bezier curve then reaching total length (based on the capabilities of the backend.).
For the sake of the experiment, we choose a maximum length of 1cm. This should be chosen based on the maximum bezier interpolation parameter value for each motors.
Once we decided where to cut, we merge all the primitives and fit the original trajectory with a single bezier curve.
We estimate (as exactly as possible) the analitical error of the representation initially we could simply estimate it using least squares and some uniform sampling). 
If the error is too big, we split the obtained trajectory in the maximum error location and interpolate the two chunks while enforcing continuity
if the first derivative and repeat recursively the process until the error reached the required eps.
A similar process is described [here](https://link.springer.com/article/10.1186/s10033-019-0360-8).
See [this](http://www.inf.ed.ac.uk/teaching/courses/cg/d3/bezierJoin.html#:~:text=When%20two%20B%C3%A9zier%20curves%20are,tangent%20vector%20at%20the%20join) for an example.

## SMOOTHING

Compression and smoothing could also be made as a single step by interpolating or fitting with a maximum tolerance
the original trajectory resulting in a smooth trajectory defined as sequence of smooth bezier curves.
Without the compression steps, the trajectory is defined as a set of trajectory primitives that we try to smootly blend.
A sample algorithm is presented [here](https://link.springer.com/article/10.1007/s00170-014-6386-2).

## SPEED-PROFILE PLANNING
At this point, we have a sequence of cubic bezier curves, for each of these, there's an associated gcode command and a feedrate.
Feedrates define a discontinuous feed profile which is simply smoothed to respect the provided motor constraints.
Once for each segment we get a speed-start, which is the actual speed at which the segment will start and a target speed speed-end.
Speed-end is not required to be met. Provided the start and target speed, we define a cubic bezier profile such that the acceleration
due to the curvature and speed motor-constraints are met. If the constraints are not met, the speed is reduced.
The digested speed profile is continuosly updated whenever new trajectory segments are enqueued.
The behavior of the system should be that the lookahead buffer should always provide a deceleration profile 
to be followed whenever the enqueued amount of steps per motor is not sufficient to reach zero speed.
This is easily implemented by backstepping the last speed from zero until reaching the last speed.

Finally we pass the motion segments to the backend, ready to be executed.
