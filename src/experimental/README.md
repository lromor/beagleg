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
  

