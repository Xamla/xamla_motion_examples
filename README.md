# xamla_motion_examples

This repository contains several examples being part of a Rosvita project

## example_01_pick_and_place

The first example shows a simple pick and place operation.
It should show several aspects:

* reading JointValues from the Rosvita WorldView
* executing a Supervised MoveJoints operation
* executing a sequence of MoveJoints operations
* simple use of a Gripper (e.g. close & open)

All joint values needed for the example are read from the world view.
The movement to the start joint values is supervised, the rest of the movement in this example is not.

## example_02_palletizing

This example shows how the robot can do pick and place operations for a set of poses.
It should cover several aspects:

* calculating a grid of poses using a single pose in world view for orientation and translation
* generating the joint values for the place and pre place poses
* collision free pick and place movement from a home configuration addressing every pose in the grid

The generated joint values are saved in world view for visualization.
