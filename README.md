# xamla_motion_examples
This repository contains a rosvita project containing several examples

## example_01_pick_and_place
The first example shows a simple pick and place operation.
It should show several aspects:
* reading JointValues from the Rosvita WorldView
* executing a Supervised MoveJoints operation 
* executing a sequence of MoveJoints operations
* simple use of a Gripper (e.g. close & open)

All joint values needed for the example are read from the world view.
The movement to the start joint values is supervised, the rest of the movement in this example is not. 
