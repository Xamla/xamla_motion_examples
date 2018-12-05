# xamla_motion_examples

This repository contains several examples being part of a Rosvita project.
Every example script lies in a corresponding folder in the Rosvita project.
For some examples, a corresponding directory can be found in the Rosvita WorldView containing objects which are used or generated by the script. 

## Example 01: Pick and place

The first [example script](example_01/example_01_pick_and_place.py) shows a simple pick and place operation.
It should show several aspects:

* reading JointValues from the Rosvita WorldView
* executing a Supervised MoveJoints operation
* executing a sequence of MoveJoints operations
* simple use of a Gripper (e.g. close & open)

All joint values needed for the example are read from the WorldView in the corresponding folder.
The movement to the start joint values is supervised, the rest of the movement in this example is not.

## Example 02: Palletizing

This example shows how the robot can do pick-and-place operations for a set of poses.

Since the example does a lot of things, it is subdivided into several simple ones which can be run individually.
Running the file [example_02_palletizing.py](example_02/example_02_palletizing.py) puts everything together.
The features are:

* Given a pose and some parameter defining the size, calculate a grid of poses with translation and rotation corresponding to the pose [example_02_1_generate_grid.py](example_02/example_02_1_generate_grid.py)
* Generate some CollisionObjects at a list of poses [example_02_2_create_collision_boxes.py](example_02/example_02_2_create_collision_boxes.py)
* Using inverse kinematics, get corresponding JointValues for a list of poses [example_02_3_create_joint_values_from_poses.py] (example_02/example_02_3_create_joint_values_from_poses.py)
* Do a pick and place operation from the pick JointValues to every Pose, using `move_joints_collision_free` for movement between pick and preplace and linear movement between preplace and place poses [example_02_4_pick_place_poses_linear.py](example_02/example_02_4_pick_place_poses_linear.py) ( [alternative linear version](example_02/example_02_5_pick_place_poses.py) ).

The script [example_02_palletizing.py](example_02/example_02_palletizing.py)  runs the subexamples 1-4 to do the following:

* Calculating a grid of place poses using a single pose (named `GridPose`) in WorldView for orientation and translation
* Showing some cubes below the poses to show where they are and as a CollisionObject to be avoided.
* Calculate a set of pre place poses hovering over the place poses in z-direction (the grid spans in x and y direction from view of the `GridPose` Pose)
* Generating the JointValues for the pre place poses using inverse kinematics.
    The JointValues at the pick position (named `Home`) is used as a seed, since we move back and forth from it to the pre place pose.
    If there could not be found JointValues for every pose, an exception is thrown.
    In that case, one can alter the `GridPose` and/or the size of the grid before running the example again, until a valid set of poses has been found.
* Collision free pick and place movement from the `Home` configuration addressing every pose in the grid.

Objects generated by the examples are written in most cases to WorldView for visualization.
For example, user can see the JointValues calculated from the poses or observe how the grid is generated differently when its defining pose is altered in WorldView.

Every example has a main function with type annotation, which should make including them into an xgraph straight-forward.

## Example 03: Shifting

This [example script](example_03/example_03_shiftig_joint_values.py) shows how to shift JointValues in cartesian space.

Joint values can be added and changed in `example_03_shifting/jointValues/` in WorldView.

The folder `example_03_shifting/poses/` in WorldView contains a `Reference` and a `Shift` pose.
The transformation applied in Cartesian Space is defined by the translation/rotation of the `Shift` pose in reference to the `Reference` pose. 

Every end effector pose is transformed this way. Inverse kinematics is used to get JointValues at this new pose.
If the inverse kinematics operation yields no result, the corresponding JointValues are skipped.

The resulting joint values are stored in `example_03_shifting/generated/`.

## Example 04: Collision objects

This [example script](example_04/example_04_collision_objects.py) shows how the robot can move while evading `CollisionObjects`.

First `CollisionObject`s are generated from poses read from `WorldView`.
The arm of the robot moves back and forth between two joint values configuration.
For every movement, a different `CollisionObject` is added to `WorldView` and taken into account when planning the movement.

## Example 05: Sinus shaking trajectory

This [example script](example_05/example_05_sinus_shaking_trajectory.py) shows how to produce sinus shaped motion given a list of poses

The script first creates a list of poses consisting of the current pose of the end effector.
Then a new `CartesianPath` is created by applying translation along a given axis following a sinus function.
Linear movement of this path is then executed.
Setting the parameter for amplitude, frequence and axis to change the behaviour of the movement.

## Example 06: Null space movement

This [example script](example_06/example_06_null_space.py)  shows how to achieve movement of the torso joint of the robot while keeping its end effectors locked at their poses.

## Example 07: Jogging

This set of examples shows how to use the jogging client using the python interface.

* The [first script](example_07/example_07_3_twist.py) reads continuously a pose from world view and uses jogging to move the EndEffector to that position.
* The [second script](example_07/example_07_2_rotate_joint.py) show how a joint can be addressed and moved directly.
* The [third script](example_07/example_07_3_twist.py) show how a twist can be applied to an end effector.
