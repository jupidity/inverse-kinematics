[image1]: ./photos/originDiagram.png
[image2]: ./photos/angle.png
[image3]: ./photos/stack.png


# Udacity Pick and Place Project Writeup
---


The Amazon/Udacity pick and place challenge consists of controlling a robotic arm Gazebo simulation to pick up a spawned object and place it in a bin. The simulation environment is available at:

https://github.com/udacity/RoboND-Kinematics-Project

Most of the robotics tasks are covered by the Github repo, the package comes with:

- a urdf file containing the description of the robotic arm.
- a gazebo world file containing the description of a simulated environment with which to place the arm
- a collection of launch files to launch almost all ROS nodes required for the project to function
- shell script to start ROS master, MoveIt trajectory planning nodes, joint controller nodes, Gazebo simulation, and Rviz ROS visualizations


There are two main problems that this project requires us to solve in order for the Kuka arm to function autonomously.

forward kinematics:

*"if I know my joint angles, whats my end effector position and orientation?"*

and inverse kinematics:

*"if I know my end effector position and orientation, what joint angles could have produced it?"*

# Forward Kinematics
---

Forward kinematics begins by representing each joint as the coordinate reference frame that describes its position and orientation. Changes to the joint state can be seen as changes to the coordinate frame relative to a global frame. The parameters of interest are the locations of each joint, which can be found by applying a series of homogeneous transforms to the previous coordinate reference frame dependent on its current state. If all homogeneous transformations are chained together, we can derive a matrix that represents the total transform between the base link and end effector.



In general, there are six parameters required to completely specify an object in 3d space ``[x,y,x,row,psi,theta]``. Hence, if we were to specify a complete homogeneous transformation from one joint reference frame to another, we would need six different elementary transforms (three rotations and three translations). In an attempt to minimize these calculations, the DH parameters specify a convention for assigning the coordinate frame such that only 4 parameters and thus 4 elementary transformations per reference frame are required. The conventions are as follows:

For each reference frame `i`, define the `z_i` axis to point about the axis of rotation for rotary joints, and along the direction of motion for prismatic joints. Then specify the `x_i` axis to be the mutually perpendicular normal to both the `z_i+1` and `z_i` axes.

Then in order to specify the homogeneous transform between the reference frames, we need to specify the correct rotation and translations between these axes.

The `a` parameters are the rotations and translations of the `z` axis, and are measured relative to the `x` axis.
- ``α​i`` (twist angle)
    The twist angle α​i is the angle between the `z_i` and `z_i+1` axes measured about the mutually perpendicular `x_i` axis.
- `a​i−1`​​ (link length)
    The link length `ai-1` is the distance between the `z` axes. Since the `x_i-1` axes is mutually perpendicular to both `z_i` and `z_i-1`, this marks the shortest line between the two, and the distance is measured along this axis.
- ``d​i​​`` (link offset) = signed distance from ​`x​_​​​i−1`​​ to ​`x_​​​i`​​ measured along ​`z_​​​i`​​. Note that this quantity will be a variable in the case of prismatic joints.
- ``θi``​​ (joint angle) = angle between ​`x_​​​i−1`​​ to ​`x_​​​i`​​ measured about ​`z_​​​i`​​ in a right-hand sense. Note that this quantity will be a variable in the case of a revolute joint.


Once these quantities are specified, the full homogeneous transform between the links can be derived. This is essentially the entirety of the forward kinematics problem, find the complete transformation between the joints as a function of their rotation and/or extension to derive the end effector location and orientation. For prismatic joints, the `d` length is the variable parameter, and for rotary joints the theta value is the only variable parameter. All other parameters can be found in the robots urdf file and passed into the homogeneous transform matrix upon definition.




first, lets determine the FK solution. From the urdf file, the relative positions and orientations of the joints are defined in relation to its parent joint. Thus, starting with the base joint, we can find the d and a values from the urdf file. The only complication is determining which translation corresponds to which parameter. In order to accomplish this, we just need to be observant of the orientations of each joint.

The following diagram was provided by the Udacity team to give helpful suggestions as to the origin placement and axes assignment for optimal homogeneous transformation matrices between reference frames.


![alt text][image1]


Using the above diagram and the urdf file, the DH parameter table can be filled in. In order to be verbose about my thought process, I'll outline one such parameter assignment process for the 1st joint.  

- `a1` is the distance between `z1` and `z2` along the `x1` axis. `J1` rotates about the global `z` axis, which means `z1` is parallel to the global. `Z2` points along the global `y` axis, meaning the `x1` axis points along the global `x` axis, so `a1` corresponds to the `x` displacement between `j1`,`j2` on the urdf file, or .35m.
- `α1` is the angle between `z1` and `z2` about `x1`. The `z2` axis is 90 degrees in the negative direction from the `z1` axis about `x1` according to the right hand rule, so `α1` = -90.
- `θ1` is the angle between x0 and x1 about z1. `θ1` is 0 since `z0` is coincident with `z1`.
- `d1` is the signed distance between `x0` and `x1` along `z1`. `x0` is at `z` = 0, and `x1` is at the global `z` location of `j2`, or .33m + .42m = .75m   


i | `α` (degrees) |​ `a` (m) | `d` (m) | `θ` (degrees)
-------------|---|---|---|---
0|0|0|0|0
1|-90|.35|.75|0
2|0|1.25|0|q2 - 90
3|-90|.054|1.5|0
4|90|0|0|0
5|-90|0|0|0
6|0|0|0|0
7|0|0|.303|0

Since the Kuka arm consists entirely of rotary joints, the d values of the DH parameter table are constants and can be filled in at declaration. The variable parameter at each computation step is the θ_i parameter, leading to the following homogeneous transformation between links:

        T(i-1)_ (i) =
             [                  cos(θi),                  -sin(θi),              0,                 a_i-1]
             [ sin(θi) * cos(alpha_i-1), cos(θi) * cos(alpha_i-1), -sin(alpha_i-1),    -sin(alpha0) * d_i]
             [ sin(θi) * sin(alpha_i-1), cos(θi) * sin(alpha_i-1),  cos(alpha_i-1),  cos(alpha_i-1) * d_i]
             [                        0,                        0,               0,                     1]


which can be filled in by the above table

The DH parameter reference frames are defined in relation to the local coordinate system, so reference frame transformations must be intrinsicly rotated. Once each transform between joint frames is known, we can construct the full homogeneous transform by pre multiplying each individual transform.

      R0_G = ( ( ( ( ( ( T0_1 * T1_2) * T2_3 ) * T3_4 ) * T4_5 ) * T5_6 ) * T6_7)


The result is the full homogeneous transform from base to end effector in the DH frame. There is still one step required if we with to compare results with the Rviz simulation. The DH gripper frame is defined differently from the gripper frame in Rviz, so we must rotate the R0_G frame such that it coincides with the Rviz axes definition to compare results.

     T_corr = simplify(R_z.evalf(subs = {r3:np.pi}) * R_y.evalf(subs ={r2:-np.pi/2} ) )
    T_tot = T0_G * T_corr

Finally, we have a complete homogeneous transformation from base frame to gripper frame.




# Inverse Kinematics
---

 The inverse kinematics problem is the opposite: given an end effector location and orientation, what are the joint settings that give that location and orientation? While analytic methods exist, the suggested method is to find a particular solution for the application in question that requires you to solve a series of equations given by the layout of actuators and bounded by their ranges. For the Kuka arm, there are 6 rotary joints. Since we have a gripper arm with a spherical wrist joint, we can decouple the translation and rotation steps into two separate problems.

A decoupled `RRR` - `RRR` robotic arm can be thought of as comprised of two separate components: a `RRR` robotic arm the controls the `x`,`y`,`z` position of the wrist center, and a `RRR` spherical wrist that controls the orientation of the end effector in relation to the wrist center.  

From observation of the joint types and orientations, it appears joints `J1`-`J3` control the `x`,`y`,`z` component of the `ee`, while joints `J4`-`J6` control the orientation.

Lets start with `J1`-`J3`. In this architecture, the `IK_server.py` node is given a location and orientation for the end effector. Since `J1`-`J3` are responsible for controlling the `x`,`y`,`z` location of the wrist center, not the end effector, we must first determine the position of the wrist center if we wish to determine the required values of `J1`-`J3`. Thus we need to derive a unit vector pointing in the direction of the end effector relative to the wrist center, and then step backwards from the end effector along this vector a length equal to the distance from the end effector to the wrist center.

first, we need to determine exactly what information we are provided with. By running the `rosmsg` command, we can obtain the exact contents of the message sent to the `IK_server.py` node.

    $ cat CalculateIK.srv

produces

    geometry_msgs/Pose[] poses
    ---
    trajectory_msgs/JointTrajectoryPoint[] points

which tells us the `IK_Server.py` node receives a Geometry_msgs/Pose, while

    $ rosmsg show geometry_msgs/Pose


shows that we are given an `x`,`y`,`z` pose in 3D, and orientation in quaternions. We can convert the quaternion representation to roll, pitch, and yaw, and generate a rotation matrix between the base frame and desired frame from the `rpy` values.

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([req.poses[x].orientation.x,req.poses[x].orientation.y,req.poses[x].orientation.z, req.poses[x].orientation.w])

 A 3D rotation matrix is generated by post multiplying three elementary rotations about each of the principle axes. In this case, the rotation matrix is given by:

    R0_G = R_z(yaw) * R_y(pitch) * R_x(roll)


Where `R0_G` is the rotation matrix from the base frame to the gripper frame as defined in the urdf file. Lastly, we need to apply the rotation from the urdf gripper frame to the DH parameter gripper frame. Since we solved for the matrix from the DH gripper frame to the urdf gripper frame in the forward kinematics solution above, we can just invert this matrix for the reverse transformation:


      R_corr = R_rot * (R_z.evalf(subs={r3:pi}) * R_y.evalf(subs={r2:-pi/2})).inv()




According to our mathematical representation, the joints are rotating the reference frames relative to each-other, while the links and joints have a static position and orientation within their local reference frame. Thus within the gripper frame, the gripper will always point along the `z`-axis, but adjustments to the joint angles will change the gripper frames' `z`-axis relative to the base frame. Thus if we have the rotation matrix from the base frame to the DH parameter gripper frame, the 3rd column will specify the orientation of the gripper frame `z` axis in the base frame, and thus specify a unit vector pointing in the direction of the gripper. Once we have the base frame `x,y,z` components of the gripper frame `z`-axis, we can step backwards a length `l` in each direction (where `l` is the scalar length from the gripper to the wrist center) to obtain the wrist centers `x,y,z` coordinates in the base frame.


The wrist center is at the same location as `J5`, so to find `l` we must find the length between `J5` and the gripper along the kuka arm. From the urdf file, we can see that the gripper joint has length ``.11m`` in the `x` direction relative to `J6`, and `J6` is ``.196m`` from `J5` so `l = .11m + .196m = .303m`. Everything together produces the equation:

    wc_pos = ee_pos -(.303) * R_corr * [0,0,1]


where `wc_pos` is the `x`,`y`,`z` coordinate of the wrist center, and `ee_pos` is the `x`,`y`,`z`, coordinates of the end effector




once we have the wrist centers `x`,`y`,`z` coordinates in the base frame, we can move on to solving for the orientation of `J1` - `J3`.

From observation, if we drop the z component for the wrist center, the remaining `wc_x`,`wc_y` parameters define the angle of the first joint `J1` from the expression `q1 = atan2(wc_y,wc_x)`.

All that remains is to solve for the two remaining joint rotations `q2`,` q3`

Once we have the angle `q1`, it becomes useful to think about the setup in terms of cylindrical coordinates to solve for `q2` and `q3` since `J2` and `J3` lie in the `θ` = `q1` plane.

Below is a drawing of a sample arm orientation in the `z`, `theta = q1` plane

![alt text][image2]


Here `L1` represents the length from `J1` to `J3` along link 1, `L2` represents the length from `J3` to `J5`, and `r` is a vector pointing from `J2` to the wrist center at `J5`.  

The `J2` `z` component can be found in the urdf file, the `J2` `x,y` component is given by `J2_x = a1*cos(q1)`, `J2_y = a1*sin(q1)`, and the three can be used to find `d`

      d = atan2((wc_z - d1), sqrt((wc_x-a1*cos(theta1))^2 + (wc_y-a1*sin(theta1)) ^2))

and the angles `a`, `b`, and `c` can be found from the law of cosines applied to the `L1`,`L2`,`r` triangle, resulting in the expressions:

      cos(b) = (r**2 - L1**2 -L2**2) / (-2*L1*L2) := B
    cos(a) = (L2**2 - L1**2 -r**2) / (-2*L1*r) := A

Since cosines and sines can return ambiguous values, we can take advantage of the relationships `tan(q) = sin(q)/cos(q)` and `1 = sin^2(q) cos^2(q)` to write the above relationships as

      a = atan2(sqrt(1-A^2), A))
    b = atan2(sqrt(1-B^2),B)

Once known, `q2` and `q3` can be found by solving the equations

      90 = q2 + a + d
    90 = q3 + b

However, after inspecting the urdf file, it appears that while `J4` and `J5` are separated only be a distance along the global `x` axis, `J4` is slightly offset from `J3` in the `z` direction as well as the global `x` direction. As a result, `J5` ,and thus the wrist center,  does not lie on the `y3` axis. In order to account for this, we can subtract the difference in angle from `y3` and `z4` from the value we derived for `q3` above. From inspection of the urdf file, we can see that this angle equates to  `atan2(.056,1.5)`.

Everything together results in the complete expressions for `q1`,`q2`,`q3`:

      q1 = float(atan2(wc_y,wc_x))
    q2 = float(pi/2 - atan2((wc_z - d1), sqrt((wc_x-a1*cos(q1))^2 + (wc_y-a1*sin(q1)) ^2)) - atan2(sqrt(1-A ^2), A))
    q3 = float(pi/2 - atan2(sqrt(1-B^2), B) - atan2(.056,1.5))

Once the first three joints are known, the remaining challenge is to find the final three joints that control the orientation of the end effector. We can use the knowledge of homogeneous transformations to equate the forward kinematics solution derived earlier to `R_corr` used to determine the wrist center:

      R_corr = R0_6

We can split ``R0_6`` into two terms:

      R0_6 = R0_3 * R3_6

leading to

      R3_6 = inv(R0_3) * R_corr := r

here, `inv(R0_3)` consists entirely of known quantities since we now know `q1`,`q2`, and `q3`, meaning the RHS can be solved for numerically. On the LHS, we can solve for `R3_6` symbolically using the same techniques from the forward kinematics section:

      R3_6 =  (T3_4 * T4_5) * T5_6 )

resulting in the matrix:

    [-sin(q4) * sin(q6) + cos(q4)* cos(q5)* cos(q6), -sin(q4)* cos(q6) - sin(q6)* cos(q4)* cos(q5), -sin(q5)* cos(q4)],
        [                             sin(q5)* cos(q6),                           -sin(q5)* sin(q6),          cos(q5)],
      [  -sin(q4)* cos(q5)* cos(q6) - sin(q6)* cos(q4),  sin(q4)* sin(q6)* cos(q5) - cos(q4)* cos(q6),  sin(q4)* sin(q5)]



Inspection of the above matrix shows we are immediately given a numerical value for the `cos(q5)` in `r23`. This is still ambiguous for `q5`, but does provide us with a numerical value for the sine as `sin(q5) = sqrt(1 - r23^2)`. we can substitute the sine value into `r21` and `r22` to get a numerical value for `cos(q6)`, `sin(q6)`, leading to a numerical value for `q6` from `q6 = atan2(sin(q6),cos(q6))`. Similarly, we can substitute `sin(q5)` into `r13` and `r31` to solve for `q4` via `atan2`. Everything together results in the equations:


    theta6 = float(atan2((-R3_6[4]/sqrt(1-R3_6[5]^2)),(R3_6[3]/sqrt(1-R3_6[5]^ 2))))
    theta4 = float(atan2((R3_6[8]/sqrt(1-R3_6[5]^2)),(-R3_6[2]/sqrt(1-R3_6[5]^2))))
    theta5 = float(atan2((R3_6[3]/cos(theta6)),R3_6[5]))

And we now have numeric values for all six thetas.

## Results
---
The above inverse kinematics solution gets very good accuracy and repeatability when used in the Gazebo simulation provided

![alt text][image3]

Above: The Kuka arm successfully stacking 6 cylinders in simulation

The main problem is the speed of execution. IK_server.py is written using Sympy primarily, and as a result takes noticeable time to return the joint angle solutions when called. Sometimes resulting in the warning:

[ WARN] [1499277460.747253884, 1277.949000000]: Dropping first 1 trajectory point(s) out of 28, as they occur before the current time.
First valid point will be reached in 0.041s.

when computation took too long.

A better node would retreive the homogeneous transformation matrices from the parameter server for re-usability, and use numpy for matrix calculations to improve execution speed. Since this exercise was more focused on finding the mathematical solution to the inverse kinematics problem, I concluded with the Sympy solution.

Additionally, there are moments in the simulation where the arm adjusts its ee orientation by pi about the z_G axis before continuing its trajectory. This is likely due to not specifying a single quadrant for joint angle solutions. A better IK node would compute all working joint angle configurations and return the one with minimal joint difference from the previous state.
