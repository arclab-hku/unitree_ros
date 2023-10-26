These files are derived from the `develop` branch of the [franka_description repository](https://github.com/frankaemika/franka_ros/tree/develop/franka_description)

The `panda_arm.xacro` was originally used for modeling the real robot. In that case, it does not contain collision
geometries and inertia properties, besides, the joints does not contain acceleration and dynamics. 
To compensate that, we modified it based on `panda_gazebo.xacro`. But notice that
the joint limits in `panda_gazebo` is larger than official values given in the [doc](https://frankaemika.github.io/docs/control_parameters.html#constants), which are the same as the values in
`panda_arm.xacro`. Hence, these values are not changed.

