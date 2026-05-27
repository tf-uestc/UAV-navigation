# uav_vln_bringup launch files

- `mavros_px4_sitl.launch`: starts MAVROS using `mavros/px4.launch` with a local SITL `fcu_url`.
- `takeoff_land_demo.launch`: starts MAVROS and runs the `takeoff_land.py` Offboard demo.

> Note: This bringup package **does not** start PX4 SITL/Gazebo itself. Start PX4 SITL first (e.g. in `/home/tf/PX4-Autopilot`) and then run the demo launch.
