#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/lib/python3.9/site-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/lib/python3.9/site-packages:/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/tesse_ros_bridge/lib/python3.9/site-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/tesse_ros_bridge" \
    "/opt/anaconda3/envs/ros_env/bin/python3.9" \
    "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/src/tesse-ros-bridge/ROS/setup.py" \
     \
    build --build-base "/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/build/tesse_ros_bridge" \
    install \
    --root="${DESTDIR-/}" \
     --prefix="/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install" --install-scripts="/Users/stanley/Documents/COURSE/PROGRAMME/C_Family/Robotics_MIT/Labs/Lab3/src/catkin_ws/install/bin"
