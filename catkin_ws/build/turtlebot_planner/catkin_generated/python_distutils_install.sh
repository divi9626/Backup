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

echo_and_run cd "/home/divyam/catkin_ws/src/turtlebot_planner"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/divyam/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/divyam/catkin_ws/install/lib/python2.7/dist-packages:/home/divyam/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/divyam/catkin_ws/build" \
    "/usr/bin/python2" \
    "/home/divyam/catkin_ws/src/turtlebot_planner/setup.py" \
     \
    build --build-base "/home/divyam/catkin_ws/build/turtlebot_planner" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/divyam/catkin_ws/install" --install-scripts="/home/divyam/catkin_ws/install/bin"
