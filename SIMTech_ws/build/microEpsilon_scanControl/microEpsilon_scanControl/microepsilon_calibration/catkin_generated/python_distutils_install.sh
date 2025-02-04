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

echo_and_run cd "/home/jeeva/GitHub/SIMTech_ws/src/microEpsilon_scanControl/microEpsilon_scanControl/microepsilon_calibration"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jeeva/GitHub/SIMTech_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jeeva/GitHub/SIMTech_ws/install/lib/python3/dist-packages:/home/jeeva/GitHub/SIMTech_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jeeva/GitHub/SIMTech_ws/build" \
    "/usr/bin/python3" \
    "/home/jeeva/GitHub/SIMTech_ws/src/microEpsilon_scanControl/microEpsilon_scanControl/microepsilon_calibration/setup.py" \
     \
    build --build-base "/home/jeeva/GitHub/SIMTech_ws/build/microEpsilon_scanControl/microEpsilon_scanControl/microepsilon_calibration" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jeeva/GitHub/SIMTech_ws/install" --install-scripts="/home/jeeva/GitHub/SIMTech_ws/install/bin"
