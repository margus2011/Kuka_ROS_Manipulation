execute_process(COMMAND "/home/jeeva/GitHub/SIMTech_ws/build/microEpsilon_scanControl/microEpsilon_scanControl/microepsilon_calibration/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jeeva/GitHub/SIMTech_ws/build/microEpsilon_scanControl/microEpsilon_scanControl/microepsilon_calibration/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
