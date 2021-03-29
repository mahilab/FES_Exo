# CALIBRATION/DATA COLLECTION EXECUTABLES

add_executable(fes_calibration fes_calibration.cpp icon/fes_exo_icon.rc)
target_link_libraries(fes_calibration mahi::fesexo)

add_executable(fes_calibration_dup fes_calibration_dup.cpp icon/fes_exo_icon.rc)
target_link_libraries(fes_calibration_dup mahi::fesexo)

add_executable(recruitment_curve recruitment_curve.cpp icon/fes_exo_icon.rc)
target_link_libraries(recruitment_curve mahi::fesexo)

add_executable(fes_max_recording fes_max_recording.cpp)
target_link_libraries(fes_max_recording mahi::fes)

add_executable(fes_max_recording_gui fes_max_recording_gui.cpp)
target_link_libraries(fes_max_recording_gui mahi::fesexo)

# EXPERIMENT EXECUTABLES

add_executable(single_dof_poc single_dof_poc.cpp)
target_link_libraries(single_dof_poc mahi::fesexo)

add_executable(multi_dof_poc multi_dof_poc.cpp)
target_link_libraries(multi_dof_poc mahi::fesexo)

add_executable(general_movement general_movement.cpp)
target_link_libraries(general_movement mahi::fesexo)

# TEST EXECUTABLES

add_executable(test test.cpp)
target_link_libraries(test meii::meii)