install(PROGRAMS create_thruster_vsp.sh
        DESTINATION lib/${PROJECT_NAME})
install(FILES create_thruster_vsp.sh
        DESTINATION share/${PROJECT_NAME}/scripts)

install(PROGRAMS start_switch.py
        DESTINATION lib/${PROJECT_NAME})
