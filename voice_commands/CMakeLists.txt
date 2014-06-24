cmake_minimum_required(VERSION 2.4.6)
cmake_minimum_required(VERSION 2.8.3)
project(voice_commands)



install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY nodes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY model
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
