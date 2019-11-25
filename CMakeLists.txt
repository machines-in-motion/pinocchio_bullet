cmake_minimum_required(VERSION 2.8.3)

project(pinocchio_bullet)

find_package(catkin REQUIRED COMPONENTS
    mpi_cmake_modules
)

catkin_package()
catkin_python_setup()

##########################
# building documentation #
##########################
build_doxygen_documentation()