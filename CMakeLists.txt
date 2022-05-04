cmake_minimum_required(VERSION 3.22.0)

project(3DRP CXX)

###### DOWNLOAD ALL THE CLOUDVIEWER SUBMODULE ###### 
find_package(Git QUIET)
option(GIT_SUBMODULE "Check submodules during build" ON)

if(GIT_FOUND AND GIT_SUBMODULE AND NOT EXISTS "${PROJECT_SOURCE_DIR}/CloudViewer/CMakeLists.txt")
    message(STATUS "Submodule Cloudviewer update")
    execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                    RESULT_VARIABLE GIT_SUBMOD_RESULT)
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
        message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
    endif()
endif()
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/CloudViewer/CMakeLists.txt")
    message(FATAL_ERROR "The CloudViewer submodules was not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

# CMAKE SETUP FROM: https://stackoverflow.com/questions/34438357/cmake-one-build-directory-for-multiple-projects
######  3DRP Core Inclusion ###### 
add_subdirectory(core)

######  GUI Inclusion & Initialization ###### 
add_subdirectory(CloudViewer)
add_subdirectory(ui)


message(WARNING "Note: If encountered 'Unable to start program', go to build/CloudViewer/src/[Debug|Release] OR set CloudViewer as 'Startup Project'")
