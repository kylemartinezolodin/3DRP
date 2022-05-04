file(GLOB_RECURSE DIR_SRCS    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE DIR_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

add_library(3DRPCore ${DIR_SRCS} ${DIR_HEADERS})

find_package(CGAL)
find_package(PCL 1.10.1 REQUIRED)

target_include_directories(3DRPCore 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(3DRPCore 
    PRIVATE CGAL::CGAL
    ${PCL_LIBRARIES}
)

