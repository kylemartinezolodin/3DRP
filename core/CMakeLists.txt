file(GLOB_RECURSE DIR_SRCS    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE DIR_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

add_library(3DRPCore ${DIR_SRCS} ${DIR_HEADERS})

find_package(CGAL)

target_include_directories(3DRPCore 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(3DRPCore 
    CGAL::CGAL
)

