# CMakeLists.txt - HEAD directory CMakeLists file
#
# M.Overdick 10/21/16
cmake_minimum_required(VERSION 2.8)

# Some Boost components require being set here (improves compile time)
set(BOOST_REQUIRED_COMPONENTS
    system
    thread
)

find_package(Boost "1.46" REQUIRED ${BOOST_REQUIRED_COMPONENTS})
find_package(UHD "3.10.0" REQUIRED)

# Directories to be included (for header files)
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
    ../include/
)

link_directories(
    ${Boost_LIBRARY_DIRS}
)

set(RESOURCES
    ../include/sinc.cpp
    ../include/writebuff.cpp
    ../include/checklo.cpp
)

add_executable(masternode masternode.cpp ${RESOURCES})

SET(CMAKE_BUILD_TYPE "RELEASE")
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mavx2")

target_link_libraries(masternode ${UHD_LIBRARIES} ${Boost_LIBRARIES})
