
# Require minimum standard version of CMake
cmake_minimum_required (VERSION 3.10)

# Set C++ 17 compiler flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set project name
project(Flocking)

add_subdirectory("External/ImFrame")
set_property(TARGET ImFrame PROPERTY FOLDER "External")

# Add source files and dependencies to executable
set(
	source_list
	"src/main.cc"
    "src/boid.cc"
)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")
add_executable(${PROJECT_NAME} WIN32 ${source_list})
target_link_libraries(${PROJECT_NAME} PRIVATE ImFrame)
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/External/ImFrame/include")  


# Set compiler options
if(MSVC)
	target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)
else()
	target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

# Create folder structure
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${source_list})
