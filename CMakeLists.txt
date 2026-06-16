cmake_minimum_required(VERSION 4.2)
project(PhysicsApp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(GLEW REQUIRED)

add_executable(GsimTerm GravitySimulatorPrototype/physics.cpp)
add_executable(OpenGLingsim
        OpenGL/src/Application.cpp
        OpenGL/src/IndexBuffer.cpp
        OpenGL/src/VertexBuffer.cpp
        OpenGL/src/Renderer.cpp
)

target_include_directories(GsimTerm PRIVATE GravitySimulatorPrototype)
target_include_directories(OpenGLingsim PRIVATE OpenGL)
target_link_libraries(OpenGLingsim OpenGL::GL glfw GLEW::GLEW)