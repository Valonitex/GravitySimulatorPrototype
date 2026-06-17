cmake_minimum_required(VERSION 4.2)
project(PhysicsApp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(RAYLIB_DIR ${CMAKE_SOURCE_DIR}/dep/raylib-6.0_linux_amd64)

find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(GLEW REQUIRED)

add_executable(GsimTerm GravitySimulatorPrototype/physics.cpp)
add_executable(OpenGLingsim
        OpenGL/src/Application.cpp
        OpenGL/src/IndexBuffer.cpp
        OpenGL/src/VertexBuffer.cpp
        OpenGL/src/Renderer.cpp
        OpenGL/src/VertexArray.cpp
        OpenGL/src/Shader.cpp
        OpenGL/src/Renderer.cpp
        #OpenGL/src/
)
add_executable(RaylibDemo RaylibDemo/main.cpp)

target_include_directories(RaylibDemo PRIVATE ${RAYLIB_DIR}/include)
target_link_directories(RaylibDemo PRIVATE ${RAYLIB_DIR}/lib)
target_link_libraries(RaylibDemo raylib m pthread dl rt X11)

target_include_directories(GsimTerm PRIVATE ../GravitySimulatorPrototype)
target_include_directories(GsimTerm PRIVATE ${RAYLIB_DIR}/include)
target_link_directories(GsimTerm PRIVATE ${RAYLIB_DIR}/lib)
target_link_libraries(GsimTerm raylib m pthread dl rt X11)

target_include_directories(OpenGLingsim PRIVATE OpenGL)
target_link_libraries(OpenGLingsim OpenGL::GL glfw GLEW::GLEW)