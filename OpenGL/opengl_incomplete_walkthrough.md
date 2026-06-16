# Complete OpenGL Code Walkthrough — Zero to Everything

---

## Before Any Code: What Are We Actually Doing?

### CPU vs GPU

Your computer has two main processors:

**CPU (Central Processing Unit)** — This is where your C++ program runs. It executes code mostly sequentially. It's fast and flexible but has relatively few cores (4-16 typically). It's bad at doing the same simple operation millions of times in parallel.

**GPU (Graphics Processing Unit)** — Your graphics card. It has *thousands* of tiny cores (a mid-range GPU has 2,000–6,000). Each core is slow and simple compared to a CPU core, but they all run *simultaneously in parallel*. This makes the GPU extraordinary at tasks like "compute the color of every one of the 2,000,000 pixels on screen at the same time."

To draw something on screen, your C++ code (running on the CPU) needs to:
1. Upload geometry data to GPU memory
2. Tell the GPU how to interpret that data
3. Tell the GPU which programs (called "shaders") to run on that data
4. Issue a draw command

OpenGL is the language your CPU-side C++ uses to give all those instructions to the GPU.

### What OpenGL Actually Is

OpenGL is not a traditional library. It's a **specification** — a document written by the Khronos Group that says "here is a list of functions, and here is exactly what each one must do." Your **graphics driver** (software that ships with your GPU — NVIDIA, AMD, Intel, or Mesa on Linux) *implements* that spec.

So when you call `glGenBuffers(1, &buffer)`, you're calling code that NVIDIA (or AMD, or Intel) wrote in their driver, which conforms to what Khronos specified `glGenBuffers` should do.

OpenGL is a **state machine**. It has a global "current state" — which buffer is bound, which shader is active, what blend mode is set, etc. Almost every function either:
- **Changes state**: "bind this buffer," "use this shader"
- **Uses current state to do work**: "draw using whatever is currently bound"

This is why you see so many `glBind*` calls before drawing — you're assembling state, then firing.

---

## Part 1: Header Includes

```cpp
#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <locale>
#include <sstream>
#include <csignal>
```

`#include` is a C preprocessor directive. Before compilation, the preprocessor literally copies the contents of the named file into your source file. This gives your code access to the functions and types defined there.

### `<iostream>`
C++ standard I/O streams.
Gives you: `std::cout` (print to terminal), `std::endl` (newline + flush buffer), `std::cin` (read from terminal).

### `<GL/glew.h>` — The GLEW Library

**GLEW** = OpenGL Extension Wrangler Library. Here's the problem it solves:

Only OpenGL 1.1 functions are guaranteed to be "statically linkable" on most platforms — meaning you can just call them like normal C functions. But virtually everything in modern OpenGL (shaders, buffer objects, vertex arrays) came after 1.1 and is *not* statically available. These functions are provided by your graphics driver and must be **loaded at runtime** by looking up their memory addresses.

GLEW does this lookup for you. When you call `glewInit()`, it asks the driver "where is `glGenBuffers`? Where is `glShaderSource`?" and stores those addresses in function pointers. From then on, your calls to `glGenBuffers(...)` actually invoke the driver's code via those pointers.

**Key rule**: `glewInit()` must be called *after* you have a valid, active OpenGL context (provided by GLFW). Without a live context, GLEW has nothing to query.

### `<GLFW/glfw3.h>` — The GLFW Library

**GLFW** = Graphics Library Framework. Creating an OS window with an OpenGL context attached requires different, painful code on every platform: X11/GLX on Linux, Win32/WGL on Windows, Cocoa/CGL on macOS. GLFW hides all that. You call one function, and it creates the window + context for your platform.

GLFW also handles:
- Keyboard/mouse/joystick input events
- Window resize/close/move events
- Timing

GLFW knows **nothing** about drawing triangles — that's entirely OpenGL's job.

### `<fstream>`
File stream library. Gives you `std::ifstream` (input file stream, for reading files).

### `<locale>`
Not actually used in this code — likely a leftover include. It provides locale-aware string/number formatting.

### `<sstream>`
String stream library. Gives you `std::stringstream` — a stream that reads/writes to a string in memory instead of a file or terminal.

### `<csignal>`
C signal handling. Gives you `raise()` (send a signal to the current process) and `SIGTRAP` (the "trap" signal that tells a debugger to pause execution, exactly like a breakpoint).

---

## Part 2: The Error Checking System

This is the most practically important boilerplate in the file. OpenGL's error reporting is infamous — functions don't throw exceptions or return error codes. They silently fail and set an internal error flag. If you never check it, you waste hours debugging invisible problems.

### How OpenGL Reports Errors: `glGetError()`

OpenGL maintains an internal **error queue**. When something goes wrong (invalid arguments, wrong state, etc.), OpenGL pushes an error code onto that queue. Error codes are integer constants like:
- `GL_NO_ERROR` (0) — no error
- `GL_INVALID_ENUM` — you passed an enum value that isn't valid for that function
- `GL_INVALID_VALUE` — a numeric argument is out of range
- `GL_INVALID_OPERATION` — the operation isn't allowed in the current state

`glGetError()` pops **one** error from the front of the queue and returns it. If the queue is empty, it returns `GL_NO_ERROR` (0). If multiple errors occurred, you must call `glGetError()` multiple times to drain them all.

### `GLClearError()`

```cpp
static void GLClearError()
{
    while (glGetError() != GL_NO_ERROR);
}
```

**`static`** — This function has "internal linkage." It's only visible within this `.cpp` file. Other source files in the project cannot see or call it. Good hygiene for helper functions.

**`void`** — Returns nothing.

**`while (glGetError() != GL_NO_ERROR);`** — A loop with an empty body (the `;` is the entire body). Each iteration:
1. Calls `glGetError()`, popping one error from the queue
2. Compares it to `GL_NO_ERROR` (0)
3. If not equal (an error existed), loop again and pop the next one
4. If equal (queue empty), stop

**Purpose**: Before checking if a GL call caused an error, we need to drain any *old* errors left over from earlier code. This guarantees we're starting fresh.

### `GLLogCall()`

```cpp
static bool GLLogCall(const char* function, const char* file, int line)
{
    while (GLenum error = glGetError())
    {
        std::cout << "[OpenGL Error] ("<< error <<"): " << function
                  << " " << file << ":" << line << std::endl;
        return false;
    }
    return true;
}
```

**`bool`** — Returns `true` if no errors were found, `false` if any error was found.

**`const char* function`** — A C-string (null-terminated `char` array) containing the name of the GL function that was called. Passed from the macro using the stringification operator.

**`const char* file`** — A C-string with the source filename, from `__FILE__`.

**`int line`** — The source line number, from `__LINE__`.

**`GLenum`** — OpenGL's typedef for `unsigned int`. All OpenGL enumeration values (`GL_NO_ERROR`, `GL_TRIANGLES`, etc.) are this type.

**`while (GLenum error = glGetError())`** — Three things at once:
1. Declares a new variable `error` of type `GLenum`
2. Calls `glGetError()` and assigns its result to `error`
3. Uses `error` as the loop condition — in C++, any nonzero value is "true"

So: if the error is nonzero (there was an error), run the loop body. Each iteration pops the next error, until `glGetError()` returns 0 (empty queue), ending the loop.

Inside the loop: print a formatted message showing the error code, the function name, the file, and the line. Then `return false` — once we've confirmed an error, we exit the function (reporting it back as failure).

**`return true`** at the end — If the `while` condition was false from the very start (no errors at all), we skip the loop and return `true` (success).

### `ASSERT` Macro

```cpp
#define ASSERT(x) if (!(x)) raise(SIGTRAP);
```

**`#define ASSERT(x)`** — Defines a function-like macro. Every occurrence of `ASSERT(something)` in your code gets textually replaced with `if (!(something)) raise(SIGTRAP);` before compilation.

**`if (!(x))`** — If the expression `x` evaluates to false/zero.

**`raise(SIGTRAP)`** — Send the `SIGTRAP` signal to the current process. When running under a debugger (GDB, CLion, LLDB), this pauses execution exactly at that line, just like a manual breakpoint — you can then inspect all variables and the call stack. Without a debugger, `SIGTRAP` typically crashes the program, which is still better than silently continuing in a broken state.

### `GLCall` Macro

```cpp
#define GLCall(x) GLClearError();\
    x;\
    ASSERT(GLLogCall(#x , __FILE__, __LINE__ ))
```

**`#define GLCall(x)`** — Function-like macro with parameter `x`, which will be an OpenGL function call like `glBindBuffer(GL_ARRAY_BUFFER, buffer)`.

**`\` at line ends** — In macros, a backslash-newline tells the preprocessor "this macro definition continues on the next line." Without it, `#define` would end at the first newline.

**`GLClearError();`** — Drain all old errors first.

**`x;`** — Run the actual OpenGL call you wrapped.

**`ASSERT(GLLogCall(#x, __FILE__, __LINE__))`**:

- **`#x`** — The **stringification operator**. `#` before a macro parameter turns the literal code text into a string literal. If `x` is `glBindBuffer(GL_ARRAY_BUFFER, buffer)`, then `#x` becomes the string `"glBindBuffer(GL_ARRAY_BUFFER, buffer)"`. This is passed to `GLLogCall` so you can see exactly which call failed.

- **`__FILE__`** — A predefined compiler macro. It expands to the current source file's path as a string literal, e.g. `"/home/you/project/main.cpp"`.

- **`__LINE__`** — A predefined compiler macro. It expands to the current line number as an integer, e.g. `142`.

- `GLLogCall` returns `true` (no error) or `false` (error found). `ASSERT` triggers `SIGTRAP` if it gets `false`.

**Full expansion example**: `GLCall(glGenBuffers(1, &buffer))` expands to:

```cpp
GLClearError();
glGenBuffers(1, &buffer);
if (!(GLLogCall("glGenBuffers(1, &buffer)", "/home/.../main.cpp", 88))) raise(SIGTRAP);
```

So: clear stale errors → run the call → check for new errors → pause debugger if something's wrong.

---

## Part 3: What Is a Shader?

Before looking at `ParseShader`, `CompileShader`, and `CreateShader`, you need to understand what a shader is from scratch.

### The GPU Rendering Pipeline

When you tell OpenGL "draw these triangles," your geometry flows through a sequence of steps called the **rendering pipeline**:

```
[Vertex data in GPU memory]
          ↓
[VERTEX SHADER — your code, runs on GPU, once per vertex]
          ↓
[Primitive Assembly — GPU forms triangles from processed vertices]
          ↓
[Rasterization — GPU determines which pixels each triangle covers]
          ↓
[FRAGMENT SHADER — your code, runs on GPU, once per pixel]
          ↓
[Output to framebuffer — the image you see on screen]
```

The two stages **you write code for** are the vertex shader and the fragment shader. Everything else is automatic GPU hardware.

### GLSL — The Shading Language

Shaders are written in **GLSL** (OpenGL Shading Language). It looks like C. You write it as a string, pass it to OpenGL, and the GPU driver **compiles it at runtime**. Example:

**Vertex shader** (minimal example):
```glsl
#version 330 core
layout(location = 0) in vec2 position;

void main()
{
    gl_Position = vec4(position.x, position.y, 0.0, 1.0);
}
```

**Fragment shader** (minimal example):
```glsl
#version 330 core
uniform vec4 u_Color;
out vec4 color;

void main()
{
    color = u_Color;
}
```

### What Does the Vertex Shader Do?

The vertex shader runs **once per vertex**. Its job: take each vertex's input data (position, UV, color, etc.) and output a final 4D position in **clip space**, stored in the built-in variable `gl_Position`.

In this program, each vertex has a 2D position `(-0.5,-0.5)` etc. The vertex shader outputs it as `vec4(x, y, 0.0, 1.0)`:
- `z = 0.0`: depth (irrelevant for 2D drawing)
- `w = 1.0`: the homogeneous coordinate, required by OpenGL; `1.0` means "a regular point in space" (not a direction vector)

### What Does the Fragment Shader Do?

After rasterization (the GPU figures out which pixels are inside each triangle), the fragment shader runs **once per pixel** (technically "fragment" — a pixel + some metadata like depth). Its job: output a final color (`vec4` of R, G, B, A) for that pixel.

In this program, the fragment shader just outputs `u_Color` — whatever value you set from the C++ side.

### What Is a Uniform?

A **uniform** is a shader variable that:
- Is set from your C++ code before a draw call
- Stays the **same for every vertex and every pixel** in that entire draw call
- Can be changed between draw calls

Think of it as a global knob you turn from outside the shader. In this program, `u_Color` is a `vec4` (4 floats: R, G, B, A) that changes each frame to animate the red channel.

Contrast with **vertex attributes** (like `position`): those are *per-vertex* — each vertex gets its own value from the vertex buffer.

---

## Part 4: `ShaderProgramSource` Struct

```cpp
struct ShaderProgramSource
{
    std::string VertexSource;
    std::string FragmentSource;
};
```

**`struct`** — Defines a plain data structure. In C++, `struct` members are `public` by default (unlike `class` where they default to `private`).

**`std::string`** — C++'s standard string class. Manages its own memory, knows its length, supports `+`, `find`, `substr`, etc. Much safer than raw `char*` arrays.

This struct is just a container. `ParseShader` will read one shader file that contains both shader types mixed together, split them apart, and return both as strings in this struct.

---

## Part 5: `ParseShader` — Every Line Explained

The shader file (`res/shaders/Basic.shader`) uses a custom format where both shaders live in one file, separated by `#shader vertex` / `#shader fragment` tags:

```glsl
#shader vertex
#version 330 core
layout(location = 0) in vec2 position;
void main() { ... }

#shader fragment
#version 330 core
uniform vec4 u_Color;
...
```

`ParseShader` reads this file and splits it into two separate strings.

```cpp
static ShaderProgramSource ParseShader(const std::string& filepath)
{
```

**`const std::string& filepath`** — The file path, passed as a **const reference**. `&` means no copy is made of the string, `const` means we promise not to modify it. Efficient for string parameters.

```cpp
    std::ifstream stream(filepath);
```

**`std::ifstream`** — Input file stream. The constructor `stream(filepath)` opens the file at `filepath` for reading. If the file doesn't exist, the stream enters a failed state (though this code doesn't explicitly check for that).

```cpp
    enum class ShaderType
    {
        NONE = -1, VERTEX = 0, FRAGMENT = 1
    };
```

**`enum class`** — A strongly-typed enumeration. Unlike a plain `enum`, values can't be implicitly converted to/from `int` — you must cast explicitly. This prevents accidentally passing an enum where an int is expected.

Values are assigned deliberately:
- `NONE = -1`: initial state, before any `#shader` directive. `-1` prevents accidental use as an array index.
- `VERTEX = 0`: maps to `ss[0]` (first string stream)
- `FRAGMENT = 1`: maps to `ss[1]` (second string stream)

```cpp
    std::string line;
    std::stringstream ss[2];
    ShaderType type = ShaderType::NONE;
```

**`std::string line`** — A reusable string buffer. `getline` writes each line into this variable, overwriting it each iteration.

**`std::stringstream ss[2]`** — Array of 2 string streams. Works like `cout` but writes to an internal string you retrieve with `.str()`. `ss[0]` accumulates vertex shader source, `ss[1]` accumulates fragment shader source.

**`ShaderType type = ShaderType::NONE`** — Current parsing mode. Starts as NONE because we haven't seen any `#shader` tag yet.

```cpp
    while (getline(stream, line))
    {
```

**`getline(stream, line)`** — Reads characters from `stream` until it finds a newline (`\n`), stores everything *except* the newline in `line`, then returns a reference to `stream`. The while-loop converts the stream to bool: `true` if a line was successfully read, `false` at end-of-file or error. So this iterates one line at a time until the file ends.

```cpp
        if (line.find("#shader") != std::string::npos)
        {
```

**`line.find("#shader")`** — Searches `line` for the substring `"#shader"`. Returns the **index** (position) where it starts, or `std::string::npos` if not found.

**`std::string::npos`** — A `static const` member of `std::string`, equal to the maximum value of `size_t` (usually `18446744073709551615` on 64-bit systems). It's the sentinel "not found" value since a real position is always much smaller.

So this condition reads: "if this line contains the text '#shader'."

```cpp
            if (line.find("vertex") != std::string::npos)
                type = ShaderType::VERTEX;
            else if (line.find("fragment") != std::string::npos)
                type = ShaderType::FRAGMENT;
```

If the `#shader` line also contains "vertex," switch mode to VERTEX. If it contains "fragment," switch to FRAGMENT. We don't add these directive lines to any stream.

```cpp
        else
        {
            ss[(int)type] << line << "\n";
        }
```

For all other lines (actual GLSL code): append the line to the appropriate string stream.

**`(int)type`** — Explicit cast of the enum to int, needed to use it as an array index. `VERTEX` (= 0) → `ss[0]`, `FRAGMENT` (= 1) → `ss[1]`.

**`ss[(int)type] << line << "\n"`** — The `<<` operator on a stringstream appends data, exactly like `cout`. We add `"\n"` because `getline` stripped the newline when reading — we put it back so the GLSL source has correct line breaks.

Note: if `type` is still `NONE` (= -1) when we hit this `else` block, `ss[-1]` is undefined behavior — it reads memory before the array. In practice, a well-formed shader file always starts with `#shader vertex` before any actual GLSL, so this shouldn't occur.

```cpp
    return {ss[0].str(), ss[1].str()};
}
```

**`ss[0].str()`** — Retrieves the complete accumulated string from stringstream 0 — the entire vertex shader source.

**`ss[1].str()`** — Same for stringstream 1 — the entire fragment shader source.

**`return { ... }`** — Brace/aggregate initialization of `ShaderProgramSource`. Fields are initialized in order: `VertexSource = ss[0].str()`, `FragmentSource = ss[1].str()`.

---

## Part 6: GPU Object Handles — A Critical Mental Model

Before `CompileShader` and `CreateShader` make sense, you need to understand how OpenGL "objects" work.

OpenGL manages its own memory on the GPU. You cannot use `new` or `malloc` for GPU memory — those are CPU-side allocators. Instead, OpenGL gives you **handles**: small unsigned integers that identify GPU-side objects.

This is exactly like file descriptors in Linux. `open()` gives you an `int` (not a pointer to the file's contents) — just an ID. You then pass that ID to `read(fd, ...)`, `write(fd, ...)`, etc. OpenGL handles work identically.

Every OpenGL object has:
- A **create** function: `glCreateShader`, `glCreateProgram`, `glGenBuffers`, `glGenVertexArrays`
- A **delete** function: `glDeleteShader`, `glDeleteProgram`, `glDeleteBuffers`
- Various operations that take the handle: `glShaderSource(id, ...)`, `glBindBuffer(target, id)`

---

## Part 7: `CompileShader` — Every Line Explained

```cpp
static unsigned int CompileShader(unsigned int type, const std::string& source)
{
```

`type` is either `GL_VERTEX_SHADER` or `GL_FRAGMENT_SHADER` — OpenGL constants (unsigned ints) that identify which type of shader to create.

```cpp
    unsigned int id = glCreateShader(type);
```

**`glCreateShader(type)`** — Allocates a new shader object on the GPU side and returns its handle (unsigned int). The `type` argument tells the driver whether this is a vertex or fragment shader. Returns 0 on failure.

At this point the shader object exists on the GPU but has no source code yet.

```cpp
    const char* src = source.c_str();
```

**`source.c_str()`** — `std::string` stores its characters in its own internal buffer. OpenGL's C API expects `const char*` (a null-terminated character array). `.c_str()` returns a pointer to the `std::string`'s internal buffer, guaranteed to be null-terminated.

We store it in a local `const char* src` variable because `glShaderSource` takes a `const char**` (pointer-to-pointer), and you need an actual *variable* to take the address of with `&`.

```cpp
    GLCall(glShaderSource(id, 1, &src, nullptr));
```

**`glShaderSource(shader, count, strings, lengths)`** — Hands GLSL source text to a shader object.

- `id` — which shader object to give the source to
- `1` — we're providing 1 string. OpenGL allows you to split source across multiple strings (which it concatenates internally), but we give it just one.
- `&src` — address of our `const char*` variable. The parameter type is `const char**` (pointer to an array of C-strings). Our "array" has exactly one element: `src`.
- `nullptr` — if provided, this would be an array of string lengths. `nullptr` tells OpenGL to find the end of each string via null-termination.

After this call, the shader object on the GPU holds your GLSL source text, ready to compile.

```cpp
    GLCall(glCompileShader(id));
```

**`glCompileShader(id)`** — Invokes the GLSL compiler (built into your graphics driver) on the source in shader object `id`. The driver's compiler parses the GLSL text, type-checks it, and compiles it to GPU machine code. This is synchronous — the call doesn't return until compilation finishes.

```cpp
    int result;
    GLCall(glGetShaderiv(id, GL_COMPILE_STATUS, &result));
```

**`glGetShaderiv(shader, pname, params)`** — Queries an integer property of a shader object.

- `id` — which shader
- `GL_COMPILE_STATUS` — which property to query (compilation success/failure)
- `&result` — pointer to an int; OpenGL writes the answer here

The `iv` suffix is OpenGL naming convention: **i**nteger **v**alue(s). After this call, `result` is `GL_TRUE` (1) if compilation succeeded, `GL_FALSE` (0) if it failed.

```cpp
    if (result == GL_FALSE)
    {
        int length;
        GLCall(glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length));
```

Compilation failed. Now query the byte length of the human-readable error message. `GL_INFO_LOG_LENGTH` includes the null terminator in its count.

```cpp
        char* message = (char*)alloca(length * sizeof(char));
```

**`alloca(n)`** — Allocates `n` bytes on the **stack** of the current function, not the heap.

| | Stack (`alloca`) | Heap (`malloc`/`new`) |
|---|---|---|
| Speed | Instant (adjusts stack pointer) | Slower (has to manage free lists) |
| Freeing | Automatic when function returns | Manual `free`/`delete` required |
| Max size | ~1-8MB total stack | Gigabytes |
| Portability | POSIX only, not standard C++ | Fully standard |

For a small, short-lived buffer (a compiler error message), `alloca` is fast and self-cleaning — appropriate here.

**`(char*)`** — Cast the `void*` that `alloca` returns to `char*` so we can use it as a character buffer.

**`length * sizeof(char)`** — Total bytes needed. `sizeof(char)` is always 1 by definition (a `char` is exactly 1 byte), so this is just `length`, but writing it explicitly is clear.

```cpp
        GLCall(glGetShaderInfoLog(id, length, &length, message));
```

**`glGetShaderInfoLog(shader, bufSize, actualLength, buffer)`**:

- `id` — which shader
- `length` (first, input) — max characters to write including null terminator
- `&length` (second, output) — OpenGL overwrites this with the actual number of characters written (excluding null terminator)
- `message` — the char buffer to write the error text into

After this call, `message` is a C-string like `"ERROR: 0:4: 'position' : undeclared identifier"`.

```cpp
        std::cout << "Failed to compile " <<
            (type == GL_VERTEX_SHADER ? "vertex" : "fragment")
            << "shader" << std::endl;
        std::cout << message << std::endl;
```

**`type == GL_VERTEX_SHADER ? "vertex" : "fragment"`** — The ternary operator: `condition ? value_if_true : value_if_false`. Selects a string based on which shader type failed.

```cpp
        GLCall(glDeleteShader(id));
        return 0;
    }

    return id;
}
```

If compilation failed: free the broken shader object (`glDeleteShader` releases GPU memory), return 0 to signal failure. If compilation succeeded (we never entered the `if` block), return the valid handle.

---

## Part 8: `CreateShader` — Every Line Explained

```cpp
static unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader)
{
    unsigned int program = glCreateProgram();
```

**`glCreateProgram()`** — Creates a **program object** on the GPU and returns its handle. A program is the container that links vertex and fragment shaders into a complete, executable GPU pipeline. Think of it like an executable binary — it links multiple "compiled object files" (shaders) into one runnable thing. Returns 0 on failure.

```cpp
    unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader);
    unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader);
```

Compile both shaders using our function above. `vs` and `fs` are the handles returned by the internal `glCreateShader` calls — valid handles on success, 0 on failure (which isn't checked here; production code would check).

```cpp
    GLCall(glAttachShader(program, vs));
    GLCall(glAttachShader(program, fs));
```

**`glAttachShader(program, shader)`** — Associates a compiled shader with a program object. You can think of it as: "add this compiled shader to the pipeline." Each program needs exactly one vertex shader and one fragment shader attached. You can attach and detach multiple times before linking.

```cpp
    GLCall(glLinkProgram(program));
```

**`glLinkProgram(program)`** — The GPU-side linker step. During linking:
- The vertex shader's output variables are matched to the fragment shader's input variables (ensuring compatibility — mismatches are link errors)
- Uniform variable locations are resolved and assigned integers
- The two compiled shaders are merged into one executable GPU pipeline
- Driver-level optimizations may be applied

This is analogous to linking `.o` files into an executable in a normal C++ build.

```cpp
    GLCall(glValidateProgram(program));
```

**`glValidateProgram(program)`** — Additional validation against the **current OpenGL state**. It's somewhat optional — `glLinkProgram` catches most errors. `glValidateProgram` catches state-dependent issues (e.g., "this program is valid on its own, but will fail given what else is currently bound"). Good debug practice, often omitted in release builds.

```cpp
    GLCall(glDeleteShader(vs));
    GLCall(glDeleteShader(fs));
```

Once linked into the program, the individual shader objects are redundant — the program has its own internal compiled representation. Deleting them frees GPU-side shader objects. This does **not** affect the program, which has already internalized them. Analogy: deleting `.o` files after the `.exe` is built — the executable still works.

```cpp
    return program;
}
```

Return the program handle for use with `glUseProgram`.

---

## Part 9: `main()` — The Window and Context

```cpp
GLFWwindow* window;
```

**`GLFWwindow`** — An opaque struct defined by GLFW. "Opaque" means you never look at its internals — you only ever hold a pointer to it and pass that pointer to GLFW functions.

**`* window`** — A pointer to a `GLFWwindow`. Declared here but not yet assigned (happens in `glfwCreateWindow`).

```cpp
if (!glfwInit())
    return -1;
```

**`glfwInit()`** — Initializes GLFW's internal state. Must be called before any other GLFW function. Returns `GLFW_TRUE` (1) on success, `GLFW_FALSE` (0) on failure. `!glfwInit()` = "if init returned false (failure)," exit with error code -1.

```cpp
glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
```

**`glfwWindowHint(hint, value)`** — Sets desired properties for the *next* window/context to be created. Hints are requests to the driver, not guarantees.

- **`GLFW_CONTEXT_VERSION_MAJOR, 3`** + **`GLFW_CONTEXT_VERSION_MINOR, 3`** — Request OpenGL version 3.3.

- **`GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE`** — The "compatibility profile" keeps all legacy OpenGL features from the 1990s (like `glBegin`/`glEnd` immediate mode drawing). The alternative, `GLFW_OPENGL_CORE_PROFILE`, strips those out and forces the fully modern API. Compatibility profile is more lenient, useful in tutorial code.

```cpp
window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
if (!window)
{
    glfwTerminate();
    return -1;
}
```

**`glfwCreateWindow(width, height, title, monitor, share)`**:
- `640, 480` — window dimensions in pixels
- `"Hello World"` — title bar text
- `NULL` monitor — windowed mode (non-null = fullscreen on a specific monitor)
- `NULL` share — don't share OpenGL resources with another window

When this runs: GLFW asks the OS to create an actual window, then creates an **OpenGL context** tied to that window. A context is the complete collection of OpenGL state (which buffers exist, which shader is active, what GPU memory is allocated, etc.) — everything lives inside a context. Returns `NULL` on failure.

```cpp
glfwMakeContextCurrent(window);
```

**`glfwMakeContextCurrent(window)`** — Makes this window's OpenGL context **current** on the calling thread.

OpenGL is a **thread-local state machine**: each thread can have at most one "current context," and all `gl*` calls on that thread operate on that context. A context isn't active just because it exists — you explicitly bind it. Since we have one window and one thread, we bind it once and it stays current.

```cpp
glfwSwapInterval(1);
```

**`glfwSwapInterval(1)`** — Enables vertical sync (vsync).

You need **double buffering** to understand this: OpenGL renders into a "back buffer" (a region of GPU memory not currently displayed). When a frame is done, you "swap" back buffer → front buffer. This prevents displaying a half-rendered frame.

Without vsync, `glfwSwapBuffers` swaps instantly, potentially causing "tearing" (the monitor catches the swap mid-refresh, showing half of the old frame and half of the new). `SwapInterval(1)` makes the swap wait for the monitor's vertical blank signal, limiting framerate to the monitor's refresh rate (typically 60fps) and eliminating tearing. `SwapInterval(0)` = no vsync, uncapped framerate, possible tearing.

```cpp
if (glewInit() != GLEW_OK)
    std::cout << "Error ! " << std::endl;
else
    std::cout << "Success ! " << std::endl;
```

**`glewInit()`** — Queries the graphics driver for all modern OpenGL function addresses and stores them in internal function pointers. Must be called after `glfwMakeContextCurrent` — it needs an active context to query. Returns `GLEW_OK` on success.

```cpp
std::cout << glGetString(GL_VERSION) << std::endl;
```

**`glGetString(GL_VERSION)`** — Returns a `const unsigned char*` (treated as a C-string) describing the OpenGL version string. Might print something like `"3.3.0 NVIDIA 535.104.05"`. Useful for verifying you got the context you requested.

---

## Part 10: What Is a Vertex?

```cpp
float positions[] = {
    -0.5f, -0.5f,   // vertex 0: bottom-left
     0.5f, -0.5f,   // vertex 1: bottom-right
     0.5f,  0.5f,   // vertex 2: top-right
    -0.5f,  0.5f,   // vertex 3: top-left
};
```

A **vertex** is a point in space that forms the corner of a geometric shape. Three vertices define a triangle; four define a quad (though OpenGL splits it into two triangles internally).

A vertex can carry multiple pieces of data:
- **Position** — where in space (always required)
- **Color** — a per-vertex color
- **UV coordinates** — where on a texture this vertex maps to
- **Normal vector** — used for lighting calculations
- **Any custom data you want per-vertex**

In this program, each vertex carries only a 2D position (x, y).

**The `f` suffix** on literals like `0.5f` makes them `float` literals. Without `f`, `0.5` is a `double`, and the implicit narrowing to `float` may generate compiler warnings.

### NDC Coordinates (Normalized Device Coordinates)

The values -0.5 to 0.5 are in **Normalized Device Coordinates (NDC)**. OpenGL defines a coordinate system where:

```
(-1, +1) ────────────────── (+1, +1)
    │                            │
    │       CENTER (0,0)         │
    │                            │
(-1, -1) ────────────────── (+1, -1)
```

- Center of screen = (0, 0)
- Left edge = x = -1, Right edge = x = +1
- Bottom edge = y = -1, Top edge = y = +1

This is always true regardless of the window's pixel dimensions. OpenGL maps this -1 to +1 box onto whatever the window size is. So our square (0.5 units from center in each direction) occupies the middle half of the screen.

---

## Part 11: What Is an Index Buffer?

```cpp
unsigned int indices[] = {
    0, 1, 2,    // triangle 1: bottom-left → bottom-right → top-right
    2, 3, 0     // triangle 2: top-right → top-left → bottom-left
};
```

OpenGL draws **triangles**. A square = 2 triangles. Without an index buffer, you'd list all 6 vertex positions, repeating two of them:

```
Without indices: (-0.5,-0.5), (0.5,-0.5), (0.5,0.5),   <- triangle 1
                 (0.5,0.5), (-0.5,0.5), (-0.5,-0.5)   <- triangle 2 (2 vertices repeated!)
```

An **index buffer** (also called Element Buffer Object / EBO or IBO) stores each unique vertex *once*, then refers to them by integer index. Instead of 6 vertices, you have 4 vertices + 6 indices. The GPU assembles the triangles by reading vertices at the specified indices.

For a square, savings are small. For a complex 3D mesh with 50,000 vertices sharing many edges, index buffers can nearly halve memory usage and drastically improve GPU cache performance.

---

## Part 12: VAO and VBO — The Core of Modern OpenGL

These are the two concepts that trip up almost everyone new to OpenGL.

### What Is a VBO (Vertex Buffer Object)?

A VBO is a **chunk of raw memory on the GPU**. It's analogous to `malloc` for GPU memory. When you upload your `positions` array to a VBO, the GPU stores 32 bytes (8 floats × 4 bytes). But those 32 bytes are just bytes — the GPU does not inherently know that they represent 2D positions. You could have equally uploaded 32 bytes of temperature sensor readings.

### What Is a VAO (Vertex Array Object)?

A VAO stores the **schema for how to read data from VBOs**. It records:
- Which VBO to read for attribute slot 0 (position)?
- How many components per vertex? (2 here)
- What data type? (float)
- What byte offset does attribute 0 start at within each vertex?
- How far apart (stride) are consecutive vertices?
- Which index buffer (IBO) is associated with this geometry?

| Concept | Analogy |
|---|---|
| VBO | A CSV file — raw data, no labels |
| VAO | The schema / header row — says what each column means |

When you issue a draw call, you bind the VAO. OpenGL reads the VAO's schema to know how to interpret the VBO's bytes.

### `glGenVertexArrays` and `glBindVertexArray`

```cpp
unsigned int vao;
GLCall(glGenVertexArrays(1, &vao));
GLCall(glBindVertexArray(vao));
```

**`unsigned int vao`** — A variable to hold the VAO's handle (an unsigned int ID).

**`glGenVertexArrays(count, array)`** — Generates `count` VAO IDs and writes them into `array`. Passing `1` and `&vao` generates one VAO handle and stores it in `vao`. The VAO exists on the GPU but is unconfigured.

**`glBindVertexArray(vao)`** — Makes this the currently active VAO. All subsequent vertex attribute configuration calls (`glEnableVertexAttribArray`, `glVertexAttribPointer`) will be **recorded into this VAO**. This is the state machine in action — binding sets up which object receives subsequent configuration.

### `glGenBuffers`, `glBindBuffer`, `glBufferData`

```cpp
unsigned int buffer;
GLCall(glGenBuffers(1, &buffer));
GLCall(glBindBuffer(GL_ARRAY_BUFFER, buffer));
GLCall(glBufferData(GL_ARRAY_BUFFER, 4 * 2 * sizeof(float), positions, GL_STATIC_DRAW));
```

**`glGenBuffers(count, array)`** — Same pattern as `glGenVertexArrays` but for buffer objects (VBOs, IBOs, etc.). Generates one buffer handle, stores it in `buffer`.

**`glBindBuffer(target, buffer)`** — Binds this buffer to a specific "target slot" in OpenGL's state. `GL_ARRAY_BUFFER` is the slot for vertex attribute data. From this point on, GL calls that reference `GL_ARRAY_BUFFER` use this buffer.

**`glBufferData(target, size, data, usage)`** — Allocates GPU memory and copies data into it.

- `GL_ARRAY_BUFFER` — upload to the currently-bound array buffer (our VBO)
- `4 * 2 * sizeof(float)` — total size in bytes: 4 vertices × 2 floats × 4 bytes/float = **32 bytes**
- `positions` — pointer to the CPU-side data to copy across to GPU memory
- `GL_STATIC_DRAW` — a **usage hint** telling the driver how you'll use this data:
  - `STATIC` = the data won't change after this upload (driver may put it in the fastest VRAM)
  - `DRAW` = it will be used for rendering (vs. `READ` or `COPY`)
  - Other options: `GL_DYNAMIC_DRAW` (data changes frequently), `GL_STREAM_DRAW` (changes every frame)

After `glBufferData`, your 32 bytes of vertex positions live in GPU memory.

---

## Part 13: `glVertexAttribPointer` — The Most Confusing OpenGL Call

```cpp
GLCall(glEnableVertexAttribArray(0));
GLCall(glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), 0));
```

This tells OpenGL exactly how to interpret the bytes in the VBO when drawing.

**`glEnableVertexAttribArray(0)`** — Enables attribute slot 0. OpenGL has at least 16 attribute slots (indices 0–15), all disabled by default. You must enable each slot you use. Slot 0 corresponds to `layout(location = 0)` in the vertex shader.

**`glVertexAttribPointer(index, size, type, normalized, stride, pointer)`** — Defines the memory layout for attribute `index`:

- **`index = 0`** — Which attribute slot this applies to. The `location = 0` in `layout(location = 0) in vec2 position` in the vertex shader refers to this same slot index.

- **`size = 2`** — Number of components per vertex for this attribute. Our positions are 2D (x and y), so size = 2. 3D positions would be 3, RGBA colors would be 4.

- **`type = GL_FLOAT`** — The data type of each component. Our array contains `float` values.

- **`normalized = GL_FALSE`** — Only matters for integer types. `GL_TRUE` would rescale the integer to the range [-1, 1] or [0, 1]. Since we're using floats (already real numbers), we say `GL_FALSE`.

- **`stride = 2 * sizeof(float) = 8`** — The number of bytes from the start of one vertex's data to the start of the next vertex's data. With 2 floats per vertex: 2 × 4 = 8 bytes. If each vertex also had a 2D UV coordinate (another 2 floats), the stride would be 4 × 4 = 16.

- **`pointer = 0`** — The byte offset from the start of the buffer where this attribute begins, cast as `void*`. Since position is the very first (and only) thing in each vertex, the offset is 0. If UV came before position in memory, position would be at offset `2 * sizeof(float) = 8`.

**The VAO records all of this**. When you later bind just the VAO, it automatically restores all attribute configurations. Here's a visual of the VBO memory layout:

```
VBO Memory (32 bytes):
Byte:  0  1  2  3 | 4  5  6  7 | 8  9 10 11 | 12 13 14 15 | 16 ...
       [x vertex 0] [y vertex 0] [x vertex 1] [y vertex 1] | ...

glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 8, 0):
  Slot 0 starts at byte 0 (offset = 0)
  Read 2 floats = 8 bytes
  Next vertex's slot 0 is 8 bytes later (stride = 8)
  Reads: -0.5, -0.5 | 0.5, -0.5 | 0.5, 0.5 | -0.5, 0.5
```

---

## Part 14: Setting Up the Index Buffer

```cpp
unsigned int ibo;
GLCall(glGenBuffers(1, &ibo));
GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo));
GLCall(glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(unsigned int), indices, GL_STATIC_DRAW));
```

Nearly identical to the VBO setup, but uses `GL_ELEMENT_ARRAY_BUFFER` as the target.

**`glGenBuffers(1, &ibo)`** — Generate a buffer handle.

**`glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo)`** — Bind as an element array buffer (index buffer). `GL_ELEMENT_ARRAY_BUFFER` is the specific target for index data. This is separate from `GL_ARRAY_BUFFER` (vertex data).

**`glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(unsigned int), indices, GL_STATIC_DRAW)`**:
- `6 * sizeof(unsigned int)` = 6 × 4 = **24 bytes**
- `indices` — the CPU-side array `{0,1,2,2,3,0}`

**Important VAO interaction**: While a VAO is bound, binding `GL_ELEMENT_ARRAY_BUFFER` is **recorded into the VAO**. So when you later bind just the VAO before drawing, it automatically rebinds the associated index buffer. This is why unbinding order matters (covered below).

---

## Part 15: Loading Shaders and Uniforms

```cpp
ShaderProgramSource source = ParseShader("res/shaders/Basic.shader");
unsigned int shader = CreateShader(source.VertexSource, source.FragmentSource);
GLCall(glUseProgram(shader));
```

**`ParseShader("res/shaders/Basic.shader")`** — Reads the file and splits it into vertex/fragment source strings.

**`CreateShader(source.VertexSource, source.FragmentSource)`** — Compiles both, links them into a program, returns the program handle.

**`glUseProgram(shader)`** — Makes this program the active shader pipeline. All subsequent draw calls will use this program's vertex and fragment shaders.

### Getting a Uniform Location

```cpp
GLCall(int location = glGetUniformLocation(shader, "u_Color"));
ASSERT(location != -1);
GLCall(glUniform4f(location, 0.8f, 0.3f, 0.8f, 1.0f));
```

**`GLCall(int location = ...)`** — Note: wrapping a variable declaration in `GLCall` is slightly unconventional. The macro expansion includes the declaration before the error check. It works, but production code would typically separate this: declare `location`, then `GLCall(location = glGetUniformLocation(...))`.

**`glGetUniformLocation(program, name)`** — Finds the integer "location" (index) of the uniform named `"u_Color"` in the program. This string lookup must match the name in your GLSL source exactly (case-sensitive). Returns -1 if the uniform doesn't exist or was **optimized away** — if the GLSL compiler determines a uniform is never actually used in computing the output, it removes it, and `glGetUniformLocation` returns -1 for it.

**`ASSERT(location != -1)`** — Assert we found the uniform. If this fires, you either misspelled the name or the shader removed it.

**`glUniform4f(location, 0.8f, 0.3f, 0.8f, 1.0f)`** — Set the uniform at `location` to a vec4.

The `4f` in the function name is OpenGL's naming convention: `glUniform[count][type]`:
- `4` = 4 values
- `f` = float
So `glUniform4f` sets a `vec4` uniform. `glUniform3f` would set a `vec3`, `glUniform1i` would set a single int, etc.

Arguments: red=0.8, green=0.3, blue=0.8, alpha=1.0 → an initial pinkish-purple color.

---

## Part 16: Unbinding

```cpp
GLCall(glBindVertexArray(0));
GLCall(glUseProgram(0));
GLCall(glBindBuffer(GL_ARRAY_BUFFER, 0));
GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
```

Binding the value `0` to any target means "unbind" — clear the slot, no object is active for this target.

**Why unbind?** Defensive programming. If code elsewhere accidentally calls a state-modifying function without explicitly setting up the right context, unbinding prevents it from silently corrupting objects you carefully configured.

**Order matters for VAO and IBO**:
- The VAO *remembers* which IBO was bound while the VAO was active
- If you unbind the IBO *while the VAO is still bound*, the VAO forgets the IBO
- Here: VAO is unbound first (`glBindVertexArray(0)`), then IBO is unbound — **safe order**
- The VAO is no longer recording when IBO is unbound, so its internal IBO reference is preserved

---

## Part 17: The Render Loop — Every Line Explained

```cpp
float r = 0.0f;
float increment = 0.05f;
```

`r` — the red channel value (0.0 to 1.0) for the animated color, starting at 0 (no red).

`increment` — how much `r` changes per frame, initially positive (increasing). Reverses sign at the boundaries to create a bounce.

```cpp
while (!glfwWindowShouldClose(window))
{
```

**`glfwWindowShouldClose(window)`** — Returns 1 if the user has requested the window to close (clicking the X, pressing Alt+F4, etc.). `!` inverts: run the loop as long as the user hasn't requested close.

---

```cpp
    GLCall((GL_COLOR_BUFFER_BIT));
```

### ⚠️ BUG — THIS LINE IS WRONG

This should be:
```cpp
GLCall(glClear(GL_COLOR_BUFFER_BIT));
```

What the code **as written** does: `(GL_COLOR_BUFFER_BIT)` is just a parenthesized expression that evaluates the constant (= `0x00004000 = 16384`) and immediately discards the result. Nothing happens. No clearing occurs.

What `glClear(GL_COLOR_BUFFER_BIT)` **should** do: clear the back buffer to the current clear color (default: black). Without this call, each new frame is rendered on top of whatever was drawn in the previous frame. You'd see ghosting/trails as the square was drawn at full opacity over the previous frame at the same position — in this specific program it might not be visually obvious since the square doesn't move, but it's still a real bug.

**`GL_COLOR_BUFFER_BIT`** — A bitmask constant. You can OR multiple bits: `glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)` clears both color and depth. The depth buffer stores per-pixel depth values used for 3D occlusion testing.

---

```cpp
    GLCall(glUseProgram(shader));
    GLCall(glUniform4f(location, r, 0.3f, 0.8f, 1.0f));
```

Re-bind the shader program (we unbound it after initial setup). Update `u_Color`'s red channel to the current value of `r`. Green stays 0.3, blue stays 0.8, alpha stays 1.0.

```cpp
    GLCall(glBindVertexArray(vao));
    GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo));
```

Bind the VAO — this restores all vertex attribute configuration (which VBO to use, how many floats per vertex, stride, offset). The explicit IBO rebind is somewhat redundant since the VAO already stores the IBO reference, but it makes the intent explicit and is harmless.

```cpp
    GLCall(glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr));
```

**This is the actual draw command** — it triggers the entire GPU rendering pipeline.

**`glDrawElements(mode, count, type, indices)`**:

- **`GL_TRIANGLES`** — Primitive draw mode. Every 3 consecutive indices form one triangle. 6 indices → 2 triangles. Other modes: `GL_LINES` (every 2 indices = a line), `GL_POINTS` (each index = a point), `GL_TRIANGLE_STRIP` (triangles sharing edges).
- **`6`** — Total indices to process: `{0,1,2,2,3,0}` = 6 values.
- **`GL_UNSIGNED_INT`** — Data type of each index in the IBO. Must match what you uploaded (`unsigned int`). Other options: `GL_UNSIGNED_SHORT` (2-byte indices, enough for <65535 vertices), `GL_UNSIGNED_BYTE`.
- **`nullptr`** — Location of index data. When an IBO is bound (via the VAO), this is `nullptr` — it means "indices are in the currently bound `GL_ELEMENT_ARRAY_BUFFER`." Without a bound IBO, you'd pass a CPU-side pointer.

**What physically happens when `glDrawElements` fires**:
1. GPU reads 6 indices from IBO: 0, 1, 2, 2, 3, 0
2. For each index, the GPU reads the corresponding vertex from the VBO (using the VAO's layout schema)
   - Index 0 → vertex at byte offset 0 → (-0.5, -0.5)
   - Index 1 → vertex at byte offset 8 → (0.5, -0.5)
   - etc.
3. The vertex shader runs once per unique vertex, receiving the position via `layout(location=0) in vec2 position`, outputting `gl_Position`
4. The GPU assembles 2 triangles from the 6 processed vertices
5. The rasterizer calculates which screen pixels fall inside each triangle — roughly tens of thousands of pixels
6. The fragment shader runs once per pixel, reading `u_Color` (same value for all pixels this frame) and writing it as the pixel color
7. Resulting pixels are written to the back buffer

```cpp
    if (r > 1.0f)
        increment = -0.05f;
    else if (r < 0.0f)
        increment = 0.05f;

    r += increment;
```

Animate `r` between 0.0 and 1.0. When `r` exceeds 1.0, reverse increment to negative (start decreasing). When `r` drops below 0.0, reverse back to positive. Then add increment to `r` for use next frame.

At 60fps with `increment = 0.05`, `r` cycles from 0→1 in about 1/3 of a second.

```cpp
    glfwSwapBuffers(window);
```

**`glfwSwapBuffers(window)`** — Swap the back buffer (what we just drew) with the front buffer (currently on screen).

Because `glfwSwapInterval(1)` was set, this call **blocks** (waits) until the monitor's vertical blank signal — the moment between scan lines when the monitor is not actively refreshing. At that precise moment, the buffers swap instantaneously from the monitor's perspective, preventing tearing. At 60Hz, this means the CPU thread is blocked ~16.7ms per frame.

After the swap: the just-drawn frame is on screen; the old front buffer is now the new back buffer, ready to be drawn into next frame.

```cpp
    glfwPollEvents();
```

**`glfwPollEvents()`** — Processes all pending OS events: keyboard presses/releases, mouse movement/clicks, window resize, window close requests, etc. Without this call in your loop, the window's UI is frozen and unresponsive — the OS thinks your application is hung. Events are dispatched to any GLFW callbacks you've registered, or queryable with `glfwGetKey(window, GLFW_KEY_ESCAPE)` etc.

---

## Part 18: Cleanup

```cpp
GLCall(glDeleteProgram(shader));
glfwTerminate();
return 0;
```

**`glDeleteProgram(shader)`** — Frees the GPU-side program object. Proper cleanup also includes `glDeleteVertexArrays(1, &vao)`, `glDeleteBuffers(1, &buffer)`, and `glDeleteBuffers(1, &ibo)`, but since the OS and GPU driver reclaim all GPU memory on process exit anyway, it's only strictly necessary for long-running applications that load/unload scenes.

**`glfwTerminate()`** — Destroys all remaining GLFW windows, frees GLFW's internal resources, returns the system to its pre-`glfwInit` state. Should always be called before the program exits.

**`return 0`** — `main` returning 0 signals success to the OS. Nonzero = error.

---

## Full Picture: How Everything Connects

```
CPU Side (your C++ code)             GPU Side (graphics driver executes)
────────────────────────────────     ──────────────────────────────────

float positions[] = {...}  ──────→  [VBO: 32 bytes of raw float data]
unsigned int indices[] = {...} ──→  [IBO: 24 bytes of index data]

glVertexAttribPointer(...)  ──────→ [VAO: "slot 0 = 2 floats, stride 8, offset 0"]
                                     └─ VAO also stores: IBO reference

ParseShader() + CompileShader()      [Vertex Shader:  compiled GLSL]
            + glLinkProgram() ─────→ [Fragment Shader: compiled GLSL]
                                     [Program: linked pipeline]

glUniform4f(location, r, ...) ────→ [u_Color uniform value: (r, 0.3, 0.8, 1.0)]

─────── Per-frame ───────────────────────────────────────────────────────

glUseProgram(shader) ─────────────→ Activate this pipeline

glBindVertexArray(vao) ───────────→ Restore: "slot 0 reads from VBO,
                                              2 floats, stride 8, IBO bound"

glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr)
    │
    ├─ Read 6 indices from IBO: 0, 1, 2, 2, 3, 0
    ├─ Read vertices from VBO per VAO schema
    ├─ Vertex shader × 4 vertices → 4 clip-space positions
    ├─ Rasterize 2 triangles → ~50,000 fragments (pixels)
    ├─ Fragment shader × 50,000 → color = u_Color each
    └─ Write pixels to back buffer

glfwSwapBuffers() ────────────────→ Show back buffer; wait for vsync
glfwPollEvents()  ────────────────→ Process input, window events
```

Each frame, the CPU updates `r`, pushes the new uniform value to the GPU, re-issues the same draw call (same geometry, same shaders), and swaps buffers. The GPU renders the same square every frame with a smoothly cycling red channel.
