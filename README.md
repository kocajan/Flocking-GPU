# Flocking Simulation

This project builds a C++ / CUDA application using OpenGL and GLFW.
The final executable is produced as `bin/flocking`.

---

## Requirements

### Operating System

* Linux (the Makefile assumes a GNU toolchain and system OpenGL)

---

### Compilers & Toolchains

You must have the following installed:

* **GCC / G++** with C++20 support
* **NVIDIA CUDA Toolkit** (provides `nvcc`)

Check versions:

```bash
gcc --version
g++ --version
nvcc --version
```

---

### System Libraries

The project links against the following system libraries:

* OpenGL (`libGL`)
* GLFW
* POSIX threads (`pthread`)
* Dynamic loader (`dl`)
* CUDA runtime (`cudart`)

On **Ubuntu / Debian**, install everything with:

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    libglfw3-dev \
    libgl1-mesa-dev \
    mesa-common-dev \
    nvidia-cuda-toolkit
```

Make sure NVIDIA drivers are installed and compatible with the CUDA toolkit version.

---

## Build

From the project root directory:

```bash
make
```

This will:

* Compile all `.cpp`, `.c`, and `.cu` files
* Generate dependency files (`.d`)
* Place object files in `build/`
* Produce the executable at `bin/flocking`

---

## Run

### Default run mode

```bash
make run
```

Equivalent to:

```bash
./bin/flocking run cfg
```

---

### Experiment / lab mode

```bash
make lab
```

Equivalent to:

```bash
./bin/flocking experiment cfg
```

---

### Custom configuration directory

You can override the configuration directory name:

```bash
make run CFG=my_config
```

This runs:

```bash
./bin/flocking run my_config
```

---

## Cleaning

Remove only build artifacts:

```bash
make clean
```

Remove build artifacts and the executable:

```bash
make fclean
```

Full rebuild:

```bash
make re
```

---