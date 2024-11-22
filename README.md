# FLAME GPU 2 Tutorial (CUDA C++)

Ths repository provides the Tutorial for FLAME GPU 2 using the CUDA C++ interface.

[FLAMEGPU/FLAMEGPU2](https://github.com/FLAMEGPU/FLAMEGPU2) is downloaded via CMake and configured as a dependency of the project.

The version of FLAME GPU fetched is pinned to a specific release of FLAME GPU, in case of API breaking changes.
This is controlled using the `FLAMEGPU_VERSION` CMake variable, which can be modified in `CMakeLists.txt`, or as a configuration argument.

## Tutorial

The Tutorial is designed to be delivered via a jupyter notebook running on a cloud instance using a specific VM, so the notebook makes certain assumptions about this.

This may prevent the notebook from being usable locally until we address this issue.

## Dependencies

The dependencies below are required for building FLAME GPU 2.

Only documentation can be built without the required dependencies (however Doxygen is still required).

## Requirements

Building FLAME GPU has the following requirements. 
There are also optional dependencies which are required for some components, such as Documentation or Python bindings.

+ [CMake](https://cmake.org/download/) `>= 3.18`
+ [CUDA](https://developer.nvidia.com/cuda-downloads) `>= 11.0` and a [Compute Capability](https://developer.nvidia.com/cuda-gpus) `>= 3.5` NVIDIA GPU.
+ C++17 capable C++ compiler (host), compatible with the installed CUDA version
  + [Microsoft Visual Studio 2019 or 2022](https://visualstudio.microsoft.com/) (Windows)
    + *Note:* Visual Studio must be installed before the CUDA toolkit is installed. See the [CUDA installation guide for Windows](https://docs.nvidia.com/cuda/cuda-installation-guide-microsoft-windows/index.html) for more information.
  + [make](https://www.gnu.org/software/make/) and [GCC](https://gcc.gnu.org/) `>= 8.1` (Linux)
+ [git](https://git-scm.com/)

Optionally:

+ [cpplint](https://github.com/cpplint/cpplint) for linting code
+ [Doxygen](http://www.doxygen.nl/) to build the documentation
+ [Python](https://www.python.org/) `>= 3.6` for python integration
+ [swig](http://www.swig.org/) `>= 4.0.2` for python integration
  + Swig `4.x` will be automatically downloaded by CMake if not provided (if possible).
+ [FLAMEGPU2-visualiser](https://github.com/FLAMEGPU/FLAMEGPU2-visualiser) dependencies
  + [SDL](https://www.libsdl.org/)
  + [GLM](http://glm.g-truc.net/) *(consistent C++/GLSL vector maths functionality)*
  + [GLEW](http://glew.sourceforge.net/) *(GL extension loader)*
  + [FreeType](http://www.freetype.org/)  *(font loading)*
  + [DevIL](http://openil.sourceforge.net/)  *(image loading)*
  + [Fontconfig](https://www.fontconfig.org/)  *(Linux only, font detection)*

## Building with CMake

Building via CMake is a three step process, with slight differences depending on your platform.

1. Create a build directory for an out-of tree build
2. Configure CMake into the build directory
    + Using the CMake GUI or CLI tools
    + Specifying build options such as the CUDA Compute Capabilities to target, the inclusion of Visualisation or Python components, or performance impacting features such as `SEATBELTS`. See [CMake Configuration Options](#CMake-Configuration-Options) for details of the available configuration options
3. Build compilation targets using the configured build system
    + See [Available Targets](#Available-targets) for a list of available targets.

### Linux

To build under Linux using the command line, you can perform the following steps.

For example, to configure CMake for `Release` builds, for consumer Pascal GPUs (Compute Capability `61`), with python bindings enabled, producing the static library and `boids_bruteforce` example binary.

```bash
# Create the build directory and change into it
mkdir -p build && cd build

# Configure CMake from the command line passing configure-time options. 
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=61

# Build the target(s)
cmake --build . --target all -j 8

# Alternatively make can be invoked directly
make flamegpu all -j8

```

### Windows

Under Windows, you must instruct CMake on which Visual Studio and architecture to build for, using the CMake `-A` and `-G` options.
This can be done through the GUI or the CLI.

I.e. to configure CMake for consumer Pascal GPUs (Compute Capability `61`), with python bindings enabled, and build the producing the static library and `boids_bruteforce` example binary in the Release configuration:

```cmd
REM Create the build directory 
mkdir build
cd build

REM Configure CMake from the command line, specifying the -A and -G options. Alternatively use the GUI
cmake .. -A x64 -G "Visual Studio 17 2022" -DCMAKE_CUDA_ARCHITECTURES=61

REM You can then open Visual Studio manually from the .sln file, or via:
cmake --open . 
REM Alternatively, build from the command line specifying the build configuration
cmake --build . --config Release --target ALL_BUILD --verbose
```

#### CMake Configuration Options

| Option                               | Value                       | Description                                                                                                |
| -------------------------------------| --------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `CMAKE_BUILD_TYPE`                   | `Release` / `Debug` / `MinSizeRel` / `RelWithDebInfo` | Select the build configuration for single-target generators such as `make`   |
| `CMAKE_CUDA_ARCHITECTURES`           | e.g `60`, `"60;70"`         | [CUDA Compute Capabilities][cuda-CC] to build/optimise for, as a `;` separated list. See [CMAKE_CUDA_ARCHITECTURES][cmake-CCA]. Defaults to `all-major` or equivalent. Alternatively use the `CUDAARCHS` environment variable. |
| `FLAMEGPU_SEATBELTS`                 | `ON`/`OFF`                  | Enable / Disable additional runtime checks which harm performance but increase usability. Default `ON`     |
| `FLAMEGPU_VISUALISATION`             | `ON`/`OFF`                  | Enable Visualisation. Default `OFF`.                                                                       |
| `FLAMEGPU_VISUALISATION_ROOT`        | `path/to/vis`               | Provide a path to a local copy of the visualisation repository.                                            |
| `FLAMEGPU_ENABLE_NVTX`               | `ON`/`OFF`                  | Enable NVTX markers for improved profiling. Default `OFF`                                                  |
| `FLAMEGPU_WARNINGS_AS_ERRORS`        | `ON`/`OFF`                  | Promote compiler/tool warnings to errors are build time. Default `OFF`                                     |
| `FLAMEGPU_SHARE_USAGE_STATISTICS`    | `ON`/`OFF`                  | Share usage statistics ([telemetry](https://docs.flamegpu.com/guide/telemetry)) to support evidencing usage/impact of the software. Default `ON`. |
| `FLAMEGPU_TELEMETRY_SUPPRESS_NOTICE` | `ON`/`OFF`                  | Suppress notice encouraging telemetry to be enabled, which is emitted once per binary execution if telemetry is disabled. Defaults to `OFF`, or the value of a system environment variable of the same name. |

See the [FLAMEGPU/FLAMEGPU2 Readme](https://github.com/FLAMEGPU/FLAMEGPU2#cmake-configuration-options) for a full list of CMake options for the main repository.

For a list of available CMake configuration options, run the following from the `build` directory:

```bash
cmake -LH ..
```

### Available Targets

| Target         | Description                                                                                                   |
| -------------- | ------------------------------------------------------------------------------------------------------------- |
| `all`          | Linux target containing default set of targets, including everything but the documentation and lint targets   |
| `ALL_BUILD`    | The windows equivalent of `all`                                                                               |
| `all_lint`     | Run all available Linter targets                                                                              |
| `example`      | The `example` target created by the `CMakeLists.txt` in the root of this repository                           |
| `lint_example` | Lint the `example` target.                                                                                    |
| `flamegpu`     | Build the FLAME GPU static library                                                                                |
| `docs`         | The FLAME GPU API documentation (if available)                                                                |

For a full list of available targets, run the following after configuring CMake:

```bash
cmake --build . --target help
```

## Usage Statistics (Telemetry)

Support for academic software is dependant on evidence of impact. Without evidence it is difficult/impossible to justify investment to add features and provide maintenance. We collect a minimal amount of anonymous usage data so that we can gather usage statistics that enable us to continue to develop the software under a free and permissible licence.

Information is collected when a simulation, ensemble or test suite run have completed.

The [TelemetryDeck](https://telemetrydeck.com/) service is used to store telemetry data. 
All data is sent to their API endpoint of https://nom.telemetrydeck.com/v1/ via https. For more details please review the [TelmetryDeck privacy policy](https://telemetrydeck.com/privacy/).

We do not collect any personal data such as usernames, email addresses or hardware identifiers but we do generate a random user identifier. This identifier is salted and hashed by Telemetry deck.

More information can be found in the [FLAMEGPU documentation](https://docs.flamegpu.com/guide/telemetry).

Telemetry is enabled by default, but can be opted out by:

+ Setting an environment variable `FLAMEGPU_SHARE_USAGE_STATISTICS` to `OFF`, `false` or `0` (case insensitive).
  + If this is set during the first CMake configuration it will be used for all subsequent CMake configurations until the CMake Cache is cleared, or it is manually changed.
  + If this is set during simulation, ensemble or test execution (i.e. runtime) it will also be respected
+ Setting the `FLAMEGPU_SHARE_USAGE_STATISTICS` CMake option to `OFF` or another false-like CMake value, which will default telemetry to be off for executions.
+ Programmatically overriding the default value by:
  + Calling `flamegpu::io::Telemetry::disable()` or `pyflamegpu.Telemetry.disable()` prior to the construction of any `Simulation`, `CUDASimulation` or `CUDAEnsemble` objects.
  + Setting the `telemetry` config property of a `Simulation.Config`, `CUDASimulation.SimulationConfig` or `CUDAEnsemble.EnsembleConfig` to `false`.