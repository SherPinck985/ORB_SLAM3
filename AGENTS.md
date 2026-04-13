# Repository Guidelines

## Project Structure & Module Organization
Core library headers live in `include/`, with camera models under `include/CameraModels/`. Implementations live in `src/` and `src/CameraModels/`. Runnable demos and calibration YAMLs are under `Examples/` by sensor mode (`Monocular`, `Stereo`, `RGB-D`, and inertial variants). `Thirdparty/` vendors DBoW2, g2o, and Sophus; avoid editing these unless the change is dependency-specific. `Vocabulary/` stores the compressed ORB vocabulary, and `evaluation/` contains trajectory evaluation scripts.

## Build, Test, and Development Commands
Use the provided build script for normal development:

```bash
./build.sh
```

This builds bundled dependencies, unpacks `Vocabulary/ORBvoc.txt`, and compiles the shared library plus example binaries. For an incremental rebuild, use:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

ROS support in this checkout is legacy code under `Examples_old/ROS/ORB_SLAM3`; verify paths before using `./build_ros.sh`.

## Coding Style & Naming Conventions
Match the existing C++11 style in `src/` and `include/`: 4-space indentation, opening braces on the next line for classes and functions, and no repository-wide reformatting. Class and method names use PascalCase (`LocalMapping`, `TrackMonocular`); member fields commonly use prefixes such as `m`, `mb`, and `mp`. Keep paired changes aligned across headers and implementations, for example `include/System.h` and `src/System.cc`.

## Testing Guidelines
There is no top-level unit test suite or coverage gate. Validate changes by rebuilding and running the closest example binary for the affected sensor path, then use the Python scripts in `evaluation/` when trajectory accuracy changes matter. Keep generated artifacts such as `build/`, `lib/`, `Vocabulary/ORBvoc.txt`, and trajectory logs out of commits.

## Commit & Pull Request Guidelines
Recent history uses short, imperative subjects such as `Fix typos`, `Update README.md`, and `Update ROS scripts`. Follow that pattern and keep commits focused. No PR template is present; include a concise description, affected datasets or sensor modes, exact build/run commands used for verification, and screenshots or trajectory/error summaries when viewer output or accuracy changes.


这个是开源代码ORB-SLAM3的源码，请你以中文回答我的提问，并且告诉我你每一步都干了什么。
