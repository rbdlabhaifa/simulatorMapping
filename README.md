# Simulator Mapping

> **Repository:** [rbdlabhaifa/simulatorMapping](https://github.com/rbdlabhaifa/simulatorMapping)

## What is this project?

**Simulator Mapping** is an open-source **C++ robotics simulation framework** maintained by the [RBD Lab at the University of Haifa](https://github.com/rbdlabhaifa). It wraps a vendored **ORB-SLAM2** stack inside a **Pangolin** 3D viewer so you can develop SLAM and navigation code on a laptop—no drone, no ROS stack, no cloud infra.

The repo ships as a **CMake monolith**: three small in-house libraries (`simulator`, `exitRoom`, `Auxiliary`) plus six example executables that cover mapping, simulation, navigation, and room-exit detection.

## What does it do?

In plain terms, the project lets you:

1. **Fly through a virtual room** — Load a Blender `.obj` model, open an interactive 3D window, and drive a virtual camera with keyboard commands (`W/A/S/D`, `E/Q`, `R/F`).
2. **Run SLAM on what the camera sees** — Each rendered frame is fed into ORB-SLAM2, which builds a sparse 3D map and tracks camera pose in real time.
3. **Save and reload maps** — Export `.bin` map files and `.csv` point clouds for offline analysis or reuse.
4. **Map from real sensors** — Use a webcam (`mapping`) or a pre-recorded video (`offline_orb_slam`) instead of the simulator.
5. **Detect room exits** — The `RoomExit` module analyzes SLAM map points to guess doorway locations (used by `runSimulator`).
6. **Implement navigation** — `navigate_to_point` is the student assignment entry point: start the simulator, read `target_pose` from config, and write your own path-planning logic.

**Typical workflow for a new developer:**

```bash
git clone https://github.com/rbdlabhaifa/simulatorMapping.git
cd simulatorMapping
cp generalSettings.json.example generalSettings.json   # then edit paths
./install.sh
cd Thirdparty/slam/Vocabulary && tar -xf ORBvoc.txt.tar.gz && cd -
cd build && ./examples/runSimulator
```

Press **Tab** in the model viewer when prompted to start SLAM scanning.

---

# 1. Project Overview

**Simulator Mapping** solves a specific robotics education problem: *how do you teach SLAM and indoor navigation when students don't all have drones, calibrated labs, or repeatable environments?*

The answer is **software-in-the-loop simulation**. Instead of flying a DJI Tello (though the camera configs are Tello-calibrated), you render synthetic views from a 3D model. ORB-SLAM2 treats those frames exactly like camera input, producing maps and poses you can use to test algorithms.

**Primary users:**

| Audience | How they use it |
|---|---|
| **University students** | Complete assignments in `navigate_to_point.cc`; follow the [Student Guide (PDF)](https://drive.google.com/file/d/1HKZ_ecanpfv_IFJg1f98f6U__ivAlVef/view?usp=sharing) |
| **Course TAs / lab staff** | Provide the [all-data zip](https://drive.google.com/file/d/1McpMjSu7-ziM0-tqMumX0LM7evFS9Irs/view?usp=sharing) with pre-built models and maps |
| **Researchers** | Extend `RoomExit` or swap SLAM configs for new camera setups |

**What it is not:** a web service, cloud deployment, or production robotics stack. It is a local, GPU-accelerated research/education toolchain that runs as compiled executables on a developer workstation or VM (Ubuntu 22.04 recommended).

---

# 2. Architecture & Tech Stack

## Languages & Build

| Layer | Technology |
|---|---|
| Language | C++17 |
| Build system | CMake 3.0+ |
| Package management (optional) | vcpkg (Windows path only) |
| Configuration | JSON (`generalSettings.json`) |

## Core Libraries

| Library | Role |
|---|---|
| **ORB-SLAM2** (vendored in `Thirdparty/slam/`) | Monocular SLAM: tracking, mapping, loop closure, map serialization |
| **Pangolin** (`Thirdparty/Pangolin/`) | OpenGL 3D model rendering, windowing, framebuffer capture |
| **OpenCV 3.4.16** | Image I/O, video capture, ORB feature pipeline |
| **Eigen 3.4** | Linear algebra (poses, SVD surface alignment) |
| **g2o + DBoW2** | Graph optimization and bag-of-words loop detection (ORB-SLAM2 deps) |
| **Boost** (serialization, system) | Map persistence |
| **nlohmann/json** | Runtime configuration parsing |
| **Python headers** | Required by ORB-SLAM2 build (linked, not scripted) |

## Architectural Pattern

**Monolithic native application** with **library + executable** layering:

```
┌─────────────────────────────────────────────────────────────┐
│  examples/  (entry-point executables)                       │
│  runSimulator | navigate_to_point | mapping | offline_orb…  │
└────────────┬───────────────────────────────┬────────────────┘
             │                               │
    ┌────────▼────────┐              ┌───────▼───────┐
    │  simulator lib  │              │  exitRoom lib │
    │  (Pangolin+SLAM)│              │  (RoomExit)   │
    └────────┬────────┘              └───────────────┘
             │
    ┌────────▼────────────────────────────────────────┐
    │  ORB_SLAM2  │  Auxiliary  │  Pangolin  │  g2o   │
    └─────────────────────────────────────────────────┘
```

**Component interaction (simulator path):**

1. `Simulator` loads a `.obj` model into a Pangolin OpenGL window.
2. Each rendered frame is captured via `glReadPixels`, converted to grayscale, and passed to `ORB_SLAM2::System::TrackMonocular`.
3. SLAM runs on a dedicated thread (`Simulator::SLAMThread`); rendering runs on the main Pangolin loop.
4. Optional `RoomExit` analyzes the SLAM point cloud to infer doorway/exit candidates.
5. Map artifacts (`.bin`, `.csv`) are written to paths configured in `generalSettings.json`.

---

# 3. Prerequisites

## Required (Linux — primary platform)

| Requirement | Version / Notes |
|---|---|
| **OS** | **Ubuntu 22.04** (officially supported; other distros may work but are untested) |
| **CPU** | ≥ 4 cores (6 recommended); VM users must allocate enough cores or `make` will fail |
| **RAM** | ≥ 8 GB (16 GB recommended for OpenCV + SLAM compile) |
| **GPU / Display** | OpenGL-capable GPU + X11/Wayland display (required for Pangolin window) |
| **Disk** | ≥ 10 GB free (OpenCV build from source is large) |
| **Git** | For cloning repo and dependency sources during `install.sh` |
| **sudo** | Required by `install.sh` for `apt-get` and system-wide library installs |

## Installed automatically by `install.sh`

The install script pulls these via `apt-get` and source builds:

- `cmake`, `build-essential`, `python3-numpy`, `python3-dev`
- OpenGL: `libglew-dev`, `libglu1-mesa-dev`, `freeglut3-dev`, `mesa-common-dev`
- FFmpeg / video: `libavcodec-dev`, `libavformat-dev`, `libswscale-dev`, `libdc1394-22-dev`
- Image libs: `libjpeg-dev`, `libpng-dev`, `libtiff5-dev`, `libopenexr-dev`
- Math / ML: `libboost-all-dev`, `libopenblas-dev`, `libeigen3-dev` (also built from source)
- Misc: `libncurses5-dev`, `libpcl-dev`, X11/XCB dev packages, `libbluetooth-dev`

## Required (Windows — partial / experimental)

| Requirement | Notes |
|---|---|
| Windows 10/11 x64 | No official Linux-equivalent `install.sh` |
| Visual Studio with C++ toolchain | For MSVC builds |
| vcpkg | See `vcpkg/installWithVcpkg.bat` |
| Git | Clone vcpkg and Pangolin fork |

## External data (not in repo)

| Asset | Source |
|---|---|
| Pre-built lab simulator + maps + videos | [all-data zip (Google Drive)](https://drive.google.com/file/d/1McpMjSu7-ziM0-tqMumX0LM7evFS9Irs/view?usp=sharing) |
| ORB vocabulary (text) | Extract locally: `Thirdparty/slam/Vocabulary/ORBvoc.txt.tar.gz` → `ORBvoc.txt` |
| 3D model (`.obj`) | User-provided or from all-data zip |

---

# 4. Environment Setup

## Clone the repository

```bash
git clone https://github.com/rbdlabhaifa/simulatorMapping.git
cd simulatorMapping
```

## First-run configuration (use the example file)

A template with placeholder paths ships at **`generalSettings.json.example`**. Copy it before your first run:

```bash
cp generalSettings.json.example generalSettings.json
```

Edit `generalSettings.json` and replace every `/ABSOLUTE/PATH/TO/...` segment with real paths on your machine. At minimum, update:

| Placeholder in example | Replace with |
|---|---|
| `/ABSOLUTE/PATH/TO/simulatorMapping/...` | Your clone root, e.g. `/home/you/dev/simulatorMapping/...` |
| `/ABSOLUTE/PATH/TO/your-data/models/room.obj` | A Blender `.obj` model (from the [lab data zip](https://drive.google.com/file/d/1McpMjSu7-ziM0-tqMumX0LM7evFS9Irs/view?usp=sharing) or your own) |
| `/ABSOLUTE/PATH/TO/your-data/slamMaps/...` | A writable output directory (create it if missing) |
| `/ABSOLUTE/PATH/TO/your-data/videos/mapping.avi` | A test video for `offline_orb_slam` |

**In-repo paths you can set immediately after clone** (once vocabulary is extracted):

```text
.../Thirdparty/slam/Vocabulary/ORBvoc.txt
.../Thirdparty/slam/config/tello_9F5EC2_640.yaml   # or webcam.yml for laptop camera
```

> **Note:** `generalSettings.json` is local machine config (listed in `.gitignore`). Copy from `generalSettings.json.example` on first setup; do not commit paths specific to your machine.

## Configuration model (no `.env` file)

This project **does not use a `.env` file or environment variables for application config**. All runtime settings live in **`generalSettings.json`** at the repository root.

Executables resolve this file relative to the **current working directory**:

```cpp
// src/Auxiliary.cpp
settingPath = currentDirPath + "/../generalSettings.json";
```

**Critical:** always run binaries from the `build/` directory so `../generalSettings.json` resolves correctly.

### `generalSettings.json` schema

| Key | Type | Purpose |
|---|---|---|
| `slam_configuration.vocabulary_path` | string (abs path) | Path to `ORBvoc.txt` |
| `slam_configuration.drone_yaml_path` | string (abs path) | ORB-SLAM2 camera/settings YAML (e.g. `tello_9F5EC2_640.yaml`) |
| `slam_configuration.load_map` | bool | Load existing SLAM map on startup |
| `slam_configuration.load_map_path` | string (abs path) | Binary map file (`.bin`) |
| `slam_configuration.save_map` | bool | Persist map after offline SLAM run |
| `simulator_configuration.model_path` | string (abs path) | Blender `.obj` model |
| `simulator_configuration.run_with_slam` | bool | Wait for Tab + enable SLAM in navigation examples |
| `simulator_configuration.movement_factor` | float | Base movement step size in simulator |
| `simulator_configuration.simulator_starting_speed` | float | Command speed multiplier |
| `simulator_configuration.align_model_to_texture.align` | bool | Auto-align camera to named texture surface |
| `simulator_configuration.align_model_to_texture.texture` | string | Texture/object name in `.obj` (e.g. `"floor"`) |
| `simulator_configuration.starting_pose.{x,y,z}` | float | Initial camera position |
| `simulator_configuration.simulator_output_dir` | string (abs path) | Output directory for maps/CSVs |
| `offline_video_test_path` | string (abs path) | Input video for `offline_orb_slam` |
| `continue_scanning` | bool | Resume from existing map in `mapping` |
| `use_webcam` | bool | `true` = webcam; `false` = drone (drone code currently stubbed) |
| `target_pose.{x,y,z}` | float | Navigation target for `navigate_to_point` |

### Where to obtain path values / credentials

There are **no API keys or cloud secrets**. Replace every absolute path in `generalSettings.json` with paths valid on your machine:

1. **In-repo paths** — point to your clone root, e.g. `/home/you/dev/simulatorMapping/Thirdparty/slam/Vocabulary/ORBvoc.txt`
2. **Lab assets** — download the [all-data zip](https://drive.google.com/file/d/1McpMjSu7-ziM0-tqMumX0LM7evFS9Irs/view?usp=sharing) and map external paths to equivalent files inside it
3. **Camera calibration** — use bundled configs under `Thirdparty/slam/config/` or your own calibrated YAML

### Optional environment variables (Pangolin only)

Pangolin may read `PANGOLIN_WINDOW_URI` and `HOME` for window/display defaults. No project-specific env vars are required.

---

# 5. Build & Run Instructions

## First-time install (Ubuntu 22.04)

```bash
cd simulatorMapping
chmod +x install.sh opencv3.4.16Install.sh
./install.sh
```

Extract the ORB vocabulary (required once):

```bash
cd Thirdparty/slam/Vocabulary
tar -xf ORBvoc.txt.tar.gz   # produces ORBvoc.txt
cd -
```

Configure paths (first time only):

```bash
cp generalSettings.json.example generalSettings.json
# Replace every /ABSOLUTE/PATH/TO/... placeholder with your real paths
nano generalSettings.json
```

## Rebuild after partial install failure

If `install.sh` fails mid-`make` (common on under-provisioned VMs), **do not re-run the full script**. Resume from the build directory:

```bash
cd build
cmake ..
make -j$(nproc)
```

## Run executables

All commands assume `cd build` first:

```bash
cd build
```

| Executable | Command | Purpose |
|---|---|---|
| Interactive simulator + room exit demo | `./examples/runSimulator` | Loads model, runs scripted scan, detects exits, navigates |
| Student navigation stub | `./examples/navigate_to_point` | Template for target-point navigation (`target_pose` in JSON) |
| Simulator API template | `./examples/simulatorUsageTemplate` | Example `simulator.command()` usage |
| Live mapping (webcam) | `./examples/mapping` | Build SLAM map from webcam feed |
| Offline video SLAM | `./examples/offline_orb_slam` | Build/export map from video file |
| Room exit visualization | `./examples/getExitPoints <pointcloud.csv>` | Plot exits from CSV (requires matplotlib-cpp) |

### Simulator keyboard controls (in model viewer)

| Key | Action |
|---|---|
| **Tab** | Start SLAM scanning / signal ready |
| **W/A/S/D** | Move forward / right / back / left |
| **E/Q** | Yaw left / right |
| **R/F** | Down / up (Y axis inverted vs ORB-SLAM) |
| **T** | Toggle SLAM tracking |
| **M** | Save current map |
| **K** | Stop simulator |
| **1/2** | Decrease / increase movement speed |

## Tests

**No project-level unit or integration test suite exists.** `catch2` appears in `vcpkg/vcpkg.json` but is not wired into the root `CMakeLists.txt`. There is no `ctest` target.

To validate a build manually:

```bash
cd build
./examples/runSimulator          # Requires display + configured model
./examples/offline_orb_slam      # Headless-friendly if video path is valid
```

## Linters / formatters

**No linter or formatter configuration is checked into this repository** (no `.clang-format`, no CI lint step). Use your IDE defaults or add tooling locally if needed.

---

# 6. Repository Structure

```
simulatorMapping/
├── CMakeLists.txt              # Root build: libraries + subdirs
├── generalSettings.json        # ★ Local runtime config (create from .example; not committed)
├── generalSettings.json.example # ★ Template with placeholder paths — copy on first clone
├── install.sh                  # ★ Ubuntu bootstrap: deps + cmake + make
├── opencv3.4.16Install.sh      # Builds OpenCV 3.4.16 from source into /usr/local
├── README.md
│
├── examples/                   # Executable entry points (student/workflow code)
│   ├── runSimulator.cpp        # Full demo: scan → RoomExit → navigate
│   ├── navigate_to_point.cc    # ★ Student assignment stub
│   ├── mapping.cc              # Webcam/drone live mapping
│   ├── offline_orb_slam.cc     # Video → SLAM map + CSV export
│   └── simulatorUsageTemplate.cpp
│
├── include/                    # Public headers
│   ├── simulator/simulator.h   # ★ Core Simulator API
│   ├── RoomExit/RoomExit.h     # Exit-point detection from point cloud
│   ├── Auxiliary.h             # Config path helper, draw utilities
│   └── RunModel/TextureShader.h# Pangolin GLSL shader for textured models
│
├── src/                        # Library implementations
│   ├── simulator/simulator.cpp # Pangolin render loop + SLAM feed
│   ├── RoomExit/RoomExit.cpp   # Geometric exit detection algorithm
│   └── Auxiliary.cpp
│
├── Thirdparty/                 # Vendored + install-time cloned deps
│   ├── slam/                   # ★ ORB-SLAM2 fork (System, Tracking, Map, …)
│   │   ├── config/             # Camera YAML configs (Tello, webcam, KITTI, …)
│   │   └── Vocabulary/         # ORBvoc.txt.tar.gz (extract before use)
│   ├── Pangolin/               # 3D viewer / OpenGL
│   ├── g2o/                    # Graph optimization
│   └── DBoW2/                  # Bag-of-words
│
└── vcpkg/                      # Windows-only helper manifest + install script
    ├── vcpkg.json
    └── installWithVcpkg.bat
```

---

# 7. Core Workflows & Data Flow

## Workflow A — Simulator + SLAM + Room Exit (`runSimulator`)

**Start here to understand the system.**

```
generalSettings.json
        │
        ▼
examples/runSimulator.cpp::main()
        │
        ├─► Simulator(config, model, …)     ← include/simulator/simulator.h
        │         │
        │         ├─► simulator.run()       → spawns simulatorRunThread()
        │         │       ├─ Pangolin: load .obj, render loop
        │         │       ├─ glReadPixels → grayscale frame
        │         │       └─ SLAMThread → ORB_SLAM2::System::TrackMonocular()
        │         │
        │         ├─ setTrack(true) after Tab pressed
        │         ├─ command("forward 0.5") → virtual robot motion
        │         └─ getCurrentMap() → vector<MapPoint*>
        │
        ├─► RoomExit(mapPoints).getExitPoints()
        │         └─ src/RoomExit/RoomExit.cpp (polygon + variance heuristics)
        │
        └─► command(rotate + forward) toward best exit
```

**Key files to read first:**

1. `examples/runSimulator.cpp` — orchestration
2. `include/simulator/simulator.h` — public API
3. `src/simulator/simulator.cpp` — render ↔ SLAM pipeline (`feedSLAM`, `simulatorRunThread`)
4. `Thirdparty/slam/include/System.h` — ORB-SLAM2 interface
5. `src/RoomExit/RoomExit.cpp` — exit detection logic

## Workflow B — Offline map from video (`offline_orb_slam`)

```
generalSettings.json → offline_video_test_path
        │
        ▼
offline_orb_slam.cc::main()
        │
        ├─► ORB_SLAM2::System(voc, yaml, MONOCULAR)
        ├─► cv::VideoCapture → TrackMonocular() per frame
        ├─► saveMap() → CSV point clouds + per-point keyframe data
        └─► SLAM->SaveMap(simulatorOutputDir/simulatorMap.bin)
```

## Workflow C — Student navigation (`navigate_to_point`)

```
generalSettings.json → target_pose + simulator_configuration
        │
        ▼
navigate_to_point.cc::main()
        │
        ├─► Simulator startup (same as above)
        └─► // Implement moving to target  ← STUDENT CODE HERE
```

## Data artifacts

| Output | Producer | Location |
|---|---|---|
| `simulatorMap.bin` | `offline_orb_slam`, simulator (`M` key) | `simulator_output_dir` |
| `cloud*.csv` | SLAM map points + observations | `simulator_output_dir` |
| `mapping-<timestamp>/` | `mapping` executable | `./` (cwd) |
| `Slam_latest_Map.bin` | `mapping` | inside timestamp dir |

---

# 8. Deployment & CI/CD

## Deployment

**There is no production deployment.** This is a local desktop/VM application:

- No Docker images
- No Kubernetes manifests
- No cloud hosting (AWS/GCP/Azure)
- No server process or REST API

**"Deployment" = build on a Linux workstation/VM with a display**, configure `generalSettings.json`, and run executables from `build/`.

For headless or remote use, you still need an X11 display (or X forwarding) because Pangolin opens an OpenGL window.

## CI/CD

**No GitHub Actions, GitLab CI, or other automated pipeline is configured** for this repository. Builds are manual via `install.sh` or local `cmake && make`.

The bundled `Thirdparty/Pangolin/scripts/vcpkg/` tree contains upstream Microsoft vcpkg CI scripts; these are **vendor artifacts**, not project CI.

### Recommended manual release checklist

1. Clean clone on Ubuntu 22.04 VM (≥ 6 cores)
2. Run `./install.sh` successfully
3. Extract `ORBvoc.txt`
4. Point `generalSettings.json` at lab data
5. Verify `./examples/runSimulator` and `./examples/offline_orb_slam`
6. Archive `build/examples/*` + config + vocabulary (not typically done today)

---

# 9. Known Quirks & Technical Debt

| Issue | Context | Impact |
|---|---|---|
| **Machine-specific paths in `generalSettings.json`** | Local config must be created from `generalSettings.json.example` | Fresh clones fail until paths are set; use the example template |
| **CWD-dependent config resolution** | `Auxiliary::GetGeneralSettingsPath()` assumes run from `build/` | Running from another directory silently breaks config loading |
| **No `.env` / secrets management** | All config in JSON | Fine for local use; no multi-environment support |
| **`install.sh` is destructive & slow** | Re-clones Eigen, json, OpenCV into `Thirdparty/` every run; requires `sudo` | Not idempotent; risky to re-run on partial success |
| **OpenCV 3.4.16 pinned** | Built from source via `opencv3.4.16Install.sh` | Conflicts with system OpenCV; long compile; outdated API (`CV_CAP_PROP_*`) |
| **Drone integration stubbed** | `mapping.cc` Tello calls commented out | `use_webcam: false` path does nothing useful |
| **`navigate_to_point` incomplete** | Student assignment placeholder | Target navigation not implemented in repo |
| **Duplicate Pangolin keybinding `'a'`** | `simulator.cpp` registers `'a'` for both axis toggle and strafe-right | `'a'` behavior is ambiguous |
| **Y-axis inversion** | ORB-SLAM Y vs Pangolin Y differ | Documented in API; easy to get wrong when drawing points |
| **`getExitPoints` likely broken link** | `examples/CMakeLists.txt` links only `exitRoom`, but source uses `matplotlibcpp` | May fail to link unless matplotlib-cpp installed and CMake updated |
| **No automated tests** | Zero `ctest`/Catch2 integration | Regressions caught only manually |
| **ORB-SLAM2 GPL** | Vendored under `Thirdparty/slam/` | License constraints for commercial closed-source use |
| **Windows path is unmaintained** | `vcpkg/installWithVcpkg.bat` references `ThirdParty/` (wrong casing) and a Pangolin fork | Windows build likely broken without manual fixes |
| **README vs JSON key mismatch** | Old README says `target_point`; JSON uses `target_pose` | Confusing for new developers |
| **Resource starvation on VMs** | Parallel `make -j3` + OpenCV compile | Documented make failures; need more CPU/RAM |

---

# 10. Troubleshooting / FAQ

### 1. `make` fails or runs out of memory during `./install.sh`

**Symptom:** Compiler killed, OOM, or cryptic build errors in OpenCV/ORB-SLAM2.

**Fix:**

1. Allocate **≥ 6 CPU cores and 8+ GB RAM** to the VM.
2. Do **not** re-run `./install.sh`. Resume manually:

```bash
cd simulatorMapping/build
cmake ..
make -j$(nproc)
```

If still failing, reduce parallelism: `make -j2`.

---

### 2. Executable starts but immediately errors on missing vocabulary, model, or video

**Symptom:** ORB-SLAM2 fails to load vocabulary; Pangolin cannot open model; video capture fails.

**Fix:**

1. Create config from the template if you have not already:

```bash
cp generalSettings.json.example generalSettings.json
```

2. Extract vocabulary:

```bash
cd Thirdparty/slam/Vocabulary && tar -xf ORBvoc.txt.tar.gz && cd -
```

3. Edit **`generalSettings.json`** — set **absolute paths** for:
   - `vocabulary_path` → `.../Thirdparty/slam/Vocabulary/ORBvoc.txt`
   - `model_path` → your `.obj` file
   - `offline_video_test_path` → valid video file
   - `simulator_output_dir` → writable directory

4. Confirm you run from `build/`:

```bash
cd build && ./examples/runSimulator
```

---

### 3. Pangolin window does not appear / `OpenGL` / display errors

**Symptom:** Crash on startup, "cannot open display", blank window, or GL errors over SSH.

**Fix:**

1. Run on a machine with a **local graphical session** (Ubuntu desktop).
2. Over SSH, enable X forwarding: `ssh -X user@host` and ensure an X server is available.
3. Verify OpenGL libraries installed (re-run apt packages from `install.sh` lines 9–19).
4. Inside the simulator, press **Tab** to start SLAM scanning after the model loads — the program waits for this signal.

---

## Quick reference links

| Resource | URL |
|---|---|
| Repository | https://github.com/rbdlabhaifa/simulatorMapping |
| Student guide (PDF) | https://drive.google.com/file/d/1HKZ_ecanpfv_IFJg1f98f6U__ivAlVef/view?usp=sharing |
| Lab data bundle (zip) | https://drive.google.com/file/d/1McpMjSu7-ziM0-tqMumX0LM7evFS9Irs/view?usp=sharing |
