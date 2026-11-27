# Recordings of the code before the gyro (Issue #92)

`golden_<toolchain>_mapping_short_course.txt` and `golden_<toolchain>_localization_short_course.txt` hold
every output of the `drive_short_course` drive (see `../golden_output.hpp`) as it was produced by the
`Localizer` before the gyro work. In `test/test_localizer.cpp`, `LocalizerGyro.*IsIdentical…` compares today's
code with the recording **of the same toolchain** bit for bit (a hash of every output). Floating point
differs between toolchains (compiler, libm, CPU), so a toolchain without a recording **fails** the test
("golden for toolchain <name> is missing; regenerate with test/data/README.md"); there is no loose comparison.

`<toolchain>` is `${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}`
with every character outside `A-Za-z0-9._-` replaced by `_`; CMake passes it as `LIGHTNAV_GOLDEN_TOOLCHAIN`
(see `CMakeLists.txt`). The test prints the name it looks for when the recording is missing.

One line per output: `<hash of every bit> <state> x y z roll pitch yaw position_std` (the numbers are
informative; only the hash and, through it, the state are compared).

## Which recordings exist

| Toolchain | Made from (base commit) | Made on | Notes |
|---|---|---|---|
| `Darwin-arm64-Clang-21.1.8` | `7b909df` (PR-1 head); the same bytes also come out of `a1771d0` | macOS 26 (Darwin 25.6.0), arm64, the pixi environment's clang 21.1.8 | The first recording was made from `a1771d0`; recording again from `7b909df` with the procedure below gave identical files. |
| `Linux-x86_64-GNU-13.3.0` | `7b909df` | The build machine (AMD Ryzen 7 4700GE, Ubuntu 24.04), inside the image of the `test` stage of `docker/Dockerfile` (the CI `docker` job; the same apt gcc 13.3.0 and glibc as the production X13) | Written twice on the same machine: byte for byte identical. Also matched on both CI runners (see the next row) on 2026-11-28. |
| `Linux-x86_64-GNU-15.3.0` | `7b909df` | The same build machine, in the pixi linux-64 environment (the same gcc 15.3.0 as the CI `build-test (ubuntu-latest)`; libm is the glibc of the machine that runs it) | Matched on both CI runners: Ubuntu 24.04 (glibc 2.39) and Ubuntu 26.04 (checked on 2026-11-28). libm comes from the glibc of the running machine, so a runner with yet another glibc may still differ; then record again. |

Add a row for every recording you commit.

## Recording again

The recording must come from the sources **before the gyro**: use `7b909df` (the head of
`feat/mapping-tilt-height-loop`, PR-1). It builds with gcc as well (`a1771d0` does not: `-Wdangling-else`
in `test_mapping.cpp`). `a1771d0 .. 7b909df` leaves the outputs of the drive unchanged with the default
parameters (reviewed), and on the Mac both commits give identical files. The writer is
`GoldenRecording.WriteWhenRequested`; it does nothing unless `LIGHTNAV_WRITE_GOLDEN` is set, and it only uses
what `Localizer` offered at that commit.

```sh
# 1. a work tree of the base commit (needs its own pixi environment, or a link to an existing one)
git worktree add ../lightnav-golden-base 7b909df
cd ../lightnav-golden-base

# 2. copy the support header from this branch, add the writer to the base test file and give the tests
#    the toolchain name (the same lines as in this branch's CMakeLists.txt)
cp <this-branch>/src/lightnav_localization/test/golden_output.hpp src/lightnav_localization/test/
python3 - <<'PY'
p = 'src/lightnav_localization/test/test_localizer.cpp'
s = open(p).read()
include_marker = '#include "synthetic_world.hpp"'
assert include_marker in s, 'include marker not found in test_localizer.cpp'
s = s.replace(include_marker, include_marker + '\n#include "golden_output.hpp"', 1)
s += '''
TEST(GoldenRecording, WriteWhenRequested) {
  const char* const directory = std::getenv("LIGHTNAV_WRITE_GOLDEN");
  if (directory == nullptr) return;
  lightnav_localization::testing::write_golden_recordings(directory);
}
'''
open(p, 'w').write(s)
p = 'src/lightnav_localization/CMakeLists.txt'
s = open(p).read()
anchor = '  target_include_directories(test_matcher SYSTEM PRIVATE'
assert anchor in s, 'anchor not found in CMakeLists.txt'
s = s.replace(anchor, '''  string(REGEX REPLACE "[^A-Za-z0-9._-]" "_" lightnav_golden_toolchain
         "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}")
  target_compile_definitions(test_localizer PRIVATE LIGHTNAV_GOLDEN_TOOLCHAIN="${lightnav_golden_toolchain}")
''' + anchor, 1)
open(p, 'w').write(s)
PY

# 3. build and run with the variable set (the files are named after the toolchain)
pixi run build
mkdir -p /tmp/golden
LIGHTNAV_WRITE_GOLDEN=/tmp/golden pixi run bash -c \
  'DYLD_LIBRARY_PATH=$PIXI_PROJECT_ROOT/install/lib ./build/lightnav_localization/test_localizer --gtest_filter=GoldenRecording.*'

# 4. copy the two files into this directory, add a row to the table above, and commit
cp /tmp/golden/*.txt <this-branch>/src/lightnav_localization/test/data/
```

### The docker recording (`Linux-x86_64-GNU-13.3.0`)

This is the toolchain of the image of the `test` stage of `docker/Dockerfile` (the CI `docker` job; the
production machine uses the same apt gcc and glibc). It is recorded inside that image, on the build machine
(do not run `docker build` on the Mac). Use the work tree of `7b909df` prepared by steps 1 and 2 above (the
copied header, the writer test and the CMake lines), and run in it:

```sh
docker build -f docker/Dockerfile --target test -t lightnav:golden-7b909df .

mkdir -p <out>
docker run --rm --user $(id -u):$(id -g) -e TSUKUCHALLE_UID=$(id -u) \
  -e LIGHTNAV_WRITE_GOLDEN=/out -v <out>:/out --entrypoint bash lightnav:golden-7b909df \
  -c 'cd /ws && . install/setup.bash && ./build/lightnav_localization/test_localizer --gtest_filter=GoldenRecording.*'
```

Copy the two files in `<out>` into `test/data/` and add or update the row of the table.

On Linux use `LD_LIBRARY_PATH` instead of `DYLD_LIBRARY_PATH` (or whatever the repository's test command
sets). The files change when the drive in `golden_output.hpp` changes, or when the compiler, its version or
the CPU architecture changes (a new toolchain name, hence a new recording).
