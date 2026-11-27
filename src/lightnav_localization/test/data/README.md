# Recordings of the code before the gyro (Issue #92)

`golden_mapping_short_course.txt` and `golden_localization_short_course.txt` hold every output of the
`drive_short_course` drive (see `../golden_output.hpp`) as it was produced by the `Localizer` of commit
**a1771d0** (`feat/mapping-tilt-height-loop` before the gyro work, no `gyro_yaw_delta_rad` argument yet).
`LocalizerGyro.*IsIdentical…` compares today's code with them: a bit-for-bit hash per output, and, on a
platform whose floating point rounds differently, the state (exact) and the numbers (1e-4) of every output.

One line per output: `<hash of every bit> <state> x y z roll pitch yaw position_std` (numbers with 10
significant digits, for the tolerance path only).

## Recording again

The recording must come from the sources of a1771d0, never from the current ones. The writer is
`GoldenRecording.WriteWhenRequested` (it does nothing unless `LIGHTNAV_WRITE_GOLDEN` is set) and it only
uses what `Localizer` offered at a1771d0.

```sh
# 1. a work tree of the base commit (needs its own pixi environment, or a link to an existing one)
git worktree add ../lightnav-golden-base a1771d0
cd ../lightnav-golden-base

# 2. copy the two support headers from this branch and add the writer to the base test file
cp <this-branch>/src/lightnav_localization/test/golden_output.hpp src/lightnav_localization/test/
python3 - <<'PY'
p = 'src/lightnav_localization/test/test_localizer.cpp'
s = open(p).read()
s = s.replace('#include "synthetic_world.hpp"', '#include "synthetic_world.hpp"\n#include "golden_output.hpp"', 1)
s += '''
TEST(GoldenRecording, WriteWhenRequested) {
  const char* const directory = std::getenv("LIGHTNAV_WRITE_GOLDEN");
  if (directory == nullptr) return;
  lightnav_localization::testing::write_golden_recordings(directory);
}
'''
open(p, 'w').write(s)
PY

# 3. build and run with the variable set
pixi run build
mkdir -p /tmp/golden
LIGHTNAV_WRITE_GOLDEN=/tmp/golden pixi run bash -c \
  'DYLD_LIBRARY_PATH=$PIXI_PROJECT_ROOT/install/lib ./build/lightnav_localization/test_localizer --gtest_filter=GoldenRecording.*'

# 4. copy the two files into this directory and commit them
cp /tmp/golden/*.txt <this-branch>/src/lightnav_localization/test/data/
```

(On Linux use `LD_LIBRARY_PATH` instead of `DYLD_LIBRARY_PATH`, or whatever the repository's test command
sets.) The files change when the drive in `golden_output.hpp` changes, and they depend on the compiler and
CPU (Apple clang / arm64 fuses multiply-add; gcc on x86-64 does not by default), so a recording made on
one platform matches bit for bit on that platform only.
