# CLAUDE.md

## What this repo is

A fork of **MPEG PCC TMC13** (G-PCC reference software, branched at `a3d15c5`,
release-v23.0-rc2) called **LiteLogs**. The fork adds:

1. **Tile-level and slice-level multithreading** to the encoder
   (`tmc3/encoder.cpp`).
2. A **second binary** (`litelogs`) that batch-processes a list of `.ply`
   frames: it extracts pedestrian/cyclist points out of each cloud, encodes
   only the remainder, decodes the bitstream straight back, and writes
   per-frame timings to CSV (`tmc3/LiteLogs.cpp`).

Everything else is stock TMC13 — do not treat unmodified upstream files as
fork code. To see exactly what this fork owns:

```
git diff --stat a3d15c5..HEAD
```

Files that diverge from upstream: `CMakeLists.txt`, `tmc3/CMakeLists.txt`,
`tmc3/encoder.cpp`, `tmc3/PCCTMC3Encoder.h`, `tmc3/LiteLogs.cpp` (new),
`decode.sh` (new), `encoder_fast_l1.cfg` / `encoder_fast_l3.cfg` (new),
`run_configs.sh` (new), `README.md`, `.gitignore`. The older single
`encoder_fast.cfg` has been **deleted** in favour of the two `_l1` / `_l3`
rate points.

## Building

```
mkdir -p build && cd build && cmake .. && make
```

Produces two binaries in `build/tmc3/`: `tmc3` (stock encoder+decoder) and
`litelogs` (batch tool). Both link `Threads::Threads`.

- The fork bumped the standard from **C++11 to C++17** in the top-level
  `CMakeLists.txt` — `LiteLogs.cpp` needs `<filesystem>`. Don't revert this.
- `tmc3/CMakeLists.txt` moved `TMC3.cpp` out of the shared `PROJECT_CPP_FILES`
  glob into a new `PROJECT_COMMON_FILES` set, so `tmc3` and `litelogs` share
  all objects but each get their own `main()`. Add new shared sources to
  `PROJECT_CPP_FILES`; never add `TMC3.cpp` or `LiteLogs.cpp` there (duplicate
  `main`).
- The existing `build/` cache is configured `CMAKE_BUILD_TYPE=Debug` with
  clang++-14. For timing measurements, reconfigure with
  `-DCMAKE_BUILD_TYPE=Release` — Debug numbers are not meaningful.

## Running

### `litelogs` (the fork's main tool)

```
./build/tmc3/litelogs -c encoder_fast_l1.cfg
```

**All paths are hardcoded** in `tmc3/LiteLogs.cpp:317-329` and must be edited +
rebuilt to point elsewhere. They are NOT command-line options:

| constant | current value | role |
|---|---|---|
| `dataPath` | `samsung_evo/LiteLogs/KITTI_Attr/val_ply` | input `.ply` frames |
| `detPath` | `samsung_evo/LiteLogs/KITTI_Attr/lidar_raw_dets_val` | per-frame detections |
| `stemListPath` | `test_folder/val_peds.txt` | which frames to process |
| `compressedPath` | `test_folder/compressed_slicing_new_lite/` | `<stem>.bin` |
| `reconPath` | `test_folder/recon_ply/` | decoded `<stem>.ply` |
| `pedPath` | `test_folder/ped/` | `<stem>/ped.ply` |
| `csvPath` | `test_folder/slicing_encoding_times_new_lite.csv` | results |

`litelogs` ignores `uncompressedDataPath`/`compressedStreamPath` from the cfg
and overrides them per file; it also forces `firstFrameNum=0`, `frameCount=1`
(one frame per file). The required-argument checks for those two options are
commented out in `ParseParameters`.

**The work list comes from `stemListPath`, not from the directory.** One file
stem per line; `dataPath` is never enumerated. Lines are trimmed (CRLF and
indentation tolerated) and blanks skipped; frames run **in the order written**,
so there is deliberately no `std::sort`. A stem with no matching `.ply` is
warned about and counted in the summary's `missing=` field, but does not fail
the run.

CSV columns:

```
input_ply,compressed_bin,extract_ms,ped_points,rest_points,encode_only_ms,status
```

`status` is `ok`, `failed` (encoder) or `decode_failed`.

`litelogs` **does not support bi-prediction GOF mode** —
`biPredictionEnabledFlag` causes an immediate error exit, and `compressOneGOF`
/ `setMotionVectorFileName` are stripped from `SequenceEncoder::compress`.

- **`if (successCount == 200) break;`** near the end of the per-file loop
  (`LiteLogs.cpp:515`) is a debug cap. It silently truncates any run to 200
  frames — `val_peds.txt` has 1291 stems. Remove it for real runs.

### Configs

`encoder_fast_l1.cfg` and `encoder_fast_l3.cfg` are two rate points over the
same KITTI-style lidar setup (`srcUnit: 1`, `inputScale: 1000`, RAHT
`transformType: 3`, `attribute: reflectance`). They differ in exactly two
lines:

| | `_l1` | `_l3` |
|---|---|---|
| `positionQuantizationScale` | 0.05 | 0.0075 |
| `tileSize` | 1000 | 5000 |

`tileSize` is what enables the parallel path — see below.

The `val_ply` frames carry `property uint8 reflectance`, which matches
`attribute: reflectance`. Beware other PLY sets: the Open3D-written clouds
under `hdd_vol/KITTI/g-pcc/ply_files` carry `uchar red/green/blue` instead and
trip `assert(codeReflectance == pointCloud.hasReflectances())`.

`outputBinaryPly: 0` means ASCII recon output, roughly **3 MB per frame**.
Set it to `1` unless you need text.

### `run_configs.sh`

```
./run_configs.sh                       # encoder_fast_l1.cfg then _l3.cfg
./run_configs.sh some_other.cfg
```

Because the output paths are hardcoded and the CSV is opened `std::ios::trunc`,
consecutive configs would overwrite each other. The script therefore **moves**
each run's outputs into `test_folder/runs/<config stem>/` (along with a copy of
the `.cfg` and a `run.log`) before starting the next. Pre-existing output found
on the first run is stashed to `test_folder/runs/_prior_<timestamp>/` rather
than deleted. A config that exits nonzero still gets its partial output
archived; the script reports it and exits 1 at the end.

Output-path variables at the top of the script **must track the hardcoded paths
in `LiteLogs.cpp`** — they are duplicated, not derived.

### Standalone decoding

`decode.sh <bin_dir> <out_dir>` loops the stock `tmc3 --mode 1` over `*.bin`.
Mostly redundant now that `litelogs` decodes in-process, but still useful for
re-decoding an archived run. Note it writes to `$OUTDIR/recon_ply/` but only
`mkdir -p "$OUTDIR/"`, and the `set -e` makes the `ec=$?` check dead code —
create `recon_ply/` yourself.

Options must use `--opt=value`. A space-separated value is silently dropped by
`program_options_lite` (it warns "Unhandled argument ignored" and leaves the
option at its default). `decode.sh`'s `--mode 1` survives only by accident:
`mode` maps to a `bool` (`params.isDecoder`), so bare `--mode` sets it true and
the stray `1` is discarded. Don't copy that style for anything else.

## Pedestrian/cyclist extraction (`tmc3/LiteLogs.cpp`)

Points inside pedestrian/cyclist boxes are routed **around** the codec: written
verbatim to `<pedPath>/<stem>/ped.ply`, with only the remainder encoded. This
replaces an external Python step (`OpenPCDet/tools/extract_ped_plys.py`) and
saves an ASCII PLY write + read per frame.

The split lives in `compressOneFrame` immediately after `ply::read`
(`LiteLogs.cpp:2618-2659`), before the azimuth sort. Supporting pieces:
`Detection` / `OrientedBox` (265, 284), `isPedestrian` (2480), `readDetections`
(2496), `makeBoxes` (2545), `splitPedestrianPoints` (2568). Sub-clouds are
built with `pcc::getPartition` (external linkage from `encoder.cpp`,
forward-declared at `LiteLogs.cpp:66`) so attribute copying isn't duplicated.

### Detection file format — read this before touching `readDetections`

Detections are produced by `OpenPCDet/tools/eval_lidar_only.py` and are
**already in the velodyne frame**. No calib file is needed or read; the
`RectToVelo` / `readCalib` plumbing that an earlier camera-frame version
required has been removed.

That writer deliberately reorders its columns:

```python
# pred_boxes: [x, y, z, dx, dy, dz, heading], write as class x y z w h l ry
w, h, l = box[4], box[5], box[3]
```

So the file is `class x y z w h l ry` — columns 4,5,6 are **`(w, h, l)`**, i.e.
`(dy, dz, dx)`, with the height in the *middle*. `readDetections` must map them
back:

```cpp
det.size = Vec3<double>(l, w, h);   // (dx, dy, dz)
det.heading = ry;                   // already a lidar heading
```

**This has been got wrong once already.** Reading the trio straight into
`dx, dy, dz` permutes the extents without changing their product, so the boxes
stay plausible-looking while the vertical extent nearly halves — the counts
come out wrong (e.g. 2328 instead of 4642) with nothing obviously broken. The
counts are validated against `extract_ped_plys.py`; agreement should be exact
to within ±2 points (mm truncation vs float32, and closed-boundary `<=` vs
Delaunay `in_hull`).

Also note: because these are 7-DOF lidar boxes with a true centre, there is
**no** `+h/2` and **no** `-(ry + π/2)`. Both of those belong to the KITTI
camera-frame `label_2` convention and would be double-applied here.

### Geometry of the test

`OrientedBox::half` holds half-extents in **millimetres** (`det.size *
inputScale / 2`), matching `ply::read`, which multiplies by `inputScale` and
truncates into `Vec3<int32_t>`. `contains` tests, in order: a squared
xy-circumradius early reject (`xyRadiusSq = half[0]² + half[1]²`, valid because
rotation preserves length), the axis-aligned z slab (`half[2]`), then the exact
half-extent compare after rotating by `-heading`.

Consequence worth remembering: the split runs on **mm-truncated** positions, so
`ped.ply` is mm-precision, not the source float32. That is lossless relative to
the codec but not bit-exact with the input. It is well under KITTI's ~2 cm
ranging noise.

`detScoreThreshold` (cfg option, default `0.`) filters detections by score.
A failed `operator>>` zeroes its operand, so the optional score is parsed into
a scratch variable and only committed on success — otherwise score-less labels
would read as 0 and be filtered out.

## The decode pass

After a successful encode, `main()` decodes the bitstream back in-process
(`LiteLogs.cpp:474-481`) using the `SequenceDecoder` already present in the
file, writing `<reconPath>/<stem>.ply`. Verified byte-identical to a stock
`tmc3 --mode=1` decode of the same `.bin`.

**`reconstructedDataPath` is deliberately left unset during the encode.**
`runParams` is a per-file copy, so it is assigned only just before constructing
the decoder. Setting it earlier would make the encoder emit its own
reconstruction *and* pull the per-tile recon merge (`encoder.cpp:851-864`)
inside the window that `getLastEncodeOnlyMs()` measures. Don't "tidy" this by
hoisting the assignment.

The ped cloud is **not** merged back in — `ped.ply` and the decoded rest stay
separate, and `litelogs_eval.py` concatenates them
(`run_class_separated_compression`). Merging in C++ would need the ped cloud
mapped from input-mm into the recon's coding coords via `outputScale` /
`outputOrigin`, plus an attribute-parity check (`PCCPointSet3::append` silently
leaves attributes uninitialised on mismatch).

Note the decoder is **entirely serial** — `tmc3/encoder.cpp` is the only file in
the tree that uses threads, and the tile inventory is stored as metadata and
never used for dispatch. So runs pair a parallel encode with a single-threaded
decode.

Because the stock decoder is unmodified, its own per-slice chatter now appears
in batch output ("positions bitstream size", "reflectances processing time")
from `decoder.cpp:576, 771, 981, 986`. Those are upstream lines, unlike the
encoder's equivalents which the fork commented out. To silence them, swap
`std::cout.rdbuf` around the `decompress` call rather than editing
`decoder.cpp` — keeping that file stock is what keeps the fork rebasable.

## The parallel encode path (`tmc3/encoder.cpp`)

`PCCTMC3Encoder3::compress` now has two branches over `partitions.slices`. The
original serial loop is kept intact as the fallback; the threaded path only runs
when `canThreadTiles` holds:

```
tileSize > 0 && tileMaps.size() > 1
&& !predgeom && !trisoup
&& !interPrediction && !inter_frame_prediction && !biPrediction
&& !entropy_continuation && !inter_entropy_continuation
```

Those exclusions are load-bearing: each of them introduces cross-slice or
cross-frame state that the per-thread encoders do not share. **If you add a
feature that carries state across slices, add it to this guard.**

Structure when threading:

- Slices are bucketed by `tileId` into `tileToSlices` up front.
- An outer pool (`hwThreads`, currently **hardcoded to 8**, with the
  `hardware_concurrency()` call commented out just above) pulls tile indices off
  an atomic counter. An inner pool (`kMaxSliceWorkers = 8`) does the same for
  slices within a tile. So worst case ~64 threads.
- Each slice gets a **fresh `PCCTMC3Encoder3`** with its own copy of
  `EncoderParams`, its own `_ctxtMemAttrs`, and `_firstSliceInFrame = true`,
  `_prevSliceId = 0`. This is why entropy continuation must be off — every
  slice restarts its contexts.
- Callbacks are not invoked from worker threads. `BufferedCallbacks` records an
  ordered `events` list of payload/recolour indices; results are merged per
  tile, then replayed on the calling thread in tile order after the join. Keep
  this discipline — `callback->onOutputBuffer` is not thread-safe.
- Exceptions are captured per pool into a `std::exception_ptr` behind an
  `std::atomic<bool>` latch and rethrown after the join.

One behavioural difference from the serial path to be aware of:

- **Reconstructed cloud ordering**: the threaded path appends recon points
  grouped by tile (each tile's slices merged first, then tiles in index
  order); the serial path appends in slice-partition order. Bitstreams decode
  identically, but recon point order can differ.

## Timing instrumentation

Upstream's per-slice `std::cout` lines in **`encoder.cpp`** ("positions
bitstream size", "processing time (user)", "Tile number", "Slice number") are
**deliberately commented out**, not deleted, so the batch runs produce clean
output. Leave them commented; uncomment locally for debugging only. (The
decoder's copies are still live — see above.)

Real timing comes from `_lastTileEncodingMs` on `PCCTMC3Encoder3`
(getter `getLastTileEncodingMs()`), measured with `gettimeofday` around the
tile worker join — there's an explicit comment that `chrono` was distrusted here.
`SequenceEncoder::compressOneFrame` copies it into `_lastEncodeOnlyMs`, which
becomes the `encode_only_ms` CSV column. Alongside it, `_lastExtractMs`,
`_lastPedPoints` and `_lastRestPoints` feed the `extract_ms`, `ped_points` and
`rest_points` columns; all four reset at the top of `SequenceEncoder::compress`.

`encode_only_ms` covers the encode alone. The run summary's **"Total processing
time (wall/user, encode + decode)"** shares one `clock_user` across both passes,
so those totals include decode; the per-file wall line does too. There is no
per-frame decode column — add one if you need the split.

**Gotcha:** `_lastTileEncodingMs` is reset to `-1.0` at the top of `compress`
and only assigned inside the threaded branch. If `canThreadTiles` is false
(e.g. `tileSize` too large to yield >1 tile), every CSV row reads `NA` for
timing even though encoding succeeded. If a run produces all-`NA` timings,
check the tile count and the guard conditions first. Note that extraction
shrinks the cloud and can therefore change the tile count — the `rest_points`
column makes that diagnosable.

## Conventions

- Match upstream TMC13 style: 2-space indent, 80-column, `.clang-format` at the
  repo root. Fork additions follow it.
- Files under `tmc3/` other than the ones listed above are upstream; prefer
  making changes in `encoder.cpp` / `LiteLogs.cpp` so the fork delta stays small
  and rebasable onto newer TMC13 releases.
- `test_folder/` is gitignored scratch space: `val_peds.txt` (the 1291-stem work
  list), compressed/recon/ped output, result CSVs, `runs/` archives from
  `run_configs.sh`, and `extract_ped_sample/` (reference output from
  `extract_ped_plys.py`, useful for validating the extraction).
- Reference implementations live outside this repo, under
  `/home/dungrup/wd_black/OpenPCDet/tools/`: `eval_lidar_only.py` (writes the
  detections), `extract_ped_plys.py` (the Python extraction this port must
  match), `litelogs_eval.py` (downstream evaluation).
