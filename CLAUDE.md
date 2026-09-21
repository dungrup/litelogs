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
3. A **third binary** (`ply-cat`) that concatenates the extracted ped cloud
   back onto the decoded remainder, one merged `.ply` per frame
   (`tools/ply-cat.cpp`).

Everything else is stock TMC13 — do not treat unmodified upstream files as
fork code. To see exactly what this fork owns:

```
git diff --stat a3d15c5..HEAD
```

Files that diverge from upstream: `CMakeLists.txt`, `tmc3/CMakeLists.txt`,
`tmc3/encoder.cpp`, `tmc3/PCCTMC3Encoder.h`, `tmc3/LiteLogs.cpp` (new),
`tools/ply-cat.cpp` (new), `decode.sh` (new), `encoder_fast_l1.cfg` /
`encoder_fast_l3.cfg` (new), `README.md`, `.gitignore`. The older single
`encoder_fast.cfg` has been **deleted** in favour of the two `_l1` / `_l3`
rate points.

`tools/ply-merge.cpp` is **upstream**, not fork code — don't confuse it with
`ply-cat`. Its "merge" means grouping N frames into one cloud tagged by
`frameindex`, which is a different operation entirely.

## Building

```
mkdir -p build && cd build && cmake .. && make
```

Produces three binaries in `build/tmc3/`:

| binary | what it is | links |
|---|---|---|
| `tmc3` | stock encoder+decoder | all codec objects, `Threads::Threads` |
| `litelogs` | batch extract/encode/decode tool | all codec objects, `Threads::Threads` |
| `ply-cat` | ped + rest merge tool | `ply.cpp`, `misc.cpp`, `program_options_lite.cpp`, `version.cpp` |

All three are in the default `all` target, so a plain `make` builds them.
To build just one: `cmake --build build --target ply-cat`.

`ply-cat` deliberately links **no codec objects** — it only needs the ply
reader/writer and `PCCPointSet3` (header-only). Keep it that way: it is a
~5-second build, which is the point of having it separate from `litelogs`.
`ply-merge` (upstream) has the same link set but is `EXCLUDE_FROM_ALL`.

- The fork bumped the standard from **C++11 to C++17** in the top-level
  `CMakeLists.txt` — `LiteLogs.cpp` needs `<filesystem>`. Don't revert this.
- `tmc3/CMakeLists.txt` moved `TMC3.cpp` out of the shared `PROJECT_CPP_FILES`
  glob into a new `PROJECT_COMMON_FILES` set, so `tmc3` and `litelogs` share
  all objects but each get their own `main()`. Add new shared sources to
  `PROJECT_CPP_FILES`; never add `TMC3.cpp`, `LiteLogs.cpp` or `ply-cat.cpp`
  there (duplicate `main`).
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

Paths below are relative to `samsung_evo/LiteLogs/`; `ICRA` abbreviates
`KITTI_Attr_ICRA/litelogs_l3`. **They name an `_l3` output tree, so switching
rate point means editing them** — nothing derives the directory from the cfg.

| constant | current value | role |
|---|---|---|
| `dataPath` | `KITTI_Attr/val_ply` | input `.ply` frames |
| `detPath` | `KITTI_Attr_ICRA/lidar_raw_dets_val` | per-frame detections |
| `stemListPath` | `<repo>/test_folder/val_peds.txt` | which frames to process |
| `compressedPath` | `ICRA/rest_only_compressed_bin` | `<stem>.bin` |
| `reconPath` | `ICRA/rest_only_decoded_ply/` | decoded `<stem>.ply` |
| `pedPath` | `ICRA/ped_ply/` | `<stem>/ped.ply` |
| `csvPath` | `ICRA/encoding_times.csv` | results |

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

- A debug cap, `if (successCount == 200) break;`, sits **commented out** at
  `LiteLogs.cpp:515-516`. Left in place on purpose for quick 200-frame smoke
  runs; uncomment it and every run silently truncates to 200 of the 1291
  stems in `val_peds.txt`. Check it before trusting a short run.

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

### `ply-cat` (merge ped + decoded rest)

```
./build/tmc3/ply-cat \
  --reconDir=<ICRA>/rest_only_decoded_ply \
  --pedDir=<ICRA>/ped_ply \
  --outDir=<ICRA>/merged_ply \
  --stemList=test_folder/val_peds.txt \
  --csvPath=<ICRA>/merge_summary.csv
```

Unlike `litelogs`, nothing is hardcoded — every path is an option, and
`--stemList` takes the same `val_peds.txt` so both tools walk the same frames
in the same order. Options are `--opt=value` only (see below).

| option | default | role |
|---|---|---|
| `reconDir` | — | decoded remainder, `<stem>.ply` |
| `pedDir` | — | extracted peds, `<stem>/ped.ply` |
| `outDir` | — | merged `<stem>.ply` (created if absent) |
| `stemList` | — | work list, one stem per line |
| `csvPath` | unset | optional per-frame summary |
| `positionScale` | `1000.` | read scale, inverted on write |
| `outputBinaryPly` | `true` | binary vs ascii output |

CSV columns: `stem,recon_points,ped_points,total_points,status`, where `status`
is `ok`, `ok_no_ped`, `recon_failed`, `ped_failed`, `attr_mismatch` or
`write_failed`. A frame fails without aborting the batch; the exit code is 1
if any frame failed.

Three things that are easy to get wrong here:

- **`positionScale` is load-bearing.** `ply::read` multiplies by it and
  truncates into `Vec3<int32_t>` (`ply.cpp:407-409`, `PCCPointSet.h:60`), so
  the default of `1.0` used by `ply-merge` would collapse every point to
  integer **metres**. 1000 matches `inputScale` and keeps the ped cloud's
  millimetre precision exactly. The recon lattice is
  `1/(inputScale × positionQuantizationScale)` = 133.3 mm at `_l3`, which is
  not a whole number of mm, so recon points shift by **≤0.67 mm** on the
  round-trip — measured, and ~200× below the quantization step.
- **A missing `ped.ply` is normal.** `compressOneFrame` only writes one when
  `_lastPedPoints` is nonzero; in the 1291-frame `_l3` run, 9 frames have none.
  Those pass through as `ok_no_ped`, not an error.
- **Attribute parity is checked before the append**, because
  `PCCPointSet3::append` guards each copy on `hasX() && src.hasX()` and
  silently leaves the rest uninitialised. A mismatch is refused outright rather
  than written out.

`ply::write` always emits `property uint16 refc` regardless of the
`PropertyNameMap` (`ply.cpp:132`) — only positions are named by it — and
`ply::read` accepts `reflectance` or `refc` (`ply.cpp:349`), so the round-trip
closes. In binary mode positions are written as `float64`, in ascii as `float`.

Note `litelogs_eval.py` also concatenates these two clouds in
`run_class_separated_compression`. Decide which one owns the merge rather than
letting both drift.

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

`OrientedBox::half` holds **inflated** half-extents in **millimetres**
(`det.size * inputScale * inflation / 2`), matching `ply::read`, which
multiplies by `inputScale` and truncates into `Vec3<int32_t>`. `Detection::size`
keeps the detector's true extents — the inflation lives only in the box, so
anything reporting box dimensions still sees real ones. `contains` tests, in order: a squared
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

`detBoxInflation` (cfg option, default `1.`) scales the box extents about their
centre before the point test, pulling in more points. It is applied in
`makeBoxes` — deliberately not in `readDetections` (which stays a pure parser
of that error-prone column order) and not in `contains` (see below). Values
`<= 0` are rejected in `ParseParameters`: they would collapse or invert every
box and silently extract nothing rather than fail visibly.

Note `box.xyRadiusSq` is computed **from** `box.half`, so the early reject grows
with the box automatically. Preserve that assignment order: deriving the radius
before applying inflation would leave the reject cutting at the original
circumradius, and extraction counts would climb a little and then stop
responding to the factor.

Two behaviours to expect when raising it. A pedestrian box is roughly
0.6 × 0.6 × 1.8 m, so a uniform factor adds three times as much vertically as
laterally, and half of that vertical growth goes **downward into the ground
plane** — extra points skew towards ground return, not person. And the union
of `ped.ply` and the coded cloud is invariant, so `ply-cat`'s `total_points`
column should be identical at every inflation setting; that makes a good
end-to-end check.

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

The ped cloud is **not** merged back in here — `ped.ply` and the decoded rest
stay separate, and `ply-cat` (or `litelogs_eval.py`) joins them afterwards.

Merging *inside* `LiteLogs.cpp` is the hard version, and that is why it is not
done: the recon cloud is still in integer coding coords at that point, so the
ped cloud would need mapping from input-mm through `outputScale` /
`outputOrigin`. **`ply-cat` faces no such problem** — by the time both clouds
are files, `writeOutputFrame` and `ply::write` have already applied those
scales, so the two are in the same external system (metres, velodyne) and a
plain `append` is correct. Verified on frame `000001`: ped x ∈ [11.35, 11.82]
sits inside recon x ∈ [-79.47, 77.07], against source x ∈ [-79.43, 77.01].

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
(getter `getLastTileEncodingMs()`), measured with `gettimeofday` — there's an
explicit comment that `chrono` was distrusted here.
`SequenceEncoder::compressOneFrame` copies it into `_lastEncodeOnlyMs`
(`LiteLogs.cpp:2702`), which becomes the `encode_only_ms` CSV column. Alongside
it, `_lastExtractMs`, `_lastPedPoints` and `_lastRestPoints` feed the
`extract_ms`, `ped_points` and `rest_points` columns; all four reset at the top
of `SequenceEncoder::compress`.

**`_lastTileEncodingMs` is assigned unconditionally, on both encode paths.**
The `frame_time` stopwatch starts at `encoder.cpp:347-348` and stops at
`866-869` — four lines *after* the `if (!canThreadTiles) … } else { … }` branch
closes at 864 — so the serial path is timed exactly like the threaded one. The
only `return`s in that span (686, 690, 763, 767, 776, 827) are inside the
worker lambdas and return from the lambda, not from `compress`. A CSV row
reads `NA` only if `compress` throws.

There is a *second*, older stopwatch, `tile_time`, at `encoder.cpp:843-846`.
That one **is** inside the threaded branch, and it now feeds nothing but the
commented-out `"Tile encoding took"` print. Don't mistake it for the value that
reaches the CSV. (Earlier revisions did assign `_lastTileEncodingMs` from it,
which is why "timings go `NA` when `canThreadTiles` is false" was true once and
is not any more.)

Know what `frame_time` actually spans, because it is wider than "encode only":
`quantization()`, the SPS/GPS/APS parameter-set writes, tile and slice
partitioning, all slice encoding, and the per-tile recon merge. The run
summary's **"Total processing time (wall/user, encode + decode)"** shares one
`clock_user` across both passes, so those totals include decode; the per-file
wall line does too. There is no per-frame decode column — add one if you need
the split.

Extraction shrinks the coded cloud and can therefore change the tile count. If
it drops below 2, `canThreadTiles` goes false and the frame encodes serially —
that shows up as a **slower** `encode_only_ms`, never as `NA`. The
`rest_points` column makes it diagnosable.

## Conventions

- Match upstream TMC13 style: 2-space indent, 80-column, `.clang-format` at the
  repo root. Fork additions follow it.
- Files under `tmc3/` other than the ones listed above are upstream; prefer
  making changes in `encoder.cpp` / `LiteLogs.cpp` / `tools/ply-cat.cpp` so the
  fork delta stays small and rebasable onto newer TMC13 releases.
- `.clang-format` is enforced by eye, not CI, but the binary on PATH may be a
  broken pip shim — use `/usr/bin/clang-format-18` explicitly.
- `test_folder/` is gitignored scratch space: `val_peds.txt` (the 1291-stem work
  list, still read by both `litelogs` and `ply-cat`), older compressed/recon/ped
  output, result CSVs, and `extract_ped_sample/` (reference output from
  `extract_ped_plys.py`, useful for validating the extraction). Current runs
  write to `samsung_evo/LiteLogs/KITTI_Attr_ICRA/` instead.
- Reference implementations live outside this repo, under
  `/home/dungrup/wd_black/OpenPCDet/tools/`: `eval_lidar_only.py` (writes the
  detections), `extract_ped_plys.py` (the Python extraction this port must
  match), `litelogs_eval.py` (downstream evaluation).
