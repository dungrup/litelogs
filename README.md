# LiteLogs: A fork of TMC13

## Building

### OSX
- mkdir build
- cd build
- cmake .. -G Xcode 
- open the generated xcode project and build it

### Linux

```
> mkdir build
> cmake -B ./build/ -DCMAKE_BUILD_TYPE=Release 
> cmake --build ./build -j$(nproc)
```

This builds all three binaries. To rebuild just one:

```
> cmake --build ./build --target ply-cat
```

`ply-cat` links only the ply reader/writer and the option parser — no codec
objects — so it rebuilds in a few seconds.

Use `-DCMAKE_BUILD_TYPE=Release` for any run you intend to take timings from;
Debug numbers are not meaningful.

### Windows
- md build
- cd build
- cmake .. -G "Visual Studio 15 2017 Win64"
- open the generated visual studio solution and build it


## Running

**UPDATE:** the build produces three binaries in `./build/tmc3/`:

| binary | role |
|---|---|
| `tmc3` | stock TMC13 encoder + decoder |
| `litelogs` | batch tool: extract peds, encode the rest, decode it straight back |
| `ply-cat` | merge the extracted ped cloud back onto the decoded remainder |

`litelogs` adds tile-level and slice-level multithreading to the encode path.
It is no longer encoder-only — after each successful encode it decodes the
bitstream back in-process and writes the reconstructed `.ply`.

### litelogs

1. Point to correct paths in `LiteLogs.cpp` (L317-329) — they are hardcoded
   constants, not command line options
2. Re-build using above instructions
3. The frames to process are read from __stemListPath__ (one file stem per
   line, processed in the order written), *not* by scanning __dataPath__
4. For each frame, litelogs writes the ped cloud to __pedPath__, the bitstream
   to __compressedPath__ and the decoded remainder to __reconPath__. Timing
   stats land in __csvPath__

```
> ./build/tmc3/litelogs -c encoder_fast_l3.cfg
```

Tune necessary params in __encoder_fast_l1.cfg__ / __encoder_fast_l3.cfg__ —
the two files are one rate point apart, differing only in
`positionQuantizationScale` and `tileSize`.

### ply-cat

`litelogs` keeps the ped cloud and the decoded remainder in separate files.
`ply-cat` joins them into one `.ply` per frame. Every path is an option here,
and `--stemList` takes the same work list, so both tools walk the same frames:

```
> ./build/tmc3/ply-cat \
    --reconDir=<reconPath> \
    --pedDir=<pedPath> \
    --outDir=<merged output dir> \
    --stemList=test_folder/val_peds.txt \
    --csvPath=<merge summary csv>
```

Options must be given as `--opt=value`; a space-separated value is silently
ignored. Frames whose detections held no pedestrians have no `ped.ply` at all
and simply pass through. `--positionScale` (default 1000) is the scale applied
on read before truncation to integers and inverted on write — leaving it at
1000 matches `inputScale` and preserves the ped cloud's millimetre precision.


This TMC13 codec implementation encodes frame sequences.  A single binary
contains the encoder and decoder implementation, with selection using
the `--mode` option.  Documentation of options is provided via the
`--help` command line option.


### Runtime configuration and configuration files

All command line parameters may be specified in a configuration file.
A set of configuration file templates compliant with the current Common
Test Conditions is provided in the cfg/ directory.

### Example

To generate the configuration files, run the gen-cfg.sh script:

```console
mpeg-pcc-tmc13/cfg$ ../scripts/gen-cfg.sh --all
```

An example script (`scripts/Makefile.tmc13-step`) demonstrates how
to launch the encoder, decoder and metric software for a single
input frame.  The VERBOSE=1 make variable shows the detailed command
execution sequence.  Further documentation of the parameters are
contained within the script.

The following example encodes and decodes frame 0100 of the sequence
`Ford_01_q_1mm`, making use of the configuration file
`cfg/lossy-geom-no-attrs/ford_01_q1mm/r01/encoder.cfg` and storing
the intermediate results in the output directory
`experiment/lossy-geom-no-attrs/ford_01_q1mm/r01/`.

```console
mpeg-pcc-tmc13$ make -f $PWD/scripts/Makefile.tmc13-step \
    -C experiment/lossy-geom-no-attrs/ford_01_q1mm/r01/ \
    VPATH=$PWD/cfg/octree-predlift/lossy-geom-no-attrs/ford_01_q1mm/r01/ \
    ENCODER=$PWD/build/tmc3/tmc3 \
    DECODER=$PWD/build/tmc3/tmc3 \
    PCERROR=/path/to/pc_error \
    SRCSEQ=/path/to/Ford_01_q_1mm/Ford_01_vox1mm-0100.ply \
    NORMSEQ=/path/to/Ford_01_q_1mm/Ford_01_vox1mm-0100.ply

  [encode]  Ford_01_vox1mm-0100.ply.bin <- /path/to/Ford_01_q_1mm/Ford_01_vox1mm-0100.ply
  [md5sum]  Ford_01_vox1mm-0100.ply.bin.md5
  [md5sum]  Ford_01_vox1mm-0100.ply.bin.ply.md5
  [decode]  Ford_01_vox1mm-0100.ply.bin.decoded.ply <- Ford_01_vox1mm-0100.ply.bin
  [md5sum]  Ford_01_vox1mm-0100.ply.bin.decoded.ply.md5
  [metric]  Ford_01_vox1mm-0100.ply.bin.decoded.ply.pc_error <- Ford_01_vox1mm-0100.ply.bin.decoded.ply
```

## Intra and inter prediction

The yaml files stored directly under the cfg/ folder correspond to intra prediction, and yaml files stored under cfg/inter/ folder correspond to inter prediction. The gen-cfg.sh script is updated such that intra/inter prediction may be specified as an additional option to produce the configuration files corresponding to intra/inter prediction; alternately, the "--all" option may be used to generate the configuration for intra and inter prediction for all tool configurations.

After running the gen-cfg.sh script, the configuration files for intra and inter prediction are generated in separate folders. The configuration files corresponding to inter prediction are generated in folders with "-inter" suffix. For example, configuration files corresponding to octree and predicting/lifting transform using intra prediction are generated in the folder octree-predlift/ (as was the case in some earlier versions of the test model), and configuration files corresponding to octree and predicting/lifting transform using inter prediction are generated in the folder octree-predlift-inter/.




