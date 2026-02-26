#!/usr/bin/env bash
# Usage: ./kitti_expt.sh /path/to/ply_dir /path/to/output_dir /path/to/tmc3 \
#   /path/to/encoder.cfg 
set -euo pipefail

BINDIR="$1"
OUTDIR="$2"
TMC3_BIN="./build/tmc3/tmc3"

mkdir -p "$OUTDIR/"

for src in "$BINDIR"/*.bin; do
  [ -e "$src" ] || continue
  base="$(basename "$src" .bin)"
  binfile="$src"                                 # input to decoder
  dec_recon="$OUTDIR/recon_ply/${base}.ply"           # decoder output to keep

  echo "Decoding $src -> $binfile"
  "$TMC3_BIN" --mode 1 \
    --compressedStreamPath="$binfile" \
    --outputBinaryPly=0 \
    --reconstructedDataPath="$dec_recon" 
  ec=$?
  if [ $ec -ne 0 ]; then
    echo "Decoder failed on $src (exit $ec)" >&2
    continue
  fi
  
  echo "Done: $dec_recon"
done