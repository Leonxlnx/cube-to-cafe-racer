#!/bin/bash
# Final render orchestrator.  usage: bash src/run_final.sh <pass: all|A|B> <shot> [<shot> ...]
# One Blender process per shot (so the thread count can adapt when other renders on this PC finish).
# Stops starting new shots after the optional wall-clock deadline END_HHMM (e.g. END_HHMM=1500).
BL="/c/Program Files/Blender Foundation/Blender 5.2/blender.exe"
cd /e/fable3ddemos/3ddemo4 || exit 1
PASS="$1"; shift
mkdir -p logs
for shot in "$@"; do
  if [ -n "$END_HHMM" ] && [ "$(date +%H%M)" -ge "$END_HHMM" ]; then
    echo "$(date +%H:%M:%S) deadline reached, not starting $shot" >> logs/final_orch.txt
    break
  fi
  n=$(tasklist //FI "IMAGENAME eq blender.exe" //FO CSV //NH 2>/dev/null | grep -c -i "blender.exe")
  if [ "$n" -ge 1 ]; then TH=${TH_BUSY:-8}; else TH=${TH_FREE:-13}; fi
  echo "$(date +%H:%M:%S) start $shot pass $PASS threads $TH (other blender: $n)" >> logs/final_orch.txt
  "$BL" -b --factory-startup --python src/render_film.py -- --mode final --pass "$PASS" --shots "$shot" --threads "$TH" $EXTRA >> "logs/final_${PASS}.txt" 2>&1
  echo "$(date +%H:%M:%S) done  $shot pass $PASS (exit $?)" >> logs/final_orch.txt
done
echo "$(date +%H:%M:%S) orchestrator finished pass $PASS" >> logs/final_orch.txt
