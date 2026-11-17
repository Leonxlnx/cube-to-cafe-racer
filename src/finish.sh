#!/bin/bash
# Safety net: assembles the film automatically when render passes finish, even if nobody is watching.
# usage: bash src/finish.sh   (run in the background)
cd /e/fable3ddemos/3ddemo4 || exit 1
mkdir -p out logs
assemble() {   # $1 = output name
  AUD=""
  if [ -f audio/mix.wav ]; then AUD="--audio audio/mix.wav"; fi
  python3 src/post.py --src frames --out "out/$1.mp4" --sheet "out/$1_sheet.png" $AUD >> logs/finish.txt 2>&1
  echo "$(date +%H:%M:%S) assembled out/$1.mp4" >> logs/finish.txt
}
# pass A (or wall clock 11:30)
until grep -q "orchestrator finished pass A" logs/final_orch.txt 2>/dev/null || [ "$(date +%H%M)" -ge 1445 ]; do sleep 60; done
assemble cube_bike_passA
# pass B (or wall clock 16:15)
until grep -q "orchestrator finished pass B" logs/final_orch.txt 2>/dev/null || [ "$(date +%H%M)" -ge 1700 ]; do sleep 60; done
assemble cube_bike_auto
echo "$(date +%H:%M:%S) finish.sh done" >> logs/finish.txt
