#!/bin/bash
# After the main render passes: optional true-camera-move versions of two still shots.
# Waits for pass B of the main orchestrator (or 15:10), never starts new shots after END_HHMM.
cd /e/fable3ddemos/3ddemo4 || exit 1
until grep -q "orchestrator finished pass B" logs/final_orch.txt 2>/dev/null || [ "$(date +%H%M)" -ge 1510 ]; do sleep 60; done
# only run the upgrades if the main passes really finished
if grep -q "orchestrator finished pass B" logs/final_orch.txt 2>/dev/null; then
  export END_HHMM=${END_HHMM:-1620}
  bash src/run_final.sh A s20m_final s19m_front34
  bash src/run_final.sh B s20m_final s19m_front34
fi
echo "$(date +%H:%M:%S) upgrades script finished" >> logs/final_orch.txt
