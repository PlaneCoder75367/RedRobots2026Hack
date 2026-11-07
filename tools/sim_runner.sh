#!/bin/bash
# Test loop used by Claude: whenever .simruns/request appears, rebuild the robot
# controller and run one headless match up to the end of auto, logging to
# .simruns/run_<id>.log. Stop it with Ctrl-C.
cd "$(dirname "$0")/.."
export WEBOTS_HOME=/Applications/Webots.app WEBOTS_HOME_PATH=/Applications/Webots.app/Contents
export SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk
WB=/Applications/Webots.app/Contents/MacOS/webots
mkdir -p .simruns
echo "sim runner ready (Ctrl-C to stop)"
while true; do
  if [ -f .simruns/request ]; then
    id=$(cat .simruns/request); rm -f .simruns/request
    echo "run $id"
    if ! ( cd controllers/red_robot && make clean >/dev/null 2>&1 && make ) > .simruns/build_$id.log 2>&1; then
      echo BUILD_FAILED >> .simruns/build_$id.log; touch .simruns/done_$id; continue
    fi
    log=.simruns/run_$id.log
    RR_TRACE=1 "$WB" --mode=fast --no-rendering --batch --minimize --stdout --stderr worlds/red_robot_2026.wbt > "$log" 2>&1 &
    pid=$!
    for i in $(seq 1 150); do sleep 1; grep -q "end of auto" "$log" && break; done
    sleep 2
    kill $pid 2>/dev/null; sleep 1; pkill -f "MacOS/webots.*--batch" 2>/dev/null
    touch .simruns/done_$id
    echo "run $id finished"
  fi
  sleep 1
done
