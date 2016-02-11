#!/bin/bash
running=`ps -aef | grep miniterm.py | wc -l`

if [ $running -gt 1 ]; then
  echo "Miniterm Running $running"
  killall -q miniterm.py
fi;

exit 0

