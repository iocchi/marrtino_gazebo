#!/bin/bash


if [ "$1" == "" ] || [ "$1" == "true" ]; then
  gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'
  echo "Simulation paused."
else
  gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
  echo "Simulation running."
fi

sleep 1

gz topic --echo --topic /stats -n 1

