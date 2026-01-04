#!/bin/bash

X=$1
Y=$2

TYPES="-s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000"

CMD="name: \"smarrtino\", position: {x: ${X}, y: ${Y}, z: 0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}"

gz service $TYPES --req "$CMD"

