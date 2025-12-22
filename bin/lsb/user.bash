#!/bin/bash

USER="10.104.0.4"

if [ "$1" != "" ]; then
  USER="$1"
fi

curl http://10.96.0.2:5000/api/user/by-ip/$USER

echo ""

