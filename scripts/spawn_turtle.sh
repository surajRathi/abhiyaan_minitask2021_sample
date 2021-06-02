#! /usr/bin/bash

set -e

if [ "$#" == 0 ]; then
  echo "No arguments passed"
  echo "Call as ${0} <args> -- <command to run>"
  exit 1
fi

# rosservice call /turtlesim/kill "$1"
rosservice call /turtlesim/reset
rosservice call /turtlesim/spawn 0 5 0 "$1"
shift

exec "$@"
