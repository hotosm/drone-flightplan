#!/bin/bash

set -eo pipefail

# Check if the number of arguments is 0
if [ $# -eq 0 ]; then
    echo "No arguments provided. Available options:"
    ls /data | grep -v '^__';
    exit 0;
fi

python3 $@