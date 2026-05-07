#!/bin/bash
set -euo pipefail
IFS=$'\n\t'
docker build --no-cache --tag viper_dev_image .
