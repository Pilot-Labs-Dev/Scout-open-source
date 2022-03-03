#!/bin/sh
gmtzone=$(date -R|awk '{print $6}')
echo "GMT$gmtzone"
