#!/bin/bash

TODAY=$(date +\%Y\%m\%d)
LOG_PATH="/home/mattiascarpa/greenhouse/log/greenhouse_log-$TODAY.log"

/usr/bin/python -u /home/mattiascarpa/greenhouse/main_greenhouse.py >> $LOG_PATH 2>&1
