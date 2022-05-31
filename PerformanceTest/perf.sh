#!/bin/bash

# Copyright 2022 MAES
# 
# This file is part of MAES
# 
# MAES is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your option)
# any later version.
# 
# MAES is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with MAES. If not, see http://www.gnu.org/licenses/.
# 
# Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
# 
# Original repository: https://github.com/MalteZA/MAES


export ALLGOOD=yes
function report_missing_executable () { echo "$1 not installed ($2)"; export ALLGOOD=no;}

# Test for needed executables
command -v bmon   &> /dev/null || report_missing_executable "bmon" "sudo apt-get install bmon"
command -v sed    &> /dev/null || report_missing_executable "sed" "sudo apt-get install sed"
command -v gawk   &> /dev/null || report_missing_executable "awk" "sudo apt-get install gawk"
command -v free   &> /dev/null || report_missing_executable "free" "sudo apt-get install procps"

# The following two commands are only strictly necessary when testing performance for MAES in ROSMode using docker as the ROS environment.
# You can comment these out if you are measuring performance of something else...
command -v docker &> /dev/null || report_missing_executable "docker" "wget -O - https://get.docker.com | sh"
command -v xhost  &> /dev/null || report_missing_executable "xhost" "sudo apt-get install x11-xserver-utils"

# Only continue of above test for executables succeeded.
if [[ $ALLGOOD = no ]]; then
    echo -e "\n\tScript aborted..."
    exit 1
fi

# Logs network activity on a specific interface, in this case docker0 (all traffic going to/from docker containers). 
function log_network () {
    echo "Epoch KilobytesReceived/sec KilobytesTransmitted/sec" >> $NETWORK_FILE_NAME
    bmon -p docker0 -r 1 -o format:fmt='$(attr:rxrate:bytes) $(attr:txrate:bytes)\n' \
        | while read line; do 
            awk -v file_name=$NETWORK_FILE_NAME '{FS=" "} {printf("%d %.2f %.2f\n", systime(), $1/1000, $2/1000) >> file_name; fflush(stdout)}' ; 
        done;
}

# Logs overall CPU utilization
function log_cpu () {
    echo "Epoch CpuUtilizationInPercent" >> $CPU_FILE_NAME
    top -b -d1 \
        | grep --line-buffered "Cpu(s)" \
        | sed -u "s/.*, *\([0-9]*.[0-9]\)%* id.*/\1/ ; s/\,/\./" \
        | while read line; do 
              awk -v file_name=$CPU_FILE_NAME -v line=$line 'BEGIN { printf ("%d %.1f\n", systime(), 100.0 - line) >> file_name}'; 
          done;
}

function log_memory () {
    export TOTAL_MEMORY=$(free --mega | grep Mem: | sed 's/Mem:\ *\([0-9.]*\).*/\1/')
    echo "Epoch MemoryUsageInMegabytes MemoryUsageInPercent" >> $MEMORY_FILE_NAME
    free --mega -s 1 \
        | grep --line-buffered "Mem:" \
        | grep --line-buffered -Eo "[0-9]+$" \
        | while read line; do
            awk -v file_name=$MEMORY_FILE_NAME -v line=$line -v total=$TOTAL_MEMORY 'BEGIN { printf ("%d %d %.2f\n", systime(), total - line, 100 - ((line / total) * 100)) >> file_name}';
          done;
}

# Get current datetime
export CURRENT_TIME=$(date +%F_%T)
export NETWORK_FILE_NAME=${CURRENT_TIME}_network.csv
export CPU_FILE_NAME=${CURRENT_TIME}_cpu.csv
export MEMORY_FILE_NAME=${CURRENT_TIME}_memory.csv
export BOOKMARK_FILE_NAME=${CURRENT_TIME}_bookmarks.csv

# User info:
echo -e "\n\tLogging ready"
echo -e "Currently logging network activity on docker0 interface to $NETWORK_FILE_NAME"
echo -e "Currently logging CPU utilization to $CPU_FILE_NAME"
echo -e "Currently logging Memory usage to $MEMORY_FILE_NAME"
echo -e "Epoch-bookmarks will be logged to $BOOKMARK_FILE_NAME\n\n"


# Make sure to kill child-processes whe user presses Ctrl+C
trap "exit" INT TERM ERR
trap "kill 0" EXIT

# Logging starts here:
log_network &
log_cpu &
log_memory &

# Add headers to bookmark-file
echo "Epoch BookmarkEntry" >> $BOOKMARK_FILE_NAME
echo "`date +%s` logging_start" >> $BOOKMARK_FILE_NAME

sleep 1

# Add bookmarks
echo "Add bookmarks by typing the desired event-name and press enter."
echo "(Press enter with no event-name to exit the loop)"
echo -n "Next event name: " 

while read line && [[ ! -z $line ]]
do 
    export line=$(tr -d '[:punct:]' <<< $line | sed "s/\ /\_/g" | tr '[A-Z]' '[a-z]')
    echo "`date +%s` $line" >> $BOOKMARK_FILE_NAME
    echo -n "Logged ${line}-event. Next event name: "
done

echo -e "\n\tPress Ctrl+C to stop logging...\n"
wait