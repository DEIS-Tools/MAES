#!/bin/bash

export ALLGOOD=yes
function report_missing_executable () { echo "$1 not installed ($2)"; export ALLGOOD=no;}

# Test for needed executables
command -v bmon   &> /dev/null || report_missing_executable "bmon" "sudo apt-get install bmon"
command -v sed    &> /dev/null || report_missing_executable "sed" "sudo apt-get install sed"
command -v awk    &> /dev/null || report_missing_executable "awk" "sudo apt-get install gawk"
command -v free   &> /dev/null || report_missing_executable "free" "sudo apt-get install procps"
command -v docker &> /dev/null || report_missing_executable "docker" "wget -O - https://get.docker.com | sh"
command -v xhost  &> /dev/null || report_missing_executable "xhost" "sudo apt-get install x11-xserver-utils"

# Only continue of above test for executables succeeded.
if [[ $ALLGOOD = no ]]; then
    echo -e "\n\tScript aborted..."
    exit 1
fi

# Logs network activity on a specific interface, in this case docker0 (all traffic going to/from docker containers). 
function log_network () {
    echo "Epoch BytesReceived/sec BytesTransmitted/sec" >> $NETWORK_FILE_NAME
    bmon -p docker0 -r 1 -o format:fmt='$(attr:rxrate:bytes) $(attr:txrate:bytes)\n' \
        | while read line; do echo "`date +%s` $line" >> $NETWORK_FILE_NAME; done;
}

# Logs overall CPU utilization
function log_cpu () {
    echo "Epoch CpuUtilizationInPercent" >> $CPU_FILE_NAME
    top -b -d1 \
        | grep --line-buffered "Cpu(s)" \
        | sed -u "s/.*, *\([0-9]*,[0-9]\)%* id.*/\1/ ; s/\,/\./" \
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
            awk -v file_name=$MEMORY_FILE_NAME -v line=$line -v total=$TOTAL_MEMORY 'BEGIN { printf ("%d %d %.2f\n", systime(), line, 100 - ((line / total) * 100)) >> file_name}';
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