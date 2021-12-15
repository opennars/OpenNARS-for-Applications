#! /bin/bash

###############################################################################
# 1.add Additional startup programs
# start_transbot
# bash /home/jetson/Transbot/transbot/start_transbot.sh
# start transbot main program
###############################################################################


###############################################################################
# 2.add to .bashrc last line
# eval "$RUN_TRANSBOT_PROGRAMS"
###############################################################################

gnome-terminal -- bash -c "cd /home/jetson/OpenNARS-for-Applications/misc/Transbot;sleep 3;export RUN_TRANSBOT_PROGRAMS='bash /home/jetson/OpenNARS-for-Applications/misc/Transbot/basenodes.sh';exec bash"

wait
exit 0
