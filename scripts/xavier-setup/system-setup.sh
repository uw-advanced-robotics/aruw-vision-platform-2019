#!/bin/bash
set -e

# allow nvidia user to access serial
sudo usermod -a -G dialout nvidia

# disable login shell service which uses GPIO serial port
sudo systemctl stop nvgetty.service
sudo systemctl disable nvgetty.service

# add vision service management aliases
echo "
alias vision-start=\"sudo systemctl start aruw_vision.service\"
alias vision-status=\"sudo systemctl status aruw_vision.service\"
alias vision-stop=\"sudo systemctl stop aruw_vision.service\"
alias vision-log=\"sudo journalctl -u aruw_vision.service\"
alias vision-log-follow=\"vision-log --follow\"
" >> ~/.bash_aliases