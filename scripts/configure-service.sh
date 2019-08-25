#!/bin/bash
set -e

ENABLE_SERVICE=false
for i in "$@"
do
    case $i in
        -e|--enable)
            ENABLE_SERVICE=true
            shift
        ;;
        *)
            # unknown option    
        ;;
    esac
done

if [ "$ENABLE_SERVICE" = true ]; then
    if [ -f /etc/systemd/system/aruw_vision.service ]; then
        sudo systemctl stop aruw_vision.service
    fi
    sudo cp "../aruw_vision.service" "/etc/systemd/system/"
    sudo systemctl enable aruw_vision.service
    echo "Enabled systemd job"
else
    if [ -f /etc/systemd/system/aruw_vision.service ]; then
        sudo systemctl disable aruw_vision.service
        echo "Disabled systemd job"
    else
        echo "No changes need to be made"
    fi
fi
