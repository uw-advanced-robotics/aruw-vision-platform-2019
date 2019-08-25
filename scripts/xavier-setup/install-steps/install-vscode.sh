#! /bin/bash

TEMP_DEB="$(mktemp)" &&
wget -O "$TEMP_DEB" 'https://packagecloud.io/headmelted/codebuilds/packages/debian/stretch/code-oss_1.32.0-1550644676_arm64.deb/download.deb' &&
sudo dpkg -i "$TEMP_DEB"
rm -f "$TEMP_DEB"
