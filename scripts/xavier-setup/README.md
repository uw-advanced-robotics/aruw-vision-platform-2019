# Xavier Setup Scripts

**Don't run any of these scripts as root!**

Don't forget to run `sudo nvpmodel -m 0` and `sudo jetson-clocks` before this for faster installations.

## Before running:
1. Systemd auto-update service will be running if there is internet connection, it will hold the dpkg lock, preventing you from running any of the below. Check with `ps aux | grep -i apt` that its not running before starting
2. Run "install update" once and restart. It'll prompt you to reset your keyboard layout in the middle

## Order:
1. `install-ros.sh`
2. `source ~/.bashrc` (VERY IMPORTANT, DON'T FORGET)
3. `make-catkin-ws.sh`
4. `install-librealsense.sh` (This one takes a while)
5. `install-darknet.sh`
6. `install-misc.sh` (Cython takes a while as well)
7. `install-vscode.sh`
8. `system-setup.sh`

## Optional:
Unpin the useless LibreOffice shortcuts from start menu and pin vscode, chromium and terminal once we're done.

## Post-installation:
You have to restart the Jetson for the `system-setup.sh` change to tty to take effect and be able to run our stuff.