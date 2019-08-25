1. Run `./buildPatchedKernel.sh` (This will take ~an hour even with nvpmodel and jetson_clocks)
2. Copy the rebuilt kernel image from the jetson to the flashing computer and overwrite the existing Image file with (from the host PC):

```
scp nvidia@192.168.55.1:buildLibrealsense2Xavier/image/Image ~/nvidia/nvidia_sdk/JetPack_4.2_Linux_P2888/Linux_for_Tegra/kernel/Image
```

JetPack shouldn't overwrite this file, so we only need to do it once on the flashing PC.

3. Put jetson into recovery mode. 

    1. Press and hold down Force Recovery button (middle button)
    2. Press and hold down the Power button (left button)
    3. Release both buttons

You should see the power LED of the xavier light up but no HDMI display

4. From the `Linux_for_tegra` directory, run: 

`sudo ./flash.sh -k kernel jetson-xavier mmcblk0p1`