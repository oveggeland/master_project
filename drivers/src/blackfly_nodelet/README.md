# blackfly_nodelet

Simple ROS driver wrapping the spinnaker API for Blackfly Cameras.

## Timestamping
Images are timestamped using the End of Exposure event given by the Spinnaker API. When this event occurs, the current ROS time is saved in the device event handler class. The device event handler then queries the camera for its current exposure time. The exposure time is divided by 2, and this time is subtracted from the saved time stamp. This procedure is performed in order to move the image's timestamp to the middle of the camera's exposure. 

## Disclosure
This driver is untested and not field proven. Use at your own risk.

## Notes:
1. Camera Frame rate may drop if the camera is not connected to a USB3.0 port. 
2. If using auto exposure, auto gain, and gamma correction, the camera tends to choose the maximum exposure value.
3. Double Check:

In /etc/default/grub, change 
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"  
```
to 
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=32768"  
```
then 
```
sudo update-grub  
```
then restart  and verify that 
```
cat /sys/module/usbcore/parameters/usbfs_memory_mb  
```
is >= 1024

4. It is recommended to use a 330 Ohm or lower value resistor to connect the OptoIn to the trigger signal.

## Dynamic Reconfigure

Set enable_dyn_reconf to "True" to change online the following parameters:

![Dynamic Reconfigure Parameters](https://github.com/unr-arl/blackfly_nodelet/blob/master/imgs/dyn_rec.png)


