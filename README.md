#Welcome to EGB439 Advanced Robotics
  - Included in this repository is python, matlab and c code used to operate the
    PenguinPi using the Raspbian Jessie operating system. Please note all code
    stored here will restore you robot to default and changes will be lost
  - if you wish to use a diffrent operating system please speak to your tutor

##Defaults
###Raspberry pi login
  username: pi
  Password: raspberry - see instruction to change <return>
  https://www.raspberrypi.org/documentation/linux/usage/users.md

Change your host name https://www.howtogeek.com/167195/how-to-change-your-raspberry-pi-or-other-linux-devices-hostname/

server-camera.py, server-motors_fixed.py and GPIOSoftShutdown.py launch at
startup if you wish to change this please speak to your tutor

##network setup

Open in nano (text editor) - sudo nano /etc/network/interfacesallow-hotplug wlan0
auto wlan0
iface wlan0 inet dhcp
wpa-ap-scan 1
wpa-scan-ssid 1
wpa-ssid "ENB439"
wpa-proto RSN
wpa-pairwise TKIP
wpa-key-mgmt WPA-PSK
wpa-psk "enb439123"

Open in nano (text editor) - sudo nano /etc/network/interfacesallow-hotplug wlan0
allow-hotplug wlan0
auto wlan0
iface wlan0 inet dhcp
wpa-ap-scan 1
wpa-scan-ssid 1
wpa-ssid "networkName"
wpa-proto RSN
wpa-pairwise CCMP
wpa-key-mgmt WPA-PSK
wpa-psk "password" 
