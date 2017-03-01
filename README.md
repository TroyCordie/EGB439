#Welcome to EGB439 Advanced Robotics
  - Included in this repository is python, matlab and c code used to operate the
    PenguinPi using the Raspbian Jessie operating system. Please note all code
    stored here will restore you robot to default and changes will be lost
  - if you wish to use a diffrent operating system please speak to your tutor

##Defaults
###Raspberry pi login
  username: pi
  Password: raspberry - see instruction to change
  https://www.raspberrypi.org/documentation/linux/usage/users.md

Change you host name https://www.howtogeek.com/167195/how-to-change-your-raspberry-pi-or-other-linux-devices-hostname/

server-camera.py, server-motors_fixed.py and GPIOSoftShutdown.py launch at
startup if you wish to change this please speak to your tutor

##network setup

Open in nano (text editor) - sudo nano /etc/network/interfacesallow-hotplug wlan0 <return>
auto wlan0 <return>
iface wlan0 inet dhcp <return>
wpa-ap-scan 1 <return>
wpa-scan-ssid 1 <return>
wpa-ssid "ENB439" <return>
wpa-proto RSN <return>
wpa-pairwise TKIP <return>
wpa-key-mgmt WPA-PSK <return>
wpa-psk "enb439123" <return>

Open in nano (text editor) - sudo nano /etc/network/interfacesallow-hotplug wlan0 <return>
allow-hotplug wlan0 <return>
auto wlan0 <return>
iface wlan0 inet dhcp <return>
wpa-ap-scan 1 <return>
wpa-scan-ssid 1 <return>
wpa-ssid "networkName" <return>
wpa-proto RSN <return>
wpa-pairwise CCMP <return>
wpa-key-mgmt WPA-PSK <return>
wpa-psk "password" <return>
