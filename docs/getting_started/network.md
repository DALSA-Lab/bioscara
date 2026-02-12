# Network Setup
This document describes the network architecture and possibilities to establish inter-device connections.

## Architecture
If the network configuration described in the [installation instructions](installation.md) are followed, then the robot controller and control PC are set-up for the network architecture as follows:

```{mermaid}
graph TD;
subgraph Wifi["Wifi"];

laptop["Laptop
hostname: -"]

DTUsecure["DTUsecure

hostname: -"]:::wifi

laptop<-."IP: 10.209.X.X".->DTUsecure

end
subgraph LAN["LAN"];

robot["Raspberry Pi 
hostname: scara"];

dev["Control PC
hostname: scara-dev"];

Switch<--IP: 10.10.10.3-->dev
Switch<--IP: 10.10.10.2-->robot

end

Switch<--"(optional) 
IP: 10.10.10.4"-->laptop
dev<-."(optional)
IP: 10.209.X.X".->DTUsecure
robot<-."(optional)
IP: 10.209.X.X".->DTUsecure
dev<-."(optional)
IP: 192.168.137.X".->laptop
robot<-."(optional)
IP: 192.168.137.X".->laptop

classDef wifi stroke:#f00
classDef bar stroke:#0f0
classDef foobar stroke:#00f
```

The Laptop is optional, but useful during development. In case a Windows hotspot is used the IP addresses are 192.168.137.X (Subnetmask 255.255.255.0).

## Permanent Ethernet LAN
If only the control PC and the robot controller are in the network, the switch can be omitted and the devices connected directly to each other via an Ethernet cable. The devices should see each other on their respective IP adress.

:::{note}
Although this should work in theory, it has not been tested yet. A third developer PC has always been connected and a switch employed.
:::

If a third device needs to join the network, a network switch is needed to route the trafic.


## Temporary WiFi Connection
Both the robot controller and control PC should be configured to automatically connect to a WiFi network with the the following credentials:  
**SSID**: DALSA_IOT  
**Password**: dalsa_iot

It is possible to gain access wirelessly by creating such an access point. An easy solution is a smartphone or laptop WiFi hotspot.
In the hotspot's settings the IP adress of the connected device will show up if it has sucessfully connected.

## Semi-Permanent Internet Access through DTUsecure
There are two options to access the internet. 

### Through the temporary WiFi AP
Create the access point as described in the previous section, and the device will have internet access forwarded via the hotspot. This method is simple, yet the internet connection is slow and unstable. Not ideal to pull a large repository from Github for example.

### Via DTUsecure
Connect the device to the DTUsecure WiFi network.

The easiest way is to simply execute the connection script *installation/scripts/connect_DTUsecure.sh*:

If the robot should be connected to DTUsecure, for example for a faster connection, run the provided script *installation/scripts/connect_DTUsecure.sh*. DTUsecure uses the WPA-EAP authethication mechanism with your DTU id and DTU password. The script has been made to simplify the setup.

:::{caution}
Due the nature of the OS, your DTU accounts password will be saved in plain text in a netplan file on the system if the connection shall be saved persistently betweeen reboots! Follow the scripts output to see where.
:::

then the script can be run simply by invoking:
```bash
sudo bash installation/scripts/connect_DTUsecure.sh
```
The script adds a connection via nmcli, and asks the user for DTU id and password and if the credentials should be saved.

## Further Information
Managing networks on Linux can be confusing, for example if interfaces show up as 'unmanaged' the chances are high that a different network manager has control over the interface. [This](https://wiki.archlinux.org/title/Network_configuration#Network_managers) table gives a good overview which programs might be involved.
Note that Netplan is not mentioned there, since Netplan only takes a standardized configuration file as input and then depending on the renderer creates the configuration for the backend. Currently Netplan only works with systemd-networkd and NetworkManager, the latter is much simpler to use with its CLI `nmcli`, and terminal UI `nmtui`. 