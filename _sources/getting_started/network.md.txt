# Network Setup
<!-- TODO -->

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

Switch<-."(optional) 
IP: 10.10.10.4".->laptop
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
After turning on the robot, the Raspberry Pi will search for a WiFi network with the the following credentials:  
**SSID**: DALSA_IOT  
**Password**: dalsa_iot

It is possible to gain access wirelessly by creating such an access point. An easy solution is a smartphone or laptop WiFi hotspot.
In the hotspot's settings the IP adress of the Raspberry Pi will show up if it has sucessfully connected.

## Semi-Permanent Internet Access through DTUSecure
<!-- TODO -->