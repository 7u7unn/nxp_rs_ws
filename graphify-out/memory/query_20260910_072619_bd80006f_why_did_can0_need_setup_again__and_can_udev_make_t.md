---
type: "query"
date: "2026-09-10T07:26:19.794057+00:00"
question: "Why did can0 need setup again, and can udev make the SocketCAN setup persistent?"
contributor: "graphify"
outcome: "useful"
source_nodes: ["BusConfig", "RobstrideBus", "Configure the physical robot", "rs_control/README.md"]
---

# Q: Why did can0 need setup again, and can udev make the SocketCAN setup persistent?

## Answer

Expanded from original query via graph vocabulary: [can, interface, bitrate, device, bus, configure, socket, launch]. The command ip link set can0 up type can bitrate 1000000 changes runtime netdev state only. Reboot, USB adapter re-enumeration, driver reload, or another network manager can recreate can0 as DOWN/STOPPED. This host has a candleLight USB-to-CAN adapter using gs_usb; the existing system link file selects the interface name but does not configure bitrate or operational state, and no custom udev rule or systemd network configuration was found. A udev rule should trigger a oneshot systemd service on can0 add; the service should run ip link set dev can0 up type can bitrate 1000000. This gives automatic setup on boot and USB replug without putting the ip command directly in udev.

## Outcome

- Signal: useful

## Source Nodes

- BusConfig
- RobstrideBus
- Configure the physical robot
- rs_control/README.md