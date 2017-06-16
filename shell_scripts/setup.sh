#!/bin/bash

sudo ifconfig enp6s0 up 192.168.10.1
sudo ifconfig enp7s0 up 192.168.11.1

sudo sysctl -w net.core.rmem_max=50000000
sudo sysctl -w net.core.rmem_max=50000000
sudo sysctl -w net.core.wmem_max=1048576