#!/bin/bash

$HOME/pi-packetizer/setup-serial.sh > cmod.log &
sleep 1
$HOME/pi-packetizer/cmod-server $HOME/dev/ttyS1 hw:0,1 >> cmod.log
