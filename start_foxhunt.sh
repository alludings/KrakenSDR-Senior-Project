#!/bin/bash

cd..
cd /home/krakenrf/krakensdr_doa
./kraken_doa_start.sh

sleep 10

/usr/bin/python3 ./pi_bridge.py

