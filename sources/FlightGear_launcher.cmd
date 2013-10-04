:: Launch the FlightGear simulator (check proprieties on 127.0.0.1:5500)

@echo off
title Launch the FlightGear simulator whit all the options needed

"%programfiles%\FlightGear\bin\Win64\fgfs" --fg-root="%programfiles%\FlightGear\data" --fg-scenery="%programfiles%\FlightGear\data\Scenery" --language=it --control=keyboard --units-meters --enable-fuel-freeze --aircraft=c172p --airport=KHAF --enable-hud --timeofday=noon --in-air --altitude=2000 --vc=120 --enable-auto-coordination --httpd=5500 --generic=socket,out,40,localhost,6100,udp,output_protocol --generic=socket,in,45,localhost,6000,udp,input_protocol --generic=socket,out,40,localhost,5100,tcp,output_protocol --generic=socket,in,45,localhost,5000,tcp,input_protocol
:: --enable-wireframe
