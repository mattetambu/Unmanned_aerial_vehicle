:: Launch the FlightGear simulator (check proprieties on 127.0.0.1:5500)

@echo off
title Launch the FlightGear simulator whit all the options needed

"%programfiles%\FlightGear\bin\Win64\fgfs" --fg-root="%programfiles%\FlightGear\data" --fg-scenery="%programfiles%\FlightGear\data\Scenery" --language=it --control=keyboard --units-meters --enable-fuel-freeze --aircraft=c172p --airport=KHAF --enable-hud --timeofday=noon --in-air --altitude=2000 --vc=120 --enable-auto-coordination --httpd=5500 --generic=socket,out,40,127.0.0.1,8080,udp,in_out_protocol-deg --generic=socket,in,45,127.0.0.1,8081,udp,in_out_protocol-deg
:: --enable-wireframe
