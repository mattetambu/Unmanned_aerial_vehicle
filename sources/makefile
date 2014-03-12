# Compile command

CC = gcc
CFLAGS = -Wall -O2 -lm -lpthread -g
CLIBS = `pkg-config --cflags --libs gtk+-2.0 libglade-2.0 gmodule-2.0 gmodule-export-2.0` `xml2-config --cflags --libs` `gtk-config --cflags --libs gthread`


ATTITUDE_ESTIMATOR_SOURCES = ./autopilot/attitude_estimator/*.c
ATTITUDE_CONTROLLER_SOURCES = ./autopilot/fw_attitude_control/*.c
POSITION_CONTROLLER_SOURCES = ./autopilot/fw_position_control/*.c
COMMANDER_SOURCES = ./autopilot/commander/*.c
AUTOPILOT_SOURCES = ./autopilot/*.c $(ATTITUDE_ESTIMATOR_SOURCES) $(ATTITUDE_CONTROLLER_SOURCES) $(POSITION_CONTROLLER_SOURCES) $(COMMANDER_SOURCES)

MISSION_SOURCES = ./mission/*.c
ORB_SOURCES = ./ORB/*.c
SIMULATOR_SOURCES = ./simulator/*.c

CONSOLE_CONTROLLER_SOURCES = ./uav_library/console_controller/*.c
IO_CTRL_SOURCES = ./uav_library/io_ctrl/*.c
GEO_SOURCES = ./uav_library/geo/*.c
DISPLAY_SOURCES = ./uav_library/display/*.c ./uav_library/display/GUI/*.c
TIME_SOURCES = ./uav_library/time/*.c
MATHLIB_SOURCES = ./uav_library/math/*.c
PARAM_SOURCES = ./uav_library/param/*.c
LIBRARY_SOURCES = $(CONSOLE_CONTROLLER_SOURCES) $(IO_CTRL_SOURCES) $(GEO_SOURCES) $(DISPLAY_SOURCES) $(TIME_SOURCES) $(MATHLIB_SOURCES) $(PARAM_SOURCES)

SOURCES = ./*.c $(AUTOPILOT_SOURCES) $(MISSION_SOURCES) $(ORB_SOURCES) $(SIMULATOR_SOURCES) $(LIBRARY_SOURCES)
executable_name = UAV_autopilot


all:
	@echo " "
	@echo "UAV Autopilot"
	@make -s compile
	@echo " "

compile: $(SOURCES)
	@echo "Compiling sources"
	@echo " "
	$(CC) $(CFLAGS) $(SOURCES) -o $(executable_name) $(CLIBS)
	@echo "   Compiling process complete."
	@echo " "

clean:
	@echo "Removing previus build results"
	@rm -f $(executable_name)
