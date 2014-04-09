# Compile command

CC = gcc
CFLAGS = -Wall -O2 -lm -lpthread -g
CLIBS = `pkg-config --cflags --libs gtk+-2.0 libglade-2.0 gmodule-2.0 gmodule-export-2.0` `xml2-config --cflags --libs` `gtk-config --cflags --libs gthread`


ATTITUDE_ESTIMATOR_SOURCES = ./autopilot/attitude_estimator_SO3/*.c
POSITON_ESTIMATOR_SOURCES = ./autopilot/position_estimator_inav/*.c ./autopilot/position_estimator_mc/*.c ./autopilot/position_estimator_mc/codegen/*.c
ATTITUDE_CONTROLLER_SOURCES = ./autopilot/fw_attitude_control/*.c ./autopilot/fixedwing_attitude_control/*.c ./autopilot/multirotor_attitude_control/*.c
POSITION_CONTROLLER_SOURCES = ./autopilot/fw_position_control/*.c ./autopilot/fixedwing_position_control/*.c ./autopilot/multirotor_position_control/*.c
COMMANDER_SOURCES = ./autopilot/commander/*.c
MIXER_SOURCES = ./autopilot/mixer/*.c
AUTOPILOT_SOURCES = ./autopilot/*.c $(ATTITUDE_ESTIMATOR_SOURCES) $(POSITON_ESTIMATOR_SOURCES) $(ATTITUDE_CONTROLLER_SOURCES) $(POSITION_CONTROLLER_SOURCES) $(COMMANDER_SOURCES) $(MIXER_SOURCES)

MISSION_SOURCES = ./mission/*.c
ORB_SOURCES = ./ORB/*.c
SIMULATOR_SOURCES = ./simulator/*.c

CONSOLE_CONTROLLER_SOURCES = ./uav_library/console_controller/*.c
DISPLAY_SOURCES = ./uav_library/display/*.c ./uav_library/display/GUI/*.c
GEO_SOURCES = ./uav_library/geo/*.c
IO_CTRL_SOURCES = ./uav_library/io_ctrl/*.c
MATH_SOURCES = ./uav_library/math/*.c
PARAM_SOURCES = ./uav_library/param/*.c
PID_SOURCES = ./uav_library/pid/*.c
TIME_SOURCES = ./uav_library/time/*.c
LIBRARY_SOURCES = $(CONSOLE_CONTROLLER_SOURCES) $(DISPLAY_SOURCES) $(GEO_SOURCES) $(IO_CTRL_SOURCES) $(MATH_SOURCES) $(PARAM_SOURCES) $(PID_SOURCES) $(TIME_SOURCES)

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
