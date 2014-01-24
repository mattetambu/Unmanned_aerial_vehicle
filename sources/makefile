# Compile command
CC = gcc
CFLAGS = -Wall -O2 -lm -lpthread -g
CLIBS = `pkg-config --cflags --libs gtk+-2.0 libglade-2.0 gmodule-2.0 gmodule-export-2.0` `xml2-config --cflags --libs` `gtk-config --cflags --libs gthread`
executable_name = UAV_autopilot


all:
	@echo " "
	@echo "UAV Autopilot"
	@make -s compile
	@echo " "

compile: ./*.c
	@echo "Compiling sources"
	@echo " "
	$(CC) $(CFLAGS) ./*.c -o $(executable_name) $(CLIBS)
	@echo "   Compiling process complete."
	@echo " "

clean:
	@echo "Removing previus build results"
	@rm -f $(executable_name)
