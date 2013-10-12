# Compile command
CC = gcc
CFLAGS = -Wall -O2 -lm -lpthread
CLIBS = `pkg-config --cflags --libs gtk+-2.0 libglade-2.0 gmodule-2.0 gmodule-export-2.0` 
executable_name = FlightGear_comunicator


all:
	@clear
	@echo "FlightGear TCP/UDP comunicator"
	@echo ""
	@make -s clean
	@make -s compile
	@echo ""

compile: ./*.c
	@echo "Compiling sources"
	$(CC) $(CFLAGS) -g ./*.c -o $(executable_name) $(CLIBS)
	@echo "   Compiling process complete."
	@echo ""

clean:
	@echo "Removing previus build results"
	@rm -f $(executable_name)
