# Compile command
compile_command = gcc
compile_options = -Wall -Werror -O2 -lm
executable_name = FlightGear_comunicator


UDP server:
	@clear
	@echo "FlightGear TCP/UDP comunicator"
	@echo ""
	@make -s Clean
	@make -s Compile
	@echo ""

Compile: ./*.c
	@echo "Compiling sources"
	${compile_command} ${compile_options} ./*.c -o ${executable_name}
	@echo "   Compiling process complete."
	@echo ""

Clean:
	@echo "Removing previus build results"
	@rm -f ${executable_name}
