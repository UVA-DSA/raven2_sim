all: Li_DYN Writer

Li_DYN: two_arm_dyn.cpp
	g++ -I ./tnt -I ./jama125 main.cpp two_arm_dyn.cpp -o two_arm_dyn
Writer: writer.cpp
	g++ writer.cpp -o writer
