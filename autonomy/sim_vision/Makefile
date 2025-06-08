OPTS=-O
CFLAGS=-Wall -I../include -std=c++11 $(OPTS)
PROGS=sim_vision sim_markers

all: $(PROGS)

sim_vision: sim.cpp
	g++ $(CFLAGS) $< -o $@

sim_markers: sim_markers.cpp
	g++ $(CFLAGS) $< -o $@

run: sim_vision
	./sim_vision

clean:
	- rm $(PROGS)
