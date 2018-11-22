CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) $(INCLUDES) Barometer.cpp -L$(NAVIO) -lnavio -o BarometerT

clean:
	rm -f BarometerT
