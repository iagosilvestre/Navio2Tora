CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) -std=c++11 $(INCLUDES) -L$(NAVIO) teste.cpp -L$(NAVIO) -lnavio -lpthread -o teste

clean:
	rm -f teste
