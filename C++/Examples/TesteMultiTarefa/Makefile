CXX ?= g++
NAVIO = ../../Navio
INCLUDES = -I ../../Navio

all:
	$(MAKE) -C ../../Navio all
	$(CXX) -std=c++11 $(INCLUDES) -L$(NAVIO) testeMulti.cpp -L$(NAVIO) -lnavio -lpthread -o testeMulti

clean:
	rm -f testeMulti
