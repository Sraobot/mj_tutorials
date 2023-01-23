
#LINUX
COMMON=-O2 -I../../include -L../../lib -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../../lib
LIBS = -lmujoco -lglfw -lm #-lm for math
CPP = g++


ROOT = main

all:
	$(CPP) $(COMMON) main.cpp $(LIBS) -o $(ROOT)

clean:
	rm *.o $(ROOT)
