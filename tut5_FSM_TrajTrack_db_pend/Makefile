
#LINUX
COMMON=-O2 -I../../include -L../../lib -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../../lib
LIBS = -lmujoco -lglfw -lm #-lm for math
CPP = g++


ROOT = main

all:
	$(CC) $(COMMON) main.cpp $(LIBS) -o $(ROOT)

main.o:
	$(CC) $(COMMON) -c main.cpp

clean:
	rm *.o $(ROOT)
