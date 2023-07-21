FF = gfortran -O3 -Wall -fPIC

CC = g++ -O3 -Wall -fPIC -std=c++11

LIBS = -lgfortran

all: lmbm_call.o lmbm_main.o lmbm_sub.o lmbm_math.o lmbm.o lmbm.so

lmbm_call.o: lmbm_call.f
	$(FF) -c lmbm_call.f

lmbm_main.o: lmbm_main.f
	$(FF) -c lmbm_main.f

lmbm_sub.o: lmbm_sub.f
	$(FF) -c lmbm_sub.f

lmbm_math.o: lmbm_math.f
	$(FF) -c lmbm_math.f

lmbm.o: lmbm.cpp
	$(CC) -c lmbm.cpp

lmbm.so: lmbm_call.o lmbm_main.o lmbm_sub.o lmbm_math.o lmbm.o
	$(CC) -o lmbm.so lmbm_call.o lmbm_main.o lmbm_sub.o lmbm_math.o lmbm.o $(LIBS) -shared

clean:
	rm -f lmbm_call.o lmbm_main.o lmbm_sub.o lmbm_math.o lmbm.o 
