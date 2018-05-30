CC=g++
src=Arm
ext_name=Arm_ext

########################################################################################################################

all:boost clean

boost:obj.o
	$(CC) -o $(ext_name).so -shared obj.o -lboost_python -lpython2.7 

obj.o:$(src).cpp
	$(CC) -lboost_python -lboost_system -lpython2.7 -I /home/rakita -I /usr/include/python2.7/  -fpic -c -o obj.o $(src).cpp

clean:
	rm -rf *.o
