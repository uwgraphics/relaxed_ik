CC=g++
src=Arm
ext_name=Arm_ext

########################################################################################################################

all:boost clean

boost:obj.o
	$(CC) -o $(ext_name).so -shared obj.o -std=c++11 -lboost_python -lboost_numpy27 -lpython2.7 -L/usr/local/lib/ -I /home/rakita -I /usr/include/python2.7/ -I /home/rakita/boost_1_67_0/ -O2 -DNDEBUG

obj.o:$(src).cpp
	$(CC) -lboost_python -lboost_system -std=c++11 -lpython2.7 -lboost_numpy27 -L/usr/local/lib/ -I /home/rakita -I /usr/include/python2.7/ -I /home/rakita/boost_1_67_0/ -O2 -DNDEBUG -fpic -c -o obj.o $(src).cpp

clean:
	rm -rf *.o
