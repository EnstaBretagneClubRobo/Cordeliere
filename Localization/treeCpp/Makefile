SRC_PATH = ./src
BIN_PATH = ./bin
OBJ_PATH = ./obj

CXXFLAGS = $(shell pkg-config --cflags ibex)
LIBS = $(shell pkg-config --libs ibex)
CFLAGS = -Wall -g

all: bin

bin: ${OBJ_PATH}/node.o ${OBJ_PATH}/tree.o ${OBJ_PATH}/contractor.o ${OBJ_PATH}/main.o
	mkdir -p bin

	g++ -std=c++11 ${CFLAGS} -o ${BIN_PATH}/exe $^ ${CXXFLAGS} ${LIBS}


${OBJ_PATH}/%.o: ${SRC_PATH}/%.cpp
	mkdir -p obj

	g++ -std=c++11 ${CFLAGS} -c $< -o $@ ${CXXFLAGS} ${LIBS}


clean:
	rm -rf ${OBJ_PATH}/*.o ${BIN_PATH}/exe
