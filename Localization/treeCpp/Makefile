SRC_PATH = ./src
BIN_PATH = ./bin
OBJ_PATH = ./obj


all: node.o tree.o main.o
	g++ -std=c++11 -Wall ${OBJ_PATH}/*.o -o ${BIN_PATH}/exe

%.o: ${SRC_PATH}/%.cpp
	g++ -std=c++11 -Wall -c $< -o ${OBJ_PATH}/$@

clean:
	rm -rf ${OBJ_PATH}/*.o ${BIN_PATH}/exe
