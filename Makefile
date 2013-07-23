# flags
CXX = clang++
CXXFLAGS = -Wall -Wextra -Werror -pedantic -std=c++11
LDLIBS = -lglut -lGLU -lGL 
#LDLIBS += -lXmu -lX11 -lXi
#LDLIBS += -lboost_thread -lboost_system
ISPC = ispc
ISPCFLAGS = -O2 --target=avx-x2 --arch=x86-64

# code
SRC = main.cpp vector.cpp
TARGET = path

# directories
OBJ_DIR = bin
LIB_DIR = -L/usr/lib
INC_DIR = -I/usr/include
OBJ = ${SRC:%.cpp=$(OBJ_DIR)/%.o}

# default to release; other option is debug
ifeq ($(MODE),)
	MODE = release
endif
ifeq ($(MODE), release)
	CXXFLAGS += -O3
else ifeq ($(MODE), debug)
	CXXFLAGS += -g -O0
else
ERRORMSG = "unknown build mode: $(MODE)"
endif

# rules
.PHONY: clean fmt 

default: init $(OBJ)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJ) $(LDLIBS)

init: 
	@mkdir -p "$(OBJ_DIR)"

$(OBJ_DIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INC_DIR) -c $< -o $@

clean:
	@rm -rf $(OBJ_DIR) $(TARGET)

fmt:
	astyle --recursive --style=allman "*.cpp"
	astyle --recursive --style=allman "*.hpp"
