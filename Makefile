CXX = g++
CXXFLAGS = -I ~/coding/RNEA/include/
LDFLAGS = -ljsoncpp

SRC = src/main.cpp src/id.cpp src/link.cpp src/robot.cpp

OBJ = $(SRC:.cpp=.o)
EXECUTABLE = test1

.PHONY: all clean

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) $(LDFLAGS) -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(EXECUTABLE)
