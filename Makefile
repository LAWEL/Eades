TARGET = eades
SRCS := eades.cpp
OBJS := $(SRCS:.cpp=.o)

CXXFLAGS = -O2 -std=c++11 -Wall -Wextra -Wshadow

.PHONY: all
all: $(TARGET)

.PHONY: clean
clean:
	rm -f *.o

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^
