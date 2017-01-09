CC = g++
CFLAGS = -O3
EXEC = GridPathPlanner

all:
	rm -f $(EXEC)
	$(CC) $(CFLAGS) -o $(EXEC) main.cpp PartiallyKnownGrid.cpp GridPathPlanner.cpp
#	./$(EXEC)
	
