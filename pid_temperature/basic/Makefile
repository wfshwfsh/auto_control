TAR = test

CC = gcc
CFLAGS = -Wall 
LDFLAGS = -lrt -lxlsxwriter -lz

all: clean build 

build: 
	$(CC) $(TAR).c -o $(TAR).app $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(TAR).app
exec: 
	./$(TAR).app
