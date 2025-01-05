TAR = test

CC = g++
CFLAGS = -Wall 
LDFLAGS = -lrt -lGL -lGLU -lglut 

all: clean build 

build: 
	$(CC) $(TAR).c -o $(TAR).app -D_HELLO $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(TAR).app
exec: 
	./$(TAR).app
