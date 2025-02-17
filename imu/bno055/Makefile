TAR = bno055

CROSS_COMPILE=aarch64-linux-gnu-
CC = ${CROSS_COMPILE}g++
CFLAGS = -Wall 
LDFLAGS = -lrt 

all: clean build 

build: 
	$(CC) $(TAR).c -o $(TAR).app -D_HELLO $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(TAR).app
exec: 
	./$(TAR).app

install:
	cp $(TAR).app /nfsshare
