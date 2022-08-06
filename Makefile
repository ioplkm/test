CC = clang
CFLAGS = -Wall -Wextra -Wpedantic -pipe -O3 -s -std=c99
LDFLAGS = -L/usr/local/lib64
LDLIBS = -lge -lpe
all: main.c
	$(CC) $(CFLAGS) $(LDFLAGS) $(LDLIBS) main.c -o main
clean:
	rm -f main
