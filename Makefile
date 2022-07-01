CC = clang
CFLAGS = -Wall -Wextra -Wpedantic -pipe -O3 -s -std=c99
LDFLAGS = -lge -lpe -L/usr/local/lib64
all: main.c
	$(CC) $(CFLAGS) $(LDFLAGS) main.c -o main
