CC = clang
CFLAGS = -Wall -Wextra -Wpedantic -pipe -O3 -s -std=c99 -D_POSIX_C_SOURCE=200112L
LDFLAGS = -L/usr/local/lib64
LDLIBS = -lge -lpe -lwayland-client -lpthread
all: main.c
	$(CC) $(CFLAGS) $(LDFLAGS) $(LDLIBS) main.c -o main
clean:
	rm -f main
