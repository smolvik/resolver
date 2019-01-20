OPT = -Wall -O0

all:   resolver.c cos_tb.c
		gcc $(OPT) -shared -o resolver.so  -fPIC resolver.c cos_tb.c


clean:
		rm *.so *.o


