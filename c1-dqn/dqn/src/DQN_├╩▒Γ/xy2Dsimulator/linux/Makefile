all: calculator.so

clean:
	rm -f *.o *.so

calculator.so: calculator.o
	gcc -shared -o calculator.so calculator.o -lm

calculator.o: calculator.c
	gcc -c -Wall -fpic calculator.c -lm
