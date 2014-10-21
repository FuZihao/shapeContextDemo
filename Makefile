CC = g++
CFLAGS = -Wall `pkg-config --cflags opencv`
LDFLAGS = `pkg-config --libs opencv`

OBJS = shapeContext.o \
	   sc_test.o

sc_test.out : $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o : %.c %.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean :
	rm -f *.o *.a *.out
