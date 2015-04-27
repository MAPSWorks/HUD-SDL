CC	= gcc
CFLAGS	= -Wall -pthread
LIBS	= -lEGL -lGLESv2 -Wall -lm -lX11 -lbluetooth -lrt

#project
PROJBASE	= .
COMMON	= $(PROJBASE)/common
SRCS	= $(wildcard $(COMMON)/*.c)
OBJS	= $(SRCS:.c=.o)
INCDIRS	= $(PROJBASE)/inc
MAIN	= $(PROJBASE)/main.o
BINDIR	= $(PROJBASE)/bin
BIN	= $(BINDIR)/project.out

#Cleanup
O2CLEAN	= $(OBJS) $(MAIN) $(BIN)

default: project

%.o: %.c
	$(CC) $(CFLAGS) -I$(INCDIRS) -c $< -o $@

project: $(MAIN) $(OBJS)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $^ $(LIBS) -o $(BIN)

clean:
	-rm -f *.o
	-rm -f $(O2CLEAN)
	-rm -rf $(BINDIR)
