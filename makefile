CC		= gcc
CFLAGS		= -Wall -pthread -g -std=gnu99
LIBS		= -lEGL -lGLESv2 -Wall -lm -lX11 -lbluetooth -lrt -lpthread

#project
PROJBASE	= .

# dirs
COMMON		= $(PROJBASE)/common
SRCS		= $(wildcard $(COMMON)/*.c)
INCDIRS		= $(PROJBASE)/inc
MISC		= $(PROJBASE)/misc

# objects
OBJS		= $(SRCS:.c=.o)
MAIN		= $(PROJBASE)/main.o
BT_CL		= $(MISC)/bt_client.o

#outputs
BINDIR		= $(PROJBASE)/bin
BIN		= $(BINDIR)/project.out
BT_CL_BIN	= $(BINDIR)/bt_cl.out

#Cleanup
O2CLEAN		= $(OBJS) $(MAIN) $(BIN) $(BT_CL_BIN) $(BT_CL)

default: project

%.o: %.c
	$(CC) $(CFLAGS) -I$(INCDIRS) -c $< -o $@

project: $(MAIN) $(OBJS)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $^ $(LIBS) -o $(BIN)

bt_client: $(BT_CL)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $^ $(LIBS) -o $(BT_CL_BIN)

clean:
	-rm -f *.o
	-rm -f $(O2CLEAN)
