# basic
CC		= g++
CFLAGS		= -Wall -g -std=c++11

# common
LIBS		= -lm -lbluetooth -lrt -lpthread -lSDL2 -lSDL2_image -lSDL2_ttf -lSDL2_gfx 
INCS		= -I$(PROJROOT)/inc -I/usr/local/include/SDL2 -I/usr/include/SDL2

# project
PROJROOT	= .

# dirs
INCDIR		= $(PROJROOT)/inc
SRC		= $(PROJROOT)/src
MISC		= $(PROJROOT)/misc
SRCS		= $(wildcard $(SRC)/*.c)

# objects
OBJS		= $(SRCS:.c=.o)
MAIN		= $(PROJROOT)/main.o
BT_CL		= $(MISC)/bt_client.o

# outputs
BINDIR		= $(PROJROOT)/bin
BIN		= $(BINDIR)/project.out
BT_CL_BIN	= $(BINDIR)/bt_cl.out

# cleanup
O2CLEAN		= $(OBJS) $(MAIN) $(BIN) $(BT_CL_BIN) $(BT_CL)

default: project

%.o: %.c $(INCDIR)/*.h
	$(CC) $(CFLAGS) $(MYFLAGS) $(INCS) -c $< -o $@

project: $(MAIN) $(OBJS)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $(MYFLAGS) $^ $(LIBS) -o $(BIN)

bt_client: $(BT_CL)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $(MYFLAGS) $^ $(LIBS) -o $(BT_CL_BIN)

clean:
	-rm -f *.o
	-rm -f $(O2CLEAN)
