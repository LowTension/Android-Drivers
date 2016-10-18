#
# CONFIDENTIAL
# Copyright (C) 2014 GlobalMems Corporation
#

C_INCLUDES=-I../../include -I../../lib
LDFLAGS=
.SUFFXES: .o .c

PACKAGE=gmad
SRCS= \
	main.c

HEADS= 
OBJS=$(SRCS:.c=.o)

all: $(PACKAGE)

$(PACKAGE): $(OBJS)
	$(LD) $(LDFLAGS) $(OBJS) -o $@

$(OBJS): $(HEADS) Makefile

clean:
	rm -f $(OBJS) $(PACKAGE)

.c.o:
	$(CC) $(C_INCLUDES) $(CFLAGS) -c $<
