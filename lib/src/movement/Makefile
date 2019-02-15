TARGET		= libhumanoid_ik.a
OBJS		= foot_ik.o
SRCS		= $(OBJS:.o=.c)
INCLUDEDIRS	+= -I../../include
LIBDIRS		+= ../../
CFLAGS		= $(INCLUDEDIRS) -W -Wall -O2

CC			= gcc
AR			= ar

$(TARGET): $(OBJS)
	$(AR) rs $@ $^
	mv $(TARGET) $(LIBDIRS)
	
.c.o:
	$(CC) -c $< $(CFLAGS)
	
clean:
	rm -f $(OBJS) $(TARGET)
	@echo "file deleted."

dep:
	gccmakedep $(SRCS)

