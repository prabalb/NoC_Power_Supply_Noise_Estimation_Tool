# $Id $

# Copyright (c) 2007-2012, Trustees of The Leland Stanford Junior University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Makefile
#
LEX = flex
YACC   = bison -y
DEFINE = 
INCPATH = -I. -Iarbiters -Iallocators -Irouters -Inetworks -Ipower -Ipsn
IDIR = ./include
CPPFLAGS += -Wall $(INCPATH) $(DEFINE) -I $(IDIR)
CPPFLAGS += -O3
CPPFLAGS += -g
# for compilation with gprof
#CPPFLAGS += -pg
LFLAGS +=


OBJDIR := obj
PROG   := booksim

# simulator source files
CPP_SRCS = main.cpp \
   config_utils.cpp \
   booksim_config.cpp \
   module.cpp \
   buffer.cpp \
   vc.cpp \
   routefunc.cpp \
   traffic.cpp \
   flitchannel.cpp \
   trafficmanager.cpp \
   synthetictrafficmanager.cpp \
   steadystatetrafficmanager.cpp \
   batchtrafficmanager.cpp \
   workloadtrafficmanager.cpp \
   buffer_state.cpp \
   stats.cpp \
   credit.cpp \
   outputset.cpp \
   flit.cpp \
   injection.cpp\
   misc_utils.cpp\
   rng_wrapper.cpp\
   rng_double_wrapper.cpp\
   power_module.cpp \
   switch_monitor.cpp \
   buffer_monitor.cpp \
   psn.cpp \
   workload.cpp



LEX_OBJS  = ${OBJDIR}/lex.yy.o
YACC_OBJS = ${OBJDIR}/y.tab.o
NETRACE_OBJS = ${OBJDIR}/netrace.o

# networks 
NETWORKS:= $(wildcard networks/*.cpp) 
ALLOCATORS:= $(wildcard allocators/*.cpp)
ARBITERS:= $(wildcard arbiters/*.cpp)
ROUTERS:= $(wildcard routers/*.cpp)
POWER:= $(wildcard power/*.cpp)
PSN:= $(wildcard psn/*.cpp)

#--- Make rules ---
OBJS :=  $(LEX_OBJS) $(YACC_OBJS) $(NETRACE_OBJS)\
 $(CPP_SRCS:%.cpp=${OBJDIR}/%.o)\
 $(NETWORKS:networks/%.cpp=${OBJDIR}/%.o)\
 $(ALLOCATORS:allocators/%.cpp=${OBJDIR}/%.o)\
 $(ARBITERS:arbiters/%.cpp=${OBJDIR}/%.o)\
 $(ROUTERS:routers/%.cpp=${OBJDIR}/%.o)\
 $(POWER:power/%.cpp=${OBJDIR}/%.o)\
 $(PSN:psn/%.cpp=${OBJDIR}/%.o)

.PHONY: clean

all:$(PROG)

# for compilation with gprof
#$(PROG): $(OBJS)
#	 $(CXX) $(LFLAGS) -g -pg $^ -o $@

$(PROG): $(OBJS)
	 $(CXX) $(LFLAGS) $^ -o $@

# rules to compile simulator


${LEX_OBJS}: lex.yy.c y.tab.h
	$(CC) $(CPPFLAGS) -c $< -o $@

${YACC_OBJS}: y.tab.c y.tab.h
	$(CC) $(CPPFLAGS) -c $< -o $@

${NETRACE_OBJS}: netrace/netrace.c netrace/netrace.h
	$(CC) $(CPPFLAGS) -c $< -o $@

${OBJDIR}/%.o: %.cpp 
	$(CXX) $(CPPFLAGS) -c $< -o $@

# rules to compile networks
${OBJDIR}/%.o: networks/%.cpp 
	$(CXX) $(CPPFLAGS) -c $< -o $@

# rules to compile arbiters
${OBJDIR}/%.o: arbiters/%.cpp 
	$(CXX) $(CPPFLAGS) -c $< -o $@

# rules to compile allocators
${OBJDIR}/%.o: allocators/%.cpp 
	$(CXX) $(CPPFLAGS) -c $< -o $@

# rules to compile routers
${OBJDIR}/%.o: routers/%.cpp 
	$(CXX) $(CPPFLAGS) -c $< -o $@

# rules to compile power classes
${OBJDIR}/%.o: power/%.cpp
	$(CXX) $(CPPFLAGS) -c $< -o $@

# rules to compile psn
${OBJDIR}/%.o: psn/%.cpp
	$(CXX) $(CPPFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) 
	rm -f $(PROG)
	rm -f *~
	rm -f allocators/*~
	rm -f arbiters/*~
	rm -f networks/*~
	rm -f runfiles/*~
	rm -f routers/*~
	rm -f examples/*~
	rm -f y.tab.c y.tab.h lex.yy.c
	rm -f moc_bgui.cpp

y.tab.c y.tab.h: config.y
	$(YACC) -d $<

lex.yy.c: config.l
	$(LEX) $<