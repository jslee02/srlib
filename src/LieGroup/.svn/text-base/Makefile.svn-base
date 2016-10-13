################################################################################
##	Makefile for LieGroup
##
##	License: LGPL
##	Jaeyoung Haan. srlib@robotics.snu.ac.kr
##	2008
##
##	
##	Usage:
################################################################################
PROC=proc
CC=g++
CP=cp
AR=ar
RANLIB=ranlib
MAKE=make
ECHO=@echo
MD=mkdir
RM=rm

TARGET_INC=../../include
TARGET_LIB=../../lib
#INSTALL_INC=/usr/local/include/srLib
#INSTALL_LIB=/usr/local/lib/srLib
INC_DIR=../

TARGET=LieGroup.a
OBJS=LieGroup.o

.SUFFIXES: .cpp .o

.cpp.o:
	${CC} -I${INC_DIR} -I. -c $<

all: ${TARGET}

LieGroup.a:${OBJS}
	${AR} rcv $@ ${OBJS}
	${RANLIB} $@

deploy:
	${MD}  -p ${TARGET_INC}/LieGroup
	${CP} *.a ${TARGET_LIB}
	${CP} *.h ${TARGET_INC}/LieGroup
	${CP} *.inl ${TARGET_INC}/LieGroup

clean:
	${RM} -f ${OBJS}
	${RM} -f ${TARGET}

new:
	${MAKE} clean
	${MAKE} all

#install:
#	${MD}  -p ${INSTALL_INC}/LieGroup
#	${MD}  -p ${INSTALL_LIB}
#	${CP} *.h ${INSTALL_INC}/LieGroup
#	${CP} *.inl ${TARGET_INC}/LieGroup
#	${CP} *.a ${INSTALL_LIB}
