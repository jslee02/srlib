################################################################################
##	Makefile for srDyn
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

TARGET=srDyn.a
OBJS=srObject.o \
	srGeometryInfo.o \
	srEntity.o \
	srState.o \
	srLink.o \
	srJoint.o \
	srCollision.o \
	srSensor.o \
	srSystem.o \
	srSpace.o \
	srWeldJoint.o \
	srRevoluteJoint.o \
	srPrismaticJoint.o \
	srUniversalJoint.o \
	srBallJoint.o \
	srIRSensor.o \
	srRangeFinder.o \
	srTouchSensor.o \
	srDYN.o \
	srISLAND.o \
	srCollisionPair.o \
	srConstraint.o \
	srJointConstraint.o \
	srContactConstraint.o \
	srClosedLoopConstraint.o \
	srSpring.o \
	srLinearSpring.o
.SUFFIXES: .cpp .o

.cpp.o:
	${CC} -I${INC_DIR} -I. -c $<

all: ${TARGET}

srDyn.a:${OBJS}
	${AR} rcv $@ ${OBJS}
	${RANLIB} $@

deploy:
	${MD}  -p ${TARGET_INC}/srDyn
	${CP} *.a ${TARGET_LIB}
	${CP} *.h ${TARGET_INC}/srDyn

clean:
	${RM} -f ${OBJS}
	${RM} -f ${TARGET}

new:
	${MAKE} clean
	${MAKE} all

#install:
#	${MD}  -p ${INSTALL_INC}/srDyn
#	${MD}  -p ${INSTALL_LIB}
#	${CP} *.h ${INSTALL_INC}/srDyn
#	${CP} *.a ${INSTALL_LIB}
