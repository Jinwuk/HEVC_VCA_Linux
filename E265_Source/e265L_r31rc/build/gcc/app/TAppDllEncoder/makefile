# the SOURCE definiton lets you move your makefile to another position
CONFIG 				= CONSOLE

# set directories to your wanted values
SRC_DIR				= ../../../../source/App/TAppEncoder
INC_DIR				= ../../../../source/Lib
LIB_DIR				= ../../../../lib
BIN_DIR				= ../../../../bin

SRC_DIR1		= ../../../../source/Lib/TLibCommon
SRC_DIR2		= ../../../../source/Lib/Threadpool
SRC_DIR3		=
SRC_DIR4		=

USER_INC_DIRS	= -I$(SRC_DIR) -I$(SRC_DIR1) -I$(SRC_DIR2)
USER_LIB_DIRS	=

# intermediate directory for object files
OBJ_DIR				= ./objects

# set executable name
PRJ_NAME			= TAppDllEncoder

# defines to set
DEFS				= -DMSYS_LINUX -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -DMSYS_UNIX_LARGEFILE

# set objects
OBJS          		= 	\
					$(OBJ_DIR)/TAppDllEncoder.o \

# set libs to link with
LIBS				= -ldl -lpthread -lgomp

DEBUG_LIBS			=
RELEASE_LIBS		=

STAT_LIBS			= -lpthread
DYN_LIBS			=


#DYN_DEBUG_LIBS		= -lTDllEncoderd -lThreadpoold
#DYN_DEBUG_PREREQS	= $(LIB_DIR)/libTDllEncoderd.so $(LIB_DIR)/libThreadpoold.so
#STAT_DEBUG_LIBS	= -lTDllEncoderStaticd -lThreadpoolStaticd
#STAT_DEBUG_PREREQS	= $(LIB_DIR)/libTDllEncoderStaticd.a $(LIB_DIR)/libThreadpoolStaticd.a

DYN_DEBUG_LIBS		= -lTDllEncoderd 
DYN_DEBUG_PREREQS	= $(LIB_DIR)/libTDllEncoderd.so 
STAT_DEBUG_LIBS		= -lTDllEncoderStaticd 
STAT_DEBUG_PREREQS	= $(LIB_DIR)/libTDllEncoderStaticd.a 

#DYN_RELEASE_LIBS	= -lTDllEncoder -lThreadpool
#DYN_RELEASE_PREREQS= $(LIB_DIR)/libTDllEncoder.so $(LIB_DIR)/libThreadpool.so
#STAT_RELEASE_LIBS	= -lTDllEncoderStatic -lThreadpoolStatic
#STAT_RELEASE_PREREQS	= $(LIB_DIR)/libTDllEncoderStatic.a $(LIB_DIR)/libThreadpoolStatic.a

DYN_RELEASE_LIBS	= -lTDllEncoder 
DYN_RELEASE_PREREQS	= $(LIB_DIR)/libTDllEncoder.so 
STAT_RELEASE_LIBS	= -lTDllEncoderStatic 
STAT_RELEASE_PREREQS	= $(LIB_DIR)/libTDllEncoderStatic.a 

# name of the base makefile
MAKE_FILE_NAME		= ../../common/makefile.base

# include the base makefile
include $(MAKE_FILE_NAME)
