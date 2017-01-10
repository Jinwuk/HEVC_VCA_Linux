# the SOURCE definiton lets move your makefile to another position
CONFIG 			= LIBRARY

# set pathes to the correct directories
SRC_DIR			= ../../../../source/Lib/TLibEncoder
INC_DIR			= ../../../../source/Lib
LIB_DIR			= ../../../../lib
BIN_DIR			= ../../../../bin

SRC_DIR1		= ../../../../source/App/TAppEncoder
SRC_DIR2		= ../../../../source/App/utils
SRC_DIR3		= ../../../../source/Lib/libmd5
SRC_DIR4		= ../../../../source/Lib/TAppCommon
SRC_DIR5		= ../../../../source/Lib/TLibCommon
SRC_DIR6		= ../../../../source/Lib/TLibVideoIO


USER_INC_DIRS	= -I$(SRC_DIR)  -I$(SRC_DIR1) -I$(SRC_DIR2) -I$(SRC_DIR3) \
                  -I$(SRC_DIR4) -I$(SRC_DIR5) -I$(SRC_DIR6)
USER_LIB_DIRS	=

# intermediate directory for object files
OBJ_DIR			= ./objects

# the library name
PRJ_NAME	 	= TDllEncoder

# version information
MAJOR_VER		= 0
MINOR_VER		= 1
VER				= $(MAJOR_VER).$(MINOR_VER)

# defines to set
DEFS			= -DMSYS_LINUX

# set objects
OBJS          	= \
			$(OBJ_DIR)/TAppEncCfg.o \
			$(OBJ_DIR)/TAppEncTop.o \
			$(OBJ_DIR)/TDllEncoder.o \
			$(OBJ_DIR)/convert_NtoMbit_YCbCr.o \
			$(OBJ_DIR)/libmd5.o \
			$(OBJ_DIR)/program_options_lite.o \
			$(OBJ_DIR)/ContextModel.o \
			$(OBJ_DIR)/ContextModel3DBuffer.o \
			$(OBJ_DIR)/ETRI_HEVC_service.o \
			$(OBJ_DIR)/SEI.o \
			$(OBJ_DIR)/TComBitStream.o \
			$(OBJ_DIR)/TComCABACTables.o \
			$(OBJ_DIR)/TComDataCU.o \
			$(OBJ_DIR)/TComInterpolationFilter.o \
			$(OBJ_DIR)/TComLoopFilter.o \
			$(OBJ_DIR)/TComMotionInfo.o \
			$(OBJ_DIR)/TComPattern.o \
			$(OBJ_DIR)/TComPic.o \
			$(OBJ_DIR)/TComPicSym.o \
			$(OBJ_DIR)/TComPicYuv.o \
			$(OBJ_DIR)/TComPicYuvMD5.o \
			$(OBJ_DIR)/TComPrediction.o \
			$(OBJ_DIR)/TComRdCost.o \
			$(OBJ_DIR)/TComRdCostWeightPrediction.o \
			$(OBJ_DIR)/TComRom.o \
			$(OBJ_DIR)/TComSampleAdaptiveOffset.o \
			$(OBJ_DIR)/TComSlice.o \
			$(OBJ_DIR)/TComTrQuant.o \
			$(OBJ_DIR)/TComWeightPrediction.o \
			$(OBJ_DIR)/TComYuv.o \
			$(OBJ_DIR)/NALwrite.o \
			$(OBJ_DIR)/SEIwrite.o \
			$(OBJ_DIR)/SyntaxElementWriter.o \
			$(OBJ_DIR)/TEncAnalyze.o \
			$(OBJ_DIR)/TEncBinCoderCABAC.o \
			$(OBJ_DIR)/TEncBinCoderCABACCounter.o \
			$(OBJ_DIR)/TEncCavlc.o \
			$(OBJ_DIR)/TEncCu.o \
			$(OBJ_DIR)/TEncEntropy.o \
			$(OBJ_DIR)/TEncFrame.o \
			$(OBJ_DIR)/TEncGOP.o \
			$(OBJ_DIR)/TEncPic.o \
			$(OBJ_DIR)/TEncPreanalyzer.o \
			$(OBJ_DIR)/TEncProcess.o \
			$(OBJ_DIR)/TEncRateCtrl.o \
			$(OBJ_DIR)/TEncSampleAdaptiveOffset.o \
			$(OBJ_DIR)/TEncSbac.o \
			$(OBJ_DIR)/TEncSearch.o \
			$(OBJ_DIR)/TEncSlice.o \
			$(OBJ_DIR)/TEncTile.o \
			$(OBJ_DIR)/TEncTop.o \
			$(OBJ_DIR)/TEncWPP.o \
			$(OBJ_DIR)/WeightPredAnalysis.o \
			$(OBJ_DIR)/TVideoIOYuv.o \
			$(OBJ_DIR)/TComThreadPool.o \

LIBS				= -lpthread

DEBUG_LIBS			=
RELEASE_LIBS		=

STAT_LIBS			=
DYN_LIBS			= -ldl

# the libraries to link with
STAT_DEBUG_LIBS		=
STAT_RELEASE_LIBS	=
DYN_DEBUG_LIBS		=
DYN_RELEASE_LIBS	=

# name of the base makefile
MAKE_FILE_NAME		= ../../common/makefile.base

# include the base makefile
include $(MAKE_FILE_NAME)
