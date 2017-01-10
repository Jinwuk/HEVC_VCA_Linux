### enforce 32-bit build : 1=yes, 0=no
M32?= 0
 
export M32

### select output (EXE or DLL)
OUTPUT 	= EXE

export OUTPUT

ifeq ($(OUTPUT), DLL)

all:
	$(MAKE)	-C lib/Threadpool   MM32=$(M32)
	$(MAKE)	-C lib/TDllEncoder		MM32=$(M32)
	$(MAKE)	-C app/TAppDllEncoder	MM32=$(M32)


debug:
	$(MAKE)	-C lib/Threadpool	debug MM32=$(M32)
	$(MAKE)	-C lib/TDllEncoder		debug MM32=$(M32)
	$(MAKE)	-C app/TAppDllEncoder	debug MM32=$(M32)

	
release:
	$(MAKE)	-C lib/Threadpool	release MM32=$(M32)
	$(MAKE)	-C lib/TDllEncoder		release MM32=$(M32)
	$(MAKE)	-C app/TAppDllEncoder	release MM32=$(M32)


else
ifeq ($(OUTPUT), EXE)
all:
	$(MAKE)	-C lib/TLibVideoIO	MM32=$(M32)
	$(MAKE)	-C lib/TLibCommon	MM32=$(M32)
	$(MAKE)	-C lib/TLibEncoder	MM32=$(M32)
	$(MAKE)	-C lib/TAppCommon   MM32=$(M32)
	$(MAKE)	-C lib/Threadpool   MM32=$(M32)
	$(MAKE)	-C app/TAppEncoder  MM32=$(M32)

debug:
	$(MAKE)	-C lib/TLibVideoIO	debug MM32=$(M32)
	$(MAKE)	-C lib/TLibCommon	debug MM32=$(M32)
	$(MAKE)	-C lib/TLibEncoder	debug MM32=$(M32)
	$(MAKE)	-C lib/TAppCommon   debug MM32=$(M32)
	$(MAKE)	-C lib/Threadpool   debug MM32=$(M32)
	$(MAKE)	-C app/TAppEncoder  debug MM32=$(M32)

release:
	$(MAKE)	-C lib/TLibVideoIO	release MM32=$(M32)
	$(MAKE)	-C lib/TLibCommon	release MM32=$(M32)
	$(MAKE)	-C lib/TLibEncoder	release MM32=$(M32)
	$(MAKE)	-C lib/TAppCommon   release MM32=$(M32)
	$(MAKE)	-C lib/Threadpool   release MM32=$(M32)
	$(MAKE)	-C app/TAppEncoder  release MM32=$(M32)

endif
endif

clean:
	$(MAKE)	-C lib/TLibVideoIO		clean MM32=$(M32)
	$(MAKE)	-C lib/TLibCommon		clean MM32=$(M32)
	$(MAKE)	-C lib/TLibEncoder		clean MM32=$(M32)
	$(MAKE)	-C lib/TAppCommon   	clean MM32=$(M32)
	$(MAKE)	-C lib/TDllEncoder		clean MM32=$(M32)
	$(MAKE)	-C lib/Threadpool   		clean MM32=$(M32)
	$(MAKE)	-C app/TAppEncoder  	clean MM32=$(M32)
	$(MAKE)	-C app/TAppDllEncoder  	clean MM32=$(M32)
