DIRS = Echoes SpyKee Vision LittleEndian Brian/brian Brian/fuzzy Brian/checker IsAac RoboTower_Game

NO_COLOR=\033[0m
OK_COLOR=\033[0;31m
ECHO	= echo

.PHONY: all clean

all:
	-for d in $(DIRS); do (cd $$d; $(ECHO) "$(OK_COLOR)[INFO]" Building subdirectory `pwd`"$(NO_COLOR)"; $(MAKE)); done

clean:
	-for d in $(DIRS); do (cd $$d; $(ECHO) "$(OK_COLOR)"[INFO] Cleaning up in `pwd`"$(NO_COLOR)"; $(MAKE) clean); done
