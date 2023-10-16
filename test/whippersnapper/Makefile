# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/systemc-2.3.4/lib-linux64

# SYSTEMC=/usr/local/systemc-2.3.4
# SYSTEMC_ARCH=linux64
# SYSTEMC_ARCH=linux

# Libraries directories.
# LIB_DIRS=$(SYSTEMC)/lib-$(SYSTEMC_ARCH)

# Include directories
INCLUDE_DIRS = -I../common
# INCLUDE_DIRS = -I. -I$(SYSTEMC)/include -I../include

# header files used. for dependency checking
HEADERS = primate.h bfs.h

# source files used. for dependency checking
SOURCES= primate.cpp bfs.cpp main.cpp ../common/common.cpp

DEPENDENCIES = \
	Makefile \
	$(HEADERS) \
	$(SOURCES)

# LIBS= -lsystemc -lstdc++ -lm
TESTS= main

all: $(TESTS)
# 	./$(TESTS)
# 	@make cmp_result

$(TESTS): $(DEPENDENCIES)
# 	@echo LIB_DIRS = $(LIB_DIRS)
	clang++ -O0 -g -fprofile-instr-generate -fcoverage-mapping -o $@ $(SOURCES)
# 	g++ -o $@ $(SOURCES) $(INCLUDE_DIRS) -L$(LIB_DIRS) $(LIBS) -g

.PHONY: clean
clean:
	rm -rf $(TESTS) *.o *.dat

# The cmp_result target comapres the simulation results with the 
# golden results. The testbench writes out the results file (output.dat)
# in the simulation configuration directory. The cmp_result target is
# automatically run at the end of each simulations.

GOLD_DIR = ./golden
GOLD_FILE = $(GOLD_DIR)/ref_output.dat

# diff 
# -w, --ignore-all-space
#               ignore all white space
cmp_result:
	@echo "********************************************************"
	@if diff -w $(GOLD_FILE) ./output.dat ; then \
		echo "SIMULATION PASSED"; \
	else \
		echo "SIMULATION FAILED"; \
	fi
	@echo "********************************************************"
