# (c) 2013 h.zeller@acm.org
# This is free software. License: GNU 3.0

# Optmization flags. Typically, that should be -O3 for regluar use, but for
# debugging and development "-g -O0" can be more helpful. Overridable by
# environment variable.
BEAGLEG_OPT_CFLAGS?=-O3

CFLAGS+=-Wall -D_XOPEN_SOURCE=500 $(ARM_COMPILE_FLAGS) $(BEAGLEG_OPT_CFLAGS) -I../

# See toplevel Makefile comment
CXXFLAGS+=-std=c++20 $(CFLAGS)
CXX?=g++

LDFLAGS+=-lpthread -lm -lgsl

COMMON_LIBS=../common/libbeaglegbase.a ../gcode-parser/libgcodeparser.a

OBJECTS=../machine-control-config.o ../config-parser.o quad.o experimental-planner.o
MAIN_OBJECTS=sim.o
TARGETS=sim

DEPENDENCY_RULES=$(OBJECTS:=.d) $(MAIN_OBJECTS:=.d)

all : sim

../common/libbeaglegbase.a:
	$(MAKE) -C ../common

../common/libgcodeparser.a:
	$(MAKE) -C ../gcode-parser

../%.o: ../%.c
	$(MAKE) -C $@

sim: sim.o $(OBJECTS) $(COMMON_LIBS)
	$(CROSS_COMPILE)$(CXX) -o $@ $^ $(COMMON_LIBS) $(PRUSS_LIBS) $(LDFLAGS)

%.o: %.cc compiler-flags
	$(CROSS_COMPILE)$(CXX) $(CXXFLAGS)  -c  $< -o $@
	@$(CROSS_COMPILE)$(CXX) $(CXXFLAGS) -MM $< > $@.d

# Auto generated dependencies
-include $(DEPENDENCY_RULES)

clean:
	rm -rf  $(TARGETS) $(MAIN_OBJECTS) $(OBJECTS) $(DEPENDENCY_RULES) *.gcda *.gcov *.gcno *.cc.html *.h.html

compiler-flags: FORCE
	@echo '$(CXX) $(CXXFLAGS)' | cmp -s - $@ || echo '$(CXX) $(CXXFLAGS)' > $@

.PHONY: FORCE
