# Builds the Red Robot sketch in team/ as a Webots controller.
# Webots compiles this automatically (Build > Build, or it rebuilds when you
# reload the world). Replace the files in team/ with your own RR26_Arduino code;
# team/src/RedRobot.cpp and team/src/ble.h are NOT compiled (the simulator
# provides its own RedRobot implementation in sim/).

CXX_SOURCES = sim/sim_redrobot.cpp sim/team_sketch.cpp $(wildcard team/src/RemoteControl.cpp) \
              $(filter-out team/src/RedRobot.cpp,$(wildcard team/src/*.cpp))
CXX_SOURCES := $(sort $(CXX_SOURCES))
INCLUDE = -Isim -Iteam
CFLAGS = -O2 -std=c++17 -Wno-unused-function -Wno-unused-variable

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH ?= $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
