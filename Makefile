LIBNAME = detour
LUADIR = ../../build/include

BUILD_CLUALIB_DIR=../../build/clualib

COPT = -O2
# COPT = -DLPEG_DEBUG -g

CWARNS = -Wall -Wextra -pedantic \
	-Waggregate-return \
	-Wcast-align \
	-Wcast-qual \
	-Wdisabled-optimization \
	-Wpointer-arith \
	-Wshadow \
	-Wsign-compare \
	-Wundef \
	-Wwrite-strings \
	-Wbad-function-cast \
	-Wdeclaration-after-statement \
	-Wmissing-prototypes \
	-Wnested-externs \
	-Wstrict-prototypes \
# -Wunreachable-code \

includeParam = 	-I$(LUADIR) \
				-IDebugUtils/Include \
				-IDetour/Include \
				-IDetourCrowd/Include \
				-IDetourTileCache/Include \
				-IRecast/Include \
				-IRecastDemo/Include \
				-ILuaBinding/Include \

CPPFLAGS = $(CWARNS) $(COPT) $(includeParam) -fPIC -DLUA_BINDING

navDemoFiles = ChunkyTriMesh.cpp InputGeom.cpp MeshLoaderObj.cpp PerfTimer.cpp SampleInterfaces.cpp
luaBindingFiles = Nav.cpp lnav.cpp
navFiles =  $(wildcard DebugUtils/Source/*.cpp)
navFiles += $(wildcard Detour/Source/*.cpp)
navFiles += $(wildcard DetourCrowd/Source/*.cpp)
navFiles += $(wildcard DetourTileCache/Source/*.cpp)
navFiles += $(wildcard Recast/Source/*.cpp)
navFiles += $(addprefix RecastDemo/Source/,$(navDemoFiles))
navFiles += $(addprefix LuaBinding/Source/,$(luaBindingFiles))
onavFiles = $(navFiles:%.cpp=%.o)
dnavFiles = $(navFiles:%.cpp=%.d)

sinclude $(dnavFiles:.cpp=.d)

# For Linux
linux:
	make detour.so "DLLFLAGS = -shared -fPIC"

# For Mac OS
macosx:
	make detour.so "DLLFLAGS = -bundle -undefined dynamic_lookup"

detour.so: $(onavFiles)
	env $(CXX) $(DLLFLAGS) $(onavFiles) -o $(BUILD_CLUALIB_DIR)/detour.so

clean:
	rm -f $(onavFiles) $(BUILD_CLUALIB_DIR)/detour.so

