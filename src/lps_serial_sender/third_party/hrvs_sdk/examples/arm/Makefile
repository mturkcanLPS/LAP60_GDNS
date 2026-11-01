#---------------------------------------------------------------------------------------
# executables
EXE := example.out
EXE_NOSTD := example_nostd.out
EXE_DEBUG := example_debug.out
EXE_VELBODY := example_velbody.out
EXE_CONFIG := example_config.out

#---------------------------------------------------------------------------------------
# directories
BIN_DIR := bin
BUILD_DIR := build
HRVS_SDK_PATH := ../../radar_system
HRVS_LIB := $(HRVS_SDK_PATH)/lib
HRVS_INCLUDE := $(HRVS_SDK_PATH)/include
PAL_SRC := $(HRVS_SDK_PATH)/src/pal

# toolchain
TOOL_PREFIX := 
CXX := $(TOOL_PREFIX)g++
LD := $(CXX)
LDLIBS_GPIOD := -lgpiod

# global link flags (Jetson: dynamic linker yol yardımı)
LDFLAGS += -Wl,-rpath,/usr/lib/aarch64-linux-gnu

# flags
CXXFLAGS := -Wall -Wextra -Wno-unused-parameter -Wno-unused-but-set-variable -Wno-unused-function -Wno-reorder -O2

# === OpenCV 4 (JetPack 6) ===
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null)
OPENCV_LIBS   := $(shell pkg-config --libs   opencv4 2>/dev/null)
CXXFLAGS += $(OPENCV_CFLAGS)
LDLIBS_OPENCV := $(OPENCV_LIBS)
# Fallback includes
OPENCV_INC_FALLBACKS := -I/usr/include/opencv4 -I/usr/local/include/opencv4
CXXFLAGS += $(OPENCV_INC_FALLBACKS)

.PHONY: opencv-info
opencv-info:
	@echo "pkg-config --modversion opencv4:"
	@pkg-config --modversion opencv4 || true
	@echo "pkg-config --cflags opencv4:"
	@pkg-config --cflags opencv4 || true
	@echo "pkg-config --libs opencv4:"
	@pkg-config --libs opencv4 || true
	@echo "Headers at:"
	@ls -ld /usr/include/opencv4/opencv2 2>/dev/null || true
	@ls -ld /usr/local/include/opencv4/opencv2 2>/dev/null || true

# === NVIDIA VPI 3 (JetPack 6, Orin Nano) ===
VPI_CFLAGS := $(shell pkg-config --cflags vpi-3 2>/dev/null)
VPI_LIBS   := $(shell pkg-config --libs   vpi-3 2>/dev/null)
CXXFLAGS  += $(VPI_CFLAGS)
LDLIBS_VPI := $(VPI_LIBS)

# Header include fallbacks/autodetect
VPI_INC_CANDIDATES := /opt/nvidia/vpi3/include /usr/include/vpi-3 /usr/local/include/vpi-3 /opt/nvidia/vpi/include
VPI_INC_FOUND := $(firstword $(foreach d,$(VPI_INC_CANDIDATES),$(if $(wildcard $(d)/vpi/Image.h),$(d))))
ifneq ($(strip $(VPI_INC_FOUND)),)
CXXFLAGS += -I$(VPI_INC_FOUND)
endif

# Library autodetect: prefer libnvvpi.* then libvpi.*
VPI_LIB_CANDIDATES := /opt/nvidia/vpi3/lib/aarch64-linux-gnu /opt/nvidia/vpi3/targets/aarch64-l4t/lib /opt/nvidia/vpi3/targets/aarch64-l4t/lib64 /usr/lib/aarch64-linux-gnu/tegra /usr/lib/aarch64-linux-gnu /usr/local/lib /lib/aarch64-linux-gnu
VPI_LIB_FILE := $(firstword $(foreach n, nvvpi vpi, $(foreach d,$(VPI_LIB_CANDIDATES),$(wildcard $(d)/lib$(n).so))))
ifeq ($(strip $(VPI_LIB_FILE)),)
VPI_LIB_FILE := $(firstword $(foreach n, nvvpi vpi, $(foreach d,$(VPI_LIB_CANDIDATES),$(firstword $(wildcard $(d)/lib$(n).so.*) $(wildcard $(d)/lib$(n).a)))))
endif
ifneq ($(strip $(VPI_LIB_FILE)),)
VPI_LIBDIR := $(dir $(VPI_LIB_FILE))
LDLIBS_VPI := -Wl,-rpath,$(VPI_LIBDIR) $(VPI_LIB_FILE)
else
VPI_LIBDIRS := $(foreach d,$(VPI_LIB_CANDIDATES),-L$(d))
LDLIBS_VPI := $(VPI_LIBDIRS) -lnvvpi -lvpi
endif

.PHONY: vpi-info
vpi-info:
	@echo "pkg-config --modversion vpi-3:"
	@pkg-config --modversion vpi-3 || true
	@echo "pkg-config --cflags vpi-3:"
	@pkg-config --cflags vpi-3 || true
	@echo "pkg-config --libs vpi-3:"
	@pkg-config --libs vpi-3 || true
	@echo "VPI header candidates:" $(VPI_INC_CANDIDATES)
	@echo "VPI header found in:" $(VPI_INC_FOUND)
	@echo "VPI lib candidates:" $(VPI_LIB_CANDIDATES)
	@echo "VPI lib file chosen:" $(VPI_LIB_FILE)
	@echo "LDLIBS_VPI:" $(LDLIBS_VPI)

# === Intel RealSense (librealsense2) ===
RS2_CFLAGS := $(shell pkg-config --cflags realsense2 2>/dev/null)
RS2_LIBS   := $(shell pkg-config --libs   realsense2 2>/dev/null)
CXXFLAGS  += $(RS2_CFLAGS)
LDLIBS_RS2 := $(RS2_LIBS)

# aarch64 guard: pkg-config x86_64 yol dökerse yok say
UNAME_M := $(shell uname -m)
ifeq ($(UNAME_M),aarch64)
  ifneq (,$(findstring x86_64,$(LDLIBS_RS2)))
    LDLIBS_RS2 :=
  endif
endif

# Fallback headers
RS2_INC_CANDIDATES := /usr/include/librealsense2 /usr/local/include/librealsense2
RS2_INC_FOUND := $(firstword $(foreach d,$(RS2_INC_CANDIDATES),$(if $(wildcard $(d)/rs.hpp),$(d))))
ifneq ($(strip $(RS2_INC_FOUND)),)
CXXFLAGS += -I$(RS2_INC_FOUND)
endif

# Fallback libs: autodetect a concrete file if pkg-config yoksa veya temizlediysek
RS2_LIB_CANDIDATES := /usr/lib/aarch64-linux-gnu /usr/local/lib /lib/aarch64-linux-gnu
RS2_LIB_FILE := $(firstword $(foreach d,$(RS2_LIB_CANDIDATES),$(wildcard $(d)/librealsense2.so) $(wildcard $(d)/librealsense2.so.*) $(wildcard $(d)/librealsense2.a)))
ifneq ($(strip $(RS2_LIB_FILE)),)
RS2_LIBDIR := $(dir $(RS2_LIB_FILE))
LDLIBS_RS2 := -Wl,-rpath,$(RS2_LIBDIR) $(RS2_LIB_FILE)
else
ifeq ($(strip $(LDLIBS_RS2)),)
RS2_LIBDIRS := $(foreach d,$(RS2_LIB_CANDIDATES),-L$(d))
LDLIBS_RS2 := $(RS2_LIBDIRS) -lrealsense2
endif
endif

.PHONY: realsense-info
realsense-info:
	@echo "pkg-config --modversion realsense2:"
	@pkg-config --modversion realsense2 || true
	@echo "pkg-config --cflags realsense2:"
	@pkg-config --cflags realsense2 || true
	@echo "pkg-config --libs realsense2:"
	@pkg-config --libs realsense2 || true
	@echo "RS2 header candidates:" $(RS2_INC_CANDIDATES)
	@echo "RS2 header found in:" $(RS2_INC_FOUND)
	@echo "RS2 lib candidates:" $(RS2_LIB_CANDIDATES)
	@echo "RS2 lib file chosen:" $(RS2_LIB_FILE)
	@echo "LDLIBS_RS2:" $(LDLIBS_RS2)

# nostd app definitions
NOSTD_DEFINITIONS += -D NO_INFO -D BYTE_BUFFER_AVOID_TEMPLATE_CHECKS
NOSTD_DEFINITIONS += -D MSG_LOGGING

DEFINITIONS += -D MSG_LOGGING
#DEFINITIONS += -D INFO

LDFLAGS += -Wl,--exclude-libs,$(HRVS_SDK_LIB_STD) -Wl,--exclude-libs,$(HRVS_SDK_LIB_NOSTD)

# arch dependent config for sdk lib
ifeq ($(UNAME_M),aarch64)
	HRVS_SDK_LIB_STD := hrvs_sdk_arm64.a
	HRVS_SDK_LIB_NOSTD := hrvs_sdk_arm64_nostd.a
else
	HRVS_SDK_LIB_STD := hrvs_sdk_arm.a
	HRVS_SDK_LIB_NOSTD := hrvs_sdk_arm_nostd.a
endif

#---------------------------------------------------------------------------------------
all: build_dirs $(BIN_DIR)/$(EXE) $(BIN_DIR)/$(EXE_NOSTD) $(BIN_DIR)/$(EXE_DEBUG) $(BIN_DIR)/$(EXE_VELBODY) $(BIN_DIR)/$(EXE_CONFIG)

# tek yerde yönetmek için
LDLIBS_ALL := $(LDLIBS_OPENCV) $(LDLIBS_VPI) $(LDLIBS_GPIOD) $(LDLIBS_RS2)

$(BIN_DIR)/$(EXE): $(BUILD_DIR)/example.o $(BUILD_DIR)/gpio_pin_linux.o $(BUILD_DIR)/uart_linux.o $(BUILD_DIR)/thread_stdlib.o $(BUILD_DIR)/time_stdlib.o
	@echo Linking $(EXE)
	$(LD) $(LDFLAGS) $(CXXFLAGS) -std=c++11 -pthread -o $@  $^ -L $(HRVS_LIB) -l:$(HRVS_SDK_LIB_STD) $(LDLIBS_ALL)

$(BIN_DIR)/$(EXE_NOSTD): $(BUILD_DIR)/example_nostd.o $(BUILD_DIR)/gpio_pin_linux.o $(BUILD_DIR)/uart_linux.o $(BUILD_DIR)/thread_linux.o $(BUILD_DIR)/time_linux.o
	@echo Linking $(EXE_NOSTD)
	$(LD) $(LDFLAGS) $(CXXFLAGS) -o $@  $^ -L $(HRVS_LIB) -l:$(HRVS_SDK_LIB_NOSTD) $(LDLIBS_ALL)

$(BIN_DIR)/$(EXE_DEBUG): $(BUILD_DIR)/example_debug.o $(BUILD_DIR)/gpio_pin_linux.o $(BUILD_DIR)/uart_linux.o $(BUILD_DIR)/thread_stdlib.o $(BUILD_DIR)/time_stdlib.o
	@echo Linking $(EXE_DEBUG)
	$(LD) $(LDFLAGS) $(CXXFLAGS) -std=c++11 -pthread -o $@  $^ -L $(HRVS_LIB) -l:$(HRVS_SDK_LIB_STD) $(LDLIBS_ALL)

$(BIN_DIR)/$(EXE_VELBODY): $(BUILD_DIR)/example_velbody.o $(BUILD_DIR)/gpio_pin_linux.o $(BUILD_DIR)/uart_linux.o $(BUILD_DIR)/thread_stdlib.o $(BUILD_DIR)/time_stdlib.o
	@echo Linking $(EXE_VELBODY)
	$(LD) $(LDFLAGS) $(CXXFLAGS) -std=c++11 -pthread -o $@  $^ -L $(HRVS_LIB) -l:$(HRVS_SDK_LIB_STD) $(LDLIBS_ALL)

$(BIN_DIR)/$(EXE_CONFIG): $(BUILD_DIR)/example_config.o $(BUILD_DIR)/gpio_pin_linux.o $(BUILD_DIR)/uart_linux.o $(BUILD_DIR)/thread_stdlib.o $(BUILD_DIR)/time_stdlib.o
	@echo Linking $(EXE_CONFIG)
	$(LD) $(LDFLAGS) $(CXXFLAGS) -std=c++11 -pthread -o $@  $^ -L $(HRVS_LIB) -l:$(HRVS_SDK_LIB_STD) $(LDLIBS_ALL)

# object rules
$(BUILD_DIR)/example.o: src/example.cpp
	$(CXX) $(CXXFLAGS) $(DEFINITIONS) -std=c++11 -I $(HRVS_INCLUDE) -c $< -o $@

$(BUILD_DIR)/example_nostd.o: src/example_nostd.cpp
	$(CXX) $(CXXFLAGS) $(NOSTD_DEFINITIONS) -I $(HRVS_INCLUDE) -c $< -o $@

$(BUILD_DIR)/example_debug.o: src/example_debug.cpp
	$(CXX) $(CXXFLAGS) $(DEFINITIONS) -std=c++11 -I $(HRVS_INCLUDE) -c $< -o $@

$(BUILD_DIR)/gpio_pin_linux.o: $(PAL_SRC)/gpio_pin_linux.cpp
	$(CXX) $(CXXFLAGS) -I $(HRVS_INCLUDE) -c $< -o $@

$(BUILD_DIR)/uart_linux.o: $(PAL_SRC)/uart_linux.cpp
	$(CXX) $(CXXFLAGS) -I $(HRVS_INCLUDE) -c $< -o $@

$(BUILD_DIR)/thread_linux.o: $(PAL_SRC)/thread_linux.cpp
	$(CXX) $(CXXFLAGS) -I $(HRVS_INCLUDE) -c $< -o $@

$(BUILD_DIR)/time_linux.o: $(PAL_SRC)/time_linux.cpp
	$(CXX) $(CXXFLAGS) -I $(HRVS_INCLUDE) -c $< -o $@

$(BUILD_DIR)/thread_stdlib.o: $(PAL_SRC)/thread_stdlib.cpp
	$(CXX) $(CXXFLAGS) -std=c++11 -I $(HRVS_INCLUDE) -c $< -o $@

$(BUILD_DIR)/time_stdlib.o: $(PAL_SRC)/time_stdlib.cpp
	$(CXX) $(CXXFLAGS) -std=c++11 -I $(HRVS_INCLUDE) -c $< -o $@

$(BUILD_DIR)/example_velbody.o: src/example_velbody.cpp
	$(CXX) $(CXXFLAGS) $(DEFINITIONS) -std=c++11 -I $(HRVS_INCLUDE) -c $< -o $@

$(BUILD_DIR)/example_config.o: src/example_config.cpp
	$(CXX) $(CXXFLAGS) $(DEFINITIONS) -std=c++11 -I $(HRVS_INCLUDE) -c $< -o $@

build_dirs:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BIN_DIR)

clean:
	rm -rf $(BUILD_DIR)  
	rm -rf $(BIN_DIR)

