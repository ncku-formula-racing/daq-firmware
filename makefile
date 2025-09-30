BUILD_DIR = build
TARGET := stm32f103c8
RTT_ADDR := 0x20000000
BUILD_TYPE := Debug
PROJ_NAME := daq-firmware

all:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. -GNinja -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
	cd $(BUILD_DIR) && ninja
	@-mv $(BUILD_DIR)/compile_commands.json .

clean:
	rm -rf $(BUILD_DIR)

flash:
	@-pyocd flash -t $(TARGET) $(BUILD_DIR)/$(PROJ_NAME).elf

monitor:
	@-pyocd rtt -t $(TARGET) -a $(RTT_ADDR) 

debug:
	@-pyocd gdbserver -t $(TARGET) --persist

format:
	@-find . -name build -type d \! -prune -o -name Drivers -type d \! -prune -o -name RTT -type d \! -prune -o -iname '*.h' -o -iname '*.c' | xargs clang-format -i

.PHONY: all clean flash monitor