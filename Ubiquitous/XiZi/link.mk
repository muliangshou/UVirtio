OBJS := $(shell cat make.obj)

$(TARGET): $(OBJS) 
	@echo ------------------------------------------------
	@echo link $(TARGET)
	@$(CROSS_COMPILE)g++ -o $@ $($(LINK_FLAGS)) $(OBJS)  -lc -lm
	@echo ------------------------------------------------
	@$(CROSS_COMPILE)objcopy -O binary $@ XiZi_$(BOARD)$(COMPILE_TYPE).bin
	@$(CROSS_COMPILE)size $@

