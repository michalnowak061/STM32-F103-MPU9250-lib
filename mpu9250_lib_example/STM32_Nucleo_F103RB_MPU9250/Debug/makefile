################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32_Nucleo_F103RB_MPU9250.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32_Nucleo_F103RB_MPU9250.list \


# All Target
all: STM32_Nucleo_F103RB_MPU9250.elf secondary-outputs

# Tool invocations
STM32_Nucleo_F103RB_MPU9250.elf: $(OBJS) $(USER_OBJS) /Users/macbookmichal/Desktop/stm32_f103_mpu9250_lib/mpu9250_lib_example/STM32_Nucleo_F103RB_MPU9250/STM32F103RBTX_FLASH.ld
	arm-none-eabi-gcc -o "STM32_Nucleo_F103RB_MPU9250.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/Users/macbookmichal/Desktop/stm32_f103_mpu9250_lib/mpu9250_lib_example/STM32_Nucleo_F103RB_MPU9250/STM32F103RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32_Nucleo_F103RB_MPU9250.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32_Nucleo_F103RB_MPU9250.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32_Nucleo_F103RB_MPU9250.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
