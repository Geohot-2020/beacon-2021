################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/AppSw/Tricore/Driver/LQ_ADC.c \
../src/AppSw/Tricore/Driver/LQ_CCU6.c \
../src/AppSw/Tricore/Driver/LQ_DMA.c \
../src/AppSw/Tricore/Driver/LQ_EEPROM.c \
../src/AppSw/Tricore/Driver/LQ_EMEM.c \
../src/AppSw/Tricore/Driver/LQ_FFT.c \
../src/AppSw/Tricore/Driver/LQ_GPIO.c \
../src/AppSw/Tricore/Driver/LQ_GPSR.c \
../src/AppSw/Tricore/Driver/LQ_GPT12_ENC.c \
../src/AppSw/Tricore/Driver/LQ_GTM.c \
../src/AppSw/Tricore/Driver/LQ_QSPI.c \
../src/AppSw/Tricore/Driver/LQ_SOFTI2C.c \
../src/AppSw/Tricore/Driver/LQ_SPI.c \
../src/AppSw/Tricore/Driver/LQ_STM.c \
../src/AppSw/Tricore/Driver/LQ_UART.c 

OBJS += \
./src/AppSw/Tricore/Driver/LQ_ADC.o \
./src/AppSw/Tricore/Driver/LQ_CCU6.o \
./src/AppSw/Tricore/Driver/LQ_DMA.o \
./src/AppSw/Tricore/Driver/LQ_EEPROM.o \
./src/AppSw/Tricore/Driver/LQ_EMEM.o \
./src/AppSw/Tricore/Driver/LQ_FFT.o \
./src/AppSw/Tricore/Driver/LQ_GPIO.o \
./src/AppSw/Tricore/Driver/LQ_GPSR.o \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.o \
./src/AppSw/Tricore/Driver/LQ_GTM.o \
./src/AppSw/Tricore/Driver/LQ_QSPI.o \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.o \
./src/AppSw/Tricore/Driver/LQ_SPI.o \
./src/AppSw/Tricore/Driver/LQ_STM.o \
./src/AppSw/Tricore/Driver/LQ_UART.o 

COMPILED_SRCS += \
./src/AppSw/Tricore/Driver/LQ_ADC.src \
./src/AppSw/Tricore/Driver/LQ_CCU6.src \
./src/AppSw/Tricore/Driver/LQ_DMA.src \
./src/AppSw/Tricore/Driver/LQ_EEPROM.src \
./src/AppSw/Tricore/Driver/LQ_EMEM.src \
./src/AppSw/Tricore/Driver/LQ_FFT.src \
./src/AppSw/Tricore/Driver/LQ_GPIO.src \
./src/AppSw/Tricore/Driver/LQ_GPSR.src \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.src \
./src/AppSw/Tricore/Driver/LQ_GTM.src \
./src/AppSw/Tricore/Driver/LQ_QSPI.src \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.src \
./src/AppSw/Tricore/Driver/LQ_SPI.src \
./src/AppSw/Tricore/Driver/LQ_STM.src \
./src/AppSw/Tricore/Driver/LQ_UART.src 

C_DEPS += \
./src/AppSw/Tricore/Driver/LQ_ADC.d \
./src/AppSw/Tricore/Driver/LQ_CCU6.d \
./src/AppSw/Tricore/Driver/LQ_DMA.d \
./src/AppSw/Tricore/Driver/LQ_EEPROM.d \
./src/AppSw/Tricore/Driver/LQ_EMEM.d \
./src/AppSw/Tricore/Driver/LQ_FFT.d \
./src/AppSw/Tricore/Driver/LQ_GPIO.d \
./src/AppSw/Tricore/Driver/LQ_GPSR.d \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.d \
./src/AppSw/Tricore/Driver/LQ_GTM.d \
./src/AppSw/Tricore/Driver/LQ_QSPI.d \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.d \
./src/AppSw/Tricore/Driver/LQ_SPI.d \
./src/AppSw/Tricore/Driver/LQ_STM.d \
./src/AppSw/Tricore/Driver/LQ_UART.d 


# Each subdirectory must supply rules for building sources it contributes
src/AppSw/Tricore/Driver/%.src: ../src/AppSw/Tricore/Driver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gpt12" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/src/AppSw/Tricore/Driver" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/src/AppSw/Tricore/Main" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/src/AppSw/Tricore/User" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/src/AppSw/Tricore/APP" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/src/AppSw" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Infra/Platform/Tricore/Compilers" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Multican/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Infra/Platform" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cif/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Hssl/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cpu/Trap" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/If/Ccu6If" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dsadc/Dsadc" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Port" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Stm/Timer" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dts/Dts" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eth" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Flash" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Vadc" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Msc" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Qspi/SpiMaster" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Scu/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Comm" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Math" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Infra/Platform/Tricore" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Trig" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tim" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/TimerWithTrigger" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Emem" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Mtu" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Infra" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fft" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/I2c/I2c" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Asclin/Asc" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Flash/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/If" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cpu" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fce/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Stm/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Msc/Msc" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Vadc/Adc" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Asclin" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/Pwm" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Atom" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Port/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5/Psi5" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eray" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Qspi/SpiSlave" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/Icu" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cpu/CStart" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Hssl" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cif" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eth/Phy_Pef7071" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Hssl/Hssl" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Iom/Driver" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Multican/Can" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5s/Psi5s" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fft/Fft" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/PwmHl" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Iom/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/_Lib" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/Timer" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Sent" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eray/Eray" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gpt12/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dma" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fce/Crc" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Qspi" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Infra/Sfr" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Infra/Sfr/TC26B" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Bsp" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/General" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cpu/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dts" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Src" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dma/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cif/Cam" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Src/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Asclin/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/I2c/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Configurations" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/_Lib/DataHandling" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Sent/Sent" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/Timer" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5s" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Emem/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/PwmBc" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Iom" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/TPwm" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Multican" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Mtu/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Infra/Sfr/TC26B/_Reg" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/PwmHl" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dma/Dma" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/Timer" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Time" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dsadc/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cpu/Irq" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gpt12/IncrEnc" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5s/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Scu" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/_Lib/InternalMux" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Stm" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dsadc/Rdc" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Vadc/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dts/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eth/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Smu" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/_PinMap" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Asclin/Lin" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/StdIf" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dsadc" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fce" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/PwmHl" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Qspi/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tom" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tim/In" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Msc/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fft/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/Pwm" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/_Utilities" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Smu/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/I2c" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Asclin/Spi" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eray/Std" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Port/Io" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/_Impl" -I"D:/tc264_ws/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Sent/Std" --iso=99 --c++14 --language=+volatile --anachronisms --fp-model=3 --fp-model=c --fp-model=f --fp-model=l --fp-model=n --fp-model=r --fp-model=z -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file=$(@:.src=.d) --misrac-version=2012 -N0 -Z0 -Y0 2>&1; sed -i -e '/ctc\\include/d' -e '/Libraries\\iLLD/d' -e '/Libraries\\Infra/d' -e 's/\(.*\)".*\\LQ_TC26xB_LIB_ADS\(\\.*\)"/\1\.\.\2/g' -e 's/\\/\//g' $(@:.src=.d) && \
	echo $(@:.src=.d) generated
	@echo 'Finished building: $<'
	@echo ' '

src/AppSw/Tricore/Driver/%.o: ./src/AppSw/Tricore/Driver/%.src
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


