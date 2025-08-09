cmake_minimum_required(VERSION 3.29)
project(stm32f7xx_drivers C)

set(CMAKE_C_STANDARD 11)

include_directories(drivers)
include_directories(drivers/Inc)

add_executable(stm32f7xx_drivers
    drivers/Inc/stm32f746xx.h
    Src/main.c
    Src/syscalls.c
    Src/sysmem.c
        drivers/Inc/stm32f746xx_gpio_driver.h
        drivers/Src/stm32f746xx_gpio_driver.c
        drivers/Inc/stm32f746xx_spi_driver.h
        drivers/Src/stm32f746xx_spi_driver.c
        drivers/Src/stm32f746xx_i2c_driver.c
        drivers/Inc/stm32f746xx_i2c_driver.h)
