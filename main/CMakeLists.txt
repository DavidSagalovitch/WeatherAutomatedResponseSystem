idf_component_register(
    SRCS
        "main.c"           # Main application source
        "../motor/motor.c" # Motor component source
        "../led/led.c"     # LED component source
        "../testIO/gpio_write.c" 
        "../testIO/gpio_read.c" 
    INCLUDE_DIRS
        "."                # Include headers from the main folder
        "../motor"         # Include headers from the motor folder
        "../led"           # Include headers from the led folder
        "../testIO"
)