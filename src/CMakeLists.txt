add_executable(floppyorgan main.c midi.h midi.c uart2midi.h uart2midi.c scheduler.h scheduler.c)

pico_generate_pio_header(floppyorgan ${CMAKE_CURRENT_LIST_DIR}/floppy.pio)

# pull in common dependencies
target_link_libraries(floppyorgan pico_stdlib hardware_pio pico_multicore)
pico_enable_stdio_usb(floppyorgan 1)
pico_enable_stdio_uart(floppyorgan 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(floppyorgan)

# add url via pico_set_program_url
example_auto_set_url(floppyorgan)
