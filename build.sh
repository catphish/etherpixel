arm-none-eabi-gcc -mcpu=cortex-a7 -fpic -ffreestanding -c boot.s -o boot.o \
&& arm-none-eabi-gcc -O2 -mcpu=cortex-a7 -fpic -ffreestanding -std=gnu99 -c kernel.c -o kernel.o -Wall -Wextra \
&& arm-none-eabi-gcc -O0 -mcpu=cortex-a7 -fpic -ffreestanding -std=gnu99 -c sleep.c -o sleep.o -Wall -Wextra \
&& arm-none-eabi-gcc -T linker.ld -o myos.elf -ffreestanding -O2 -nostdlib boot.o sleep.o kernel.o\
&& sudo cp ./myos.elf /var/lib/tftpboot/ \
&& echo OK
