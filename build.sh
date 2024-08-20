#!/bin/sh -e
zig build-exe \
	-target arm-freestanding-none \
	-mcpu cortex_m23 \
	-femit-asm \
	-O ReleaseSmall \
	--script bluepill.ld \
	main.zig
zig objcopy -O hex main main.hex
# rm main main.o
st-flash --reset --format ihex write main.hex 
