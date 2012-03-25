#!/bin/bash
cp arch/arm/configs/u8220_defconfig .config
time make ARCH=arm CROSS_COMPILE=~/build/android-ndk-r6/toolchains/arm-linux-androideabi-4.4.3/prebuilt/linux-x86/bin/arm-linux-androideabi- -j4
