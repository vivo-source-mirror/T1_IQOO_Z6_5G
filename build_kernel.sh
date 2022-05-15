#!/bin/bash
#git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/host/x86_64-linux-glibc2.17-4.8 -b android11-mainline-release
#git clone https://android.googlesource.com/platform/prebuilts/clang/host/linux-x86 -b android11-mainline-release
#git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9 -b android11-mainline-release
PATH="`pwd`/aarch64-linux-android-4.9/bin:`pwd`/clang_tool/clang-r383902b/bin:`pwd`/x86_64-linux-glibc2.17-4.8/bin$PATH"
mkdir -p out
make -j32 O=out HOSTCC=clang HOSTAR=x86_64-linux-ar HOSTLD=x86_64-linux-ld ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- REAL_CC=clang CROSS_COMPILE=aarch64-linux-android- CLANG_TRIPLE=aarch64-linux-gnu- vendor/holi-qgki_defconfig
make -j32 O=out DTC_EXT=`pwd`/dtc CONFIG_BUILD_ARM64_DT_OVERLAY=y HOSTCC=clang HOSTAR=x86_64-linux-ar HOSTLD=x86_64-linux-ld ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- REAL_CC=clang CROSS_COMPILE=aarch64-linux-android- CLANG_TRIPLE=aarch64-linux-gnu-
