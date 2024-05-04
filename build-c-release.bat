@echo off

cmake -S c -B build/release -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=RELEASE
cmake --build build/release
