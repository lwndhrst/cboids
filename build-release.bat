@echo off

cmake -S . -B build/release -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=RELEASE
cmake --build build/release
