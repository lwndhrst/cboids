@echo off

cmake -S . -B build -G "Unix Makefiles"
cmake --build build
