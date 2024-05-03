@echo off

mkdir build

odin build odin -out:build\odin.exe

cmake -S c -B build -G "Unix Makefiles"

cmake --build build
