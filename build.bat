@echo off

cmake -S c -B build -G "Unix Makefiles"
cmake --build build

odin build odin -out:build\multilingual-boids-odin.exe
