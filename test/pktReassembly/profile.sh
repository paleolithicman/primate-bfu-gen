#!/bin/bash

make clean && make
clang++ -g -emit-llvm -S -O0 primate.cpp -o primate.ll
LLVM_PROFILE_FILE="main.profraw" ./main
llvm-profdata merge main.profraw -o main.profdata
llvm-cov export -format=text -instr-profile main.profdata ./main -sources primate.cpp > prof.json
#llvm-cov export -format=text -skip-functions -instr-profile main.profdata ./main -sources primate.cpp > prof.json
llvm-cov show -show-line-counts-or-regions -instr-profile=main.profdata ./main -sources primate.cpp > report.txt
