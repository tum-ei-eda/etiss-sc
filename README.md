# etiss-sc

SystemC/TLM2.0 productivity library wrapping the Extendable Instruction Set Simulator ETISS as a tlm cpu core.

## Install
### Dependencies
Requires the following dependencies
1. [ETISS](https://github.com/tum-ei-eda/etiss.git) - tested with [0e9a9c8738bada27b840795b7cfdaa6c7e4e1a44](https://github.com/tum-ei-eda/etiss/commit/0e9a9c8738bada27b840795b7cfdaa6c7e4e1a44)

the following infrastructure
1. gcc - tested with v9.3.0
2. cmake (>= 3.15)

### Standalone

####  Build
After cloning the repo and entering into it:
```
cmake -S . -B build -DETISS_PREFIX=<path/to/etiss/install> -DSystemCLanguage_DIR=/path/to/systemc/lib/cmake/SystemCLanguage [-DCMAKE_BUILD_TYPE={...}]
cmake --build build [--parallel $(nproc)]
```

### Alongside SCC (as submodule in another VP)

In the VP's CMakeLists.txt add `etissvp` submodule after the `scc` submodule.

```
...
add_subdirectory(<path/to/scc>)
add_subdirectory(<path/to/etissvp>)
...
```

## Compile and Run Minimal Example

1. Minimal Target Software
```
export RISCV=<path/to/riscv/gnu/toolchain>
$RISCV/bin/riscv64-unknown-elf-gcc -march=rv32im -mabi=ilp32 -O3 -T examples/barebone_vp/target_software/link.ld -nostartfiles examples/barebone_vp/target_software/crt0.s examples/barebone_vp/target_software/helloworld.c examples/barebone_vp/target_software/syscall.c -DETISSVP_LOGGER=0xf0000000 -o target_software.elf
```
2. Execute Target Software
```
build/examples/barebone_vp/barebone_vp --etiss examples/barebone_vp/ini/etiss.ini --elfs target_software.elf
```
