#!/usr/bin/bash

# on change cmakefile (parameter is for clang in nvim)
# cd build
# cmake ../  -DCMAKE_EXPORT_COMPILE_COMMANDS=1 

if [ "$1" = "cmake" ]; then
    rm -r build
    mkdir build
    cd build
    cmake ../ -DCMAKE_EXPORT_COMPILE_COMMANDS=1
    cd ..
fi

cmakelists_file="CMakeLists.txt"
if [ ! -f "$cmakelists_file" ]; then
    echo "Could not find CMakeLists.txt file"
    exit 1
fi

project_name=$(grep -oP '(?<=project\().*(?=\))' "$cmakelists_file")
cmake --build build 1> /dev/null
if [ $? -eq 0 ]; then
    ./build/$project_name
else
    echo "Build gives error!"
fi
