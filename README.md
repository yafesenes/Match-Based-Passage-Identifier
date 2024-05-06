# Match-Based Passage Identifier

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

Before building the repo, you need to have the following software installed on your system:

- Boost
- OpenCV

### Installing

Follow these steps to build the MBPI library on your system:

1. **Clone the Repository**

   ```bash
   git clone https://github.com/yafesenes/Match-Based-Passage-Identifier.git
   cd Match-Based-Passage-Identifier
   ```
2. **Build the project**   
   Use CMake to configure and build the project:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```
   This will build the MBPI library.

### Usage
To use the MBPI in your project, include the library in your project's build script. Ensure that the MBPI, Boost, and OpenCV libraries are correctly linked.

Here is an example of how you might include it in your CMake project:
   ```cmake
   add_subdirectory(path/to/MBPI)
   add_executable(your_project src/main.cpp)
   target_link_libraries(your_project MBPI)
   ```

## Contributors
- Yafes Enes Şahiner
- Esat Yusuf Gündoğdu

