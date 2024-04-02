# NarrowPassageSampler

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

Before building the repo, you need to have the following software installed on your system:

- Boost
- OpenCV

### Installing

Follow these steps to build the NarrowPassageSampler library on your system:

1. **Clone the Repository**

   ```bash
   git clone https://github.com/yourusername/NarrowPassageSampler.git
   cd NarrowPassageSampler
   ```

2. **Build the Project**

   Use CMake to configure and build the project:

   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

   This will build the `NarrowPassageSampler` library.

### Usage

To use the NarrowPassageSampler in your project, include the library in your project's build script. Ensure that the `NarrowPassageSampler`, `Boost`, and `OpenCV` libraries are correctly linked.

Here is an example of how you might include it in your CMake project:

```cmake
# Assuming NarrowPassageSampler is installed in /usr/local
include_directories(/usr/local/include/NarrowPassageSampler)
link_directories(/usr/local/lib)

add_executable(your_project_name your_source_files.cpp)
target_link_libraries(your_project_name NarrowPassageSampler ${Boost_LIBRARIES} ${OpenCV_LIBS})
```

## Contributors

- Yafes Enes Şahiner
- Esat Yusuf Gündoğdu


