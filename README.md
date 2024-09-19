# uvgVoluCap

## Getting started

To build this project from source, you'll need to have vcpkg installed and added to your system's PATH environment variable. Here's how you can do it:

1. **Download vcpkg**: 
   - You can download vcpkg from its GitHub repository: [Microsoft/vcpkg](https://github.com/microsoft/vcpkg):
    ```
    git clone https://github.com/Microsoft/vcpkg.git
    ```
   - Setup executable file:
    ```
    cd vcpkg
    ./bootstrap-vcpkg.sh # bootstrap-vcpkg.bat for Powershell
    ./vcpkg integrate install
    ```
    At this point, vcpkg is set up and ready to use

2. **Add vcpkg to PATH**:

   - Add the directory containing the vcpkg executable to your system's PATH environment variable. This step allows you to run vcpkg from any directory in your command prompt or terminal.

3. **Build the project**:
    - Navigate to this project folder ( Same location with vcpkg-configuration.json file ).
    - Run the following command to build:
    ```
    mkdir build
    cd build
    cmake .. --preset=default
    cmake --build . --config Release --parallel
    ```
    
After following these steps, you should be able to build the project successfully.
