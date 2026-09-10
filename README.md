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

3. **Preparing**:
    - Clone the main repository:
    ```
    git clone https://gitlab.tuni.fi/cs/ultravideo/pcc/uvgvolucap.git
    cd uvgvolucap
    ```

    - Initialize the submodules
    ```
    git submodule update --init --recursive
    ```

    - To Pull Updates Later:
    ```
    git submodule update --remote
    ```

4. **Build the project**:
    - Navigate to this project folder (same location as the vcpkg-configuration.json file).
    - Run the following command to build:
    ```
    mkdir build
    cd build
    cmake .. --preset=default
    cmake --build . --config Release --parallel --target install
    ```
    
After following these steps, you should be able to build the project successfully.

5. **Calibration**:
    + To run our method:
    ```
    cd /src/calibration
    python Calibrate.py
    ```
    If a camera is connected, a window of its color image will appear like this:
    
    ![Initial ROI img](demo/initial_ROI.png)

    Select the ROI - make sure it can cover most of the human body - Then press Space:

    ![Selected ROI](demo/Selected_ROI.png)

    Then a Window of 3D scene appears - Press K to lock - Select the aruco area - Press C to trim to scene - Press ESC to exit.
    Worth noting that: Focus only on the area containing the aruco, leave some details of the wall to recognize the correct surface to click later.

    ![Selected ROI](demo/Trim3D.png)

    Then repeat these steps for all connected cameras.
    After that, a scene of 4 points appears, Shift + click on this order: Pink - Blue - Red - Gray

    ![Selected ROI](demo/4points.png)

    Then a 3D scene containing the aruco appears, click the color on the Aruco: Pink - Blue - Red - Gray. Then press ESC
    At this stage, the calibrated scene appears, the first scene has been corrected to the correct space/position. Now it will become the reference for other cameras.
    Click Pink - Blue - Red - Gray on the reference scene. Then repeat same order for other scenes and follow instruction on the terminal.

    + Use oneshot ChArUco board method:

    This method estimates the camera extrinsics in a single shot by detecting a ChArUco board visible to all cameras, using `src/calibration/calibrate_charuco.py`. It also lets you visually fine-tune the merged point clouds before saving the calibration.
    ```
    cd /src/calibration
    python calibrate_charuco.py
    ```
    Useful options:
    ```
    python calibrate_charuco.py --calibrate --data_dir ./camera_data   # also run intrinsic ChArUco calibration on the captures
    python calibrate_charuco.py --config ./cameraconfig.json            # reuse existing extrinsics instead of solving the pose
    python calibrate_charuco.py --load --data_dir ./camera_data        # reuse previously captured images/point clouds
    ```
    Adjust the board geometry with `--squares_x`, `--squares_y`, `--charuco_square_len`, `--charuco_marker_len` and `--dict` to match your printed board. The resulting transformations are written to `cameraconfig.json`.

    + Use CWIPC method:

    Create an empty calib file:
    ```
    cwipc_register --noregister
    ```
    Run this to start calibrate, make sure the aruco is visible for all cameras:
    ```
    cwipc_register --nofine --rgb --interactive --verbose
    ```
    A Window of all views from all cameras appears, then press W. Check that all views can detect the aruco marker. Then press ESC to finish.
    Now you have cameraconfig.json from CWIPC software. Go to /src/calibration to convert it to uvgVolucap format:
    ```
    cd /src/calibration
    python3 convert_cwipc_calib.py --cwipc_calib_path /path/to/cwipc_cameraconfig.json --template ./template.json --output /path/to/uvgvolucap_calib.json
    ```

6. **Run capturing**:
    + Run Visualizer -> Switch to ZMQ mode -> Start listening:
    + Go to build/bin:
    ```
    ./uvgVoluCap --config /path/to/uvgvolucap_calib.json --addr_color tcp://localhost:5555 --addr_position tcp://localhost:5556 --running_time 60 --cam_type 2 --running_mode 0
    ```
    Note that: --addr_color tcp://localhost:5555 --addr_position tcp://localhost:5556 must match the addresses on the Visualizer.