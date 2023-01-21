# Intel Realsense D435/D435i

## SDK
You can get the SDK from an apt install. Please follow the instructions listed [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).
This will give you the C/C++ SDk and associated libraries, however given most of this course is taught in Python you will want some bindings.

```bash
pip install pyrealsense2
```

Documentation for the SDKs is provided below.

[C/C++ Documentation](https://intelrealsense.github.io/librealsense/doxygen/annotated.html) | 
[Python Documentation](https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html)

## Realsense Viewer
The best way to acquaint yourself with the capabilities of the Realsense cameras is to use the `realsense-viewer` application shipped as part of the SDK. Here you can check which streams are available, which configurations of streams are available in a combination off the camera and get a feel for field of view, maximum sensing distances etc, as well as dialing in manual exposure settings.

## Examples

Find some example code for getting the Realsense camera running at the below links.

[C/C++](https://dev.intelrealsense.com/docs/code-samples) | [Python Examples](https://dev.intelrealsense.com/docs/python2)

## Calibration
The calibration and camera parameters are stored on the Realsense camera themselves. *DO NOT OVERWRITE THESE CONFIGURATIONS ON THE CAMERAS*, however we recommend that as part of your set-up you perform calibration of the cameras to compare against the stored parameters, and if needed use your own calibrations. The above documentation will assist you in retrieving these parameters.

## Tools for Interfacing with Data
We recommend that you use OpenCV for doing any image processing from the Realsense cameras, and Open3D for any pointcloud work.
