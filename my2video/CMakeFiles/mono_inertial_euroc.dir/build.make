# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/telemoro/ORB-SLAM3/orb_slam3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/telemoro/ORB-SLAM3/orb_slam3/my2video

# Include any dependencies generated for this target.
include CMakeFiles/mono_inertial_euroc.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mono_inertial_euroc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mono_inertial_euroc.dir/flags.make

CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o: CMakeFiles/mono_inertial_euroc.dir/flags.make
CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o: ../Examples/Monocular-Inertial/mono_inertial_euroc.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/telemoro/ORB-SLAM3/orb_slam3/my2video/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o -c /home/telemoro/ORB-SLAM3/orb_slam3/Examples/Monocular-Inertial/mono_inertial_euroc.cc

CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/telemoro/ORB-SLAM3/orb_slam3/Examples/Monocular-Inertial/mono_inertial_euroc.cc > CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.i

CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/telemoro/ORB-SLAM3/orb_slam3/Examples/Monocular-Inertial/mono_inertial_euroc.cc -o CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.s

CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o.requires:

.PHONY : CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o.requires

CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o.provides: CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o.requires
	$(MAKE) -f CMakeFiles/mono_inertial_euroc.dir/build.make CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o.provides.build
.PHONY : CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o.provides

CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o.provides.build: CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o


# Object files for target mono_inertial_euroc
mono_inertial_euroc_OBJECTS = \
"CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o"

# External object files for target mono_inertial_euroc
mono_inertial_euroc_EXTERNAL_OBJECTS =

../Examples/Monocular-Inertial/mono_inertial_euroc: CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o
../Examples/Monocular-Inertial/mono_inertial_euroc: CMakeFiles/mono_inertial_euroc.dir/build.make
../Examples/Monocular-Inertial/mono_inertial_euroc: ../lib/libORB_SLAM3.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/local/lib/libpangolin.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libGLX.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libEGL.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libGLX.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libEGL.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libdc1394.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libavcodec.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libavformat.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libavutil.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libswscale.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libavdevice.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libpng.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libz.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libjpeg.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libtiff.so
../Examples/Monocular-Inertial/mono_inertial_euroc: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../Examples/Monocular-Inertial/mono_inertial_euroc: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/Monocular-Inertial/mono_inertial_euroc: ../Thirdparty/g2o/lib/libg2o.so
../Examples/Monocular-Inertial/mono_inertial_euroc: CMakeFiles/mono_inertial_euroc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/telemoro/ORB-SLAM3/orb_slam3/my2video/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/Monocular-Inertial/mono_inertial_euroc"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mono_inertial_euroc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mono_inertial_euroc.dir/build: ../Examples/Monocular-Inertial/mono_inertial_euroc

.PHONY : CMakeFiles/mono_inertial_euroc.dir/build

CMakeFiles/mono_inertial_euroc.dir/requires: CMakeFiles/mono_inertial_euroc.dir/Examples/Monocular-Inertial/mono_inertial_euroc.cc.o.requires

.PHONY : CMakeFiles/mono_inertial_euroc.dir/requires

CMakeFiles/mono_inertial_euroc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mono_inertial_euroc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mono_inertial_euroc.dir/clean

CMakeFiles/mono_inertial_euroc.dir/depend:
	cd /home/telemoro/ORB-SLAM3/orb_slam3/my2video && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/telemoro/ORB-SLAM3/orb_slam3 /home/telemoro/ORB-SLAM3/orb_slam3 /home/telemoro/ORB-SLAM3/orb_slam3/my2video /home/telemoro/ORB-SLAM3/orb_slam3/my2video /home/telemoro/ORB-SLAM3/orb_slam3/my2video/CMakeFiles/mono_inertial_euroc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mono_inertial_euroc.dir/depend
