# Hello

These Ansible scripts are created to setup the stack for our robot.

[How to install ansible](https://www.cyberciti.biz/python-tutorials/linux-tutorial-install-ansible-configuration-management-and-it-automation-tool/)

Don't forget to change the host in the inventory.yml and in the setup.yml

To check ssh connection

`ansible virtual -m ping`


Do the setup

`ansible-playbook setup.yml`


-v for stderr stdout
-vvvvv for all the debug

----

# Flags and packages


Flags that we don't use

```
# our: cmake -D INSTALL_PYTHON_EXAMPLES=ON -D WITH_XIMEA=BOOL:ON ..
# CFLAGS="-pipe -O3 -march=native" cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_C_EXAMPLES=OFF -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH={{ download_dir }}/opencv_contrib-{{ opencv_version }}/modules -D BUILD_EXAMPLES=ON -D WITH_XIMEA=BOOL:ON ..
# cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_QT=ON -D WITH_OPENGL=ON {{ opencv_extra_cmake_options }} ..

-D BUILD_TIFF=ON
-D WITH_CUDA=OFF
-D ENABLE_AVX=OFF
-D WITH_OPENGL=OFF
-D WITH_OPENCL=OFF
-D WITH_IPP=OFF
-D WITH_TBB=ON
-D BUILD_TBB=ON
-D WITH_EIGEN=OFF
-D WITH_V4L=OFF
-D WITH_VTK=OFF
-D BUILD_TESTS=OFF
-D BUILD_PERF_TESTS=OFF
-D CMAKE_BUILD_TYPE=RELEASE
-D CMAKE_INSTALL_PREFIX=/usr/local
-D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules /opt/opencv/

-D WITH_XIMEA=BOOL:ON
-D BUILD_NEW_PYTHON_SUPPORT=ON
-D WITH_QT=ON
-D WITH_OPENGL=ON
```

Some packages list that I have not yet gone through.

```
# PROVISION
# SETUP: https://github.com/mattskone/ansible-opencv-python3/blob/master/roles/opencv/tasks/main.yml
# PKG: https://github.com/chrismeyersfsu/playbook-opencv/blob/master/roles/common/defaults/main.yml
 - build-essential
 - cmake
 - cmake-qt-gui
 - pkg-config
 - libpng12-0
 - libpng12-dev
 - libpng++-dev
 - libpng3
 - libpnglite-dev
 - zlib1g-dbg
 - zlib1g
 - zlib1g-dev
 - pngtools
 - libtiff4-dev
 - libtiff4
 - libtiffxx0c2
 - libtiff-tools
 - libjpeg8
 - libjpeg8-dev
 - libjpeg8-dbg
 - libjpeg-progs
 - ffmpeg
 - libavcodec-dev
 - libavcodec53
 - libavformat53
 - libavformat-dev
 - libgstreamer0.10-0-dbg
 - libgstreamer0.10-0
 - libgstreamer0.10-dev
 - libxine1-ffmpeg
 - libxine-dev
 - libxine1-bin
 - libunicap2
 - libunicap2-dev
 - libdc1394-22-dev
 - libdc1394-22
 - libdc1394-utils
 - swig
 - libv4l-0
 - libv4l-dev
 - python-numpy
 - libpython2.6
 - python-dev
 - python2.6-dev
 - libgtk2.0-dev
 - pkg-config
```

```
- libjpeg8-dev
- libtiff5
- libtiff5-dev
- libjasper-dev  # TODO: might cause issues on newer OS versions
- libpng12-dev
- libjpeg-dev
- libtiff5-dev
- libavcodec-dev
- libavformat-dev
- libswscale-dev
- libv4l-dev
- libatlas-base-dev
- gfortran
```
