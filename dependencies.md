Dependencies
===

## Libraries list

|     Name     |                                                              Sources                                                              |       Versions        | Notes                                                                                                                                                                                                                                                                                                                                 |
| :----------: | :-------------------------------------------------------------------------------------------------------------------------------: | :-------------------: | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
|    gflags    |                       [Github](https://github.com/gflags/gflags)</br>[Doc](https://gflags.github.io/gflags)                       | v2.2.2</br>(e171aa2)  | -                                                                                                                                                                                                                                                                                                                                     |
|     glog     |                                             [Github](https://github.com/google/glog)                                              | v0.6.0</br>(b33e3ba)  | -                                                                                                                                                                                                                                                                                                                                     |
|  googletest  |                   [Github](https://github.com/google/googletest)</br>[Doc](https://google.github.io/googletest)                   | v1.14.0</br>(f8d7d77) | -                                                                                                                                                                                                                                                                                                                                     |
|  benchmark   |            [Github](https://github.com/google/benchmark)</br>[Doc](https://google.github.io/benchmark/user_guide.html)            | v1.8.3</br>(3441176)  | -                                                                                                                                                                                                                                                                                                                                     |
|    Eigen     |                      [Gitlab](https://gitlab.com/libeigen/eigen)</br>[Doc](https://eigen.tuxfamily.org/dox/)                      |   v3.4.x</br>(???)    | Somewhere on branch `3.4`                                                                                                                                                                                                                                                                                                             |
| ceres solver |                    [Github](https://github.com/ceres-solver/ceres-solver) </br>[Doc](http://ceres-solver.org/)                    | v2.1.0</br>(f68321e)  | Fix Fundamental manifold to upgrade to v2.2.0                                                                                                                                                                                                                                                                                         |
|   ~~fmt~~    |                                [Github](https://github.com/fmtlib/fmt)</br>[Doc](https://fmt.dev)                                 |           -           | <ul><li>Introduced because of Sophus, remove after Sophus 1.24.6.</li><li>Not exactly the same as the format library offered by Cpp20</li></ul>                                                                                                                                                                                       |
|    Sophus    |                                           [Github](https://github.com/strasdat/Sophus)                                            | v1.24.6</br>(d0b7315) | <ul><li>`Sophus` is under maintenance mode, no more features will be added. The actively developed `Sophus 2` is in another repo. An alternative library is [`manif`](https://github.com/artivis/manif)</li><li>The author is a Computer Vision researcher. According to his profile, he started SLAM research around 2010.</li></ul> | Spectra |  |  |  |
|  ~~fftw3~~   |                              [Github](https://github.com/FFTW/fftw3)</br>[Website](https://fftw.org)                              |           -           | <ul><li>Use Eigen unsupported fft library instead, fftw3 is one of the backend solver options.</li><li>__DONT__ try to build from github source, use the source from [Download](https://www.fftw.org/download.html) page.</li></ul>                                                                                                   |
|    OpenCV    | [Release](https://opencv.org/releases/)</br>[Github](https://github.com/opencv)</br>[Doc](https://docs.opencv.org/4.x/index.html) |        4.10.0         | <ul><li>Use the pre-built binary if contrib is not neccessary.</li><li>Encounter download problem when configuring `xfeatures2d` module.</li></ul>                                                                                                                                                                                    |
|   yaml-cpp   |                                           [Github](https://github.com/jbeder/yaml-cpp)                                            | v0.8.0</br>(f732014)  | -                                                                                                                                                                                                                                                                                                                                     |         |
|      Qt      |                          [Download](https://www.qt.io/download-dev)</br>[Github](https://github.com/qt)                           |         6.5.2         | -                                                                                                                                                                                                                                                                                                                                     |
|   ~~gRPC~~   |                       [Github](https://github.com/grpc/grpc)</br>[Doc](https://grpc.io/docs/languages/cpp/)                       |           -           | <ul><li>Used to communicate with camera modules, remove after enabling zmq.</li><li>Not easy to build from source, too many dependencies.</li>                                                                                                                                                                                        |
|    ZeroMQ    |                     [Github](https://github.com/zeromq/libzmq)</br>[Website](https://zeromq.org/get-started/)                     |           -           | Read the documentation carefully.                                                                                                                                                                                                                                                                                                     |
|    libssh    |                                                [Website](https://www.libssh.org/)                                                 |           -           | Need `OpenSSL` and `zlib` to build from source                                                                                                                                                                                                                                                                                        |
|    cppzmq    |             [Github](https://github.com/zeromq/cppzmq)</br>[Doc](https://brettviren.github.io/cppzmq-tour/index.html)             |           -           | Many alternatives, not sure the differences.                                                                                                                                                                                                                                                                                          |
|   freeglut   |                  [Github](https://github.com/freeglut/freeglut)</br>[Website](https://freeglut.sourceforge.net/)                  |           -           | An alternative of the abandoned GLUT                                                                                                                                                                                                                                                                                                  |
|     glew     |                     [Github](https://github.com/nigels-com/glew)</br>[Website](https://glew.sourceforge.net/)                     |           -           | -                                                                                                                                                                                                                                                                                                                                     |
|     ROS      |                                                                                                                                   |                       |                                                                                                                                                                                                                                                                                                                                       |


## Instructions

Most C++ developers need to know a few essential steps to build a CMake-organized project.

```bash
# cd cmake-project
mkdir build
cd build
cmake ..
make
make install
```

Adding just one more argument can make your libraries much cleaner.

```bash
cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/your/local/root
```

There are more tricks worth knowing to build with CMake, but I won't cover them here. I want to mention two important points:

1. Don't feel ashamed to use CMake GUI

CMake is a powerful language with thorough but sometimes hard-to-understand documentation. If you only know how to use a few commands to build projects or write basic project descriptions, you won't fully understand CMake's capabilities. The GUI allows you to see what's happening under the hood after you enter `cmake ..`, and it provides a searchable log. When it comes to highly automated builds and configuration, the GUI helps you understand the core framework more clearly.


2. Install source-built libraries to a local directory

On Linux-like systems, we have many handy package management tools, such as `apt` on Ubuntu and `brew` on macOS. On Windows, we configure around Visual Studio and environment paths. It's important to note that these package management tools handle __global__ dependencies, which may be used by system applications you don't want to interfere with. Before tools like `virtualenv` introduced to C++, my suggestion for managing source-built libraries is to install all built targets in a specific local directory. Use the library name as the package root directory and the library version as a sub-directory name. Some preferable directory locations could be

+ Windows: D:/Dependencies, D:/Local
+ Linux: ${HOME}/Dependencies, ${HOME}/.local (e.g. /home/bobblelaw/Dependencies, /home/bobblelaw/.local)

Take OpenCV for example, say you try to build the latest OpenCV(4.10.0) source. Then the built targets should be found in

+ Windows: D:/Dependencies/opencv/4.10.0, D:/Dependencies/opencv/4.10.0n
+ Linux: ${HOME}/Dependencies/opencv/4.10.0, ${HOME}/Dependencies/opencv/4.10.0x

For the suffix, you can have your own style. Like "-stable", "-dev", or "n" stands for nightly, "x" stands for extra, etc. In real production scenario, we __MUST__ choose stable version.