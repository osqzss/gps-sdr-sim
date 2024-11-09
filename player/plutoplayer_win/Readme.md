# plutoplayer_win

### Build on Windows with Visual Studio 2022

The libiio library can be obtained on the [Github](https://github.com/analogdevicesinc/libiio/releases) page of the project. Download __Windows.zip__ for the latest stabel build and extract it.

1. Start Visual Studio 2022
2. Create an empty project for a console application
3. On the _Solution Explorer_ at right, add the source files to the project.
4. Add the paths to the following folder in `Configuration Properties -> C/C++ -> General -> Additional Include Directories`:
    * __Windows/include__ for iio.h
5. Add the paths to the following folder in `Configuration Properties -> Linker -> General -> Additional Library Directories`:
    * __Windows/Windows-VS-2022-x64__ for libiio.lib
6. Specify the name of the additional library in `Configuration Properties -> Linker -> Input -> Additional Dependencies`:
    * __libiio.lib__
7. Select __Release__ in the _Solution Configurations_ drop-down list
8. Select __X64__ in the _Sofution Platforms_ drop-down list
9. Run `Build -> Build Solution`

After a successful build, you can find the executable in the __Release/x64__ folder. Copy it into the __Windows/Windows-VS-2022-x64__ folder and run it.
