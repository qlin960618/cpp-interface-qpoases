# cpp-interface-qpoases 
Interface between DQ Robotics and [qpOASES](https://github.com/coin-or/qpOASES)

Refer to the [docs](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/cpp.html)

## Recommended installation process for qpOASES

The following commands will clone the repository and install qpOASES as a shared library.

```bash
cd ~/Downloads
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
sed -i -e 's/option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" OFF)/option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" ON)/g' CMakeLists.txt
mkdir build
cd build
cmake ..
make -j16
sudo make install
sudo ldconfig
```
