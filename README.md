# About this REPO
This repo contains a suite of different trajectory planning algorithms for robotic applications 

# Preinstallation requirements
- This repo utilizes matplotplusplus https://github.com/alandefreitas/matplotplusplus and is installed with vcpkg, for plotting applications 
- Follow the instructions to install vcpkg with the link here https://vcpkg.io/en/getting-started.html 
- If you are using Linux you will need to do the following commands
```
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg/ #cd into the directory 
./bootstrap-vcpkg.sh #run the shell script
./vcpkg integrate install
./vcpkg integrate bash 
./vcpkg install matplotplusplus #install the matplot library
```

# How to run
```
mkdir build 
cd build 
cmake ../
make #builds code into the build directory 
```


