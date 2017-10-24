# Raspi-Self-Balancing-Robot-cpp
基于Raspberry Pi 2 的二轮自平衡机器人，C++版

新Python版项目地址：https://github.com/zytx/Raspi-Self-Balancing-Robot-Python

## 用法

环境
```
sudo apt-get update
sudo apt-get install pigpio
```

编译
```
g++ Robot.cpp -o robot -lpigpio -lrt -pthread
```

运行
```
sudo ./robot
```
