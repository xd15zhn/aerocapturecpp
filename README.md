# 火星气动捕获仿真
第一部分：验证被控对象建立的模型是否正确。

# 依赖库
- <https://gitee.com/xd15zhn/simucpp>
- <https://gitee.com/xd15zhn/raylib>

# 文件说明
- [main.cpp] 带制导律的精确模型仿真主程序。
- [test.cpp] 简化模型测试主程序。
- [camera.c/h] 3D图形库`raylib`的摄像头设置。
- [spacecraft.cpp/hpp] `simucpp`建立的火星探测器被控对象模型。简化模型和精确模型都适用，仅参数不同。
- [identifier.hpp] 使用了特征模型方法的制导律。
- [parameters.hpp] 需要用到的常数参数。

# log
`main.cpp`运行后的结果如下
```
Calculating trajectory inside atmosphere......
56.5s, height: 150.202        
Flew out of the atmosphere.
Target Apoapsis: 5652.5
Calculated Apoapsis: 5652.5
Calculating trajectory outside atmosphere......
5652.49    
Trajectory calculating finished.
```
