# 四旋翼比赛

Copyright (c) 2017, HIT-E304 CORPORATION. All rights reserved.

## 目前小组成员：李博、老薛、东升、小宋、莫墨

### 程序说明

#### mode_test_pkg

1.**px4Lib**

+ 文件：
> px4Lib.cpp
> px4Lib.h

+ 功能：
> - 无人机初始化 `bool init(ros::NodeHandle& nodeHandle);`
> - 无人机解锁`bool arm();`
> - 无人机任务执行`void spin();`

2.**position_ctrl_with_rc**

+ 文件：
> position_ctrl_with_rc.cpp

+ 功能：
> 可执行无人机控制节点

3.**drone_state**

+ 文件：
> drone_state.cpp

+ 功能：
> 可执行无人机状态读取节点

4.**tracking_camera**

+ 文件：
> tracking_camera.launch

+ 功能：
> 将所有需要执行的节点写到此launch文件中，以实现跟踪摄像头的功能。

#### usb_cam
此pkg为下载的公用包，用来是用来连接摄像头的驱动，可执行节点为`usb_cam_node.cpp`。

#### opencvtest
1.**opencvtest**

+ 文件：
> opencvtest.cpp

+ 功能：
> 可执行节点，摄像头对红色方框的识别。

2.**img_pro_info.msg**

+ 功能：自定义msg文件，为摄像头最终识别的信息，即识别结果。
``` cpp
    bool out_flag
    int32 width
    int32 height
    int32 x_pos
    int32 y_pos
```

### 运行方法
1. 在硬件接线正确的前提下进入到本地工作空间所在路径`cd 'workspace'`
2. source此工作空间下的bash文件，将此工作空间写入环境变量中`source devel/setup.bash`
3. 运行跟踪摄像头的程序`roslaunch mode_test_pkg tracking_camera.launch`