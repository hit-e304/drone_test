# 四旋翼比赛

## 目前小组成员：莫墨、老薛、东升、小宋、李博

### 无人机测试步骤

1 . 上电前准备工作：

+ 在实验室一定要记好每个树莓派在连接李老师那个WiFi的时候的ip地址；
+ 打开地面站，插好数传；
+ 打开WiFi（一定要先打开WiFi之后才能给树莓派上电，否则树莓派不会自动连接WiFi）；
+ 打开远程控制端的电脑，准备测试。

2 . 上电到起飞前：

+ 首先打开多个Teminal，输入`ssh dqn3@192.168.1.147`，准备好起飞和读取飞行数据；
+ 读取飞行器当前位姿的命令`rostopic echo /mavros/local_position/pose`
+ 读取飞行器状态的节点：
``` bash
source laoxue_ws/devel/setup.bash
rosrun mode_test_pkg drone_state
```
**以上两条命令在teminal中先敲好，但是先不要执行，等待其他测试无误后再执行**

3 . 飞行器起飞：

+ 