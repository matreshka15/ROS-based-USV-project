上位机ROS系统的核心Package
========================
### 说明
* workingfile/目录下用于存放无人船运行过程中需要的文件，例如：路径的经纬度记录文件、GPS坐标点记录文件等。

-----------
### Jetson开发环境配置
* WiFi链接  
搜索WiFi信号:nmcli dev wifi  
连接WiFi:sudo nmcli dev wifi connect wifi_name password 12345678

* 修改DNS  
sudo nano /etc/systemd/resolved.conf  
修改好了之后：systemctl restart systemd-resolved.service

* 创建swap空间
$ sudo fallocate -l 8G /mnt/8GB.swap  
$ sudo mkswap /mnt/8GB.swap  
$ sudo swapon /mnt/8GB.swap  
然后sudo nano /etc/fstab  
/mnt/8GB.swap  none  swap  sw 0  0  

* 配置frp穿透:
https://www.jianshu.com/p/a921e85280ed  
启动frp:    ./frpc -c ./frpc.ini  

* 解压缩tar.gz文件
tar -zxvf java.tar.gz  
解压到指定的文件夹tar -zxvf java.tar.gz  -C /usr/java  
x : 从 tar 包中把文件提取出来  
z : 表示 tar 包是被 gzip 压缩过的，所以解压时需要用 gunzip 解压  
v : 显示详细信息  
f xxx.tar.gz :  指定被处理的文件是 xxx.tar.gz  

* 换源：
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic main restricted universe multiverse  
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic main restricted universe multiverse  
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-updates main restricted universe multiverse  
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-updates main restricted universe multiverse  
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-backports main restricted universe multiverse  
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-backports main restricted universe multiverse  
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-security main restricted universe multiverse  
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-security main restricted universe multiverse  

* OpenGl模组
apt-get install python3-pyqt4.qtopengl  
pip3 install PyOpenGl PyOpenGl_accerate  

-----------------
