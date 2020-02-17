# 3D printing with Moveo
![image](https://github.com/Szu-Chi/3d-printing-with-moveo/blob/Feature_Position_Control/img/moveo.gif)
### 將BCN3D-Moveo結合ROS、Marlin以及 Ultimaker Cura，產生新型態的3D列印機。
我們變更3D列印切片軟體Cura的列印機模型與列印空間，並將Cura與ROS中的逆運動學求解器(IK-Slover)整合，把Cura生成的笛卡爾座標系轉換成更機械軸的轉動角度，最終透過Marlin控制BCN3D-Moveo移動噴嘴進行3D列印
### 系統架構 System Structure:
![image](https://github.com/Szu-Chi/3d-printing-with-moveo/blob/Feature_Position_Control/img/system_structure.png)


## How to Use:
### 1. 環境準備
- 1. 安裝ROS、MoveIt和trac-ik
        1.  安裝ros-melodic-desktop-full\
            參考 http://wiki.ros.org/melodic/Installation/Ubuntu

        2.  使用原始碼安裝MoveIt\
            參考 https://moveit.ros.org/install/source/
        3.  安裝trac-ik
            ```
            sudo apt-get install ros-melodic-trac-ik
            ```
- 2. 建立工作區 moveo_ws
    ```
    cd ~
    mkdir moveo_ws 
    ``` 
- 3. 下載3d-printing-with-movoe資料
    ```
    cd ~/moveo_ws
    git clone https://github.com/Szu-Chi/3d-printing-with-moveo.git
    ``` 
- 4. 變更資料夾名稱為src
    ```
    mv ~/moveo_ws/3d-printing-with-moveo ~/moveo_ws/src
    ``` 
- 5. 建構原始碼build source code
    ```
    catkin_make
    source ~/moveo_ws/devel/setup.bash
    echo "source ~/moveo_ws/devel/setup.bash" >> ~/.bashrc
    ```

### 2. 安裝 Cura SIPE version
![image](https://github.com/Szu-Chi/3d-printing-with-moveo/blob/Feature_Position_Control/img/curaLoadingImg.png)
- 使用autoInstall.bash
    ```
    cd ~/moveo_ws/src/Cura
    bash autoInstall.bash
    source ~/.bashrc
    ```
or

- 參考 https://github.com/Ultimaker/Cura/wiki/Running-Cura-from-Source-on-Ubuntu 手動安裝

### 3. 燒錄Marlin
- 1. 開啟Arduino ~/moveo_ws/src/Marlin/Marlin.ino
- 2. Tool->Board->Arduino Mega or Mega 2560
- 3. 按下上傳按鈕(Ctrl+U) 
### 4. 開始列印
- 1. 開啟Cura
    ```
    cura
    ```
- 2. 放入要列印的STL
- 3. 按下Slice開始切片
- 4. 存檔並等待G-code生成
- 5. 開啟3D列印機
- 6. Prepare->Auto home->Home ALL
- 7. Print from SD
- 8. 開始列印
## About Directories
### CAD
我們更改過後的Moveo，變更內容如下:
- 1. 加上Titan擠出軸以及噴頭組
- 2. 移除木板，新增旋轉台底座
- 3. 變更3M2和4M(加上齒輪組)
- 4. 增進4M1D與3M2之間的穩定度
- 5. 在機構內部新增走線空間

原始版本:https://github.com/BCN3D/BCN3D-Moveo
### Cura
我們更改過後的Cura，變更內容如下:
- 1. 新增列印機型 SPIE Joint Lab Moveo
- 2. 在機型為Moveo時，改變空間界線判斷方式
- 3. 在機型為Moveo時，連接ROS轉換座標與機械軸位置

原始版本:https://github.com/Ultimaker/Cura/wiki/Running-Cura-from-Source-on-Ubuntu 
### Marlin
我們更改過後的Marlin，變更內容如下:
- 1. 變更Mother Board Arduino腳位
- 2. 馬達歸位方式
- 3. 平面測量方式
- 4. Motion Planner規劃方式

原始版本:https://github.com/MarlinFirmware/Marlin
### circuit
- 1. Arduino 3D印表機擴充版電路圖及PCB layout\
     原始版本:https://github.com/MarlinFirmware/Marlin

- 2. 馬達閉迴路控制板電路圖及PCB layout\
     原始版本:https://github.com/uStepper/uStepper1

### gcode_translation
將原始G-code的笛卡爾座標轉換成機械軸

### moveo_moveit_move_group
ROS中用來測試Moveo動作、列印範圍及IK解答效能

### moveo_moveit_config
使用MoveIt Setup Assistant設定Moveo的參數檔(如:各軸活動極限、虛擬機械軸、規劃運動群組...等)，此參數檔用於MoveIt
### moveo_urdf
由SolidWorks外掛模組SW2URDF所生成的Moveo的URDF(Unified Robot Description File)
### progressbar
在Cura切片時產生當前進度條
### virtual_shift
讀取G-code中機械軸的位置，利用Rviz模擬Moveo列印情況
## Troubleshooting
- 