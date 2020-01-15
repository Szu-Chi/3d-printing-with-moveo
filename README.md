# 3D printing with Moveo
### 將BCN3D-Moveo結合ROS、Marlin以及 Ultimaker Cura，產生新型態的3D列印機。
我們變更3D列印切片軟體Cura的列印機模型與列印空間，並將Cura與ROS中的逆運動學求解器(IK-Slover)整合，把Cura生成的笛卡爾座標系轉換成更機械軸的轉動角度，最終透過Marlin控制BCN3D-Moveo移動噴嘴進行3D列印
### 系統架構 System Structure:
![image](https://github.com/Szu-Chi/3d-printing-with-moveo/blob/Feature_Position_Control/system_structure.png)


## How to Use:
### 環境準備



### 安裝 Cura SIPE version
![image](https://github.com/Szu-Chi/3d-printing-with-moveo/blob/Feature_Position_Control/moveo.gif)
1. 

### 燒錄Marlin

### 開始列印

## About Directories
### CAD

### Cura

### Marlin

### circuit

### gcode_translation

### moveo_moveit

### moveo_moveit_config

### moveo_urdf

### progressbar

### progressbar

## Troubleshooting
- 