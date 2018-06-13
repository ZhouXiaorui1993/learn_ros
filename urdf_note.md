#urdf

##link标签的属性

- <inertial>:定义惯性
- <visual>:定义一个link的外观，大小，颜色和材质纹理图

    - <geometry>：定义了一个几何形状，三个参数分别是长宽高，单位是米。选项：<box>立方体，<cylinder>圆柱，<sphere>球体。
    - <material>：指定颜色rgb和透明度a。
- <collision>:定义碰撞检测属性

