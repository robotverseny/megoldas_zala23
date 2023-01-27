---
layout: default
title: Launch files
---

# Launch files

## `lidar_and_motor_bringup.launch`

A LIDAR és a motor indítása

## `tf_laser_base.launch`

Egyszerű statikus transzform a `laser` és a `base_link` között.

```xml
<launch>
  <node args="0.26 0.0 0.228 0.0 0.0 0.999999682932 0.000796326710733 base_link laser 50" name="laser_tf_publisher" pkg="tf" type="static_transform_publisher"/>  
</launch>
```
