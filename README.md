# Computer Animation and Special Effects HW2

## Build on Microsoft Windows with Visual Studio 2017/2019

### Instruction

- Open ForwardKinematics.sln
- Build
- Executable will be in ./bin

## Build on other platforms and/or compilers

### :warning: **This method is not well-tested, so it may not work properly.**

### :warning: **Thus, you are expected to solve buggy or trivial problems yourself.**

### Some tested platforms (cmake 3.20):

- Ubuntu Groovy Gorilla (20.10) with GCC   10.2.0

### Prerequisite

- [Cmake](https://cmake.org) (version >= 3.14)
- Compiler (e.g. GCC)

### Instruction

- Run:
```bash=
cmake -S . -B build
cmake --build build --config Release --target install --parallel 8
```
- Executable will be in ./bin

### If you are building on Linux, you need one of these dependencies, usually `xorg-dev`

- `xorg-dev` (For X11)
- `libwayland-dev wayland-protocols extra-cmake-modules libxkbcommon-dev` (For Wayland)
- `libosmesa6-dev` (For OSMesa)

# 110550047 Animation HW2

## Introduction

This project is to practice Kinematics system to standard people bones and import a series of motions to simulate walking and shooting.
Concerning topics includes 3D-Rotations, Kinematics and Keyframing.

## Fundamentals

### Coordinates

Global coordinates refer to a fixed, absolute coordinate system that is consistent across an entire scene or project. It serves as a reference point for positioning objects relative to the world or scene origin. 

Local coordinates, on the other hand, are relative to individual objects or parenting system within a scene. They define an object's position, rotation, and scale relative to its own local coordinate system, which may be different from the global coordinate system.

### Rotations

#### Homogeneous Coordinates

$(\frac{x}{w}, \frac{y}{w}, \frac{z}{w}) = [x,y,z,w]\\(x,y,z)=[x,y,z,1]$

#### Euler Angle

$E = (x,y,z)$

#### Quaternions

$Q = (s,x,y,z)\ or\ (s,v)$

### Kinematics

Kinematics is the branch of physics that studies the motion of objects without considering the forces that cause that motion. It focuses on the position, velocity, and acceleration of objects, as well as their relationships and patterns of motion.

### Keyframing

Keyframing allows animators to control various properties such as position, scale, rotation, opacity, and more, over time. By adjusting the values and implementing interpolation of these properties at different keyframes, animators can create lifelike movements, transitions, and visual effects.

## Implementation

### Forward Kinematics

```c++
// Initialize
// BFS travel
while(!bones.isEmpty())
{
    bone = bones.pop();
    // Apply Kinematics
    Vi = vi*Length;
	R = RiRj...;
    Ti = R_{i+1} * V_{i+1} + T_{i-1};
    
    // Breadth-First
    bones.add(bone.siblings); 
}
```

#### Acclaim file

Contains skeletal animation data, including bone rotations, translations, and other parameters, captured from real-world motion. Acclaim files are widely used in fields such as animation, gaming, and biomechanics, serving as a standard format for storing and exchanging motion capture data between different software and hardware systems.

### Keyframing

```c#
double scaler = frame_old / frame_new;
foreach(bone in bones)
{
    leftframe = i * scaler;
    right = i * scaler;
    ratio = bone in [leftframe, rightframe];
	// Lerp Interpolation
    T = (1-ratio)*T[leftframe] + ratio * T[rightframe];
    // Slerp Interpolation
    R = R[leftframe].slerp(R[rightframe], ratio);
}
```

## Result and Discussion

### Forward Kinematics

![image-20230424104633023](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424104633023.png) ![image-20230424104557817](../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424104557817.png)<img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424104623449.png" alt="image-20230424104623449" style="zoom: 150%;" />

### Time Warping

+ 328 Frames
  <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424104719952.png" width = "300" height = "200" alt="image-20230424104719952" /> <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424104741182.png" width = "300" height = "200" alt="image-20230424104741182" /> <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424104749766.png" width = "300" height = "200" alt="image-20230424104749766" />
  <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105506727.png" width = "300" height = "200" alt="image-20230424105506727" /> <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105522916.png" width = "300" height = "200" alt="image-20230424105522916" /> <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105540910.png" width = "300" height = "200" alt="image-20230424105540910" />
+ 164 Frames
  <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424104853127.png" width = "300" height = "200" alt="image-20230424104853127" /><img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424104904586.png" width = "300" height = "200" alt="image-20230424104904586" /><img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424104913141.png" width = "300" height = "200" alt="image-20230424104913141" />
  <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105408295.png" width = "300" height = "200" alt="image-20230424105408295" /> <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105419652.png" width = "300" height = "200" alt="image-20230424105419652" /><img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105434182.png" width = "300" height = "200" alt="image-20230424105434182" />
+ 492 Frames
  <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105016247.png" width = "300" height = "200" alt="image-20230424105016247" /> <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105031839.png" width = "300" height = "200" alt="image-20230424105031839" /> <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105041845.png" width = "300" height = "200" alt="image-20230424105041845" />
  <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105259638.png" width = "300" height = "200" alt="image-20230424105259638" /> <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105308133.png" width = "300" height = "200" alt="image-20230424105308133" /> <img src="../../../../../../AppData/Roaming/Typora/typora-user-images/image-20230424105321216.png" width = "300" height = "200" alt="image-20230424105321216" />

### Rotation System

+ `util::rotateDegreeZYX`, commonly expressed by rotating X first, then rotating Y , end up rotating Z,  convert into Q = ZYX。
+ The [official document](http://eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b) shows `toRotationMatrix().eulerAngles(0, 1, 2)`, which convert into Q.M = XYZ.
+ `util::rotateDegree` and `toRotationMatrix().eulerAngles` order need to be same (Q=Q.M=XYZ or ZYX), which can be implemented with
  1. `util::rotateDegreeZYX` with `toRotationMatrix().eulerAngles(2, 1, 0)`
  2. `util::rotateDegreeXYZ` with `toRotationMatrix().eulerAngles(0, 1, 2)` 
+ It is more handy if we use Quaternions to represent any of motions. It would be a bit confusing when converting between Quaternions and Euler Angles. In addition, we need to construct a criterion to specify rotation matrix.

## Conclusion

In conclusion, the introduction of forward kinematics and keyframing techniques in animation and motion graphics opens up new possibilities for creating dynamic and realistic animations. Forward kinematics allows animators to create complex movements and articulation of joints, providing a realistic and natural motion for characters or objects. Keyframing, on the other hand, provides precise control over the timing and spacing of animations, allowing for smooth transitions, dynamic movements, and visually appealing effects.

The use of global and local coordinates further enhances the accuracy and flexibility of animations, enabling animators to position and transform objects in 3D space with precision. Global coordinates provide a consistent reference point for positioning objects in relation to the world or scene origin, while local coordinates allow for object-specific transformations relative to their own coordinate system. Understanding the distinction between global and local coordinates is crucial for maintaining proper spatial relationships in a scene.

Furthermore, the use of Acclaim files as a standard format for motion capture data facilitates the integration of realistic motion data into animations. This allows animators to capture real-world motion and apply it to virtual characters or objects, resulting in more authentic and lifelike animations.
