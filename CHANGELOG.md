## 0.5.0

Backwards incompatible change to library internals; the IMUHandle implementation now assumes the buffer is ordered in "xyzw"
layout instead of "wxyz" layout. This is to keep with the convention that Eigen uses with quaternions, given its popularity.

## 0.4.0

This is the first edition of the changelog. For a detailed overview of existing features, please check README.md

### Features:

- "Whole Robot" controllers can now access JointStateHandles in addition to the existing

### Breaking Changes:

- NativeJointHandleHolder now inherits from NativeJointStateHandleHolder to mimic the way that JointHandleHolders extend from JointStateHandles in ROS Control.

