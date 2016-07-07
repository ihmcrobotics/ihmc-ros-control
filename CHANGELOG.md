## 0.4.0

This is the first edition of the changelog.

### Features:

- "Whole Robot" controllers can now access JointStateHandles

### Breaking Changes:

- NativeJointHandleHolder now inherits from NativeJointStateHandleHolder to mimic the way that JointHandleHolders extend from JointStateHandles in ROS Control.
