package us.ihmc.rosControl.wholeRobot;

import us.ihmc.rosControl.NativeUpdateableInterface;

import java.nio.ByteBuffer;

/**
 * Created by dstephen on 7/6/16.
 */
class JointStateHandleImpl implements JointStateHandle, NativeUpdateableInterface
{
   private final String jointName;

   private double effort;
   private double position;
   private double velocity;

   JointStateHandleImpl(String jointName)
   {
      this.jointName = jointName;
   }

   public void readFromBuffer(ByteBuffer buffer)
   {
      effort = buffer.getDouble();
      position = buffer.getDouble();
      velocity = buffer.getDouble();
   }

   @Override
   public String getName()
   {
      return this.jointName;
   }

   @Override
   public double getEffort()
   {
      return this.effort;
   }

   @Override
   public double getPosition()
   {
      return this.position;
   }

   @Override
   public double getVelocity()
   {
      return this.velocity;
   }

   @Override
   public void writeToBuffer(ByteBuffer buffer)
   {
      //do nothing
   }
}
