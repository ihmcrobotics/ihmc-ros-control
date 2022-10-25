package us.ihmc.rosControl.wholeRobot;

import us.ihmc.rosControl.NativeUpdateableInterface;

import java.nio.ByteBuffer;

class JointImpedanceHandleImpl implements JointImpedanceHandle, NativeUpdateableInterface
{
   private final String jointName;

   private double stiffness;
   private double damping;
   private double position;
   private double velocity;

   JointImpedanceHandleImpl(String jointName)
   {
      this.jointName = jointName;
   }

   public void readFromBuffer(ByteBuffer buffer)
   {
      // Nothing to do
   }

   @Override
   public String getName()
   {
      return this.jointName;
   }

   @Override
   public void setStiffness(double stiffness)
   {
      this.stiffness = stiffness;
   }

   @Override
   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   @Override
   public void setPosition(double position)
   {
      this.position = position;
   }

   @Override
   public void setVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   @Override
   public void writeToBuffer(ByteBuffer buffer)
   {
      buffer.putDouble(this.stiffness);
      buffer.putDouble(this.damping);
      buffer.putDouble(this.position);
      buffer.putDouble(this.velocity);
   }
}
