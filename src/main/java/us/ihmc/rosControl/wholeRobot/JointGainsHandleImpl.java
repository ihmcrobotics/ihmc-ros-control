package us.ihmc.rosControl.wholeRobot;

import us.ihmc.rosControl.NativeUpdateableInterface;

import java.nio.ByteBuffer;

class JointGainsHandleImpl implements JointGainsHandle, NativeUpdateableInterface
{
   private final String jointName;

   private double stiffness;
   private double damping;

   JointGainsHandleImpl(String jointName)
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
   public void writeToBuffer(ByteBuffer buffer)
   {
      buffer.putDouble(this.stiffness);
      buffer.putDouble(this.damping);
   }
}
