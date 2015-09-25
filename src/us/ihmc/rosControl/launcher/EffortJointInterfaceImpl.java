package us.ihmc.rosControl.launcher;

import java.nio.ByteBuffer;

class EffortJointInterfaceImpl implements EffortJointInterface, NativeUpdateableInterface
{
   private final String jointName;
   
   private double effort;
   private double position;
   private double velocity;
   
   private double desiredEffort;
   
   
   public EffortJointInterfaceImpl(String jointName)
   {
      this.jointName = jointName;
   }

   public void readFromBuffer(ByteBuffer buffer)
   {
      effort = buffer.getDouble();
      position = buffer.getDouble();
      velocity = buffer.getDouble();
   }
   
   public void writeToBuffer(ByteBuffer buffer)
   {
      buffer.putDouble(desiredEffort);
   }
   
   
   @Override
   public double getEffort()
   {
      return effort;
   }

   @Override
   public double getPosition()
   {
      return position;
   }

   @Override
   public double getVelocity()
   {
      return velocity;
   }

   @Override
   public void setDesiredEffort(double desiredEffort)
   {
      this.desiredEffort = desiredEffort;
   }

   @Override
   public String getName()
   {
      return jointName;
   }
   
}
