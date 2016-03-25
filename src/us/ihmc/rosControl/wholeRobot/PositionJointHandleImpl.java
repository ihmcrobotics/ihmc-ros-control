package us.ihmc.rosControl.wholeRobot;

import us.ihmc.rosControl.NativeUpdateableInterface;

import java.nio.ByteBuffer;

class PositionJointHandleImpl implements PositionJointHandle, NativeUpdateableInterface
{
   private final String jointName;

   private double effort;
   private double position;
   private double velocity;

   private double desiredPosition;


   PositionJointHandleImpl(String jointName)
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
      buffer.putDouble(desiredPosition);
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
   public void setDesiredPosition(double desiredPosition)
   {
      this.desiredPosition = desiredPosition;
   }

   @Override
   public String getName()
   {
      return jointName;
   }
   
}
