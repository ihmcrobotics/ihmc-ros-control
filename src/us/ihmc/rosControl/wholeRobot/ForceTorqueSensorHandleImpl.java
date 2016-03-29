package us.ihmc.rosControl.wholeRobot;

import java.nio.ByteBuffer;

import us.ihmc.rosControl.NativeUpdateableInterface;

class ForceTorqueSensorHandleImpl implements ForceTorqueSensorHandle, NativeUpdateableInterface
{
   private final String name;
   private double fx, fy, fz;
   private double tx, ty, tz;
   
   public ForceTorqueSensorHandleImpl(String name)
   {
      this.name = name;
   }
   
   @Override
   public void readFromBuffer(ByteBuffer buffer)
   {
      fx = buffer.getDouble();
      fy = buffer.getDouble();
      fz = buffer.getDouble();
      
      tx = buffer.getDouble();
      ty = buffer.getDouble();
      tz = buffer.getDouble();
   }
   @Override
   public void writeToBuffer(ByteBuffer buffer)
   {
      // Only state
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public double getFx()
   {
      return fx;
   }

   @Override
   public double getFy()
   {
      return fy;
   }

   @Override
   public double getFz()
   {
      return fz;
   }

   @Override
   public double getTx()
   {
      return tx;
   }

   @Override
   public double getTy()
   {
      return ty;
   }

   @Override
   public double getTz()
   {
      return tz;
   }
   
   
}
