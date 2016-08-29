package us.ihmc.rosControl.wholeRobot;

import java.nio.ByteBuffer;

import us.ihmc.rosControl.NativeUpdateableInterface;

class IMUHandleImpl implements IMUHandle, NativeUpdateableInterface
{
   private final String name;                       ///< The name of the sensor
   
   
   // Orientation
   private double q_x, q_y, q_z, q_w;
   
   
   
   /** 
    * A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z)
    */
   private final double[] orientationCovariance = new double[9];
   
   // Angular velocity
   private double theta_x, theta_y, theta_z;
   
   
   
   /** 
    * A pointer to the storage of the angular velocity covariance value: a row major 3x3 matrix about (x,y,z)
    */
   private final double[] angularVelocityCovariance = new double[9];
   
   // Acceleration
   private double xdd, ydd, zdd;
   
   /**
    * A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z)
    */
   private final double[] linearAccelerationCovariance = new double[9];


   
   public IMUHandleImpl(String name)
   {
      this.name = name;
   }
   
   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public double getQ_x()
   {
      return q_x;
   }

   @Override
   public double getQ_y()
   {
      return q_y;
   }

   @Override
   public double getQ_z()
   {
      return q_z;
   }

   @Override
   public double getQ_w()
   {
      return q_w;
   }

   @Override
   public void getOrientationCovariance(double[] orientationCovariance)
   {
      if(orientationCovariance.length != this.orientationCovariance.length)
      {
         throw new RuntimeException("Expecting 9 element array");
      }
      
      System.arraycopy(this.orientationCovariance, 0, orientationCovariance, 0, this.orientationCovariance.length);
   }

   @Override
   public double getTheta_x()
   {
      return theta_x;
   }

   @Override
   public double getTheta_y()
   {
      return theta_y;
   }

   @Override
   public double getTheta_z()
   {
      return theta_z;
   }

   @Override
   public void getAngularVelocityCovariance(double[] angularVelocityCovariance)
   {
      if(angularVelocityCovariance.length != this.angularVelocityCovariance.length)
      {
         throw new RuntimeException("Expecting 9 element array");
      }
      
      System.arraycopy(this.angularVelocityCovariance, 0, angularVelocityCovariance, 0, this.angularVelocityCovariance.length);

   }

   @Override
   public double getXdd()
   {
      return xdd;
   }

   @Override
   public double getYdd()
   {
      return ydd;
   }

   @Override
   public double getZdd()
   {
      return zdd;
   }

   @Override
   public void getLinearAccelerationCovariance(double[] linearAccelerationCovariance)
   {
      if(linearAccelerationCovariance.length != this.linearAccelerationCovariance.length)
      {
         throw new RuntimeException("Expecting 9 element array");
      }
      
      System.arraycopy(this.linearAccelerationCovariance, 0, linearAccelerationCovariance, 0, this.linearAccelerationCovariance.length);

   }

   @Override
   public void readFromBuffer(ByteBuffer buffer)
   {
      q_x = buffer.getDouble();
      q_y = buffer.getDouble();
      q_z = buffer.getDouble();
      q_w = buffer.getDouble();
      
      for(int i = 0; i < orientationCovariance.length; i++)
      {
         orientationCovariance[i] = buffer.getDouble();
      }
      
      theta_x = buffer.getDouble();
      theta_y = buffer.getDouble();
      theta_z = buffer.getDouble();
      
      
      for(int i = 0; i < angularVelocityCovariance.length; i++)
      {
         angularVelocityCovariance[i] = buffer.getDouble();
      }
      
      xdd = buffer.getDouble();
      ydd = buffer.getDouble();
      zdd = buffer.getDouble();
      
      
      
      for(int i = 0; i < linearAccelerationCovariance.length; i++)
      {
         linearAccelerationCovariance[i] = buffer.getDouble();
      }

   }

   @Override
   public void writeToBuffer(ByteBuffer buffer)
   {
      // Only has state
   }
   
   

}
