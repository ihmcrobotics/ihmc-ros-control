package us.ihmc.rosControl.wholeRobot;

public interface IMUHandle
{

   void getLinearAccelerationCovariance(double[] linearAccelerationCovariance);

   double getZdd();

   double getYdd();

   double getXdd();

   void getAngularVelocityCovariance(double[] angularVelocityCovariance);

   double getTheta_z();

   double getTheta_y();

   double getTheta_x();

   void getOrientationCovariance(double[] orientationCovariance);

   double getQ_w();

   double getQ_z();

   double getQ_y();

   double getQ_x();

   String getName();
   
}
