package us.ihmc.rosControl.wholeRobot;

public interface ForceTorqueSensorHandle
{

   double getTz();

   double getTy();

   double getTx();

   double getFz();

   double getFy();

   double getFx();

   String getName();

}
