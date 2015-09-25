package us.ihmc.rosControl.launcher;

interface EffortJointInterface
{
   public String getName();
   
   public double getEffort();
   public double getPosition();
   public double getVelocity();
   
   public void setDesiredEffort(double desiredEffort);
}
