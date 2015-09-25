package us.ihmc.rosControl.launcher;

public interface JointHandle
{
   public String getName();
   
   public double getEffort();
   public double getPosition();
   public double getVelocity();
   
   public void setDesiredEffort(double desiredEffort);
}
