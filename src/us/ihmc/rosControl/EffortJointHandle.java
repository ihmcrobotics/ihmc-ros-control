package us.ihmc.rosControl;

public interface EffortJointHandle
{
   public String getName();
   
   public double getEffort();
   public double getPosition();
   public double getVelocity();
   
   public void setDesiredEffort(double desiredEffort);
}
