package us.ihmc.rosControl;

/**
 * Created by parallels on 3/21/16.
 */
public interface PositionJointHandle
{
   public String getName();

   public double getEffort();
   public double getPosition();
   public double getVelocity();

   public void setDesiredPosition(double desiredEffort);
}
