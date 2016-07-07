package us.ihmc.rosControl.wholeRobot;

/**
 * Created by dstephen on 7/6/16.
 */
public interface JointStateHandle
{
   public String getName();

   public double getEffort();
   public double getPosition();
   public double getVelocity();
}
