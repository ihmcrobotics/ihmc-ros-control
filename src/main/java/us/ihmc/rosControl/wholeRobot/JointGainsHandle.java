package us.ihmc.rosControl.wholeRobot;

public interface JointGainsHandle {
    public String getName();

    public void setDamping(double desiredDamping);
    public void setStiffness(double desiredStiffness);
    public void setPosition(double desiredPosition);
    public void setVelocity(double desiredVelocity);
}
