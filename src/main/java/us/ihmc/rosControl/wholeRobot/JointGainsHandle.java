package us.ihmc.rosControl.wholeRobot;

public interface JointGainsHandle {
    public String getName();

    public void setDamping(double desiredDamping);
    public void setStiffness(double desiredStiffness);
}
