package us.ihmc.rosControl.wholeRobot;

public interface JointImpedanceHandle {
    public String getName();

    public void setDamping(double desiredDamping);
    public void setStiffness(double desiredStiffness);
    public void setPosition(double desiredPosition);
    public void setVelocity(double desiredVelocity);

    public double getDamping();
    public double getStiffness();
    public double getPosition();
    public double getVelocity();
}
