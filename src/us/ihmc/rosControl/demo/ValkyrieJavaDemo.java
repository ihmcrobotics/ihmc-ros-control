package us.ihmc.rosControl.demo;

import us.ihmc.rosControl.JointHandle;
import us.ihmc.rosControl.valkyrie.IHMCValkyrieControlJavaBridge;

public class ValkyrieJavaDemo extends IHMCValkyrieControlJavaBridge
{
   private final static String jointName = "leftKneePitch";
   private JointHandle joint;

   private double q;
   private double direction;
   
   private boolean firstTick = true;
   
   @Override
   protected void init()
   {
      joint = createJointHandle(jointName);
   }

   @Override
   protected void doControl(long time, long duration)
   {
      if(firstTick)
      {
         q = joint.getPosition();
         firstTick = false;
      }
      
      q += 0.001 * direction;
      if(q <= 0.0)
      {
         direction = 1.0;
      }
      else if(q > 1.0)
      {
         direction = -1.0;
      }
      
      double tau = 1000 * (q - joint.getPosition()) + 10 * (0 - joint.getVelocity());
      joint.setDesiredEffort(tau);

   }

}
