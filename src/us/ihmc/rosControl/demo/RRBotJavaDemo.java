package us.ihmc.rosControl.demo;

import us.ihmc.rosControl.IHMCRosControlJavaBridge;
import us.ihmc.rosControl.EffortJointHandle;

public class RRBotJavaDemo extends IHMCRosControlJavaBridge
{

   private EffortJointHandle joint1;
   private EffortJointHandle joint2;
   
   private double qDesiredJoint1 = -1.0;
   private double qDesiredJoint2 = 0.0;
   
   private double joint2Direction = 1.0;
   
   @Override
   protected void init()
   {
      joint1 = createEffortJointHandle("joint1");
      joint2 = createEffortJointHandle("joint2");

      System.out.println("Finish loading RRBot controller");
   }

   @Override
   protected void doControl(long time, long duration)
   {
      double tau1 = 10000 * (qDesiredJoint1 - joint1.getPosition()) + 10 * (0 - joint1.getVelocity());
      double tau2 = 100 * (qDesiredJoint2 - joint2.getPosition()) + 10 * (0 - joint2.getVelocity());
      
      qDesiredJoint2 += joint2Direction * 0.001;
      
      if(qDesiredJoint2 > 1.0)
      {
         joint2Direction = -1.0;
      }
      else if (qDesiredJoint2 < -1.0)
      {
         joint2Direction = 1.0;
      }
      
      joint1.setDesiredEffort(tau1);
      joint2.setDesiredEffort(tau2);
   }

}
