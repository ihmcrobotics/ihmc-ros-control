package us.ihmc.rosControl.launcher;

import java.util.ArrayList;

public abstract class IHMCRosControlJavaBridge
{
   protected final native void addJointToBufferN(String jointName);

   
   private final ArrayList<NativeUpdateableInterface> updatables = new ArrayList<>();
   
   protected final EffortJointInterface createJointEffortInterface(String jointName)
   {
      addJointToBufferN(jointName);
      EffortJointInterfaceImpl effortJointInterface = new EffortJointInterfaceImpl(jointName);
      updatables.add(effortJointInterface);
      return effortJointInterface;
   }
   
}
