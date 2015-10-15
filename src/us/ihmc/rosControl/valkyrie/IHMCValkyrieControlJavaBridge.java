package us.ihmc.rosControl.valkyrie;

import us.ihmc.rosControl.IHMCRosControlJavaBridge;

public abstract class IHMCValkyrieControlJavaBridge extends IHMCRosControlJavaBridge
{
   private final native boolean addIMUToBufferN(long thisPtr, String imuName);
   private final native boolean addForceTorqueSensorToBufferN(long thisPtr, String forceTorqueSensorName);

   
   
   /**
    * Return a new IMUHandle. Call from init()
    * 
    * @param imuName
    * @return
    */
   protected final IMUHandle createIMUHandle(String imuName)
   {
      if(!inInit())
      {
         throw new RuntimeException("createIMUHandle should only be called from init()");
      }
      if(!addIMUToBufferN(getDelegatePtr(), imuName))
      {
         throw new IllegalArgumentException("Cannot find IMU with name " + imuName);
      }
      IMUHandleImpl imuHandle = new IMUHandleImpl(imuName);
      addUpdatable(imuHandle);
      return imuHandle;
   }

   /**
    * Return a new force torque sensor handle. Call from init()
    * 
    * @param forceTorqueSensorName
    * @return
    */
   protected final ForceTorqueSensorHandle createForceTorqueSensorHandle(String forceTorqueSensorName)
   {
      if(!inInit())
      {
         throw new RuntimeException("createForceTorqueSensorHandle should only be called from init()");
      }
      if(!addForceTorqueSensorToBufferN(getDelegatePtr(), forceTorqueSensorName))
      {
         throw new IllegalArgumentException("Cannot find force torque sensor with name " + forceTorqueSensorName);
      }
      ForceTorqueSensorHandleImpl imuHandle = new ForceTorqueSensorHandleImpl(forceTorqueSensorName);
      addUpdatable(imuHandle);
      return imuHandle;
   }
}
