package us.ihmc.rosControl;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

public abstract class IHMCRosControlJavaBridge
{
   private ByteBuffer readBuffer;
   private ByteBuffer writeBuffer;
   
   private final native boolean addJointToBufferN(long thisPtr, String jointName);
   
   private final native static void rosError(String error);
   private final native static void rosInfo(String msg);
   
   private final native ByteBuffer createReadBuffer(long thisPtr);
   private final native ByteBuffer createWriteBuffer(long thisPtr);
   
   private boolean inInit = false;
   private long thisPtr;
   private long delegatePtr;

   
   private final ArrayList<NativeUpdateableInterface> updatables = new ArrayList<>();
   
   
   protected IHMCRosControlJavaBridge()
   {
      rosInfo("Binding System.out to ROS_INFO\n");
      System.setOut(new PrintStream(new RosInfoOutputStream()));
      rosError("Binding system.err to ROS_ERROR\n");
      System.setErr(new PrintStream(new RosErrorOutputStream()));
   }
   
   protected boolean inInit()
   {
      return inInit;
   }
   
   protected long getDelegatePtr()
   {
      return delegatePtr;
   }
   
   protected void addUpdatable(NativeUpdateableInterface updatable)
   {
      if(!inInit)
      {
         throw new RuntimeException("Cannot call from outside init()");
      }
      
      updatables.add(updatable);
   }
   
   /**
    * Return a new JointHandle. Call from init()
    * 
    * @param jointName
    * @return
    */
   protected final JointHandle createJointHandle(String jointName)
   {
      if(!inInit)
      {
         throw new RuntimeException("createJointHandle should only be called from init()");
      }
      if(!addJointToBufferN(thisPtr, jointName))
      {
         throw new IllegalArgumentException("Cannot find joint with name " + jointName);
      }
      JointHandleImpl jointHandle = new JointHandleImpl(jointName);
      updatables.add(jointHandle);
      return jointHandle;
   }
   
   
   boolean initFromNative(long thisPtr, long delegatePtr)
   {
      try
      {
         this.thisPtr = thisPtr;
         this.delegatePtr = delegatePtr;
         inInit = true;
         init();
         inInit = false;
         
         readBuffer = createReadBuffer(thisPtr);
         writeBuffer = createWriteBuffer(thisPtr);
         
         readBuffer.order(ByteOrder.nativeOrder());
         writeBuffer.order(ByteOrder.nativeOrder());
         
         return true;
      }
      catch(Throwable e)
      {
         e.printStackTrace();
         return false;
      }
   }
   
   void updateFromNative(long time, long duration)
   {
      try
      {
         readBuffer.clear();
         for(int i = 0; i < updatables.size(); i++)
         {
            updatables.get(i).readFromBuffer(readBuffer);
         }
         
         doControl(time, duration);
         
         writeBuffer.clear();
         for(int i = 0; i < updatables.size(); i++)
         {
            updatables.get(i).writeToBuffer(writeBuffer);
         }
      }
      catch(Throwable e)
      {
         e.printStackTrace();
      }
   }
   
   
   /**
    * This function gets called when the controller gets initialized. Use this function to setup
    * the effort interfaces.
    * 
    * @see createEffortJointInterface
    */
   protected abstract void init();
   
   /**
    * This method gets called cyclically in the update.
    */
   protected abstract void doControl(long time, long duration);
   
   
   private static class RosInfoOutputStream extends ByteArrayOutputStream
   {  
      @Override
      public synchronized void flush()
      {
         rosInfo(toString());
         reset();
      }
      
   }
   private static class RosErrorOutputStream extends ByteArrayOutputStream
   {  
      @Override
      public synchronized void flush()
      {
         rosError(toString());
         reset();
      }
      
   }
}
