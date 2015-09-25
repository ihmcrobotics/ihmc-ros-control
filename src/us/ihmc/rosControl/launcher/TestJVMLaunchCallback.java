package us.ihmc.rosControl.launcher;

public class TestJVMLaunchCallback
{
   public native void callVoidFunctionWithString(String string);
   public native int callIntFunctionWithBoolean(boolean a, boolean b);
   
   public static void execute()
   {
      
      try
      {
         TestJVMLaunchCallback testJVMLaunchCallback = new TestJVMLaunchCallback();
         testJVMLaunchCallback.callVoidFunctionWithString("Hello from java");
         System.out.println("Integer return: " + testJVMLaunchCallback.callIntFunctionWithBoolean(true, false));
      }
      catch(Throwable e)
      {
         e.printStackTrace();
      }
      
      
   }
}
