package us.ihmc.rosControl;

import java.nio.ByteBuffer;

public interface NativeUpdateableInterface
{
   void readFromBuffer(ByteBuffer buffer);
   void writeToBuffer(ByteBuffer buffer);
}
