package us.ihmc.rosControl.launcher;

import java.nio.ByteBuffer;

interface NativeUpdateableInterface
{
   void readFromBuffer(ByteBuffer buffer);
   void writeToBuffer(ByteBuffer buffer);
}
