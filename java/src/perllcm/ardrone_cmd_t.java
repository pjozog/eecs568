/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package perllcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class ardrone_cmd_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public boolean takeoff;
    public boolean hover;
    public boolean camera;
    public boolean emergency;
    public int change_speed;
 
    public ardrone_cmd_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x53c93cf2802d6956L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(perllcm.ardrone_cmd_t.class))
            return 0L;
 
        classes.add(perllcm.ardrone_cmd_t.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.utime); 
 
        outs.writeByte( this.takeoff ? 1 : 0); 
 
        outs.writeByte( this.hover ? 1 : 0); 
 
        outs.writeByte( this.camera ? 1 : 0); 
 
        outs.writeByte( this.emergency ? 1 : 0); 
 
        outs.writeInt(this.change_speed); 
 
    }
 
    public ardrone_cmd_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public ardrone_cmd_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static perllcm.ardrone_cmd_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        perllcm.ardrone_cmd_t o = new perllcm.ardrone_cmd_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.takeoff = ins.readByte()!=0;
 
        this.hover = ins.readByte()!=0;
 
        this.camera = ins.readByte()!=0;
 
        this.emergency = ins.readByte()!=0;
 
        this.change_speed = ins.readInt();
 
    }
 
    public perllcm.ardrone_cmd_t copy()
    {
        perllcm.ardrone_cmd_t outobj = new perllcm.ardrone_cmd_t();
        outobj.utime = this.utime;
 
        outobj.takeoff = this.takeoff;
 
        outobj.hover = this.hover;
 
        outobj.camera = this.camera;
 
        outobj.emergency = this.emergency;
 
        outobj.change_speed = this.change_speed;
 
        return outobj;
    }
 
}
