/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package vs;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class obj_t implements lcm.lcm.LCMEncodable
{
    public long id;
    public double x;
    public double y;
    public double z;
    public double yaw;
    public double pitch;
    public double roll;
 
    public obj_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x8b9b80be1f1185d3L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(vs.obj_t.class))
            return 0L;
 
        classes.add(vs.obj_t.class);
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
        outs.writeLong(this.id); 
 
        outs.writeDouble(this.x); 
 
        outs.writeDouble(this.y); 
 
        outs.writeDouble(this.z); 
 
        outs.writeDouble(this.yaw); 
 
        outs.writeDouble(this.pitch); 
 
        outs.writeDouble(this.roll); 
 
    }
 
    public obj_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public obj_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static vs.obj_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        vs.obj_t o = new vs.obj_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.id = ins.readLong();
 
        this.x = ins.readDouble();
 
        this.y = ins.readDouble();
 
        this.z = ins.readDouble();
 
        this.yaw = ins.readDouble();
 
        this.pitch = ins.readDouble();
 
        this.roll = ins.readDouble();
 
    }
 
    public vs.obj_t copy()
    {
        vs.obj_t outobj = new vs.obj_t();
        outobj.id = this.id;
 
        outobj.x = this.x;
 
        outobj.y = this.y;
 
        outobj.z = this.z;
 
        outobj.yaw = this.yaw;
 
        outobj.pitch = this.pitch;
 
        outobj.roll = this.roll;
 
        return outobj;
    }
 
}
