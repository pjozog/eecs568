/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package mrlcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class actuator_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public double thrust;
    public double rudder;
    public double planes;
 
    public actuator_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x869697604da855a8L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(mrlcm.actuator_t.class))
            return 0L;
 
        classes.add(mrlcm.actuator_t.class);
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
 
        outs.writeDouble(this.thrust); 
 
        outs.writeDouble(this.rudder); 
 
        outs.writeDouble(this.planes); 
 
    }
 
    public actuator_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public actuator_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static mrlcm.actuator_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        mrlcm.actuator_t o = new mrlcm.actuator_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.thrust = ins.readDouble();
 
        this.rudder = ins.readDouble();
 
        this.planes = ins.readDouble();
 
    }
 
    public mrlcm.actuator_t copy()
    {
        mrlcm.actuator_t outobj = new mrlcm.actuator_t();
        outobj.utime = this.utime;
 
        outobj.thrust = this.thrust;
 
        outobj.rudder = this.rudder;
 
        outobj.planes = this.planes;
 
        return outobj;
    }
 
}
