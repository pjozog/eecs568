/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package mrlcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class robot_status_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public int state;
    public long faults;
 
    public robot_status_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x737050846e567c5dL;
 
    public static final int UNDEFINED = 0;
    public static final int OVERRIDE = 1;
    public static final int STOPPED = 2;
    public static final int STANDBY = 3;
    public static final int ACTIVE = 4;
    public static final int ERROR = 5;

    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(mrlcm.robot_status_t.class))
            return 0L;
 
        classes.add(mrlcm.robot_status_t.class);
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
 
        outs.writeInt(this.state); 
 
        outs.writeLong(this.faults); 
 
    }
 
    public robot_status_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public robot_status_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static mrlcm.robot_status_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        mrlcm.robot_status_t o = new mrlcm.robot_status_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.state = ins.readInt();
 
        this.faults = ins.readLong();
 
    }
 
    public mrlcm.robot_status_t copy()
    {
        mrlcm.robot_status_t outobj = new mrlcm.robot_status_t();
        outobj.utime = this.utime;
 
        outobj.state = this.state;
 
        outobj.faults = this.faults;
 
        return outobj;
    }
 
}
