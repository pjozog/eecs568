/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package hauv;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class bs_dvl_2_t implements lcm.lcm.LCMEncodable
{
    public long time_received;
    public long time;
    public double velocity1;
    public double velocity2;
    public double velocity3;
    public double velocity4;
    public double range1;
    public double range2;
    public double range3;
    public double range4;
    public double temperature;
    public long time_measured;
 
    public bs_dvl_2_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x7b1c3f8e759facfbL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(hauv.bs_dvl_2_t.class))
            return 0L;
 
        classes.add(hauv.bs_dvl_2_t.class);
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
        outs.writeLong(this.time_received); 
 
        outs.writeLong(this.time); 
 
        outs.writeDouble(this.velocity1); 
 
        outs.writeDouble(this.velocity2); 
 
        outs.writeDouble(this.velocity3); 
 
        outs.writeDouble(this.velocity4); 
 
        outs.writeDouble(this.range1); 
 
        outs.writeDouble(this.range2); 
 
        outs.writeDouble(this.range3); 
 
        outs.writeDouble(this.range4); 
 
        outs.writeDouble(this.temperature); 
 
        outs.writeLong(this.time_measured); 
 
    }
 
    public bs_dvl_2_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public bs_dvl_2_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static hauv.bs_dvl_2_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        hauv.bs_dvl_2_t o = new hauv.bs_dvl_2_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.time_received = ins.readLong();
 
        this.time = ins.readLong();
 
        this.velocity1 = ins.readDouble();
 
        this.velocity2 = ins.readDouble();
 
        this.velocity3 = ins.readDouble();
 
        this.velocity4 = ins.readDouble();
 
        this.range1 = ins.readDouble();
 
        this.range2 = ins.readDouble();
 
        this.range3 = ins.readDouble();
 
        this.range4 = ins.readDouble();
 
        this.temperature = ins.readDouble();
 
        this.time_measured = ins.readLong();
 
    }
 
    public hauv.bs_dvl_2_t copy()
    {
        hauv.bs_dvl_2_t outobj = new hauv.bs_dvl_2_t();
        outobj.time_received = this.time_received;
 
        outobj.time = this.time;
 
        outobj.velocity1 = this.velocity1;
 
        outobj.velocity2 = this.velocity2;
 
        outobj.velocity3 = this.velocity3;
 
        outobj.velocity4 = this.velocity4;
 
        outobj.range1 = this.range1;
 
        outobj.range2 = this.range2;
 
        outobj.range3 = this.range3;
 
        outobj.range4 = this.range4;
 
        outobj.temperature = this.temperature;
 
        outobj.time_measured = this.time_measured;
 
        return outobj;
    }
 
}
