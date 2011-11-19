/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package hauv;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class bs_rnv_2_t implements lcm.lcm.LCMEncodable
{
    public long time_received;
    public long time;
    public double horizontal;
    public double vertical;
    public double distance;
    public double heading;
    public double depth;
    public double absheading;
    public double absroll;
    public double abspitch;
    public long time_nav;
 
    public bs_rnv_2_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x4e7e9db68f6943cbL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(hauv.bs_rnv_2_t.class))
            return 0L;
 
        classes.add(hauv.bs_rnv_2_t.class);
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
 
        outs.writeDouble(this.horizontal); 
 
        outs.writeDouble(this.vertical); 
 
        outs.writeDouble(this.distance); 
 
        outs.writeDouble(this.heading); 
 
        outs.writeDouble(this.depth); 
 
        outs.writeDouble(this.absheading); 
 
        outs.writeDouble(this.absroll); 
 
        outs.writeDouble(this.abspitch); 
 
        outs.writeLong(this.time_nav); 
 
    }
 
    public bs_rnv_2_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public bs_rnv_2_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static hauv.bs_rnv_2_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        hauv.bs_rnv_2_t o = new hauv.bs_rnv_2_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.time_received = ins.readLong();
 
        this.time = ins.readLong();
 
        this.horizontal = ins.readDouble();
 
        this.vertical = ins.readDouble();
 
        this.distance = ins.readDouble();
 
        this.heading = ins.readDouble();
 
        this.depth = ins.readDouble();
 
        this.absheading = ins.readDouble();
 
        this.absroll = ins.readDouble();
 
        this.abspitch = ins.readDouble();
 
        this.time_nav = ins.readLong();
 
    }
 
    public hauv.bs_rnv_2_t copy()
    {
        hauv.bs_rnv_2_t outobj = new hauv.bs_rnv_2_t();
        outobj.time_received = this.time_received;
 
        outobj.time = this.time;
 
        outobj.horizontal = this.horizontal;
 
        outobj.vertical = this.vertical;
 
        outobj.distance = this.distance;
 
        outobj.heading = this.heading;
 
        outobj.depth = this.depth;
 
        outobj.absheading = this.absheading;
 
        outobj.absroll = this.absroll;
 
        outobj.abspitch = this.abspitch;
 
        outobj.time_nav = this.time_nav;
 
        return outobj;
    }
 
}
