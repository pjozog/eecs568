/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package perllcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class van_plink_t implements lcm.lcm.LCMEncodable
{
    public long utime_i;
    public long utime_j;
    public boolean prior;
    public perllcm.pose3d_t x_ji;
    public int link_id;
    public double Ig;
    public double S_L;
 
    public van_plink_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x95adcb2e0ef49428L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(perllcm.van_plink_t.class))
            return 0L;
 
        classes.add(perllcm.van_plink_t.class);
        long hash = LCM_FINGERPRINT_BASE
             + perllcm.pose3d_t._hashRecursive(classes)
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
        outs.writeLong(this.utime_i); 
 
        outs.writeLong(this.utime_j); 
 
        outs.writeByte( this.prior ? 1 : 0); 
 
        this.x_ji._encodeRecursive(outs); 
 
        outs.writeInt(this.link_id); 
 
        outs.writeDouble(this.Ig); 
 
        outs.writeDouble(this.S_L); 
 
    }
 
    public van_plink_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public van_plink_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static perllcm.van_plink_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        perllcm.van_plink_t o = new perllcm.van_plink_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime_i = ins.readLong();
 
        this.utime_j = ins.readLong();
 
        this.prior = ins.readByte()!=0;
 
        this.x_ji = perllcm.pose3d_t._decodeRecursiveFactory(ins);
 
        this.link_id = ins.readInt();
 
        this.Ig = ins.readDouble();
 
        this.S_L = ins.readDouble();
 
    }
 
    public perllcm.van_plink_t copy()
    {
        perllcm.van_plink_t outobj = new perllcm.van_plink_t();
        outobj.utime_i = this.utime_i;
 
        outobj.utime_j = this.utime_j;
 
        outobj.prior = this.prior;
 
        outobj.x_ji = this.x_ji.copy();
 
        outobj.link_id = this.link_id;
 
        outobj.Ig = this.Ig;
 
        outobj.S_L = this.S_L;
 
        return outobj;
    }
 
}
