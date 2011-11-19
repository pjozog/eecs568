/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package perllcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class van_feature_user_depth_t implements lcm.lcm.LCMEncodable
{
    public int npts;
    public float mu_Z[];
    public float Sigma_Z[];
 
    public van_feature_user_depth_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x377665798924fe99L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(perllcm.van_feature_user_depth_t.class))
            return 0L;
 
        classes.add(perllcm.van_feature_user_depth_t.class);
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
        outs.writeInt(this.npts); 
 
        for (int a = 0; a < this.npts; a++) {
            outs.writeFloat(this.mu_Z[a]); 
        }
 
        for (int a = 0; a < this.npts; a++) {
            outs.writeFloat(this.Sigma_Z[a]); 
        }
 
    }
 
    public van_feature_user_depth_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public van_feature_user_depth_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static perllcm.van_feature_user_depth_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        perllcm.van_feature_user_depth_t o = new perllcm.van_feature_user_depth_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.npts = ins.readInt();
 
        this.mu_Z = new float[(int) npts];
        for (int a = 0; a < this.npts; a++) {
            this.mu_Z[a] = ins.readFloat();
        }
 
        this.Sigma_Z = new float[(int) npts];
        for (int a = 0; a < this.npts; a++) {
            this.Sigma_Z[a] = ins.readFloat();
        }
 
    }
 
    public perllcm.van_feature_user_depth_t copy()
    {
        perllcm.van_feature_user_depth_t outobj = new perllcm.van_feature_user_depth_t();
        outobj.npts = this.npts;
 
        outobj.mu_Z = new float[(int) npts];
        if (this.npts > 0)
            System.arraycopy(this.mu_Z, 0, outobj.mu_Z, 0, this.npts); 
        outobj.Sigma_Z = new float[(int) npts];
        if (this.npts > 0)
            System.arraycopy(this.Sigma_Z, 0, outobj.Sigma_Z, 0, this.npts); 
        return outobj;
    }
 
}
