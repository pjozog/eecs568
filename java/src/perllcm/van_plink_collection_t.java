/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package perllcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class van_plink_collection_t implements lcm.lcm.LCMEncodable
{
    public int nlinks;
    public perllcm.van_plink_t plink[];
 
    public van_plink_collection_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x23387c7955f0e8a0L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(perllcm.van_plink_collection_t.class))
            return 0L;
 
        classes.add(perllcm.van_plink_collection_t.class);
        long hash = LCM_FINGERPRINT_BASE
             + perllcm.van_plink_t._hashRecursive(classes)
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
        outs.writeInt(this.nlinks); 
 
        for (int a = 0; a < this.nlinks; a++) {
            this.plink[a]._encodeRecursive(outs); 
        }
 
    }
 
    public van_plink_collection_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public van_plink_collection_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static perllcm.van_plink_collection_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        perllcm.van_plink_collection_t o = new perllcm.van_plink_collection_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.nlinks = ins.readInt();
 
        this.plink = new perllcm.van_plink_t[(int) nlinks];
        for (int a = 0; a < this.nlinks; a++) {
            this.plink[a] = perllcm.van_plink_t._decodeRecursiveFactory(ins);
        }
 
    }
 
    public perllcm.van_plink_collection_t copy()
    {
        perllcm.van_plink_collection_t outobj = new perllcm.van_plink_collection_t();
        outobj.nlinks = this.nlinks;
 
        outobj.plink = new perllcm.van_plink_t[(int) nlinks];
        for (int a = 0; a < this.nlinks; a++) {
            outobj.plink[a] = this.plink[a].copy();
        }
 
        return outobj;
    }
 
}

