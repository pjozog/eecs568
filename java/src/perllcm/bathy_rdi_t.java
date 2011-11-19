/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package perllcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class bathy_rdi_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public float range[];
    public float xyz[][];
 
    public bathy_rdi_t()
    {
        range = new float[4];
        xyz = new float[4][3];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xb573265d6b32ac19L;
 
    public static final float RANGE_SENTINAL = 0f;

    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(perllcm.bathy_rdi_t.class))
            return 0L;
 
        classes.add(perllcm.bathy_rdi_t.class);
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
 
        for (int a = 0; a < 4; a++) {
            outs.writeFloat(this.range[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            for (int b = 0; b < 3; b++) {
                outs.writeFloat(this.xyz[a][b]); 
            }
        }
 
    }
 
    public bathy_rdi_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public bathy_rdi_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static perllcm.bathy_rdi_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        perllcm.bathy_rdi_t o = new perllcm.bathy_rdi_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.range = new float[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.range[a] = ins.readFloat();
        }
 
        this.xyz = new float[(int) 4][(int) 3];
        for (int a = 0; a < 4; a++) {
            for (int b = 0; b < 3; b++) {
                this.xyz[a][b] = ins.readFloat();
            }
        }
 
    }
 
    public perllcm.bathy_rdi_t copy()
    {
        perllcm.bathy_rdi_t outobj = new perllcm.bathy_rdi_t();
        outobj.utime = this.utime;
 
        outobj.range = new float[(int) 4];
        System.arraycopy(this.range, 0, outobj.range, 0, 4); 
        outobj.xyz = new float[(int) 4][(int) 3];
        for (int a = 0; a < 4; a++) {
            System.arraycopy(this.xyz[a], 0, outobj.xyz[a], 0, 3);        }
 
        return outobj;
    }
 
}
