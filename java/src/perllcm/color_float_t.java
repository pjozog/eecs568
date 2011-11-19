/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package perllcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class color_float_t implements lcm.lcm.LCMEncodable
{
    public float r;
    public float g;
    public float b;
 
    public color_float_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xf914f112c253de39L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(perllcm.color_float_t.class))
            return 0L;
 
        classes.add(perllcm.color_float_t.class);
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
        outs.writeFloat(this.r); 
 
        outs.writeFloat(this.g); 
 
        outs.writeFloat(this.b); 
 
    }
 
    public color_float_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public color_float_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static perllcm.color_float_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        perllcm.color_float_t o = new perllcm.color_float_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.r = ins.readFloat();
 
        this.g = ins.readFloat();
 
        this.b = ins.readFloat();
 
    }
 
    public perllcm.color_float_t copy()
    {
        perllcm.color_float_t outobj = new perllcm.color_float_t();
        outobj.r = this.r;
 
        outobj.g = this.g;
 
        outobj.b = this.b;
 
        return outobj;
    }
 
}
