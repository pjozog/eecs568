/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package vs;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class pallet_slot_pair_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public vs.pallet_slot_t left_slot;
    public vs.pallet_slot_t right_slot;
    public double confidence;
 
    public pallet_slot_pair_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xd8e22778d1df589eL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(vs.pallet_slot_pair_t.class))
            return 0L;
 
        classes.add(vs.pallet_slot_pair_t.class);
        long hash = LCM_FINGERPRINT_BASE
             + vs.pallet_slot_t._hashRecursive(classes)
             + vs.pallet_slot_t._hashRecursive(classes)
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
 
        this.left_slot._encodeRecursive(outs); 
 
        this.right_slot._encodeRecursive(outs); 
 
        outs.writeDouble(this.confidence); 
 
    }
 
    public pallet_slot_pair_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public pallet_slot_pair_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static vs.pallet_slot_pair_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        vs.pallet_slot_pair_t o = new vs.pallet_slot_pair_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.left_slot = vs.pallet_slot_t._decodeRecursiveFactory(ins);
 
        this.right_slot = vs.pallet_slot_t._decodeRecursiveFactory(ins);
 
        this.confidence = ins.readDouble();
 
    }
 
    public vs.pallet_slot_pair_t copy()
    {
        vs.pallet_slot_pair_t outobj = new vs.pallet_slot_pair_t();
        outobj.utime = this.utime;
 
        outobj.left_slot = this.left_slot.copy();
 
        outobj.right_slot = this.right_slot.copy();
 
        outobj.confidence = this.confidence;
 
        return outobj;
    }
 
}

