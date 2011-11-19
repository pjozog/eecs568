/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package senlcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class prosilica_attribute_t implements lcm.lcm.LCMEncodable
{
    public String label;
    public String value;
 
    public prosilica_attribute_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x12204fd2578d0266L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(senlcm.prosilica_attribute_t.class))
            return 0L;
 
        classes.add(senlcm.prosilica_attribute_t.class);
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
        char[] __strbuf = null;
        __strbuf = new char[this.label.length()]; this.label.getChars(0, this.label.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0); 
 
        __strbuf = new char[this.value.length()]; this.value.getChars(0, this.value.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0); 
 
    }
 
    public prosilica_attribute_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public prosilica_attribute_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static senlcm.prosilica_attribute_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        senlcm.prosilica_attribute_t o = new senlcm.prosilica_attribute_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        char[] __strbuf = null;
        __strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); this.label = new String(__strbuf);
 
        __strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); this.value = new String(__strbuf);
 
    }
 
    public senlcm.prosilica_attribute_t copy()
    {
        senlcm.prosilica_attribute_t outobj = new senlcm.prosilica_attribute_t();
        outobj.label = this.label;
 
        outobj.value = this.value;
 
        return outobj;
    }
 
}
