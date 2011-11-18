/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package bot_param;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class set_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public long server_id;
    public int sequence_number;
    public String key;
    public String value;
 
    public set_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x635fe2779e903d3cL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(bot_param.set_t.class))
            return 0L;
 
        classes.add(bot_param.set_t.class);
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
        outs.writeLong(this.utime); 
 
        outs.writeLong(this.server_id); 
 
        outs.writeInt(this.sequence_number); 
 
        __strbuf = new char[this.key.length()]; this.key.getChars(0, this.key.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0); 
 
        __strbuf = new char[this.value.length()]; this.value.getChars(0, this.value.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0); 
 
    }
 
    public set_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public set_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static bot_param.set_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        bot_param.set_t o = new bot_param.set_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        char[] __strbuf = null;
        this.utime = ins.readLong();
 
        this.server_id = ins.readLong();
 
        this.sequence_number = ins.readInt();
 
        __strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); this.key = new String(__strbuf);
 
        __strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); this.value = new String(__strbuf);
 
    }
 
    public bot_param.set_t copy()
    {
        bot_param.set_t outobj = new bot_param.set_t();
        outobj.utime = this.utime;
 
        outobj.server_id = this.server_id;
 
        outobj.sequence_number = this.sequence_number;
 
        outobj.key = this.key;
 
        outobj.value = this.value;
 
        return outobj;
    }
 
}

