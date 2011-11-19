/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package se;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class option_t implements lcm.lcm.LCMEncodable
{
    public int mode;
    public String savepath;
    public String graphfile;
    public boolean load_done;
    public long utime_conn;
    public double cov_conn;
 
    public option_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xac8942ed88ecc09dL;
 
    public static final int MODE_WAIT = 0;
    public static final int MODE_SAVE = 1;
    public static final int MODE_LOAD = 2;
    public static final int MODE_START = 3;
    public static final int MODE_BATCH = 4;

    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(se.option_t.class))
            return 0L;
 
        classes.add(se.option_t.class);
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
        outs.writeInt(this.mode); 
 
        __strbuf = new char[this.savepath.length()]; this.savepath.getChars(0, this.savepath.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0); 
 
        __strbuf = new char[this.graphfile.length()]; this.graphfile.getChars(0, this.graphfile.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0); 
 
        outs.writeByte( this.load_done ? 1 : 0); 
 
        outs.writeLong(this.utime_conn); 
 
        outs.writeDouble(this.cov_conn); 
 
    }
 
    public option_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public option_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static se.option_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        se.option_t o = new se.option_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        char[] __strbuf = null;
        this.mode = ins.readInt();
 
        __strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); this.savepath = new String(__strbuf);
 
        __strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); this.graphfile = new String(__strbuf);
 
        this.load_done = ins.readByte()!=0;
 
        this.utime_conn = ins.readLong();
 
        this.cov_conn = ins.readDouble();
 
    }
 
    public se.option_t copy()
    {
        se.option_t outobj = new se.option_t();
        outobj.mode = this.mode;
 
        outobj.savepath = this.savepath;
 
        outobj.graphfile = this.graphfile;
 
        outobj.load_done = this.load_done;
 
        outobj.utime_conn = this.utime_conn;
 
        outobj.cov_conn = this.cov_conn;
 
        return outobj;
    }
 
}
