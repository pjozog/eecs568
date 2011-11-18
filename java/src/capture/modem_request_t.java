/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package capture;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class modem_request_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public byte src;
    public byte dest;
    public byte rate;
    public byte slot_type;
 
    public modem_request_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x80d2b7059697faa3L;
 
    public static final byte SLOT_DATA = (byte) 1;
    public static final byte SLOT_PING = (byte) 2;
    public static final byte SLOT_REMUS_LBL = (byte) 3;
    public static final byte SLOT_MINI = (byte) 4;

    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(capture.modem_request_t.class))
            return 0L;
 
        classes.add(capture.modem_request_t.class);
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
 
        outs.writeByte(this.src); 
 
        outs.writeByte(this.dest); 
 
        outs.writeByte(this.rate); 
 
        outs.writeByte(this.slot_type); 
 
    }
 
    public modem_request_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public modem_request_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static capture.modem_request_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        capture.modem_request_t o = new capture.modem_request_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.src = ins.readByte();
 
        this.dest = ins.readByte();
 
        this.rate = ins.readByte();
 
        this.slot_type = ins.readByte();
 
    }
 
    public capture.modem_request_t copy()
    {
        capture.modem_request_t outobj = new capture.modem_request_t();
        outobj.utime = this.utime;
 
        outobj.src = this.src;
 
        outobj.dest = this.dest;
 
        outobj.rate = this.rate;
 
        outobj.slot_type = this.slot_type;
 
        return outobj;
    }
 
}

