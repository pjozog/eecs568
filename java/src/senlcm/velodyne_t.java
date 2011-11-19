/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package senlcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class velodyne_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public byte packet_type;
    public int datalen;
    public byte data[];
 
    public velodyne_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x8bcd6874fbc279bcL;
 
    public static final byte TYPE_DATA_PACKET = (byte) 1;
    public static final byte TYPE_POSITION_PACKET = (byte) 2;

    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(senlcm.velodyne_t.class))
            return 0L;
 
        classes.add(senlcm.velodyne_t.class);
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
 
        outs.writeByte(this.packet_type); 
 
        outs.writeInt(this.datalen); 
 
        if (this.datalen > 0)
            outs.write(this.data, 0, datalen);
 
    }
 
    public velodyne_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public velodyne_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static senlcm.velodyne_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        senlcm.velodyne_t o = new senlcm.velodyne_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.packet_type = ins.readByte();
 
        this.datalen = ins.readInt();
 
        this.data = new byte[(int) datalen];
        ins.readFully(this.data, 0, datalen); 
    }
 
    public senlcm.velodyne_t copy()
    {
        senlcm.velodyne_t outobj = new senlcm.velodyne_t();
        outobj.utime = this.utime;
 
        outobj.packet_type = this.packet_type;
 
        outobj.datalen = this.datalen;
 
        outobj.data = new byte[(int) datalen];
        if (this.datalen > 0)
            System.arraycopy(this.data, 0, outobj.data, 0, this.datalen); 
        return outobj;
    }
 
}
