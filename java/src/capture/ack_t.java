/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package capture;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class ack_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public byte xmit_second;
    public byte route_id;
    public byte route_size;
    public byte route[];
    public boolean xmitter_in_charge;
    public byte resource_origin;
    public int resource_id;
    public short rx_count;
    public byte masks[];
 
    public ack_t()
    {
        masks = new byte[56];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x287d5a8f60416b5eL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(capture.ack_t.class))
            return 0L;
 
        classes.add(capture.ack_t.class);
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
 
        outs.writeByte(this.xmit_second); 
 
        outs.writeByte(this.route_id); 
 
        outs.writeByte(this.route_size); 
 
        if (this.route_size > 0)
            outs.write(this.route, 0, route_size);
 
        outs.writeByte( this.xmitter_in_charge ? 1 : 0); 
 
        outs.writeByte(this.resource_origin); 
 
        outs.writeInt(this.resource_id); 
 
        outs.writeShort(this.rx_count); 
 
        outs.write(this.masks, 0, 56);
 
    }
 
    public ack_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public ack_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static capture.ack_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        capture.ack_t o = new capture.ack_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.xmit_second = ins.readByte();
 
        this.route_id = ins.readByte();
 
        this.route_size = ins.readByte();
 
        this.route = new byte[(int) route_size];
        ins.readFully(this.route, 0, route_size); 
        this.xmitter_in_charge = ins.readByte()!=0;
 
        this.resource_origin = ins.readByte();
 
        this.resource_id = ins.readInt();
 
        this.rx_count = ins.readShort();
 
        this.masks = new byte[(int) 56];
        ins.readFully(this.masks, 0, 56); 
    }
 
    public capture.ack_t copy()
    {
        capture.ack_t outobj = new capture.ack_t();
        outobj.utime = this.utime;
 
        outobj.xmit_second = this.xmit_second;
 
        outobj.route_id = this.route_id;
 
        outobj.route_size = this.route_size;
 
        outobj.route = new byte[(int) route_size];
        if (this.route_size > 0)
            System.arraycopy(this.route, 0, outobj.route, 0, this.route_size); 
        outobj.xmitter_in_charge = this.xmitter_in_charge;
 
        outobj.resource_origin = this.resource_origin;
 
        outobj.resource_id = this.resource_id;
 
        outobj.rx_count = this.rx_count;
 
        outobj.masks = new byte[(int) 56];
        System.arraycopy(this.masks, 0, outobj.masks, 0, 56); 
        return outobj;
    }
 
}

