/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package senlcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class stats_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public int ntimeouts;
    public int ngood;
    public int nbad;
    public int dt;
    public int dtmin;
    public int dtmax;
    public int latency;
    public int latencymax;
 
    public stats_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xd104377b834709e1L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(senlcm.stats_t.class))
            return 0L;
 
        classes.add(senlcm.stats_t.class);
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
 
        outs.writeInt(this.ntimeouts); 
 
        outs.writeInt(this.ngood); 
 
        outs.writeInt(this.nbad); 
 
        outs.writeInt(this.dt); 
 
        outs.writeInt(this.dtmin); 
 
        outs.writeInt(this.dtmax); 
 
        outs.writeInt(this.latency); 
 
        outs.writeInt(this.latencymax); 
 
    }
 
    public stats_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public stats_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static senlcm.stats_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        senlcm.stats_t o = new senlcm.stats_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.ntimeouts = ins.readInt();
 
        this.ngood = ins.readInt();
 
        this.nbad = ins.readInt();
 
        this.dt = ins.readInt();
 
        this.dtmin = ins.readInt();
 
        this.dtmax = ins.readInt();
 
        this.latency = ins.readInt();
 
        this.latencymax = ins.readInt();
 
    }
 
    public senlcm.stats_t copy()
    {
        senlcm.stats_t outobj = new senlcm.stats_t();
        outobj.utime = this.utime;
 
        outobj.ntimeouts = this.ntimeouts;
 
        outobj.ngood = this.ngood;
 
        outobj.nbad = this.nbad;
 
        outobj.dt = this.dt;
 
        outobj.dtmin = this.dtmin;
 
        outobj.dtmax = this.dtmax;
 
        outobj.latency = this.latency;
 
        outobj.latencymax = this.latencymax;
 
        return outobj;
    }
 
}
