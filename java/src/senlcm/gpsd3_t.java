/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package senlcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class gpsd3_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public long online;
    public senlcm.gpsd3_fix_t fix;
    public double geoidal_separation;
    public short status;
    public short satellites_used;
    public short used[];
    public senlcm.gpsd3_dop_t dop;
    public double epe;
    public double skyview_utime;
    public short satellites_visible;
    public short PRN[];
    public short elevation[];
    public short azimuth[];
    public short ss[];
    public senlcm.gpsd3_devconfig_t dev;
    public String tag;
 
    public gpsd3_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x5fd46a69f78ecfe4L;
 
    public static final short STATUS_NO_FIX = (short) 0;
    public static final short STATUS_FIX = (short) 1;
    public static final short STATUS_DGPS_FIX = (short) 2;

    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(senlcm.gpsd3_t.class))
            return 0L;
 
        classes.add(senlcm.gpsd3_t.class);
        long hash = LCM_FINGERPRINT_BASE
             + senlcm.gpsd3_fix_t._hashRecursive(classes)
             + senlcm.gpsd3_dop_t._hashRecursive(classes)
             + senlcm.gpsd3_devconfig_t._hashRecursive(classes)
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
 
        outs.writeLong(this.online); 
 
        this.fix._encodeRecursive(outs); 
 
        outs.writeDouble(this.geoidal_separation); 
 
        outs.writeShort(this.status); 
 
        outs.writeShort(this.satellites_used); 
 
        for (int a = 0; a < this.satellites_used; a++) {
            outs.writeShort(this.used[a]); 
        }
 
        this.dop._encodeRecursive(outs); 
 
        outs.writeDouble(this.epe); 
 
        outs.writeDouble(this.skyview_utime); 
 
        outs.writeShort(this.satellites_visible); 
 
        for (int a = 0; a < this.satellites_visible; a++) {
            outs.writeShort(this.PRN[a]); 
        }
 
        for (int a = 0; a < this.satellites_visible; a++) {
            outs.writeShort(this.elevation[a]); 
        }
 
        for (int a = 0; a < this.satellites_visible; a++) {
            outs.writeShort(this.azimuth[a]); 
        }
 
        for (int a = 0; a < this.satellites_visible; a++) {
            outs.writeShort(this.ss[a]); 
        }
 
        this.dev._encodeRecursive(outs); 
 
        __strbuf = new char[this.tag.length()]; this.tag.getChars(0, this.tag.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0); 
 
    }
 
    public gpsd3_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public gpsd3_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static senlcm.gpsd3_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        senlcm.gpsd3_t o = new senlcm.gpsd3_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        char[] __strbuf = null;
        this.utime = ins.readLong();
 
        this.online = ins.readLong();
 
        this.fix = senlcm.gpsd3_fix_t._decodeRecursiveFactory(ins);
 
        this.geoidal_separation = ins.readDouble();
 
        this.status = ins.readShort();
 
        this.satellites_used = ins.readShort();
 
        this.used = new short[(int) satellites_used];
        for (int a = 0; a < this.satellites_used; a++) {
            this.used[a] = ins.readShort();
        }
 
        this.dop = senlcm.gpsd3_dop_t._decodeRecursiveFactory(ins);
 
        this.epe = ins.readDouble();
 
        this.skyview_utime = ins.readDouble();
 
        this.satellites_visible = ins.readShort();
 
        this.PRN = new short[(int) satellites_visible];
        for (int a = 0; a < this.satellites_visible; a++) {
            this.PRN[a] = ins.readShort();
        }
 
        this.elevation = new short[(int) satellites_visible];
        for (int a = 0; a < this.satellites_visible; a++) {
            this.elevation[a] = ins.readShort();
        }
 
        this.azimuth = new short[(int) satellites_visible];
        for (int a = 0; a < this.satellites_visible; a++) {
            this.azimuth[a] = ins.readShort();
        }
 
        this.ss = new short[(int) satellites_visible];
        for (int a = 0; a < this.satellites_visible; a++) {
            this.ss[a] = ins.readShort();
        }
 
        this.dev = senlcm.gpsd3_devconfig_t._decodeRecursiveFactory(ins);
 
        __strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); this.tag = new String(__strbuf);
 
    }
 
    public senlcm.gpsd3_t copy()
    {
        senlcm.gpsd3_t outobj = new senlcm.gpsd3_t();
        outobj.utime = this.utime;
 
        outobj.online = this.online;
 
        outobj.fix = this.fix.copy();
 
        outobj.geoidal_separation = this.geoidal_separation;
 
        outobj.status = this.status;
 
        outobj.satellites_used = this.satellites_used;
 
        outobj.used = new short[(int) satellites_used];
        if (this.satellites_used > 0)
            System.arraycopy(this.used, 0, outobj.used, 0, this.satellites_used); 
        outobj.dop = this.dop.copy();
 
        outobj.epe = this.epe;
 
        outobj.skyview_utime = this.skyview_utime;
 
        outobj.satellites_visible = this.satellites_visible;
 
        outobj.PRN = new short[(int) satellites_visible];
        if (this.satellites_visible > 0)
            System.arraycopy(this.PRN, 0, outobj.PRN, 0, this.satellites_visible); 
        outobj.elevation = new short[(int) satellites_visible];
        if (this.satellites_visible > 0)
            System.arraycopy(this.elevation, 0, outobj.elevation, 0, this.satellites_visible); 
        outobj.azimuth = new short[(int) satellites_visible];
        if (this.satellites_visible > 0)
            System.arraycopy(this.azimuth, 0, outobj.azimuth, 0, this.satellites_visible); 
        outobj.ss = new short[(int) satellites_visible];
        if (this.satellites_visible > 0)
            System.arraycopy(this.ss, 0, outobj.ss, 0, this.satellites_visible); 
        outobj.dev = this.dev.copy();
 
        outobj.tag = this.tag;
 
        return outobj;
    }
 
}

