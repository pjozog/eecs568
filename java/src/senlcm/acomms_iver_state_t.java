/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package senlcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class acomms_iver_state_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public double latitude;
    public double longitude;
    public double heading;
    public double depth;
    public double altitude;
    public int next_wypnt;
    public double dist_nwp;
    public int batt_percent;
    public int error;
    public int aborted_mission;
    public int modem_id;
 
    public acomms_iver_state_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xc3fa30cb288b1b62L;
 
    public static final int ERR_NONE = 0;
    public static final int ERR_OVER_PITCH = 1;
    public static final int ERR_EXCEED_TIME = 2;
    public static final int ERR_LEAK = 3;
    public static final int ERR_NO_FORWARD_PROG = 4;
    public static final int ERR_EXCEED_MAX_DEPTH = 5;
    public static final int ERR_NO_UPWARD_PROG = 6;
    public static final int ERR_TOW_FLOAT_ENGAGED = 7;
    public static final int ERR_SAFETY_RET_PATH = 8;
    public static final int ERR_DFS_UNCHANGED = 9;
    public static final int ERR_COMPASS_STOPPED = 10;
    public static final int ERR_EXCEEDED_MIN_REQ_PWR = 11;
    public static final int ERR_STOP_AND_TRANSMIT_IRIDIUM = 12;

    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(senlcm.acomms_iver_state_t.class))
            return 0L;
 
        classes.add(senlcm.acomms_iver_state_t.class);
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
 
        outs.writeDouble(this.latitude); 
 
        outs.writeDouble(this.longitude); 
 
        outs.writeDouble(this.heading); 
 
        outs.writeDouble(this.depth); 
 
        outs.writeDouble(this.altitude); 
 
        outs.writeInt(this.next_wypnt); 
 
        outs.writeDouble(this.dist_nwp); 
 
        outs.writeInt(this.batt_percent); 
 
        outs.writeInt(this.error); 
 
        outs.writeInt(this.aborted_mission); 
 
        outs.writeInt(this.modem_id); 
 
    }
 
    public acomms_iver_state_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public acomms_iver_state_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static senlcm.acomms_iver_state_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        senlcm.acomms_iver_state_t o = new senlcm.acomms_iver_state_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.latitude = ins.readDouble();
 
        this.longitude = ins.readDouble();
 
        this.heading = ins.readDouble();
 
        this.depth = ins.readDouble();
 
        this.altitude = ins.readDouble();
 
        this.next_wypnt = ins.readInt();
 
        this.dist_nwp = ins.readDouble();
 
        this.batt_percent = ins.readInt();
 
        this.error = ins.readInt();
 
        this.aborted_mission = ins.readInt();
 
        this.modem_id = ins.readInt();
 
    }
 
    public senlcm.acomms_iver_state_t copy()
    {
        senlcm.acomms_iver_state_t outobj = new senlcm.acomms_iver_state_t();
        outobj.utime = this.utime;
 
        outobj.latitude = this.latitude;
 
        outobj.longitude = this.longitude;
 
        outobj.heading = this.heading;
 
        outobj.depth = this.depth;
 
        outobj.altitude = this.altitude;
 
        outobj.next_wypnt = this.next_wypnt;
 
        outobj.dist_nwp = this.dist_nwp;
 
        outobj.batt_percent = this.batt_percent;
 
        outobj.error = this.error;
 
        outobj.aborted_mission = this.aborted_mission;
 
        outobj.modem_id = this.modem_id;
 
        return outobj;
    }
 
}

