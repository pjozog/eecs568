/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package hauv;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class vehicle_state_t implements lcm.lcm.LCMEncodable
{
    public long time;
    public double x;
    public double y;
    public double z;
    public double roll;
    public double pitch;
    public double heading;
    public double altitude;
    public double dvlAngle;
    public double sonarAngle;
    public double rmin;
    public double rmax;
    public double sonarX;
    public double sonarY;
    public double sonarZ;
 
    public vehicle_state_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xe28cbbfc2a823e4aL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(hauv.vehicle_state_t.class))
            return 0L;
 
        classes.add(hauv.vehicle_state_t.class);
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
        outs.writeLong(this.time); 
 
        outs.writeDouble(this.x); 
 
        outs.writeDouble(this.y); 
 
        outs.writeDouble(this.z); 
 
        outs.writeDouble(this.roll); 
 
        outs.writeDouble(this.pitch); 
 
        outs.writeDouble(this.heading); 
 
        outs.writeDouble(this.altitude); 
 
        outs.writeDouble(this.dvlAngle); 
 
        outs.writeDouble(this.sonarAngle); 
 
        outs.writeDouble(this.rmin); 
 
        outs.writeDouble(this.rmax); 
 
        outs.writeDouble(this.sonarX); 
 
        outs.writeDouble(this.sonarY); 
 
        outs.writeDouble(this.sonarZ); 
 
    }
 
    public vehicle_state_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public vehicle_state_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static hauv.vehicle_state_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        hauv.vehicle_state_t o = new hauv.vehicle_state_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.time = ins.readLong();
 
        this.x = ins.readDouble();
 
        this.y = ins.readDouble();
 
        this.z = ins.readDouble();
 
        this.roll = ins.readDouble();
 
        this.pitch = ins.readDouble();
 
        this.heading = ins.readDouble();
 
        this.altitude = ins.readDouble();
 
        this.dvlAngle = ins.readDouble();
 
        this.sonarAngle = ins.readDouble();
 
        this.rmin = ins.readDouble();
 
        this.rmax = ins.readDouble();
 
        this.sonarX = ins.readDouble();
 
        this.sonarY = ins.readDouble();
 
        this.sonarZ = ins.readDouble();
 
    }
 
    public hauv.vehicle_state_t copy()
    {
        hauv.vehicle_state_t outobj = new hauv.vehicle_state_t();
        outobj.time = this.time;
 
        outobj.x = this.x;
 
        outobj.y = this.y;
 
        outobj.z = this.z;
 
        outobj.roll = this.roll;
 
        outobj.pitch = this.pitch;
 
        outobj.heading = this.heading;
 
        outobj.altitude = this.altitude;
 
        outobj.dvlAngle = this.dvlAngle;
 
        outobj.sonarAngle = this.sonarAngle;
 
        outobj.rmin = this.rmin;
 
        outobj.rmax = this.rmax;
 
        outobj.sonarX = this.sonarX;
 
        outobj.sonarY = this.sonarY;
 
        outobj.sonarZ = this.sonarZ;
 
        return outobj;
    }
 
}

