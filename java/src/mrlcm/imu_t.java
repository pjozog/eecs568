/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package mrlcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class imu_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public double linear_accel[];
    public double rotation_rate[];
    public double q[];
    public boolean has_heading;
    public double heading;
    public int imu_serial_number;
 
    public imu_t()
    {
        linear_accel = new double[3];
        rotation_rate = new double[3];
        q = new double[4];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x335acf906b5e190aL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(mrlcm.imu_t.class))
            return 0L;
 
        classes.add(mrlcm.imu_t.class);
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
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.linear_accel[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.rotation_rate[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeDouble(this.q[a]); 
        }
 
        outs.writeByte( this.has_heading ? 1 : 0); 
 
        outs.writeDouble(this.heading); 
 
        outs.writeInt(this.imu_serial_number); 
 
    }
 
    public imu_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public imu_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static mrlcm.imu_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        mrlcm.imu_t o = new mrlcm.imu_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.linear_accel = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.linear_accel[a] = ins.readDouble();
        }
 
        this.rotation_rate = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.rotation_rate[a] = ins.readDouble();
        }
 
        this.q = new double[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.q[a] = ins.readDouble();
        }
 
        this.has_heading = ins.readByte()!=0;
 
        this.heading = ins.readDouble();
 
        this.imu_serial_number = ins.readInt();
 
    }
 
    public mrlcm.imu_t copy()
    {
        mrlcm.imu_t outobj = new mrlcm.imu_t();
        outobj.utime = this.utime;
 
        outobj.linear_accel = new double[(int) 3];
        System.arraycopy(this.linear_accel, 0, outobj.linear_accel, 0, 3); 
        outobj.rotation_rate = new double[(int) 3];
        System.arraycopy(this.rotation_rate, 0, outobj.rotation_rate, 0, 3); 
        outobj.q = new double[(int) 4];
        System.arraycopy(this.q, 0, outobj.q, 0, 4); 
        outobj.has_heading = this.has_heading;
 
        outobj.heading = this.heading;
 
        outobj.imu_serial_number = this.imu_serial_number;
 
        return outobj;
    }
 
}

