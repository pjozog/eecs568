/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package mrlcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class gps_to_local_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public double local[];
    public double lat_lon_el_theta[];
    public float gps_cov[][];
 
    public gps_to_local_t()
    {
        local = new double[3];
        lat_lon_el_theta = new double[4];
        gps_cov = new float[4][4];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xa68613cd43dc764eL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(mrlcm.gps_to_local_t.class))
            return 0L;
 
        classes.add(mrlcm.gps_to_local_t.class);
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
            outs.writeDouble(this.local[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeDouble(this.lat_lon_el_theta[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            for (int b = 0; b < 4; b++) {
                outs.writeFloat(this.gps_cov[a][b]); 
            }
        }
 
    }
 
    public gps_to_local_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public gps_to_local_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static mrlcm.gps_to_local_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        mrlcm.gps_to_local_t o = new mrlcm.gps_to_local_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.local = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.local[a] = ins.readDouble();
        }
 
        this.lat_lon_el_theta = new double[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.lat_lon_el_theta[a] = ins.readDouble();
        }
 
        this.gps_cov = new float[(int) 4][(int) 4];
        for (int a = 0; a < 4; a++) {
            for (int b = 0; b < 4; b++) {
                this.gps_cov[a][b] = ins.readFloat();
            }
        }
 
    }
 
    public mrlcm.gps_to_local_t copy()
    {
        mrlcm.gps_to_local_t outobj = new mrlcm.gps_to_local_t();
        outobj.utime = this.utime;
 
        outobj.local = new double[(int) 3];
        System.arraycopy(this.local, 0, outobj.local, 0, 3); 
        outobj.lat_lon_el_theta = new double[(int) 4];
        System.arraycopy(this.lat_lon_el_theta, 0, outobj.lat_lon_el_theta, 0, 4); 
        outobj.gps_cov = new float[(int) 4][(int) 4];
        for (int a = 0; a < 4; a++) {
            System.arraycopy(this.gps_cov[a], 0, outobj.gps_cov[a], 0, 4);        }
 
        return outobj;
    }
 
}

