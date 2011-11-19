/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package perllcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class isam_vlink_t implements lcm.lcm.LCMEncodable
{
    public long id1;
    public long id2;
    public int link_id;
    public int publisher_id;
    public int sensor_id;
    public int n;
    public double z[];
    public int n2;
    public double R[];
    public short link_type;
    public boolean accept;
    public int accept_code;
 
    public isam_vlink_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x85e4ae2687703f12L;
 
    public static final int SENSOR_ODOMETRY = 1;
    public static final int SENSOR_CAMERA = 2;
    public static final int SENSOR_LASER = 3;
    public static final int SENSOR_SONAR = 4;
    public static final short LINK_PRIOR = (short) 1;
    public static final short LINK_POSE2D = (short) 2;
    public static final short LINK_POSE3D = (short) 3;
    public static final short LINK_POSE2DB = (short) 4;
    public static final short LINK_POSE3DB = (short) 5;
    public static final short CODE_ACCEPTED = (short) 1;
    public static final short CODE_INVALID_MODEL = (short) 2;
    public static final short CODE_LOW_THRESHOLD = (short) 3;
    public static final int CODE_MIN_CORR = 4;
    public static final int CODE_MIN_INLIERS = 5;
    public static final int CODE_SBA = 6;
    public static final int CODE_MDIST_NAV = 7;

    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(perllcm.isam_vlink_t.class))
            return 0L;
 
        classes.add(perllcm.isam_vlink_t.class);
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
        outs.writeLong(this.id1); 
 
        outs.writeLong(this.id2); 
 
        outs.writeInt(this.link_id); 
 
        outs.writeInt(this.publisher_id); 
 
        outs.writeInt(this.sensor_id); 
 
        outs.writeInt(this.n); 
 
        for (int a = 0; a < this.n; a++) {
            outs.writeDouble(this.z[a]); 
        }
 
        outs.writeInt(this.n2); 
 
        for (int a = 0; a < this.n2; a++) {
            outs.writeDouble(this.R[a]); 
        }
 
        outs.writeShort(this.link_type); 
 
        outs.writeByte( this.accept ? 1 : 0); 
 
        outs.writeInt(this.accept_code); 
 
    }
 
    public isam_vlink_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public isam_vlink_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static perllcm.isam_vlink_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        perllcm.isam_vlink_t o = new perllcm.isam_vlink_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.id1 = ins.readLong();
 
        this.id2 = ins.readLong();
 
        this.link_id = ins.readInt();
 
        this.publisher_id = ins.readInt();
 
        this.sensor_id = ins.readInt();
 
        this.n = ins.readInt();
 
        this.z = new double[(int) n];
        for (int a = 0; a < this.n; a++) {
            this.z[a] = ins.readDouble();
        }
 
        this.n2 = ins.readInt();
 
        this.R = new double[(int) n2];
        for (int a = 0; a < this.n2; a++) {
            this.R[a] = ins.readDouble();
        }
 
        this.link_type = ins.readShort();
 
        this.accept = ins.readByte()!=0;
 
        this.accept_code = ins.readInt();
 
    }
 
    public perllcm.isam_vlink_t copy()
    {
        perllcm.isam_vlink_t outobj = new perllcm.isam_vlink_t();
        outobj.id1 = this.id1;
 
        outobj.id2 = this.id2;
 
        outobj.link_id = this.link_id;
 
        outobj.publisher_id = this.publisher_id;
 
        outobj.sensor_id = this.sensor_id;
 
        outobj.n = this.n;
 
        outobj.z = new double[(int) n];
        if (this.n > 0)
            System.arraycopy(this.z, 0, outobj.z, 0, this.n); 
        outobj.n2 = this.n2;
 
        outobj.R = new double[(int) n2];
        if (this.n2 > 0)
            System.arraycopy(this.R, 0, outobj.R, 0, this.n2); 
        outobj.link_type = this.link_type;
 
        outobj.accept = this.accept;
 
        outobj.accept_code = this.accept_code;
 
        return outobj;
    }
 
}
