/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package perllcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class bathy_rdi_list_t implements lcm.lcm.LCMEncodable
{
    public long id;
    public int npts;
    public perllcm.bathy_rdi_t bathy_v[];
    public perllcm.pose3d_t x_vc[];
    public int calib_list[];
    public int ncalib;
    public perllcm.van_calib_t calib[];
 
    public bathy_rdi_list_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xf0c6270472910a00L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(perllcm.bathy_rdi_list_t.class))
            return 0L;
 
        classes.add(perllcm.bathy_rdi_list_t.class);
        long hash = LCM_FINGERPRINT_BASE
             + perllcm.bathy_rdi_t._hashRecursive(classes)
             + perllcm.pose3d_t._hashRecursive(classes)
             + perllcm.van_calib_t._hashRecursive(classes)
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
        outs.writeLong(this.id); 
 
        outs.writeInt(this.npts); 
 
        for (int a = 0; a < this.npts; a++) {
            this.bathy_v[a]._encodeRecursive(outs); 
        }
 
        for (int a = 0; a < this.npts; a++) {
            this.x_vc[a]._encodeRecursive(outs); 
        }
 
        for (int a = 0; a < this.npts; a++) {
            outs.writeInt(this.calib_list[a]); 
        }
 
        outs.writeInt(this.ncalib); 
 
        for (int a = 0; a < this.ncalib; a++) {
            this.calib[a]._encodeRecursive(outs); 
        }
 
    }
 
    public bathy_rdi_list_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public bathy_rdi_list_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static perllcm.bathy_rdi_list_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        perllcm.bathy_rdi_list_t o = new perllcm.bathy_rdi_list_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.id = ins.readLong();
 
        this.npts = ins.readInt();
 
        this.bathy_v = new perllcm.bathy_rdi_t[(int) npts];
        for (int a = 0; a < this.npts; a++) {
            this.bathy_v[a] = perllcm.bathy_rdi_t._decodeRecursiveFactory(ins);
        }
 
        this.x_vc = new perllcm.pose3d_t[(int) npts];
        for (int a = 0; a < this.npts; a++) {
            this.x_vc[a] = perllcm.pose3d_t._decodeRecursiveFactory(ins);
        }
 
        this.calib_list = new int[(int) npts];
        for (int a = 0; a < this.npts; a++) {
            this.calib_list[a] = ins.readInt();
        }
 
        this.ncalib = ins.readInt();
 
        this.calib = new perllcm.van_calib_t[(int) ncalib];
        for (int a = 0; a < this.ncalib; a++) {
            this.calib[a] = perllcm.van_calib_t._decodeRecursiveFactory(ins);
        }
 
    }
 
    public perllcm.bathy_rdi_list_t copy()
    {
        perllcm.bathy_rdi_list_t outobj = new perllcm.bathy_rdi_list_t();
        outobj.id = this.id;
 
        outobj.npts = this.npts;
 
        outobj.bathy_v = new perllcm.bathy_rdi_t[(int) npts];
        for (int a = 0; a < this.npts; a++) {
            outobj.bathy_v[a] = this.bathy_v[a].copy();
        }
 
        outobj.x_vc = new perllcm.pose3d_t[(int) npts];
        for (int a = 0; a < this.npts; a++) {
            outobj.x_vc[a] = this.x_vc[a].copy();
        }
 
        outobj.calib_list = new int[(int) npts];
        if (this.npts > 0)
            System.arraycopy(this.calib_list, 0, outobj.calib_list, 0, this.npts); 
        outobj.ncalib = this.ncalib;
 
        outobj.calib = new perllcm.van_calib_t[(int) ncalib];
        for (int a = 0; a < this.ncalib; a++) {
            outobj.calib[a] = this.calib[a].copy();
        }
 
        return outobj;
    }
 
}
