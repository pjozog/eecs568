/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package perllcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class van_options_t implements lcm.lcm.LCMEncodable
{
    public boolean self;
    public boolean vis_plot_features;
    public boolean vis_plot_scene_prior;
    public boolean vis_plot_put_corr;
    public boolean vis_plot_search_ellipses;
    public boolean vis_plot_in_and_out;
    public boolean vis_plot_inliers;
    public boolean vis_plot_summary;
    public boolean vis_plot_relpose;
    public boolean vis_plot_3pts;
    public boolean vis_plot_relpose_3pts;
    public boolean vis_plot_waitkey;
    public int n_plinks;
    public boolean manual_corr;
    public boolean mdist_plink;
 
    public van_options_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xbe69229f90cc76e1L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(perllcm.van_options_t.class))
            return 0L;
 
        classes.add(perllcm.van_options_t.class);
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
        outs.writeByte( this.self ? 1 : 0); 
 
        outs.writeByte( this.vis_plot_features ? 1 : 0); 
 
        outs.writeByte( this.vis_plot_scene_prior ? 1 : 0); 
 
        outs.writeByte( this.vis_plot_put_corr ? 1 : 0); 
 
        outs.writeByte( this.vis_plot_search_ellipses ? 1 : 0); 
 
        outs.writeByte( this.vis_plot_in_and_out ? 1 : 0); 
 
        outs.writeByte( this.vis_plot_inliers ? 1 : 0); 
 
        outs.writeByte( this.vis_plot_summary ? 1 : 0); 
 
        outs.writeByte( this.vis_plot_relpose ? 1 : 0); 
 
        outs.writeByte( this.vis_plot_3pts ? 1 : 0); 
 
        outs.writeByte( this.vis_plot_relpose_3pts ? 1 : 0); 
 
        outs.writeByte( this.vis_plot_waitkey ? 1 : 0); 
 
        outs.writeInt(this.n_plinks); 
 
        outs.writeByte( this.manual_corr ? 1 : 0); 
 
        outs.writeByte( this.mdist_plink ? 1 : 0); 
 
    }
 
    public van_options_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public van_options_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static perllcm.van_options_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        perllcm.van_options_t o = new perllcm.van_options_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.self = ins.readByte()!=0;
 
        this.vis_plot_features = ins.readByte()!=0;
 
        this.vis_plot_scene_prior = ins.readByte()!=0;
 
        this.vis_plot_put_corr = ins.readByte()!=0;
 
        this.vis_plot_search_ellipses = ins.readByte()!=0;
 
        this.vis_plot_in_and_out = ins.readByte()!=0;
 
        this.vis_plot_inliers = ins.readByte()!=0;
 
        this.vis_plot_summary = ins.readByte()!=0;
 
        this.vis_plot_relpose = ins.readByte()!=0;
 
        this.vis_plot_3pts = ins.readByte()!=0;
 
        this.vis_plot_relpose_3pts = ins.readByte()!=0;
 
        this.vis_plot_waitkey = ins.readByte()!=0;
 
        this.n_plinks = ins.readInt();
 
        this.manual_corr = ins.readByte()!=0;
 
        this.mdist_plink = ins.readByte()!=0;
 
    }
 
    public perllcm.van_options_t copy()
    {
        perllcm.van_options_t outobj = new perllcm.van_options_t();
        outobj.self = this.self;
 
        outobj.vis_plot_features = this.vis_plot_features;
 
        outobj.vis_plot_scene_prior = this.vis_plot_scene_prior;
 
        outobj.vis_plot_put_corr = this.vis_plot_put_corr;
 
        outobj.vis_plot_search_ellipses = this.vis_plot_search_ellipses;
 
        outobj.vis_plot_in_and_out = this.vis_plot_in_and_out;
 
        outobj.vis_plot_inliers = this.vis_plot_inliers;
 
        outobj.vis_plot_summary = this.vis_plot_summary;
 
        outobj.vis_plot_relpose = this.vis_plot_relpose;
 
        outobj.vis_plot_3pts = this.vis_plot_3pts;
 
        outobj.vis_plot_relpose_3pts = this.vis_plot_relpose_3pts;
 
        outobj.vis_plot_waitkey = this.vis_plot_waitkey;
 
        outobj.n_plinks = this.n_plinks;
 
        outobj.manual_corr = this.manual_corr;
 
        outobj.mdist_plink = this.mdist_plink;
 
        return outobj;
    }
 
}
