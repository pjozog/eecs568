/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package perllcm;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class auv_os_conduit_ojw_t implements lcm.lcm.LCMEncodable
{
    public int waypoint;
 
    public auv_os_conduit_ojw_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xd39e9d81f9e37d13L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(perllcm.auv_os_conduit_ojw_t.class))
            return 0L;
 
        classes.add(perllcm.auv_os_conduit_ojw_t.class);
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
        outs.writeInt(this.waypoint); 
 
    }
 
    public auv_os_conduit_ojw_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public auv_os_conduit_ojw_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static perllcm.auv_os_conduit_ojw_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        perllcm.auv_os_conduit_ojw_t o = new perllcm.auv_os_conduit_ojw_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.waypoint = ins.readInt();
 
    }
 
    public perllcm.auv_os_conduit_ojw_t copy()
    {
        perllcm.auv_os_conduit_ojw_t outobj = new perllcm.auv_os_conduit_ojw_t();
        outobj.waypoint = this.waypoint;
 
        return outobj;
    }
 
}
