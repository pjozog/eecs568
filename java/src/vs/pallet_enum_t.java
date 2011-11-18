package vs;
 
import java.io.*;
import java.util.*;
 
public final class pallet_enum_t implements lcm.lcm.LCMEncodable
{
    public int value;
 
    public static final int PHANTOM          = 0;
    public static final int UNKNOWN          = 1;
    public static final int BLOCK_FOUR_WAY   = 2;
    public static final int STRINGER_TWO_WAY = 3;
    public static final int STRINGER_FOUR_WAY = 4;
    public static final int LOADED_WITH_BOXES = 5;
    public static final int LOADED_WITH_TIRES = 6;
    public static final int FLOORPLAN_01     = 7;
    public static final int FLOORPLAN_02     = 8;
    public static final int FLOORPLAN_03     = 9;
    public static final int FLOORPLAN_04     = 10;
    public static final int FLOORPLAN_05     = 11;
    public static final int FLOORPLAN_06     = 12;
    public static final int FLOORPLAN_07     = 13;
    public static final int FLOORPLAN_08     = 14;
    public static final int FLOORPLAN_09     = 15;
 
    public pallet_enum_t(int value) { this.value = value; }
 
    public int getValue() { return value; }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeInt(this.value);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public static vs.pallet_enum_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        vs.pallet_enum_t o = new vs.pallet_enum_t(0);
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.value = ins.readInt();
    }
 
    public pallet_enum_t(DataInput ins) throws IOException
    {
        long hash = ins.readLong();
        if (hash != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
        _decodeRecursive(ins);
    }
 
    public vs.pallet_enum_t copy()
    {
        return new vs.pallet_enum_t(this.value);
    }
 
    public static final long _hashRecursive(ArrayList<Class> clss)
    {
        return LCM_FINGERPRINT;
    }
 
    public static final long LCM_FINGERPRINT = 0x055e7fb0c7b76b3eL;
}
