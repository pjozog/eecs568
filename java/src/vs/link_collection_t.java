/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package vs;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class link_collection_t implements lcm.lcm.LCMEncodable
{
    public int id;
    public String name;
    public short type;
    public boolean reset;
    public int nlinks;
    public vs.link_t links[];
 
    public link_collection_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xe4f230d9012c4ac9L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(vs.link_collection_t.class))
            return 0L;
 
        classes.add(vs.link_collection_t.class);
        long hash = LCM_FINGERPRINT_BASE
             + vs.link_t._hashRecursive(classes)
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
        outs.writeInt(this.id); 
 
        __strbuf = new char[this.name.length()]; this.name.getChars(0, this.name.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0); 
 
        outs.writeShort(this.type); 
 
        outs.writeByte( this.reset ? 1 : 0); 
 
        outs.writeInt(this.nlinks); 
 
        for (int a = 0; a < this.nlinks; a++) {
            this.links[a]._encodeRecursive(outs); 
        }
 
    }
 
    public link_collection_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public link_collection_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static vs.link_collection_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        vs.link_collection_t o = new vs.link_collection_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        char[] __strbuf = null;
        this.id = ins.readInt();
 
        __strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); this.name = new String(__strbuf);
 
        this.type = ins.readShort();
 
        this.reset = ins.readByte()!=0;
 
        this.nlinks = ins.readInt();
 
        this.links = new vs.link_t[(int) nlinks];
        for (int a = 0; a < this.nlinks; a++) {
            this.links[a] = vs.link_t._decodeRecursiveFactory(ins);
        }
 
    }
 
    public vs.link_collection_t copy()
    {
        vs.link_collection_t outobj = new vs.link_collection_t();
        outobj.id = this.id;
 
        outobj.name = this.name;
 
        outobj.type = this.type;
 
        outobj.reset = this.reset;
 
        outobj.nlinks = this.nlinks;
 
        outobj.links = new vs.link_t[(int) nlinks];
        for (int a = 0; a < this.nlinks; a++) {
            outobj.links[a] = this.links[a].copy();
        }
 
        return outobj;
    }
 
}

