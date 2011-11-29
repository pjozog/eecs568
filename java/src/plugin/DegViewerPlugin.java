package plugin;

import javax.swing.*;
import javax.swing.event.*;

import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.*;
import java.awt.geom.*;

import java.lang.reflect.*;

import lcm.spy.ChannelData;
import lcm.spy.ObjectPanel;
import lcm.lcm.LCMDataInputStream;

import perllcm.*;

public class DegViewerPlugin implements lcm.spy.SpyPlugin
{
    public boolean canHandle(long fingerprint)
    {
        if (fingerprint == perllcm.pose3d_t.LCM_FINGERPRINT)
            return true;
        else
            return false;
    }

    class MyAction extends AbstractAction
    {
	ChannelData cd;
	JDesktopPane jdp;

	public MyAction(JDesktopPane jdp, ChannelData cd)
	{
	    super("Structure Viewer (Degrees)...");
	    this.jdp = jdp;
	    this.cd = cd;
	}

	public void actionPerformed(ActionEvent e) 
	{
            Viewer v = new Viewer(cd);
	    jdp.add(v);
	    v.toFront();
	}
    }

    public Action getAction(JDesktopPane jdp, ChannelData cd)
    {
        return new MyAction(jdp, cd);
    }


    class Viewer extends JInternalFrame implements lcm.lcm.LCMSubscriber
    {
        ChannelData cd;
        ObjectPanel op;

    	public Viewer(ChannelData cd)
    	{
            super(cd.name+": Degrees", true, true);
            this.cd = cd;

            setLayout(new BorderLayout());
            op = new ObjectPanel(cd.name);
            add(new JScrollPane(op),BorderLayout.CENTER);
            setSize(550,400);
            setVisible(true);
            
            lcm.lcm.LCM.getSingleton().subscribe(cd.name, this);	    
        }        

	public void messageReceived (lcm.lcm.LCM lc, String channel, LCMDataInputStream dis)
	{
            Object o = null;

            if (!isVisible())
                return;

            try {
                dis.reset();

                /* pose3d_t */
                if (cd.fingerprint == perllcm.tag_pose3d_t.LCM_FINGERPRINT) {
                    perllcm.pose3d_t pose = new perllcm.pose3d_t(dis);
                    pose.mu[3] = Math.toDegrees(pose.mu[3]);
                    pose.mu[4] = Math.toDegrees(pose.mu[4]);
                    pose.mu[5] = Math.toDegrees(pose.mu[5]);
                    o = pose;
                }

                /* default */
                else {
                    System.out.println("DegViewerPlugin should not be active: "+cd.cls+" is not handled.");
                    Class<?> cls = cd.cls;
                    o = cls.getConstructor(DataInput.class).newInstance(dis);
                }
                
                if (op != null) 
                    op.setObject(o);
			
            } catch (NullPointerException ex) {
                cd.nerrors++;
            }
            catch (NoSuchMethodException ex) {
                cd.nerrors++;
                System.out.println("Spy.messageReceived ex: "+ex);
            } catch (InstantiationException ex) {
                cd.nerrors++;
                System.out.println("Spy.messageReceived ex: "+ex);
            } catch (IllegalAccessException ex) {
                cd.nerrors++;
                System.out.println("Spy.messageReceived ex: "+ex);
            } catch (InvocationTargetException ex) {
                cd.nerrors++;
            } catch (IOException ex) {
                cd.nerrors++;
                System.out.println("Spy.messageReceived ex: "+ex);
            }

	} // messageReceived

    } // class Viewer
} // class DegViewerPlugin
