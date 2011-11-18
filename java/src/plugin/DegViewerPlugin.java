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

import hauv.*;
import perllcm.*;
import senlcm.*;

public class DegViewerPlugin implements lcm.spy.SpyPlugin
{
    public boolean canHandle(long fingerprint)
    {
        if (fingerprint == senlcm.acomms_iver_state_t.LCM_FINGERPRINT ||
	    fingerprint == senlcm.pelican_t.LCM_FINGERPRINT ||
            fingerprint == senlcm.gpsd3_t.LCM_FINGERPRINT ||
            fingerprint == senlcm.gpsd_t.LCM_FINGERPRINT ||
            fingerprint == senlcm.kvh_dsp3000_t.LCM_FINGERPRINT ||
            fingerprint == senlcm.ms_gx1_t.LCM_FINGERPRINT ||
            fingerprint == senlcm.ms_gx3_t.LCM_FINGERPRINT ||
            fingerprint == senlcm.os_compass_t.LCM_FINGERPRINT ||
            fingerprint == senlcm.rdi_pd5_t.LCM_FINGERPRINT ||
            fingerprint == senlcm.nmea_gphdt_t.LCM_FINGERPRINT ||

            fingerprint == hauv.bs_cnv_t.LCM_FINGERPRINT ||
            fingerprint == hauv.bs_imu_t.LCM_FINGERPRINT ||
            fingerprint == hauv.bs_nvg_t.LCM_FINGERPRINT ||
            fingerprint == hauv.bs_nvr_t.LCM_FINGERPRINT ||
            fingerprint == hauv.bs_pit_t.LCM_FINGERPRINT ||
            fingerprint == hauv.bs_rnv_2_t.LCM_FINGERPRINT ||
            fingerprint == hauv.vehicle_state_t.LCM_FINGERPRINT ||

            fingerprint == perllcm.auv_os_conduit_osi_t.LCM_FINGERPRINT ||
            fingerprint == perllcm.pose3d_t.LCM_FINGERPRINT ||
            fingerprint == perllcm.van_calib_t.LCM_FINGERPRINT ||
            fingerprint == perllcm.van_feature_collection_t.LCM_FINGERPRINT
            )
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

                /* SENLCM=========================================== */
                /* acomms_iver_state_t */
                if (cd.fingerprint == senlcm.acomms_iver_state_t.LCM_FINGERPRINT) {
                    senlcm.acomms_iver_state_t acomms = new senlcm.acomms_iver_state_t(dis);
                    acomms.latitude = Math.toDegrees(acomms.latitude);
                    acomms.longitude = Math.toDegrees(acomms.longitude);
                    acomms.heading = Math.toDegrees (acomms.heading);
                    o = acomms;
                }
                /* gpsd_t */
                else if (cd.fingerprint == senlcm.gpsd_t.LCM_FINGERPRINT) {
                    senlcm.gpsd_t gpsd = new senlcm.gpsd_t(dis);
                    gpsd.latitude = Math.toDegrees(gpsd.latitude);
                    gpsd.longitude = Math.toDegrees(gpsd.longitude);
                    o = gpsd;
                }
                /* gpsd3_t */
                else if (cd.fingerprint == senlcm.gpsd3_t.LCM_FINGERPRINT) {
                    senlcm.gpsd3_t gpsd = new senlcm.gpsd3_t(dis);
                    gpsd.fix.latitude = Math.toDegrees(gpsd.fix.latitude);
                    gpsd.fix.longitude = Math.toDegrees(gpsd.fix.longitude);
                    o = gpsd;
                }
                /* kvh_dsp3000_t */
                else if (cd.fingerprint == senlcm.kvh_dsp3000_t.LCM_FINGERPRINT) {
                    System.out.println("reached\n");
                    senlcm.kvh_dsp3000_t kvh = new senlcm.kvh_dsp3000_t(dis);
                    kvh.data = Math.toDegrees(kvh.data);
                    o = kvh;
                }
                /* ms_gx1_t */
                else if (cd.fingerprint == senlcm.ms_gx1_t.LCM_FINGERPRINT) {
                    senlcm.ms_gx1_t ms = new senlcm.ms_gx1_t(dis);
                    for (int i=0; i<3; i++) {
                        ms.sAngRate[i] = Math.toDegrees(ms.sAngRate[i]);
                        ms.iAngRate[i] = Math.toDegrees(ms.iAngRate[i]);
                        ms.sEuler[i] = Math.toDegrees(ms.sEuler[i]);
                        ms.iEuler[i] = Math.toDegrees(ms.iEuler[i]);
                    }
                    o = ms;
                }
                /* ms_gx3_t */
                else if (cd.fingerprint == senlcm.ms_gx3_t.LCM_FINGERPRINT) {
                    senlcm.ms_gx3_t ms = new senlcm.ms_gx3_t(dis);
                    for (int i=0; i<3; i++) {
                        ms.sAngRate[i] = Math.toDegrees(ms.sAngRate[i]);
                        ms.iAngRate[i] = Math.toDegrees(ms.iAngRate[i]);
                        ms.sEuler[i] = Math.toDegrees(ms.sEuler[i]);
                    }
                    o = ms;
                }
                /* pelican_t */
                else if (cd.fingerprint == senlcm.pelican_t.LCM_FINGERPRINT) {
                    senlcm.pelican_t pelic = new senlcm.pelican_t(dis);
                    for (int i=0; i<3; i++) {
			pelic.angle[i] = Math.toDegrees(pelic.angle[i]);
                    }
		    for (int i=0; i<2; i++) {
			pelic.acc_angle[i] = Math.toDegrees(pelic.acc_angle[i]);
		    }
		    pelic.mag_heading	= Math.toDegrees(pelic.mag_heading);
		    pelic.heading	= Math.toDegrees(pelic.heading);
		    pelic.latitude	= Math.toDegrees(pelic.latitude);
		    pelic.longitude	= Math.toDegrees(pelic.longitude);
                    o			= pelic;
                }
                /* os_compass */
                else if (cd.fingerprint == senlcm.os_compass_t.LCM_FINGERPRINT) {
                    senlcm.os_compass_t os = new senlcm.os_compass_t(dis);
                    for (int i=0; i<3; i++)
                        os.rph[i] = Math.toDegrees(os.rph[i]);
                    o = os;
                }
                /* rdi_pd5_t */
                else if (cd.fingerprint == senlcm.rdi_pd5_t.LCM_FINGERPRINT) {
                    senlcm.rdi_pd5_t rdi = new senlcm.rdi_pd5_t(dis);
                    rdi.pitch = Math.toDegrees(rdi.pitch);
                    rdi.roll = Math.toDegrees(rdi.roll);
                    rdi.heading = Math.toDegrees(rdi.heading);
                    o = rdi;
                }
                /* nmea_gphdt_t */
                else if (cd.fingerprint == senlcm.nmea_gphdt_t.LCM_FINGERPRINT) {
                    senlcm.nmea_gphdt_t gphdt = new senlcm.nmea_gphdt_t(dis);
                    gphdt.h_true = (float)Math.toDegrees(gphdt.h_true);
                    gphdt.h_raw = (float)Math.toDegrees(gphdt.h_raw);
                    gphdt.deviation = (float)Math.toDegrees(gphdt.deviation);
                    gphdt.variation = (float)Math.toDegrees(gphdt.variation);
                    o = gphdt;
                }

                /* HAUV=========================================== */
                /* bs_cnv_t */
                else if (cd.fingerprint == hauv.bs_cnv_t.LCM_FINGERPRINT) {
                    hauv.bs_cnv_t cnv = new hauv.bs_cnv_t(dis);
                    cnv.heading = Math.toDegrees(cnv.heading);
                    o = cnv;
                }
                /* bs_imu_t */
                else if (cd.fingerprint == hauv.bs_imu_t.LCM_FINGERPRINT) {
                    hauv.bs_imu_t imu = new hauv.bs_imu_t(dis);
                    imu.rate_x = Math.toDegrees(imu.rate_x);
                    imu.rate_y = Math.toDegrees(imu.rate_y);
                    imu.rate_z = Math.toDegrees(imu.rate_z);
                    o = imu;
                }
                /* bs_pit_t */
                else if (cd.fingerprint == hauv.bs_pit_t.LCM_FINGERPRINT) {
                    hauv.bs_pit_t pit = new hauv.bs_pit_t(dis);
                    pit.pitch_dvl = Math.toDegrees(pit.pitch_dvl);
                    pit.pitch_sonar = Math.toDegrees(pit.pitch_sonar);
                    o = pit;
                }
                /* bs_nvg_t */
                else if (cd.fingerprint == hauv.bs_nvg_t.LCM_FINGERPRINT) {
                    hauv.bs_nvg_t nvg = new hauv.bs_nvg_t(dis);
                    nvg.heading = Math.toDegrees(nvg.heading);
                    nvg.roll = Math.toDegrees(nvg.roll);
                    nvg.pitch = Math.toDegrees(nvg.pitch);
                    o = nvg;
                }
                /* bs_nvr_t */
                else if (cd.fingerprint == hauv.bs_nvr_t.LCM_FINGERPRINT) {
                    hauv.bs_nvr_t nvr = new hauv.bs_nvr_t(dis);
                    nvr.pitch_rate = Math.toDegrees(nvr.pitch_rate);
                    nvr.roll_rate = Math.toDegrees(nvr.roll_rate);
                    nvr.yaw_rate = Math.toDegrees(nvr.yaw_rate);
                    o = nvr;
                }
                /* bs_rnv_2_t */
                else if (cd.fingerprint == hauv.bs_rnv_2_t.LCM_FINGERPRINT) {
                    hauv.bs_rnv_2_t rnv = new hauv.bs_rnv_2_t(dis);
                    rnv.heading = Math.toDegrees(rnv.heading);
                    rnv.absheading = Math.toDegrees(rnv.absheading);
                    rnv.absroll = Math.toDegrees(rnv.absroll);
                    rnv.abspitch = Math.toDegrees(rnv.abspitch);
                    o = rnv;
                }
                /* bs_vehicle_state_t */
                else if (cd.fingerprint == hauv.vehicle_state_t.LCM_FINGERPRINT) {
                    hauv.vehicle_state_t state = new hauv.vehicle_state_t(dis);
                    state.roll = Math.toDegrees(state.roll);
                    state.pitch = Math.toDegrees(state.pitch);
                    state.heading = Math.toDegrees(state.heading);
                    state.dvlAngle = Math.toDegrees(state.dvlAngle);
                    state.sonarAngle = Math.toDegrees(state.sonarAngle);
                    o = state;
                }


                /* PERLLCM=========================================== */
                /* auv_os_conduit_osi_t */
                else if (cd.fingerprint == perllcm.auv_os_conduit_osi_t.LCM_FINGERPRINT) {
                    perllcm.auv_os_conduit_osi_t osi = new perllcm.auv_os_conduit_osi_t(dis);
                    osi.latitude = Math.toDegrees(osi.latitude);
                    osi.longitude = Math.toDegrees(osi.longitude);
                    o = osi;
                }
                /* pose3d_t */
                else if (cd.fingerprint == perllcm.pose3d_t.LCM_FINGERPRINT) {
                    perllcm.pose3d_t pose = new perllcm.pose3d_t(dis);
                    pose.mu[3] = Math.toDegrees(pose.mu[3]);
                    pose.mu[4] = Math.toDegrees(pose.mu[4]);
                    pose.mu[5] = Math.toDegrees(pose.mu[5]);
                    o = pose;
                }
                /* van_calib_t */
                else if (cd.fingerprint == perllcm.van_calib_t.LCM_FINGERPRINT) {
                    perllcm.van_calib_t calib = new perllcm.van_calib_t(dis);
                    calib.fov[0] = Math.toDegrees(calib.fov[0]);
                    calib.fov[1] = Math.toDegrees(calib.fov[1]);
                    o = calib;
                }
                /* van_feature_collection_t */
                else if (cd.fingerprint == perllcm.van_feature_collection_t.LCM_FINGERPRINT) {
                    perllcm.van_feature_collection_t fc = new perllcm.van_feature_collection_t(dis);
                    fc.calib.fov[0] = Math.toDegrees(fc.calib.fov[0]);
                    fc.calib.fov[1] = Math.toDegrees(fc.calib.fov[1]);
                    o = fc;
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
