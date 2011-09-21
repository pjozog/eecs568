package april.vis;

import java.awt.event.*;
import april.jmat.geom.*;

public class VisEventAdapter implements VisEventHandler
{
    /** Higher priorities get first dibs on events. **/
    public int getPriority()
    {
        return 0;
    }

    /** Return true if you've consumed the event. **/
    public boolean mousePressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
    {
        return false;
    }

    public boolean mouseReleased(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
    {
        return false;
    }

    public boolean mouseClicked(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
    {
        System.out.println("**");
        return false;
    }

    public boolean mouseDragged(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
    {
        return false;
    }

    public boolean mouseMoved(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
    {
        return false;
    }

    public boolean mouseWheel(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseWheelEvent e)
    {
        return false;
    }
}
