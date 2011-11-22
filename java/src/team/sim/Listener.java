package team.sim;

import java.awt.*;
import java.util.*;
import java.io.*;

import april.vis.*;
import april.jmat.*;
import april.util.*;
import april.config.*;
import april.sim.*;

import team.common.*;


public class Listener implements OldSimulator.Listener {

    public void init(Config _config, VisWorld _vw) {

    }

    public void update(OldSimulator.odometry_t odom, ArrayList<OldSimulator.landmark_t> dets) {

        drawDummy(dets);
    }

    public void drawDummy(ArrayList<OldSimulator.landmark_t> landmarks) {

    }

}
