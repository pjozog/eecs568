package team;

import java.util.*;

import april.jmat.*;
import april.util.*;
import april.sim.*;

public class TicksUtil {
    
    public static DenseVec ticksToXYT(Simulator.odometry_t odom, double baseline) {
        
        double ticksL = odom.obs[0];
        double ticksR = odom.obs[1];
        
        DenseVec result = new DenseVec(3);

        if (ticksR == ticksL) {
            
            result.set(0,ticksL);
            result.set(1,0.0);
            result.set(2,0.0);
            
        } else {
            
            double dPhi = (ticksR - ticksL)/baseline;

            double rc = (ticksL + ticksR)/(2*dPhi);

            result.set(0,rc*Math.sin(dPhi));
            result.set(1,-rc*Math.cos(dPhi) + rc); 
            result.set(2, dPhi);

        }
        



        return result;
    }


}
