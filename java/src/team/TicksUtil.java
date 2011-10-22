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

        /*if (ticksR == ticksL) {

          result.set(0,ticksL);
          result.set(1,0.0);
          result.set(2,0.0);

          } else {*/

        // double dPhi = MathUtil.mod2pi((ticksR - ticksL)/baseline);
        // double dPhi = MathUtil.mod2pi(Math.atan2(ticksR - ticksL,baseline));
        double dPhi = Math.atan2(ticksR - ticksL, baseline);
        // double s = (ticksL + ticksR)/(2.0);

        //result.set(0,rc*Math.sin(dPhi));
        //result.set(1,-rc*Math.cos(dPhi) + rc);
        //result.set(2, dPhi);

        // result.set(0,s*Math.cos(dPhi));
        // result.set(1,s*Math.sin(dPhi));
        // result.set(2, dPhi);

        result.set(0,(ticksL+ticksR)/2);
        result.set(1,0);
        result.set(2, dPhi);

        //}




        return result;
    }


}
