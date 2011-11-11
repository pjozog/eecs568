package team.scan;

import java.util.*;

public class Corner{

    private double [] pos;
    private double theta;

    private static double deltaAngle = 10.0 * Math.PI / 180.0;
    private static double minAngle = Math.PI - deltaAngle; 
    private static double maxAngle = Math.PI + deltaAngle;

    public double getTheta(){
        return theta;

    }

    public double [] getXYPos(){
        assert(pos.length == 2);
        return pos;
    }

    Corner(double x, double y, double inTheta){
        pos = new double[]{x, y};
        theta = inTheta;
    }

    Corner(Line l1, Line l2){
        //theta = l1.getTheta()
        double []c1 = l1.getCentroid();
        double []c2 = l2.getCentroid();

        double x1 = c1[0];
        double y1 = c1[1];
        double x2 = c2[0];
        double y2 = c2[1];
        
        double theta1 = l1.getTheta();
        double theta2 = l2.getTheta();
        
        double tan1 = Math.tan(theta1);
        double tan2 = Math.tan(theta2);
        

        double x = tan1*x1 - tan2*x2 - y1 + y2;
        x /= (tan1 - tan2);

        double y = tan1 * tan2 * (x1 - x2) - tan2 * y1 + tan1 * y2;
        y /= (tan1 - tan2);

        pos = new double[]{x, y};
        theta = (theta1 + theta2) / 2;

        
    }

    public double[] getXYDiff(Corner c){
        double [] theirs = c.getXYPos();
        return new double[]{theirs[0] - pos[0], theirs[1] - pos[1]};
    }
    
    public double getThetaDiff(Corner c){
        return c.getTheta() - theta;
    }

    public static List<Corner> getAllCorners(List<Line> lines){
        ArrayList<Corner> corners = new ArrayList<Corner>();
        for(int i = 0; i < lines.size(); i++){
  
            Line l1 = lines.get(i);

            for(int j = i; j < lines.size(); j++){

                Line l2 = lines.get(j);

                double deltaTheta = Math.abs(l1.getTheta() - l2.getTheta());
               
                if(deltaTheta > minAngle && deltaTheta < maxAngle){
                    corners.add(new Corner(l1, l2));
                }
            }
        }
        return corners;
    }
}
