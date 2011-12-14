package team.slam;

import april.jmat.Matrix;

public class Rot2D {

    private double yaw;

    public Rot2D() {
        yaw = 0.0;
    }
    public Rot2D(double yaw) {
        this.yaw = yaw;
    }

 
    public double getYaw() {
        return yaw;
    }

 

    public void setYaw(double yaw) {
        this.yaw = yaw;
    }


    public Matrix getRot33() {

        //TODO: Implement this function

        return new Matrix(1,1);
    }

}
