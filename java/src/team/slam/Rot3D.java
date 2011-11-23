package team.slam;

import april.jmat.Matrix;

public class Rot3D {

    private double roll;
    private double pitch;
    private double yaw;

    public Rot3D() {
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
    }

    public Rot3D(double roll, double pitch, double yaw) {
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    public double getRoll() {
        return roll;
    }

    public double getYaw() {
        return yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public void setRoll(double roll) {
        this.roll = roll;
    }


    public void setPitch(double pitch) {
        this.pitch = pitch;
    }


    public void setYaw(double yaw) {
        this.yaw = yaw;
    }


    public Matrix getRot33() {

        //TODO: Implement this function

        return new Matrix(3,3);
    }

}
