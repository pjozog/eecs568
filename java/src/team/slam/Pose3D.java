package team.slam;

public class Pose3D {

    private Point3D t;
    private Rot3D r;

    public Pose3D(double x, double y, double z, double roll, double pitch, double yaw) {
        t = new Point3D(x, y, z);
        r = new Rot3D(roll, pitch, yaw);
    }


    // Getters
    public double getX() {
        return t.getX();
    }

    public double getY() {
        return t.getY();
    }

    public double getZ() {
        return t.getZ();
    }

    public double getRoll() {
        return r.getRoll();
    }

    public double getYaw() {
        return yaw;
    }

    public double getPitch() {
        return pitch;
    }


    // Setters
    public void setX(double x) {
        t.setX(x);
    }

    public void setY(double y) {
        t.setY(y);
    }

    public void setZ(double z) {
        t.setZ(z);
    }

    public void setRoll(double roll) {
        r.setRoll(roll);
    }

    public void setPitch(double pitch) {
        r.setPitch(pitch);
    }

    public void setYaw(double yaw) {
        r.setYaw(yaw);
    }

}
