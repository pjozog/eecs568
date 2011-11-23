package team.slam;

public class Pose3D {

    private Point3D t;
    private Rot3D r;

    public Pose3D() {
        t = new Point3D();
        r = new Rot3D();
    }

    public Pose3D(double x, double y, double z, double roll, double pitch, double yaw) {
        t = new Point3D(x, y, z);
        r = new Rot3D(roll, pitch, yaw);
    }

    public Pose3D(double[] values) {

        if (values.length != 6) {
            System.out.println("Error! Wrong length vector for Pose3D");
        } else {
            t = new Point3D(values[0], values[1], values[2]);
            r = new Rot3D(values[3], values[4], values[5]);
        }
    }

    // Copy constructor
    public Pose3D(Pose3D aPose) {

        //TODO: I think this copy is fine since they are primitives. Check with Schuyler
        t = new Point3D(aPose.getX(), aPose.getY(), aPose.getZ());
        r = new Rot3D(aPose.getRoll(), aPose.getPitch(), aPose.getYaw());

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

    public double getPitch() {
        return r.getPitch();
    }

    public double getYaw() {
        return r.getYaw();
    }

    public double[] getArray() {
        return new double[] {getX(), getY(), getZ(), getRoll(), getPitch(), getYaw()};
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

    public void setArray(double[] values) {
        if (values.length != 6) {
            System.out.println("Error! Wrong length vector for Pose3D");
        } else {
            setX(values[0]);
            setY(values[1]);
            setZ(values[2]);
            setRoll(values[3]);
            setPitch(values[4]);
            setYaw(values[5]);
        }
    }

}
