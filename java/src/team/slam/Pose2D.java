package team.slam;

public class Pose2D {

    private Point2D t;
    private Rot2D r;

    public Pose2D() {
        t = new Point2D();
        r = new Rot2D();
    }

    public Pose2D(double x, double y, double yaw) {
        t = new Point2D(x, y);
        r = new Rot2D(yaw);
    }

    public Pose2D(double[] values) {

        if (values.length != 3) {
            System.out.println("Error! Wrong length vector for Pose2D");
            assert(false);
        } else {
            t = new Point2D(values[0], values[1]);
            r = new Rot2D(values[2]);
        }
    }

    // Copy constructor
    public Pose2D(Pose2D aPose) {

        //TODO: I think this copy is fine since they are primitives. Check with Schuyler
        t = new Point2D(aPose.getX(), aPose.getY());
        r = new Rot2D(aPose.getYaw());

    }

    // Getters
    public double getX() {
        return t.getX();
    }

    public double getY() {
        return t.getY();
    }


    public double getYaw() {
        return r.getYaw();
    }

    public double[] getArray() {
        return new double[] {getX(), getY(), getYaw()};
    }


    // Setters
    public void setX(double x) {
        t.setX(x);
    }

    public void setY(double y) {
        t.setY(y);
    }


    public void setYaw(double yaw) {
        r.setYaw(yaw);
    }

    public void setStateArray(double[] values) {
        if (values.length != 3) {
            System.out.println("Error! Wrong length vector for Pose2D");
        } else {
            setX(values[0]);
            setY(values[1]);
            setYaw(values[2]);
        }
    }

}
