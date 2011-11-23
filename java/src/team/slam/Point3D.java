package team.slam;

public class Point3D {

    private double x;
    private double y;
    private double z;

    public Point3D() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    public Point3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Point3D(Point3D aPoint) {
        //TODO: I think this copy is fine since they are primitives. Check with Schuyler
        x = aPoint.getX();
        y = aPoint.getY();
        z = aPoint.getZ();
    }

    public Point3D(double[] values) {
        if (values.length != 3) {
            System.out.println("Error! Wrong length vector for Point3D");
            assert(false);
        } else {
            x = values[0];
            y = values[1];
            z = values[2];
        }
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double[] getArray() {
        return new double[] {x, y, z};
    }


    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }


}
