package team.slam;

public class Point 2D{
    

    private double x;
    private double y;

    public Point2D(){
        x = 0;
        y = 0;
    }

    public Point2D(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Point2D(Point2D point){
        x = point.getX();
        y = point.getY();
    }
    public Point2D(double[] values) {
        if (values.length != 2) {
            System.out.println("Error! Wrong length vector for Point2D");
            assert(false);
        } else {
            x = values[0];
            y = values[1];
        }
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

 
    public double[] getArray() {
        return new double[] {x, y};
    }


    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }


    public void setStateArray(double[] values) {
        if (values.length != 2) {
            System.out.println("Error! Wrong length vector for Pose2D");
        } else {
            setX(values[0]);
            setY(values[1]);
        }
    }


}