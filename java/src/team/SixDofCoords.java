package team;

import april.jmat.LinAlg;
import april.jmat.Matrix;

import java.util.*;

public class SixDofCoords {
    
    public static void main(String[] args) {

        testOps();

    }

    private static void testOps() {

        Random rand = new Random(1337);

        double[] xij = new double[6];
        xij[0] = rand.nextDouble();
        xij[1] = rand.nextDouble();
        xij[2] = rand.nextDouble();
        xij[3] = rand.nextDouble();
        xij[4] = rand.nextDouble();
        xij[5] = rand.nextDouble();

        double[] xjk = new double[6];
        xjk[0] = rand.nextDouble();
        xjk[1] = rand.nextDouble();
        xjk[2] = rand.nextDouble();
        xjk[3] = rand.nextDouble();
        xjk[4] = rand.nextDouble();
        xjk[5] = rand.nextDouble();

        double[] xik = headToTail(xij, xjk);
        double[] xji = inverse(xij);
        double[] xjkEst = tailToTail(xij, xik);

        System.out.println("xij: ");
        ArrayUtil.print1dArray(xij);
        System.out.println();

        System.out.println("xjk: ");
        ArrayUtil.print1dArray(xjk);
        System.out.println();

        System.out.println("xik = xij \\oplus xjk:");
        ArrayUtil.print1dArray(xik);
        System.out.println();

        System.out.println("xji = \\ominus xij:");
        ArrayUtil.print1dArray(xji);
        System.out.println();

        System.out.println("xjkEst = tail2tail(xij, xik):");
        ArrayUtil.print1dArray(xjkEst);
        System.out.println();

    }

    public static double[] headToTail(double[] xij, double[] xjk) {
        
        Matrix Hij = new Matrix(LinAlg.xyzrpyToMatrix(xij));
        Matrix Hjk = new Matrix(LinAlg.xyzrpyToMatrix(xjk));
        Matrix Hik = new Matrix(4,4);
        
        Hik = Hij.times(Hjk);

        double[] xik = LinAlg.matrixToXyzrpy(Hik.copyArray());

        return xik;

    }

    public static double[] inverse(double[] xij) {
        
        Matrix Hij = new Matrix(LinAlg.xyzrpyToMatrix(xij));
        
        //Get the 3x3 rotation matrix
        Matrix Rji = new Matrix(Hij.copyArray(0, 0, 3, 3));
        Matrix tij = new Matrix(Hij.copyArray(0, 3, 3, 1));
        
        //Convert to inverse coordinate transform
        Matrix Rij = Rji.transpose();
        Matrix tji = Rij.times(tij).times(-1.0);

        Matrix Hji = new Matrix(4, 4);

        //Set 3x3 rotation matrix in Hji
        Hji.set(0, 0, Rij.get(0, 0)); Hji.set(0, 1, Rij.get(0, 1)); Hji.set(0, 2, Rij.get(0, 2)); 
        Hji.set(1, 0, Rij.get(1, 0)); Hji.set(1, 1, Rij.get(1, 1)); Hji.set(1, 2, Rij.get(1, 2)); 
        Hji.set(2, 0, Rij.get(2, 0)); Hji.set(2, 1, Rij.get(2, 1)); Hji.set(2, 2, Rij.get(2, 2)); 

        //Set 3x1 translation matrix in Hji
        Hji.set(0, 3, tji.get(0));
        Hji.set(1, 3, tji.get(1));
        Hji.set(2, 3, tji.get(2));

        //Set the bottom row as '0 0 0 1'
        Hji.set(3, 3, 1.0);

        return LinAlg.matrixToXyzrpy(Hji.copyArray());

    }

    public static double[] tailToTail(double[] xij, double[] xik) {
        
        return headToTail(inverse(xij), xik);

    }

}