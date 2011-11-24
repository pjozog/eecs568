/** EECS 568 Homework 2, SLAM
    Steve Chaves
    Schuyler Cohen
    Patrick O'Keefe
    Paul Ozog
**/

package team.common;

import april.jmat.*;

import java.util.*;

/** Class of array utilities to make our lives less stressful
 **/
public class ArrayUtil {

    //Could be useful for testing and debugging matrices
    public static void print2dArray(double[][] a) {

        for (int row = 0; row < a.length; row++) {
            for (int col = 0; col < a[0].length; col++)
                System.out.print(a[row][col] + " ");
            System.out.println("");
        }
    }

    //Could be useful for testing and debugging vectors
    public static void print1dArray(double[] a) {
        for (int row = 0; row < a.length; row++)
            System.out.println(a[row]);
    }

    public static void print1dArray(int[] a) {
        for (int row = 0; row < a.length; row++)
            System.out.println(a[row]);
    }

    public static double[][] cat(double[] a, double[] b) {
        double[][] res = new double[a.length][2];

        if (a.length != b.length) {
            throw new IllegalArgumentException("Non-equal length");
        }

        for (int i = 0; i < a.length; i++) {
            res[i][0] = a[i];
            res[i][1] = b[i];
        }

        return res;
    }

}
