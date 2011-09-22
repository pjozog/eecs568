package team;

import april.jmat.*;

import java.util.*;

/** Class of array utilities to make our lives less stressful
 **/
public class ArrayUtil
{
    //Could be useful for testing and debugging matrices
    public static void print2dArray(double[][] a)
    {
	for (int row = 0; row < a.length; row++) {
	    for (int col = 0; col < a[0].length; col++)
		System.out.print(a[row][col] + " ");
	    System.out.println("");
	}
    }

    //Could be useful for testing and debugging vectors
    public static void print1dArray(double[] a)
    {
	for (int row = 0; row < a.length; row++)
	    System.out.println(a[row]);
    }
}
