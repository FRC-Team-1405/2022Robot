package frc.robot.lib;

public class MathTools {
    public static int map(int x, int inMin, int inMax, int outMin, int outMax){
        return ((x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax){
        return (x - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
    }
}