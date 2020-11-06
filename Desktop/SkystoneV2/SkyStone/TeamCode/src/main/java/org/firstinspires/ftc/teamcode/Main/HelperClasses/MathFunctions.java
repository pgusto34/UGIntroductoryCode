package org.firstinspires.ftc.teamcode.Main.HelperClasses;

public class MathFunctions {

    public double getMean(double[] m) {
        double sum = 0;
        for (double aM : m) {
            sum += aM;
        }
        return sum / m.length;
    }

    public double getStandardDeviation(double[] doubles) {
        double mean, sumAvg, sumSD = 0;
        sumAvg = 0;
        for (double aDouble1 : doubles) {
            sumAvg += aDouble1;
        }
        mean = sumAvg / doubles.length;
        for (double aDouble : doubles) {
            sumSD += Math.pow((aDouble - mean), 2);
        }
        return Math.sqrt(sumSD / (doubles.length - 1));
    }



}
