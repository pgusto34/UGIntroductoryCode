package org.firstinspires.ftc.teamcode.Testing.PurePursuit;




import java.util.ArrayList;

public class MathFunctions {

    public class DoublePoint {
        public double x;
        public double y;

        DoublePoint(double x, double y){
            x = this.x;
            y = this.y;
        }
    }

    public static double angleWrap(double angle){
        while(angle < -Math.PI){
            angle += 2*Math.PI;
        }
        while(angle > Math.PI){
            angle -= Math.PI;
        }
        return angle;
    }

    public ArrayList<DoublePoint> lineCircleIntersection(DoublePoint circleCenter, double radius,
                                                                DoublePoint linePoint1, DoublePoint linePoint2){
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003){
            linePoint1.y = linePoint2.y + 0.003; //makes sure the y values aren't the same
        }
        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003){
            linePoint1.x = linePoint2.x + 0.03; //makes sure the x values aren't the same and you don't divide by zero
        }

        //shifts all points down so cifcle origin is at zero, zero
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x); //Find the slope of the line

        double quadraticA = 1.0 + Math.pow(m1, 2);

        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1,2) * x1);

        double quadraticC = ((Math.pow(m1, 2) * Math.pow(x1, 2)) - (2.0 * y1 * m1 * x1) + Math.pow(y1,2) - Math.pow(radius, 2));

        ArrayList<DoublePoint> allPoints = new ArrayList<>();

        try{
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))/ (2.0 * quadraticA));

            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            //Reapply the offset
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new DoublePoint(xRoot1, yRoot1));
            }

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))/ (2.0 * quadraticA));

            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new DoublePoint(xRoot2, yRoot2));
            }




        }catch(Exception e){

        }
        return allPoints;
    }

}


