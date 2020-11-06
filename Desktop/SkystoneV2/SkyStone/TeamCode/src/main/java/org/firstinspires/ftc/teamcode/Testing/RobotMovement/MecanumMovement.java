package org.firstinspires.ftc.teamcode.Testing.RobotMovement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import javax.crypto.AEADBadTagException;

import static java.lang.Math.*;
@Disabled
@TeleOp(name = "Mecanum Movement Drive", group = "test")
public class MecanumMovement extends TunableOpMode {

    //Constants
    static final int wheelMotionRight = 1;
    static final int wheelMotionLeft = 2;
    static final int wheelSideLeft = 3;
    static final int wheelSideRight = 4;

    //Hardware:
    //Motors
    static public DcMotor leftFront, rightFront, leftBack, rightBack;


    public void init(){
        initMotors();
    }


    public void loop(){
        telemetry.addData("Angle", toDegrees(atan2(-gamepad1.left_stick_y,gamepad1.left_stick_x)));
        telemetry.update();

        rightFront.setPower(sin(atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + PI/4));
        rightBack.setPower(sin(atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + PI/4));
        leftBack.setPower(cos(atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + PI/4));
        leftFront.setPower(cos(atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + PI/4));
//        telemetry.addData("W1", rightBack.getPower());
//        telemetry.addData("W2", rightFront.getPower());
//        telemetry.addData("W3", leftBack.getPower());
//        telemetry.addData("W4", leftFront.getPower());
    }


    public void mecanumDrive(double x, double y, double rotation, boolean FOD){
        //Initialize Robot Class
        Robot myRobot = new Robot();

        //use X/Y coordinates of the current destination to determine angle of translation
        //This can also be used to determine translation distance
        double angle = atan2(y,x);

        //Calculate wheel multipliers
        myRobot.calculateWheels(angle, rotation);

        //Set motor speeds
//        //Made two motor powers negative for correct drive
        leftFront.setPower(myRobot.getWheelSpeed(0));
        rightFront.setPower(myRobot.getWheelSpeed(1));
        leftBack.setPower(myRobot.getWheelSpeed(2));
        rightBack.setPower(myRobot.getWheelSpeed(3));

    }

    public class Wheel{

        private int wheelDirection, wheelSide;
        private double vMulti;

        //Intitializes a wheel
         Wheel(int direction, int side){
            this.wheelDirection = direction;
            this.wheelSide = side;
        }

        /*Calculate the multiplier for the specific wheel. Do not factor in the
        speed yet (vd from equation), this will allow faster recalculation if only speed is changed
        without changing the angle or rotation
        */

         double getVoltageMultiplier(double angle, double rotation){
            angle = toRadians(angle);

            if(wheelDirection == wheelMotionRight){
                vMulti = sin(angle + PI/4);
            }else{
                vMulti = cos(angle + PI/4);
            }
            if(wheelSide == wheelSideLeft){
                vMulti = -vMulti + rotation;
            }else{
                vMulti = vMulti - rotation;
            }
            return vMulti;
        }
    }


    public class Robot{
         private double[] wheel = {0.0, 0.0, 0.0, 0.0};
         private double translationAngle = 0.0;
         private double rotationSpeed = 0.0;

         //hard code the orientation of each wheel
        Wheel frontLeft = new Wheel(wheelMotionRight, wheelSideLeft);
        Wheel frontRight = new Wheel(wheelMotionLeft, wheelSideRight);
        Wheel backLeft = new Wheel(wheelMotionLeft, wheelSideLeft);
        Wheel backRight = new Wheel(wheelMotionRight, wheelSideRight);

        public void calculateWheels(double tAngle, double rSpeed){
            //Do not recalculate if values did not change
            if(tAngle != translationAngle || rSpeed != rotationSpeed) {
                //Store angle and speed
                translationAngle = tAngle;
                rotationSpeed = rSpeed;

                //Assign multipliers to wheels
                wheel[0] = frontLeft.getVoltageMultiplier(translationAngle, rotationSpeed);
                wheel[1] = frontRight.getVoltageMultiplier(translationAngle, rotationSpeed);
                wheel[2] = backLeft.getVoltageMultiplier(translationAngle, rotationSpeed);
                wheel[3] = backRight.getVoltageMultiplier(translationAngle, rotationSpeed);
//                telemetry.addData("W1", wheel[0]);
//                telemetry.addData("W1", wheel[1]);
//                telemetry.addData("W1", wheel[2]);
//                telemetry.addData("W1", wheel[3]);
//                telemetry.update();
               // normalizeWheelMultipliers();
            }
        }

        public void normalizeWheelMultipliers(){
            double highestMultiplier = 0.0;
            //Find highest wheel multiplier
            for(int i = 0; i <= 3; ++i){
                if(abs(wheel[i]) > highestMultiplier){
                    highestMultiplier = abs(wheel[i]);
                }
            }

            //Normalize values
            for(int i = 0; i <= 3; ++i){
                wheel[i] = wheel[i]/highestMultiplier;
            }
        }

        //Returns the scaled speed for a specific wheel
        double getWheelSpeed(int wheelNumber){
            return(wheel[wheelNumber]);
        }

    }


    public void initMotors(){
        //Initialize Motors
        leftFront = hardwareMap.dcMotor.get("leftF");
        rightFront = hardwareMap.dcMotor.get("rightF");
        leftBack = hardwareMap.dcMotor.get("leftB");
        rightBack = hardwareMap.dcMotor.get("rightB");
//
//        leftIntake = hardwareMap.dcMotor.get("leftIntake");
//        rightIntake = hardwareMap.dcMotor.get("rightIntake");
//
//        leftLift = hardwareMap.dcMotor.get("leftLift");
//        rightLift = hardwareMap.dcMotor.get("rightLift");
    }


//    public void initServos(){
//        //Initialize Servos
//        leftPinch = hardwareMap.servo.get("leftPinch");
//        rightPinch = hardwareMap.servo.get("rightPinch");
//        rotator = hardwareMap.servo.get("rotator");
//        armServo = hardwareMap.servo.get("armServo");
//        //Arkie grips are not in current use
//        //arkieLeft = hardwareMap.servo.get("arkieLeft");
//        //arkieRight = hardwareMap.servo.get("arkieRight");
//    }


//    public void initSensors(){
//        //Initialize Sensors
//       // intakeSensor = (SensorREV2mDistance) hardwareMap.opticalDistanceSensor.get("intakeSensor");
//
//        //Initialize Gyro
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        imu.initialize(imuParameters);
//    }
//
//
//    public void resetZAxis(){
//        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
//        imu.initialize(imuParams);
//    }
//
//    public double getHeading(){
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        heading = angles.firstAngle;
//        return heading;
//    }

}
