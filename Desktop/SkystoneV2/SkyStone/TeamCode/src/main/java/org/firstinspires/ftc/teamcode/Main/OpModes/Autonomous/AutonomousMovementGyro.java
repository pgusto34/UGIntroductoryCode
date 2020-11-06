package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static java.lang.Math.abs;

//import org.firstinspires.ftc.teamcode.Main.HelperClasses.Stats.BaseOpMode;
//
//import static org.firstinspires.ftc.teamcode.Main.HelperClasses.Stats.BaseOpMode.*;

@Disabled
public abstract class AutonomousMovementGyro extends TunableOpMode {

    /**
     * Created by union on 18年9月28日.
     */
    //Camera ID Number
    private ElapsedTime visionRuntime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid;
    private static int valLeft;
    private static int valRight;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 2f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    public final int rows = 640;
    public final int cols = 480;

    public OpenCvCamera phoneCam;

    //Servo Positions
      double rotatorIntakePosition = 1;
//    double rotatorRotationPosition = 0.6;
//    double rotatorDepositPosition = 0.35;
//
      double rotatorGripIntakePosition = 0.075;
//    double rotatorGripFlipPosition = 0.8;
//
//    double leftGripOpenPosition = 0.303;
//    double leftGripClosedPosition = 0.227;
//
//    double rightGripOpenPosition = 0.651;
//    double rightGripClosedPosition = 0.702;

    double leftPickUpPosition = 0.34;
    double leftPickDownPosition = 0.659;

    double rightPickUpPosition = 0.472;
    double rightPickDownPosition = 0.127;

    double armIntakePosition = 0.2414;
    double armTransportPosition = 0.82;
//    double armRotatePosition = 0.3;
//    double armMaxHeightPosition = .6;
//    double armUpper45Position = .775;
//    double armMaxReachPosition = .927;
    double armDownPosition = 1;

    double arm2IntakePosition = 1 - armIntakePosition;
    double arm2TransportPosition = 1 - armTransportPosition;
    double arm2DownPosition = 1 - armDownPosition;

    double rotatorGripApproachPosition = 0.442;


    double leftGripApproachPosition = 0.51;
    double leftGripIntakePosition = 0.08;

    double rightGripApproachPosition = 0.51;

    double leftGripPosition = leftGripApproachPosition, rightGripPosition = rightGripApproachPosition;


    double armApproachPosition = 0.95;
    double arm2ApproachPosition = 1 - 0.95;


    double rotatorApproachPosition = 0;

    double armState = 0;
    double armServoPosition = armIntakePosition, rotatorServoPosition = rotatorIntakePosition, rotatorGripPosition = rotatorGripIntakePosition, armServo2Position = arm2IntakePosition;

    protected DcMotor rightFront, leftFront, leftBack, rightBack;

    protected DcMotor leftIntake, rightIntake;

    double motorSpeed;

    Servo leftGrip, rightGrip, rotatorGrip, rotatorServo, armServo, armServo2;

    Servo leftPick, rightPick;

    static final double COUNTS_PER_MOTOR_REV = 530;
    static final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
    static final double ROBOT_DIAMETER = 27.71; //Robot diameter was measured as 19 inches
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    protected double lF, rF, lB, rB, maxVector;

    ModernRoboticsI2cGyro gyro;

    public double heading;

    ElapsedTime timer = new ElapsedTime();

    private ElapsedTime runtime = new ElapsedTime();


    protected Thread autoThread = new Thread(new Runnable() {
            @Override
            public void run() {
                runAutonomous();
            }
        });

    private long maxTime;


    public void init(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC


        //Wheel Motors
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront  = hardwareMap.dcMotor.get("leftFront");
        rightBack  = hardwareMap.dcMotor.get("rightBack");
        leftBack   = hardwareMap.dcMotor.get("leftBack");

        //Grip Motors
        leftGrip = hardwareMap.servo.get("leftGrip");
        rightGrip = hardwareMap.servo.get("rightGrip");
        rotatorGrip = hardwareMap.servo.get("rotatorGrip");

        //Arm Motors
        rotatorServo = hardwareMap.servo.get("rotatorServo");
        armServo = hardwareMap.servo.get("armServo");
        armServo2 = hardwareMap.servo.get("armServo2");

        //Pick Motors
        rightPick = hardwareMap.servo.get("rightPick");
        leftPick = hardwareMap.servo.get("leftPick");

        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        timer.reset();

        while (gyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
    }

    public void init_loop() {
        telemetry.addData("Heading: ", getAngle(gyro.getHeading()));
        telemetry.addData("Right Val: ", valRight);
        telemetry.addData("Middle Val:", valMid);
        telemetry.addData("Left Val: ", valLeft);
        telemetry.addData("Red Skystone Position: ", getSkyStonePositionRed());
        telemetry.addData("Blue Skystone Position: ", getSkyStonePositionBlue());
    }


//    GyroUpdate updateGyro = new GyroUpdate(gyro, 75);
//    public Thread gyroThread = new Thread(updateGyro);

    @Override
    public void start(){
        autoThread.start();
        //gyroThread.start();
    }



    @Override
    public void stop(){
        try{
            autoThread.stop();
           // gyroThread.stop();
//            autoThread.join();
//            gyroThread.join();
//            autoThread.interrupt();
//            gyroThread.interrupt();

        }catch(Exception e){
            telemetry.addData("ENCOUNTERED AN EXCEPTION", e);
        }

    }

    public void movePicks(boolean picksUp){
            if(picksUp){
                leftPick.setPosition(leftPickUpPosition);
                rightPick.setPosition(rightPickUpPosition);
            }
            else {
                leftPick.setPosition(leftPickDownPosition);
                rightPick.setPosition(rightPickDownPosition);
            }
        }

        public void mecanumEMove(double speed, double distance, boolean forward) {

            /** distance is in INCHES **/

            setMotorRunMode(STOP_AND_RESET_ENCODER);

            int newTargetLF;
            int newTargetLB;
            int newTargetRF;
            int newTargetRB;

            if (forward) {
                newTargetLF = leftFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
                newTargetLB = leftBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
                newTargetRF = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
                newTargetRB = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            } else {
                newTargetLF = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
                newTargetLB = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
                newTargetRF = rightFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
                newTargetRB = rightBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            }

            leftFront.setTargetPosition(newTargetLF);
            leftBack.setTargetPosition(newTargetLB);
            rightFront.setTargetPosition(newTargetRF);
            rightBack.setTargetPosition(newTargetRB);


            setMotorRunMode(RUN_TO_POSITION);



            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);



            try {
                while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {

                }
            } catch (Exception e) {
                telemetry.addData("Error", e);
            }

            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            setMotorRunMode(RUN_USING_ENCODER);


        }

        public void mecanumEStrafeRight(double speed, double distance) {

        /** distance is in INCHES **/

        setMotorRunMode(STOP_AND_RESET_ENCODER);

        int newTargetLF;
        int newTargetLB;
        int newTargetRF;
        int newTargetRB;

        newTargetLF = leftFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        newTargetLB = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newTargetRF = rightFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        newTargetRB = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newTargetLF);
        leftBack.setTargetPosition(newTargetLB);
        rightFront.setTargetPosition(newTargetRF);
        rightBack.setTargetPosition(newTargetRB);

        setMotorRunMode(RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);



            try {
                while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {

                }
            } catch (Exception e) {
                telemetry.addData("Error", e);
            }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        setMotorRunMode(RUN_USING_ENCODER);

    }

        public void mecanumEStrafeLeft(double speed, double distance) {

        /** distance is in INCHES **/

        setMotorRunMode(STOP_AND_RESET_ENCODER);

        int newTargetLF;
        int newTargetLB;
        int newTargetRF;
        int newTargetRB;

        newTargetLF = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newTargetLB = leftBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        newTargetRF = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newTargetRB = rightBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newTargetLF);
        leftBack.setTargetPosition(newTargetLB);
        rightFront.setTargetPosition(newTargetRF);
        rightBack.setTargetPosition(newTargetRB);


        setMotorRunMode(RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);



            try {
                while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {


                }
            } catch (Exception e) {
                telemetry.addData("Error", e);
            }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        setMotorRunMode(RUN_USING_ENCODER);

    }

        public void mecanumETurn(double speed, double degrees, boolean clockwise) {
            /** distance is in INCHES **/

            setMotorRunMode(STOP_AND_RESET_ENCODER);

            degrees = ((ROBOT_DIAMETER * Math.PI) / 360) * degrees;

            int newTargetLF;
            int newTargetLB;
            int newTargetRF;
            int newTargetRB;

            if (clockwise) {
                newTargetLF = leftFront.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
                newTargetLB = leftBack.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
                newTargetRF = rightFront.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
                newTargetRB = rightBack.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
            } else {
                newTargetLF = leftFront.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
                newTargetLB = leftBack.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
                newTargetRF = rightFront.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
                newTargetRB = rightBack.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
            }

            leftFront.setTargetPosition(newTargetLF);
            leftBack.setTargetPosition(newTargetLB);
            rightFront.setTargetPosition(newTargetRF);
            rightBack.setTargetPosition(newTargetRB);

            setMotorRunMode(RUN_TO_POSITION);

            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);


            try {
                while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {

                }
            } catch (Exception e) {
                telemetry.addData("Error", e);
            }

            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

           setMotorRunMode(RUN_USING_ENCODER);
        }

        public void gyroEMove(double speed, double distance, boolean forward) {

        /** distance is in INCHES **/

        setMotorRunMode(STOP_AND_RESET_ENCODER);

        int newTargetLF;
        int newTargetLB;
        int newTargetRF;
        int newTargetRB;

        if (forward) {
            newTargetLF = leftFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            newTargetLB = leftBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            newTargetRF = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newTargetRB = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        } else {
            newTargetLF = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newTargetLB = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newTargetRF = rightFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            newTargetRB = rightBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        }

        leftFront.setTargetPosition(newTargetLF);
        leftBack.setTargetPosition(newTargetLB);
        rightFront.setTargetPosition(newTargetRF);
        rightBack.setTargetPosition(newTargetRB);


        setMotorRunMode(RUN_TO_POSITION);



        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);



        try {
            while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
                telemetry.addData("Function: ", heading);

                if (forward) {
                    if (abs(getCorrection()) > speed) {
                        double correction = speed;
                        if (heading < -1) {
                            leftFront.setPower(speed + correction);
                            leftBack.setPower(speed + correction);
                            rightFront.setPower(speed - correction);
                            rightBack.setPower(speed - correction);
                        } else if (heading > 1) {
                            leftFront.setPower(speed - correction);
                            leftBack.setPower(speed - correction);
                            rightFront.setPower(speed + correction);
                            rightBack.setPower(speed + correction);
                        } else {
                            leftFront.setPower(speed);
                            leftBack.setPower(speed);
                            rightFront.setPower(speed);
                            rightBack.setPower(speed);
                        }
                    } else {
                        if (heading < -1) {
                            leftFront.setPower(speed + getCorrection());
                            leftBack.setPower(speed + getCorrection());
                            rightFront.setPower(speed - getCorrection());
                            rightBack.setPower(speed - getCorrection());
                        } else if (heading > 1) {
                            leftFront.setPower(speed - getCorrection());
                            leftBack.setPower(speed - getCorrection());
                            rightFront.setPower(speed + getCorrection());
                            rightBack.setPower(speed + getCorrection());
                        } else {
                            leftFront.setPower(speed);
                            leftBack.setPower(speed);
                            rightFront.setPower(speed);
                            rightBack.setPower(speed);
                        }

                    }
                } else {
                    if (abs(getCorrection()) > speed) {
                        double correction = speed;
                        if (heading < -1) {
                            leftFront.setPower(speed - correction);
                            leftBack.setPower(speed - correction);
                            rightFront.setPower(speed + correction);
                            rightBack.setPower(speed + correction);
                        } else if (heading > 1) {
                            leftFront.setPower(speed + correction);
                            leftBack.setPower(speed + correction);
                            rightFront.setPower(speed - correction);
                            rightBack.setPower(speed - correction);
                        } else {
                            leftFront.setPower(speed);
                            leftBack.setPower(speed);
                            rightFront.setPower(speed);
                            rightBack.setPower(speed);
                        }
                    } else {
                        if (heading < -1) {
                            leftFront.setPower(speed - getCorrection());
                            leftBack.setPower(speed - getCorrection());
                            rightFront.setPower(speed + getCorrection());
                            rightBack.setPower(speed + getCorrection());
                        } else if (heading > 1) {
                            leftFront.setPower(speed + getCorrection());
                            leftBack.setPower(speed + getCorrection());
                            rightFront.setPower(speed - getCorrection());
                            rightBack.setPower(speed - getCorrection());
                        } else {
                            leftFront.setPower(speed);
                            leftBack.setPower(speed);
                            rightFront.setPower(speed);
                            rightBack.setPower(speed);
                        }


                    }

                }
            }
        } catch (Exception e) {
            telemetry.addData("Error", e); }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        setMotorRunMode(RUN_USING_ENCODER);


    }


    public void gyroEStrafeLeft(double speed, double distance) {

        /** distance is in INC-ES **/

        setMotorRunMode(STOP_AND_RESET_ENCODER);

        int newTargetLF;
        int newTargetLB;
        int newTargetRF;
        int newTargetRB;

        newTargetLF = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newTargetLB = leftBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        newTargetRF = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newTargetRB = rightBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newTargetLF);
        leftBack.setTargetPosition(newTargetLB);
        rightFront.setTargetPosition(newTargetRF);
        rightBack.setTargetPosition(newTargetRB);


        setMotorRunMode(RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);



        try {
            while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {

                if(abs(getCorrection()) > speed){
                    double correction = speed;
                    if (heading < -1) {
                        leftFront.setPower(speed - correction);
                        leftBack.setPower(speed + correction);
                        rightFront.setPower(speed - correction);
                        rightBack.setPower(speed + correction);
                    } else if (heading > 1) {
                        leftFront.setPower(speed + correction);
                        leftBack.setPower(speed - correction);
                        rightFront.setPower(speed + correction);
                        rightBack.setPower(speed - correction);
                    } else {
                        leftFront.setPower(speed);
                        leftBack.setPower(speed);
                        rightFront.setPower(speed);
                        rightBack.setPower(speed);
                    }
                } else {
                    if (heading < -1) {
                        leftFront.setPower(speed - getCorrection());
                        leftBack.setPower(speed + getCorrection());
                        rightFront.setPower(speed - getCorrection());
                        rightBack.setPower(speed + getCorrection());
                    } else if (heading > 1) {
                        leftFront.setPower(speed + getCorrection());
                        leftBack.setPower(speed - getCorrection());
                        rightFront.setPower(speed + getCorrection());
                        rightBack.setPower(speed - getCorrection());
                    } else {
                        leftFront.setPower(speed);
                        leftBack.setPower(speed);
                        rightFront.setPower(speed);
                        rightBack.setPower(speed);
                    }

                }
            }
        } catch (Exception e) {
            telemetry.addData("Error", e);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        setMotorRunMode(RUN_USING_ENCODER);

    }

    public void gyroEStrafeRight(double speed, double distance) {

        /** distance is in INC-ES **/

        setMotorRunMode(STOP_AND_RESET_ENCODER);

        int newTargetLF;
        int newTargetLB;
        int newTargetRF;
        int newTargetRB;

        newTargetLF = leftFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        newTargetLB = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newTargetRF = rightFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        newTargetRB = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newTargetLF);
        leftBack.setTargetPosition(newTargetLB);
        rightFront.setTargetPosition(newTargetRF);
        rightBack.setTargetPosition(newTargetRB);


        setMotorRunMode(RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);



        try {
            while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {

                if(abs(getCorrection()) > speed){
                    double correction = speed;
                    if (heading < -1) {
                        leftFront.setPower(speed + correction);
                        leftBack.setPower(speed - correction);
                        rightFront.setPower(speed + correction);
                        rightBack.setPower(speed - correction);
                    } else if (heading > 1) {
                        leftFront.setPower(speed - correction);
                        leftBack.setPower(speed + correction);
                        rightFront.setPower(speed - correction);
                        rightBack.setPower(speed + correction);
                    } else {
                        leftFront.setPower(speed);
                        leftBack.setPower(speed);
                        rightFront.setPower(speed);
                        rightBack.setPower(speed);
                    }
                } else {
                    if (heading < -1) {
                        leftFront.setPower(speed + getCorrection());
                        leftBack.setPower(speed - getCorrection());
                        rightFront.setPower(speed + getCorrection());
                        rightBack.setPower(speed - getCorrection());
                    } else if (heading > 1) {
                        leftFront.setPower(speed - getCorrection());
                        leftBack.setPower(speed + getCorrection());
                        rightFront.setPower(speed - getCorrection());
                        rightBack.setPower(speed + getCorrection());
                    } else {
                        leftFront.setPower(speed);
                        leftBack.setPower(speed);
                        rightFront.setPower(speed);
                        rightBack.setPower(speed);
                    }

                }
            }
        } catch (Exception e) {
            telemetry.addData("Error", e);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        setMotorRunMode(RUN_USING_ENCODER);

    }

        public void runIntake(boolean intake){
                if(intake) {
                    leftIntake.setPower(-1);
                    rightIntake.setPower(1);
                }else {
                    leftIntake.setPower(1);
                    rightIntake.setPower(-1);
                }
        }

//        public void controlGrips(boolean open){
//            if(open){
//                leftGrip.setPosition(leftGripOpenPosition);
//                rightGrip.setPosition(rightGripOpenPosition);
//            }
//            else{
//                leftGrip.setPosition(leftGripClosedPosition);
//                rightGrip.setPosition(rightGripClosedPosition);
//            }
//        }

    public void loop(){
        heading = getAngle(gyro.getHeading());
        telemetry.addData("Heading: ", heading);
        telemetry.addData("Correction: ", getCorrection());
    }

        //RED: 1 == right, 2 == middle, 3 == left
        public int getSkyStonePositionRed(){
            if(valLeft == 255 && valMid == 255){
                return 3;
            }else if(valMid == 0){
                return 1;
            }else return 2;
        }

        //BLUE 1 == left, 2 == middle, 3 == right
        public int getSkyStonePositionBlue(){
            if(valRight == 255 && valMid == 255){
                return 1;
            }else if(valMid == 0){
                return 2;
            }else return 3;
//            if(valRight == 0){
//                return 1;
//            }else if(valMid == 0){
//                return 2;
//            }else if(valLeft == 0){
//                return 3;
//            }else{
//                telemetry.addData("Skystone Not Found", "");
//                return 1;
//            }
        }

        //detection pipeline
        public static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private StageSwitchingPipeline.Stage stageToRenderToViewport = StageSwitchingPipeline.Stage.detection;
        private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }

    public void controlArm(int armState) {

        //Control Arm and Rotator Servos and Rotator Grip

        if (armState == 0) {
            /**
             * Arm Intake Position (Intaking a Stone)
             */
            armServoPosition = armIntakePosition;
            armServo2Position = arm2IntakePosition;
            rotatorServoPosition = rotatorIntakePosition;
            rotatorGripPosition = rotatorGripIntakePosition;
        } else if (armState == 1) {
            /**
             * Arm Raise to Stone Intake Position
             */
            armServoPosition = armApproachPosition;
            armServo2Position = arm2ApproachPosition;

            rotatorServoPosition = rotatorApproachPosition;
            rotatorGripPosition = rotatorGripApproachPosition;
            leftGripPosition = leftGripApproachPosition;
            rightGripPosition = rightGripApproachPosition;
        } else if (armState == 2) {
            /**
             * Flip Stone
             */
            armServoPosition = armApproachPosition;
            armServo2Position = arm2ApproachPosition;

            rotatorServoPosition = rotatorApproachPosition;
            rotatorGripPosition = rotatorGripApproachPosition;
            leftGripPosition = leftGripIntakePosition;
            rightGripPosition = rightGripApproachPosition;
        } else if (armState == 3) {
            /**
             * Max Height/4 Stone Position
             */
            armServoPosition = armTransportPosition;
            armServo2Position = arm2TransportPosition;

            rotatorServoPosition = rotatorApproachPosition;
            rotatorGripPosition = rotatorGripApproachPosition;
            leftGripPosition = leftGripIntakePosition;
            rightGripPosition = rightGripApproachPosition;
        } else if (armState == 4){
            armServoPosition = armIntakePosition;
            armServo2Position = arm2IntakePosition;
            rotatorServoPosition = 0.9;
            rotatorGripPosition = rotatorGripApproachPosition;
            leftGripPosition = leftGripIntakePosition;
            rightGripPosition = rightGripApproachPosition;

        }

        rotatorServo.setPosition(rotatorServoPosition);
        rotatorGrip.setPosition(rotatorGripPosition);
        armServo.setPosition(armServoPosition);
        armServo2.setPosition(armServo2Position);
        leftGrip.setPosition(leftGripPosition);
        rightGrip.setPosition(rightGripPosition);

    }




    protected void pause(long millis){
        maxTime = System.currentTimeMillis() + millis;
        while(System.currentTimeMillis() < maxTime && !autoThread.isInterrupted()) {}
        }


    public void encoderTurn(double power, boolean clockwise){
        setMotorRunMode(RUN_USING_ENCODER);

        if(clockwise) {
            leftFront.setPower(-power);
            rightFront.setPower(-power);
            leftBack.setPower(-power);
            rightBack.setPower(-power);
        }
        else{
            leftFront.setPower(power);
            rightFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(power);
        }

    }

    public void setMotorRunMode(DcMotor.RunMode runMode){
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
    }

    public double getEncoderMotorPowers(int currentPosition, int targetPosition, double speed){
        int countsLeft = abs(targetPosition - currentPosition);

        if(countsLeft > 500 && !(speed >= 0.8)){
            speed = speed * 0.2 * currentPosition/50;
        }else speed = 0.8;

        if(countsLeft <= 500  && !(speed >= 0.8)){
            speed = speed * 0.2 * countsLeft/100;
        }else speed = 0.8;

        return speed;
    }

    public void controlGrips(boolean open){
        if(open){
            leftGrip.setPosition(leftGripApproachPosition);
            rightGrip.setPosition(rightGripApproachPosition);
        }
    }


    private double getCorrection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

//        angle = heading;
//
//        if (angle == 0)
//            correction = 0;             // no adjustment.
//        else
//            correction = -angle;        // reverse sign of angle for correction.
//
//        correction = correction * gain;
        if(abs(heading) > 1) {
            correction = abs(heading) * 0.05;
        }
        else correction = 0;

        return correction;
    }

    private double getAngle(double angle)
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        if (angle < -180)
            angle += 360;
        else if (angle > 180)
            angle -= 360;

        return angle;
    }


    public abstract void runAutonomous();

}
