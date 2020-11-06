package org.firstinspires.ftc.teamcode.Outreach;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Outreach.OpModeGeneric;

import java.util.concurrent.TimeUnit;

/**
 * Created by union on 18年9月28日.
 */

public abstract class OpModeMecanum extends OpModeGeneric {

    protected DcMotor rightFront, rightBack, leftFront, leftBack;
    protected DcMotor extender;
    protected DcMotor rotator, rotator1;
    protected DcMotor grabber;

    protected Servo knocker;

    static final double COUNTS_PER_MOTOR_REV    = 1120 ;    // < DRIVE BASE Neverest 40:1
    static final double COUNTS_PER_MOTOR_REV2   = 2240 ; // < ROTATOR REV MOTORS
    static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double DRIVE_GEAR_REDUCTION2   = 4.0 ;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_ANGLE        = (COUNTS_PER_MOTOR_REV2 * DRIVE_GEAR_REDUCTION2) / (360);

    protected static double lF, rF, lB, rB, maxVector;

    protected static double lFE, rFE, lBE, rBE, maxVectorE;

    public int lowExtenderCount;
    public int highExtenderCount;
    public int lowRotatorCount;
    public int highRotatorCount;

    @Override
    protected void initMotors() {
        leftFront = hardwareMap.dcMotor.get("leftF");
        leftBack = hardwareMap.dcMotor.get("leftB");
        rightFront = hardwareMap.dcMotor.get("rightF");
        rightBack = hardwareMap.dcMotor.get("rightB");

        extender = hardwareMap.dcMotor.get("extender");

        rotator = hardwareMap.dcMotor.get("rotator");
        rotator1 = hardwareMap.dcMotor.get("rotator1");

        grabber = hardwareMap.dcMotor.get("grabber");
    }

    @Override
    protected void initServos(){
        knocker = hardwareMap.servo.get("knocker");
    }

    protected void initOther(){
        period = 10;
        timeUnit = TimeUnit.MILLISECONDS;
    }

    protected void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    protected void mecanumMove(double leftX, double leftY, double rightX, boolean negated){
        if(negated){
            lF = leftX + leftY - rightX;
            rF = leftX - leftY - rightX;
            lB = -leftX + leftY - rightX;
            rB = -leftX - leftY - rightX;
        }else{
            lF = -(leftX + leftY + rightX);
            rF = -(leftX - leftY + rightX);
            lB = -(-leftX + leftY + rightX);
            rB = -(-leftX - leftY + rightX);
        }

        maxVector = Math.max(Math.max(Math.abs(lF),Math.abs(rF)),
                Math.max(Math.abs(lB),Math.abs(rB)));

        maxVector = maxVector > 1 ? maxVector : 1;

        leftFront.setPower(lF / maxVector);
        rightFront.setPower(rF / maxVector);
        leftBack.setPower(lB / maxVector);
        rightBack.setPower(rB / maxVector);
    }

    protected void mecanumEMove(double speed, double distance, boolean forward){

        /** distance is in INCHES **/

        resetEncoders();

        int newTargetLF;
        int newTargetLB;
        int newTargetRF;
        int newTargetRB;

        if(forward) {
            newTargetLF = leftFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            newTargetLB = leftBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            newTargetRF = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newTargetRB = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        } else {
            newTargetLF = leftFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            newTargetLB = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newTargetRF = rightFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            newTargetRB = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        }

        leftFront.setTargetPosition(newTargetLF);
        leftBack.setTargetPosition(newTargetLB);
        rightFront.setTargetPosition(newTargetRF);
        rightBack.setTargetPosition(newTargetRB);


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        try {
            while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {}
        } catch (Exception e) {
            telemetry.addData("Error", e);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    protected void mecanumETurn(double speed, double degrees, boolean clockwise){
        /** distance is in INCHES **/

        resetEncoders();

        degrees = ((26.5 * Math.PI) / 360) * degrees;

        int newTargetLF;
        int newTargetLB;
        int newTargetRF;
        int newTargetRB;

        if(clockwise) {
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

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        try {
            while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {}
        } catch (Exception e) {
            telemetry.addData("Error", e);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void rotatorEMove(double speed, double degrees, boolean up) {
        /** angles in degrees**/

        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newTarget;
        int newTarget1;

        if (up) {
            newTarget = rotator.getCurrentPosition() + (int) (degrees * COUNTS_PER_ANGLE);
            newTarget1 = rotator1.getCurrentPosition() + (int) (degrees * COUNTS_PER_ANGLE);

        } else {
            newTarget = rotator.getCurrentPosition() - (int) (degrees * COUNTS_PER_ANGLE);
            newTarget1 = rotator1.getCurrentPosition() - (int) (degrees * COUNTS_PER_ANGLE);
        }

        rotator.setTargetPosition(newTarget);
        rotator1.setTargetPosition(newTarget1);


        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotator.setPower(speed);
        rotator1.setPower(speed);

        try {
            while (rotator.isBusy() && rotator1.isBusy()) {
            }
        } catch (Exception e) {
            telemetry.addData("Error", e);
        }

        rotator.setPower(0);
        rotator1.setPower(0);

        rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void mecanumEMoveXY(double x, double y, double speed){

        /* distance is in INCHES */

        resetEncoders();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double distance = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));

        lFE = (-x - y);
        rFE = (-x + y);
        lBE = (x - y);
        rBE = (x + y);

        maxVectorE = Math.max(Math.max(Math.abs(lFE),Math.abs(rFE)),
                Math.max(Math.abs(lBE),Math.abs(rBE)));

        maxVectorE = maxVectorE > 1 ? maxVectorE : 1;

        int newTargetLF;
        int newTargetLB;
        int newTargetRF;
        int newTargetRB;

        int directionLF = 1;
        int directionRF = 1;
        int directionLB = 1;
        int directionRB = 1;

        if(lFE < 0) directionLF = -1;
        if(rFE < 0) directionRF = -1;
        if(lBE < 0) directionLB = -1;
        if(rBE < 0) directionRB = -1;

        newTargetLF = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionLF);
        newTargetRF = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionRF);
        newTargetLB = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionLB);
        newTargetRB = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionRB);

        leftFront.setTargetPosition(newTargetLF);
        leftBack.setTargetPosition(newTargetLB);
        rightFront.setTargetPosition(newTargetRF);
        rightBack.setTargetPosition(newTargetRB);


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed * (lFE / maxVectorE));
        leftBack.setPower(speed * (lBE / maxVectorE));
        rightFront.setPower(speed * (rFE / maxVectorE));
        rightBack.setPower(speed * (rBE / maxVectorE));

        try {
            while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {}
        } catch (Exception e) {
            telemetry.addData("Error", e);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    protected void mecanumMoveGTXREY(double moveSpeed, double x,             double y,
                                     double turnSpeed, double turnDegrees,   boolean clockwise,
                                     double rotaSpeed, double rotateDegrees, boolean up,
                                     double exteSpeed, double exteDistance
    ){
        resetEncoders();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double distance = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));

        lFE = (-x - y);
        rFE = (-x + y);
        lBE = (x - y);
        rBE = (x + y);

        maxVectorE = Math.max(Math.max(Math.abs(lFE),Math.abs(rFE)),
                Math.max(Math.abs(lBE),Math.abs(rBE)));

        maxVectorE = maxVectorE > 1 ? maxVectorE : 1;

        int newTargetLF;
        int newTargetLB;
        int newTargetRF;
        int newTargetRB;

        int directionLF = 1;
        int directionRF = 1;
        int directionLB = 1;
        int directionRB = 1;

        if(lFE < 0) directionLF = -1;
        if(rFE < 0) directionRF = -1;
        if(lBE < 0) directionLB = -1;
        if(rBE < 0) directionRB = -1;

        newTargetLF = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionLF);
        newTargetRF = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionRF);
        newTargetLB = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionLB);
        newTargetRB = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionRB);

        leftFront.setTargetPosition(newTargetLF);
        leftBack.setTargetPosition(newTargetLB);
        rightFront.setTargetPosition(newTargetRF);
        rightBack.setTargetPosition(newTargetRB);


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(moveSpeed * (lFE / maxVectorE));
        leftBack.setPower(moveSpeed * (lBE / maxVectorE));
        rightFront.setPower(moveSpeed * (rFE / maxVectorE));
        rightBack.setPower(moveSpeed * (rBE / maxVectorE));

        try {
            while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {}
        } catch (Exception e) {
            telemetry.addData("Error", e);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void extenderEMoveByCounts(double speed, boolean up){
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newTarget;

        if (up) {
            newTarget = highExtenderCount;

        } else {
            newTarget = lowExtenderCount;
        }

        extender.setTargetPosition(newTarget);

        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extender.setPower(speed);

        try {
            while (extender.isBusy()) {
            }
        } catch (Exception e) {
            telemetry.addData("Error", e);
        }

        extender.setPower(0);

        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void mecanumEncoder(double x, double y, double driveSpeed,
                                  double rotatorAngle, boolean rotatorUp,
                                  double extenderDistance, boolean extenderUp){
        /*extenderDistance unit not determined*/

        resetEncoders();

        double distance = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));

        lFE = (-x - y);
        rFE = (-x + y);
        lBE = (x - y);
        rBE = (x + y);

        maxVectorE = Math.max(Math.max(Math.abs(lFE),Math.abs(rFE)),
                Math.max(Math.abs(lBE),Math.abs(rBE)));

        maxVectorE = maxVectorE > 1 ? maxVectorE : 1;

        int newTargetLF;
        int newTargetLB;
        int newTargetRF;
        int newTargetRB;
        int newTargetRotator;
        int newTargetRotator1;
        int newTargetExtender;

        int directionLF = 1;
        int directionRF = 1;
        int directionLB = 1;
        int directionRB = 1;

        if(lFE < 0) directionLF = -1;
        if(rFE < 0) directionRF = -1;
        if(lBE < 0) directionLB = -1;
        if(lBE < 0) directionRB = -1;

        newTargetLF = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionLF);
        newTargetRF = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionRF);
        newTargetLB = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionLB);
        newTargetRB = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * directionRB);
        if (rotatorUp) {
            newTargetRotator = rotator.getCurrentPosition() + (int) (rotatorAngle * COUNTS_PER_ANGLE);
            newTargetRotator1 = rotator1.getCurrentPosition() + (int) (rotatorAngle * COUNTS_PER_ANGLE);
        } else {
            newTargetRotator = rotator.getCurrentPosition() - (int) (rotatorAngle * COUNTS_PER_ANGLE);
            newTargetRotator1 = rotator1.getCurrentPosition() - (int) (rotatorAngle * COUNTS_PER_ANGLE);
        }
        if (extenderUp) {
            newTargetExtender = extender.getCurrentPosition() + (int) (extenderDistance * COUNTS_PER_MOTOR_REV);

        } else {
            newTargetExtender = extender.getCurrentPosition() - (int) (extenderDistance * COUNTS_PER_MOTOR_REV);
        }

        leftFront.setTargetPosition(newTargetLF);
        leftBack.setTargetPosition(newTargetLB);
        rightFront.setTargetPosition(newTargetRF);
        rightBack.setTargetPosition(newTargetRB);
        rotator.setTargetPosition(newTargetRotator);
        rotator1.setTargetPosition(newTargetRotator1);
        extender.setTargetPosition(newTargetExtender);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(driveSpeed * (lFE / maxVectorE));
        leftBack.setPower(driveSpeed * (lBE / maxVectorE));
        rightFront.setPower(driveSpeed * (rFE / maxVectorE));
        rightBack.setPower(driveSpeed * (rBE / maxVectorE));
        rotator.setPower(1);
        rotator1.setPower(1);
        extender.setPower(1);

        try {
            while ((leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy() )
                    || (rotator.isBusy() && rotator1.isBusy())
                    || extender.isBusy()) {}
        } catch (Exception e) {
            telemetry.addData("Error", e);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        rotator.setPower(0);
        rotator1.setPower(0);
        extender.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
