package org.firstinspires.ftc.teamcode.Testing.ServoTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import static java.lang.Math.pow;

@TeleOp(name = "Auto Test", group = "test")
public class AutoServoTest extends TunableOpMode {
    Servo rightGrip, leftGrip, rotatorGrip, rotatorServo, armServo, rightPick, leftPick;

    boolean lastDUp, lastDDown, intakeIn, intakeOut, gripOpen;
    boolean picksUp = true, lastLButton, lastB, lastDRight, lastA, lastLBumper, lastRBumper, lastY, lastX, lastDLeft, slomo;



    double rotatorGripApproach = 0.442;


    double leftGripApproach = 0.51;
    double leftGripIntake = 0.08;

    double rightGripApproach = 0.51;

    double leftGripPosition = leftGripApproach, rightGripPosition = rightGripApproach;


    double armApproach = 0.95;


    double rotatorApproach = 0;


    double rotatorGripPosition = rotatorGripApproach;
    double rotatorServoPosition = rotatorApproach;
    double armServoPosition = armApproach;
    boolean gripRotatorIntakePosition = false;
    int armState = 0;
    boolean lastUp, lastDown, lastLeft, lastRight;

    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor leftIntake, rightIntake;
    DcMotor leftLift, rightLift;

    double lF, rF, lB, rB, maxVector;


    public void init(){
        rightGrip = hardwareMap.servo.get("rightGrip");
        leftGrip = hardwareMap.servo.get("leftGrip");
        rotatorGrip = hardwareMap.servo.get("rotatorGrip");
        rotatorServo = hardwareMap.servo.get("rotatorServo");
        armServo = hardwareMap.servo.get("armServo");
        rightPick = hardwareMap.servo.get("rightPick");
        leftPick = hardwareMap.servo.get("leftPick");

        //Initialize Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");
    }

    public void loop(){

        updateBooleans();

        mecanumMove(pow(gamepad1.left_stick_x, 3), pow(gamepad1.left_stick_y, 3), pow(gamepad1.right_stick_x, 3));


        //  controlArm();

        rotatorServo.setPosition(rotatorServoPosition);
        leftGrip.setPosition(leftGripPosition);
        rightGrip.setPosition(rightGripPosition);
        armServo.setPosition(armServoPosition);
        rotatorGrip.setPosition(rotatorGripPosition);

        telemetry.addData("Rotator Servo: ", rotatorServoPosition);
        telemetry.addData("Arm Servo:     ", armServoPosition);
        telemetry.addData("Left Grip:     ", leftGripPosition);
        telemetry.addData("Right Grip:    ", rightGripPosition);
        telemetry.addData("Rotator Grip:  ", rotatorGripPosition);


    }



    public void updateBooleans(){
        if(gamepad1.y && !lastY) picksUp = !picksUp;
        if(gamepad1.y) lastY = true;
        else lastY = false;

        if(gamepad1.b && !lastB) gripOpen = !gripOpen;
        if(gamepad1.b) lastB = true;
        else lastB = false;

        if(gamepad1.a & !lastA) slomo = !slomo;
        if(gamepad1.a) lastA = true;
        else lastA = false;

        if(gamepad1.left_bumper) lastLBumper = true;
        else lastLBumper = false;

        if(gamepad1.right_bumper) lastRBumper = true;
        else lastRBumper = false;

        if(gamepad1.dpad_up && !lastDUp && armState < 6) armState += 1;
        if(gamepad1.dpad_up) lastDUp = true;
        else lastDUp = false;

        if(gamepad1.dpad_down && !lastDDown && armState > 0) armState -= 1;
        if(gamepad1.dpad_down) lastDDown = true;
        else lastDDown = false;

        if (gripOpen) {
            leftGripPosition = leftGripApproach;
            rightGripPosition = rightGripApproach;
        } else {
            leftGripPosition = leftGripIntake;
            rightGripPosition = rightGripApproach;
        }
    }

    public void controlArm() {

        //Control Grips
        if (gripOpen) {
            leftGripPosition = leftGripApproach;
            rightGripPosition = rightGripApproach;
        } else {
            leftGripPosition = leftGripIntake;
            rightGripPosition = rightGripApproach;
        }


    }

    protected void mecanumMove(double leftX, double leftY, double rightX) {
        lF = -leftX + leftY - rightX;
        rF = -leftX - leftY - rightX;
        lB = leftX + leftY - rightX;
        rB = leftX - leftY - rightX;


        maxVector = Math.max(Math.max(Math.abs(lF), Math.abs(rF)),
                Math.max(Math.abs(lB), Math.abs(rB)));

        maxVector = maxVector > 1 ? maxVector : 1;

        leftFront.setPower(lF / maxVector);
        rightFront.setPower(rF / maxVector);
        leftBack.setPower(lB / maxVector);
        rightBack.setPower(rB / maxVector);
    }

}
