package org.firstinspires.ftc.teamcode.Testing.ServoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import static java.lang.Math.*;

@TeleOp(name = "Servo Tuner", group = "test")
public class SERVOTEST extends TunableOpMode {
    Servo rightGrip, leftGrip, rotatorGrip, rotatorServo, armServo, armServo2, rightPick, leftPick;

    boolean lastDUp, lastDDown, intakeIn, intakeOut;
    boolean picksUp = true, lastLButton, lastB, lastDRight, lastA, lastLBumper, lastRBumper, lastY, lastX, lastDLeft, slomo;


    double rotatorOpen = 0;
    double rotatorDeposit = 0.239;

    double rotatorGripIntakePosition = 0.075;
    double rotatorGripFlipPosition = 0.8;

    double leftGripOpenPosition = 0.31;
    double leftGripClosedPosition = 0.207;

    double rightGripOpenPosition = 0.658;
    double rightGripClosedPosition = 0.722;

    double leftGripPosition = leftGripOpenPosition, rightGripPosition = rightGripOpenPosition;


    double armIntakePosition = 0.2414;
    double armRotatePosition = 0.4;
    double armMaxHeightPosition = .684;
    double armUpper45Position = .775;
    double armMaxReachPosition = .927;
    double armDownPosition = 0;

    double arm2IntakePosition = 1 - armIntakePosition;
    double arm2RotatePosition = 1 - 0.4;
    double arm2MaxHeightPosition = 1- .684;
    double arm2Upper45Position = 1 - .775;
    double arm2MaxReachPosition = 1 - .927;
    double arm2DownPosition = 1;

    double rotatorIntakePosition = 1;
    double rotatorDepositPosition = 0.288;

    double rotatorGripPosition = rotatorGripIntakePosition;
    boolean rotatorGripFlipped = false;
    double rotatorServoPosition = 1;
    double armServoPosition = 0.927, arm2ServoPosition = 0.073;
    boolean gripOpen;
    boolean gripRotatorIntakePosition = false;
    int armState = 0;
    boolean lastUp, lastDown, lastLeft, lastRight;

    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor leftIntake, rightIntake;
    DcMotor leftLift, rightLift;

    double lF, rF, lB, rB, maxVector;


    public void init() {
        rightGrip = hardwareMap.servo.get("rightGrip");
        leftGrip = hardwareMap.servo.get("leftGrip");
        rotatorGrip = hardwareMap.servo.get("rotatorGrip");
        rotatorServo = hardwareMap.servo.get("rotatorServo");
        armServo = hardwareMap.servo.get("armServo");
        armServo2 = hardwareMap.servo.get("armServo2");
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

    public void loop() {


        updateBooleans();

     //   controlArm();

//        if (gamepad1.dpad_right & rotatorServoPosition < 1) {
//            rotatorServoPosition += 0.0005;
//        }
//
//        if (gamepad1.dpad_left & rotatorServoPosition > 0) {
//            rotatorServoPosition -= 0.0005;
//        }

        if(gamepad1.right_trigger > 0.6 & rightGripPosition < 1){
            rightGripPosition += 0.0005;
        }

        if(gamepad1.right_bumper & rightGripPosition > 0){
            rightGripPosition -= 0.0005;
        }

        if(gamepad1.left_trigger > 0.6 & leftGripPosition < 1){
            leftGripPosition += 0.0005;
        }

        if(gamepad1.left_bumper & leftGripPosition > 0){
            leftGripPosition -= 0.0005;
        }

        if (gamepad1.a && armServoPosition < 1) {
            armServoPosition += 0.0005;
            arm2ServoPosition -= 0.0005;
        }

        if (gamepad1.b && armServoPosition > 0) {
            armServoPosition -= 0.0005;
            arm2ServoPosition += 0.0005;
        }

        if(gamepad1.y){  leftIntake.setPower(-1);
            rightIntake.setPower(1);} else

        if(gamepad1.x){

            leftIntake.setPower(1);
            rightIntake.setPower(-1);
        } else{
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }

//        if(gamepad1.a & rotatorGripPosition < 1){
//            rotatorGripPosition += 0.0005;
//        }
//
//        if(gamepad1.b & rotatorGripPosition > 0){
//            rotatorGripPosition -= 0.0005;

        //  controlArm();

        //rotatorServo.setPosition(rotatorServoPosition);
//        leftGrip.setPosition(leftGripPosition);
//        rightGrip.setPosition(rightGripPosition);
       // armServo.setPosition(armServoPosition);
//        rotatorGrip.setPosition(rotatorGripPosition);
        armServo.setPosition(armServoPosition);
        armServo2.setPosition(arm2ServoPosition);
        leftGrip.setPosition(leftGripPosition);
        rightGrip.setPosition(rightGripPosition);

//        telemetry.addData("Rotator Servo: ", rotatorServoPosition);
        telemetry.addData("Arm Servo  :     ", armServoPosition);
        telemetry.addData("Arm Servo 2:     ", arm2ServoPosition);
        telemetry.addData("Left Grip", leftGripPosition);
        telemetry.addData("Right Grip: ", rightGripPosition);

//        telemetry.addData("Left Grip:     ", leftGripPosition);
//        telemetry.addData("Right Grip:    ", rightGripPosition);
//        telemetry.addData("Rotator Grip:  ", rotatorGripPosition);

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
    }

    public void controlArm(){

        //Control Grips
        if(gripOpen)
        {
            leftGripPosition = leftGripOpenPosition;
            rightGripPosition = rightGripOpenPosition;
        }
        else {
            leftGripPosition = leftGripClosedPosition;
            rightGripPosition = rightGripClosedPosition;
        }



        //Control Arm and Rotator Servos and Rotator Grip

        if(gamepad1.dpad_right & rotatorServoPosition < 1){

        }

        if(gamepad1.dpad_left & rotatorServoPosition > 0){

        }


        if(armState == 0){

            arm2ServoPosition = arm2IntakePosition;
            armServoPosition = armIntakePosition;
            //rotatorGripPosition = rotatorGripIntakePosition;
        }
        else if(armState == 1){
            arm2ServoPosition = arm2RotatePosition;
            armServoPosition = armRotatePosition;
           // rotatorGripPosition = rotatorGripIntakePosition;
        }
        else if(armState == 2){
            arm2ServoPosition = arm2RotatePosition;
            armServoPosition = armRotatePosition;
         //   rotatorGripPosition = rotatorGripFlipPosition;
        }
        else if(armState == 3){
            arm2ServoPosition = arm2MaxHeightPosition;
            armServoPosition = armMaxHeightPosition;
            //rotatorGripPosition = rotatorGripFlipPosition;
            intakeIn = false;
            intakeOut = false;
        }
        else if(armState == 4){
            arm2ServoPosition = arm2Upper45Position;
            armServoPosition = armUpper45Position;
            //rotatorGripPosition = rotatorGripFlipPosition;
        }
        else if(armState == 5){
            arm2ServoPosition = arm2MaxReachPosition;
            armServoPosition = armMaxReachPosition;
            //rotatorGripPosition = rotatorGripFlipPosition;
        }
        else if(armState == 6){
            arm2ServoPosition = arm2DownPosition;
            armServoPosition = armDownPosition;
           // rotatorGripPosition = rotatorGripFlipPosition;
        }

//        leftGrip.setPosition(leftGripPosition);
//        rightGrip.setPosition(rightGripPosition);
//        rotatorGrip.setPosition(rotatorGripPosition);
        armServo.setPosition(armServoPosition);
        armServo2.setPosition(arm2ServoPosition);

    }

}
