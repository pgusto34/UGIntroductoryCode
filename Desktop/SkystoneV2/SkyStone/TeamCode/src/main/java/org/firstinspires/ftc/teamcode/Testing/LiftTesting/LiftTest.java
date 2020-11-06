package org.firstinspires.ftc.teamcode.Testing.LiftTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
@Disabled
@TeleOp(name = "Lift Test", group = "Test")
public class LiftTest extends OpMode {

    DcMotor leftLift, rightLift;
    int liftTarget;
    int COUNTS_PER_INCH = 500;
    double height = 0;
    boolean lastRightBumper, lastLeftBumper;

    public void init(){
        //Lift Motors
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");
    }

    public void loop(){

        lift(4);

        telemetry.addData("Left Encoder: ", leftLift.getCurrentPosition());
        telemetry.addData("Right Encoder: ", rightLift.getCurrentPosition());

        if (gamepad1.right_bumper && !lastRightBumper) {
            height += 0.1;
        }
        if (gamepad1.right_bumper) lastRightBumper = true;
        else lastRightBumper = false;

        if (gamepad1.left_bumper && !lastLeftBumper) {
            if(height > 0) height -= 0.1;
        }
        if (gamepad1.left_bumper) lastLeftBumper = true;
        else lastLeftBumper = false;
    }

    public void lift(double height){
        liftTarget = (int)(height * COUNTS_PER_INCH);
        leftLift.setTargetPosition(liftTarget);
        rightLift.setTargetPosition(liftTarget);
        setMotorRunMode(RUN_TO_POSITION);
        leftLift.setPower(.5);
        rightLift.setPower(.5);
    }


    public void controlPowerLift(){
        setMotorRunMode(RUN_USING_ENCODER);
        if(gamepad1.right_bumper){
            leftLift.setPower(1);
            rightLift.setPower(1);
        }else if(gamepad1.left_bumper){
            leftLift.setPower(-1);
            rightLift.setPower(-1);
        }else{
            leftLift.setPower(0);
            rightLift.setPower(0);
        }
    }

    //Height is in inches
    public void controlEncoderLift(double height, boolean up){
        setMotorRunMode(STOP_AND_RESET_ENCODER);
        setMotorRunMode(RUN_USING_ENCODER);
        liftTarget += (int)(height * COUNTS_PER_INCH);
        leftLift.setTargetPosition(liftTarget);
        rightLift.setTargetPosition(liftTarget);
        setMotorRunMode(RUN_TO_POSITION);
        if(up) {
            leftLift.setPower(1);
            rightLift.setPower(1);
        }
        else{
            leftLift.setPower(-1);
            rightLift.setPower(-1);
        }

        while(leftLift.isBusy() && rightLift.isBusy()){}

        leftLift.setPower(0);
        rightLift.setPower(0);

        setMotorRunMode(STOP_AND_RESET_ENCODER);
    }

    public void setMotorRunMode(DcMotor.RunMode runMode){
        leftLift.setMode(runMode);
        rightLift.setMode(runMode);
    }

}
