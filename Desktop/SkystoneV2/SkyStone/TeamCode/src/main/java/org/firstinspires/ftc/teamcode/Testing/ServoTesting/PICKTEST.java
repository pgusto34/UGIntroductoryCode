package org.firstinspires.ftc.teamcode.Testing.ServoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
@Disabled
@TeleOp(name = "PICKServoIntakeTuner", group = "test")
public class PICKTEST extends TunableOpMode {
    Servo rightPick, leftPick;

    boolean lastA, lastB;

    double leftPickUpPosition = 0.34;
    double leftPickDownPosition = 0.729;

    double rightPickUpPosition = 0.472;
    double rightPickDownPosition = 0.147;

    double leftPickPosition = 0, rightPickPosition = 0;

    public void init(){
        rightPick = hardwareMap.servo.get("rightPick");
        leftPick = hardwareMap.servo.get("leftPick");
    }

    public void loop(){

        if(gamepad1.right_bumper){
            leftPick.setPosition(leftPickDownPosition);
            rightPick.setPosition(rightPickDownPosition);
        }else{
            leftPick.setPosition(leftPickUpPosition);
            rightPick.setPosition(rightPickUpPosition);
        }


        telemetry.addData("Right Pick", rightPickPosition);
        telemetry.addData("Left Pick", leftPickPosition);

        telemetry.update();

    }
}
