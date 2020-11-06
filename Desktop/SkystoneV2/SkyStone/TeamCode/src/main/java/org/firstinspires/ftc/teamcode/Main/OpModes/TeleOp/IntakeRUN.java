package org.firstinspires.ftc.teamcode.Main.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "IntakeRUN", group = "test")
public class IntakeRUN extends OpMode {

    DcMotor leftLift, rightLift;

    public void init() {
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");
    }

    public void loop(){
       leftLift.setPower(-gamepad1.left_stick_y);
       rightLift.setPower(-gamepad1.right_stick_y);
    }
}
