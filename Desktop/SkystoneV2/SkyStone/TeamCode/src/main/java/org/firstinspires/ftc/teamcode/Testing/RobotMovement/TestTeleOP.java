package org.firstinspires.ftc.teamcode.Testing.RobotMovement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.jar.Attributes;

import static java.lang.Math.PI;
import static java.lang.Math.sin;
import static java.lang.Math.*;

@Disabled
@TeleOp(name = "GetVMultiplier", group = "test")
//@TeleOp(name = "Test Mecanum Movement", group = "test")
public class TestTeleOP extends MecanumMovement {
    double vMultiRight, vMultiLeft;
    double angle;
    double turn;

    @Override
    public void init() {

    }

    public void loop(){
        vMultiRight = sin(atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + PI/4);
        vMultiLeft = cos(atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + PI/4);
        telemetry.addData("vMultiplierRight", vMultiRight);
        telemetry.addData("vMultiplierLeft", vMultiLeft);
    }

    public void start(){}

    public void stop(){}

}
