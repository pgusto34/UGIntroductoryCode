package org.firstinspires.ftc.teamcode.Main.OpModes.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.*;

@Disabled
@TeleOp(name = "Drive")
public class Drive extends TunableOpMode {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    Double lF, rF, lB, rB, maxVector;


    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

    }


    public void loop() {
        mecanumMove(pow(gamepad1.left_stick_x, 3), pow(gamepad1.left_stick_y, 3), pow(gamepad1.right_stick_x, 3));
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
