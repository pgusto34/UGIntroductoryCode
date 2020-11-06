package org.firstinspires.ftc.teamcode.Main.OpModes.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.pow;

@Disabled
@TeleOp(name = "Gyro Drive")
public class DriveWithGyro extends TunableOpMode {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    Double lF, rF, lB, rB, maxVector;

    double heading;
    private BNO055IMU imu;
    Orientation angles;
    BNO055IMU.Parameters imuParameters = new  BNO055IMU.Parameters();

    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(imuParameters);
    }

    public void init_loop(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        telemetry.addData("rot about z", angles.firstAngle);
        telemetry.update();
    }

    public void loop() {
        heading = getAngle();
        //angles5 = imu5.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        mecanumMove(pow(gamepad1.left_stick_x, 3), pow(gamepad1.left_stick_y, 3), pow(gamepad1.right_stick_x, 3));

        telemetry.addData("rot about Z", heading);
        //telemetry.addData("IMU 5", angles5);
        telemetry.update();

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

    public double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        return heading;
    }

    public void resetZAxis(){
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imu.initialize(imuParams);
    }
}
