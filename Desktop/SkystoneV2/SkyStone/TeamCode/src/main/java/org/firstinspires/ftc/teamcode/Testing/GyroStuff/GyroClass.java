package org.firstinspires.ftc.teamcode.Testing.GyroStuff;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled
@TeleOp(name = "Test Gyro", group = "")
public abstract class GyroClass extends TunableOpMode {

    private BNO055IMU imu;
    Orientation angles;
    BNO055IMU.Parameters imuParameters;

    abstract
    public void init();

    public void init_loop(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        telemetry.addData("rot about z", angles.firstAngle);
        telemetry.update();
    }

    public void start(){
        resetZAxis();
    }


    public void loop() {
        // Get absolute orientation
        // Get acceleration due to force of gravity.

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Display orientation info.
        telemetry.addData("rot about Z", angles.firstAngle);

        telemetry.update();

    }

    public void resetZAxis(){
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imu.initialize(imuParams);
    }




}

