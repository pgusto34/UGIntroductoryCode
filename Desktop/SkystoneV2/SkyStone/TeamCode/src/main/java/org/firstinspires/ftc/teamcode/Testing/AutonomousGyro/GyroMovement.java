package org.firstinspires.ftc.teamcode.Testing.AutonomousGyro;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static java.lang.StrictMath.abs;

//import org.firstinspires.ftc.teamcode.Main.HelperClasses.Stats.BaseOpMode;
//
//import static org.firstinspires.ftc.teamcode.Main.HelperClasses.Stats.BaseOpMode.*;

@Disabled
@Autonomous(name = "Gyro Auto")
public class GyroMovement extends TunableOpMode {

    /**
     * Created by union on 18年9月28日.
     */
    private ElapsedTime visionRuntime = new ElapsedTime();

    protected DcMotor rightFront, leftFront, leftBack, rightBack;

    static final double COUNTS_PER_MOTOR_REV = 512;
    static final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
    static final double ROBOT_DIAMETER = 27.71; //Robot diameter was measured as 19 inches
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    protected double lF, rF, lB, rB, maxVector;

    //Gyro
    double heading = 0;
    private BNO055IMU imu;
    Orientation angles;
    BNO055IMU.Parameters imuParameters = new  BNO055IMU.Parameters();

    private ElapsedTime runtime = new ElapsedTime();


    private long maxTime;

    public void init(){

        initGyro();

        //Wheel Motors
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront  = hardwareMap.dcMotor.get("leftFront");
        rightBack  = hardwareMap.dcMotor.get("rightBack");
        leftBack   = hardwareMap.dcMotor.get("leftBack");

    }

    public void init_loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        telemetry.addData("rot about z", angles.firstAngle);
        telemetry.update();
    }

    @Override
    public void start(){
    }

    public void loop(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        if(heading < 0) heading += 360;
        telemetry.addData("Heading: ", heading);
        if(abs(heading - 360) > 10 && !(heading - 10 > 0)) {
            encoderTurn(.5, true);
        }else if(heading - 10 > 0) encoderTurn(.5, false);
        else encoderTurn(0, true);
    }


        public void mecanumEMove(double speed, double distance, boolean forward) {

            /** distance is in INCHES **/

            setMotorRunMode(STOP_AND_RESET_ENCODER);

            int newTargetLF;
            int newTargetLB;
            int newTargetRF;
            int newTargetRB;

            if (forward) {
                newTargetLF = leftFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
                newTargetLB = leftBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
                newTargetRF = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
                newTargetRB = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            } else {
                newTargetLF = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
                newTargetLB = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
                newTargetRF = rightFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
                newTargetRB = rightBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            }

            leftFront.setTargetPosition(newTargetLF);
            leftBack.setTargetPosition(newTargetLB);
            rightFront.setTargetPosition(newTargetRF);
            rightBack.setTargetPosition(newTargetRB);


            setMotorRunMode(RUN_TO_POSITION);

            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);

            try {
                while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
                }
            } catch (Exception e) {
                telemetry.addData("Error", e);
            }

            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            setMotorRunMode(RUN_USING_ENCODER);

        }


        public void mecanumETurn(double speed, double degrees, boolean clockwise) {
            /** distance is in INCHES **/

            setMotorRunMode(STOP_AND_RESET_ENCODER);

            degrees = ((ROBOT_DIAMETER * Math.PI) / 360) * degrees;

            int newTargetLF;
            int newTargetLB;
            int newTargetRF;
            int newTargetRB;

            if (clockwise) {
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

            setMotorRunMode(RUN_TO_POSITION);

            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);

            try {
                while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
                }
            } catch (Exception e) {
                telemetry.addData("Error", e);
            }

            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

           setMotorRunMode(RUN_USING_ENCODER);
        }




    public void initGyro(){
        // Use degrees as angle unit.
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(imuParameters);
    }

    public void veerCheck(double angle){
//
//
//            if(angle > (heading + 2)) {
//                encoderTurn(.5, false);
//            }else encoderTurn(0, false);
//
//            if(angle < (heading - 2)){
//                encoderTurn(.5, true);
//            } else encoderTurn(0, true);

        }


    protected void pause(long millis){
        maxTime = System.currentTimeMillis() + millis;
        while(System.currentTimeMillis() < maxTime) {}
        }


    public void encoderTurn(double power, boolean clockwise){
        setMotorRunMode(RUN_USING_ENCODER);

        if(clockwise) {
            leftFront.setPower(-power);
            rightFront.setPower(-power);
            leftBack.setPower(-power);
            rightBack.setPower(-power);
        }
        else{
            leftFront.setPower(power);
            rightFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(power);
        }

    }

    public void setMotorRunMode(DcMotor.RunMode runMode){
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
    }

    public double angle360(double degrees){
        if(degrees < 0) degrees += 360;
        else if(degrees > 360) degrees -= 360;

        return degrees;
    }

    public double angle180(double degrees){
        if(degrees > 179) degrees = -(360 - degrees);
        else if(degrees < -180) degrees += 360;
        else if(degrees > 360) degrees -= 360;

        return degrees;
    }

    public void gyroTurn(double degrees, double speed, boolean clockwise){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;

        double first;
        double second;


        if(clockwise){
            if(degrees > 10){
                first = (degrees - 10) + angle360(yaw);
                second = degrees + angle360(yaw);
            }
            else{
                first = angle360(yaw);
                second = degrees + angle360(yaw);
            }
        }
        else{
            if(degrees < 10){
                first = angle360(-(degrees - 10) + angle360(yaw));
                second = angle360(-degrees + angle360(yaw));
            }
            else{
                first = angle360(yaw);
                second = angle360(-degrees + angle360(yaw));
            }
        }


        double first1 = angle180(first - 5);
        double first2 = angle180(first + 5);


        encoderTurn(speed, clockwise);

        if(abs(first1 - first2) < 11) {
            while (!(first1 < yaw && yaw < first2))

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            yaw = -angles.firstAngle;
            telemetry.addData("Heading: ", yaw);
            telemetry.addData("Target", angle180(first));

            } else{

            while(!(first1 < yaw && yaw < 180) || (-180 < yaw && yaw < first2)){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
                yaw = -angles.firstAngle;
                telemetry.addData("Heading: ", yaw);
                telemetry.addData("Target", angle180(first));
            }
        }

        double second1 = angle180(second - 5);
        double second2 = angle180(second + 5);

        encoderTurn(speed/3, clockwise);

        if(abs(second1 - second2) < 11) {
            while (!(second1 < yaw && yaw < second2))

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            yaw = -angles.firstAngle;
            telemetry.addData("Heading: ", yaw);
            telemetry.addData("Target", angle180(first));

            while (!(second1 < yaw && yaw < 180) || (-180 < yaw && yaw < second2)) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
                yaw = -angles.firstAngle;
                telemetry.addData("Heading: ", yaw);
                telemetry.addData("Target", angle180(first));
            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }

        setMotorRunMode(STOP_AND_RESET_ENCODER);
        setMotorRunMode(RUN_TO_POSITION);

    }



}
