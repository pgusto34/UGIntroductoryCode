//package org.firstinspires.ftc.teamcode.Testing.AutonomousGyro;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//public class ThunderPenguinsExample extends OpMode {
//
//    public void turnWithGyro(double degrees, double speedDirection){
//        //<editor-fold desc="Initialize">
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double yaw = -angles.firstAngle;//make this negative
//        telemetry.addData("Speed Direction", speedDirection);
//        telemetry.addData("Yaw", yaw);
//        telemetry.update();
//        //
//        telemetry.addData("stuff", speedDirection);
//        telemetry.update();
//        //
//        double first;
//        double second;
//        //</editor-fold>
//        //
//        if (speedDirection > 0){//set target positions
//            //<editor-fold desc="turn right">
//            if (degrees > 10){
//                first = (degrees - 10) + devertify(yaw);
//                second = degrees + devertify(yaw);
//            }else{
//                first = devertify(yaw);
//                second = degrees + devertify(yaw);
//            }
//            //</editor-fold>
//        }else{
//            //<editor-fold desc="turn left">
//            if (degrees > 10){
//                first = devertify(-(degrees - 10) + devertify(yaw));
//                second = devertify(-degrees + devertify(yaw));
//            }else{
//                first = devertify(yaw);
//                second = devertify(-degrees + devertify(yaw));
//            }
//            //
//            //</editor-fold>
//        }
//        //
//        //<editor-fold desc="Go to position">
//        Double firsta = convertify(first - 5);//175
//        Double firstb = convertify(first + 5);//-175
//        //
//        turnWithEncoder(speedDirection);
//        //
//        if (Math.abs(firsta - firstb) < 11) {
//            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        }else{
//            //
//            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        }
//        //
//        Double seconda = convertify(second - 5);//175
//        Double secondb = convertify(second + 5);//-175
//        //
//        turnWithEncoder(speedDirection / 3);
//        //
//        if (Math.abs(seconda - secondb) < 11) {
//            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            frontleft.setPower(0);
//            frontright.setPower(0);
//            backleft.setPower(0);
//            backright.setPower(0);
//        }
//        //</editor-fold>
//        //
//        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    /*
// These functions are used in the turnWithGyro function to ensure inputs
// are interpreted properly.
//  */
//    public double devertify(double degrees){
//        if (degrees < 0){
//            degrees = degrees + 360;
//        }
//        return degrees;
//    }
//    public double convertify(double degrees){
//        if (degrees > 179){
//            degrees = -(360 - degrees);
//        } else if(degrees < -180){
//            degrees = 360 + degrees;
//        } else if(degrees > 360){
//            degrees = degrees - 360;
//        }
//        return degrees;
//    }
//    //
//    /*
//    This function is called at the beginning of the program to activate
//    the IMU Integrated Gyro.
//     */
//    public void initGyro(){
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        //
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//    }
//    //
//    /*
//    This function is used in the turnWithGyro function to set the
//    encoder mode and turn.
//     */
//    public void turnWithEncoder(double input){
//        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //
//        frontleft.setPower(input);
//        backleft.setPower(input);
//        frontright.setPower(-input);
//        backright.setPower(-input);
//    }
//}
