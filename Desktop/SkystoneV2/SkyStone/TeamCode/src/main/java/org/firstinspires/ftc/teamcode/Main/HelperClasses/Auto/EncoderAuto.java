package org.firstinspires.ftc.teamcode.Main.HelperClasses.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.Main.HelperClasses.Stats.BaseOpMode;
//
//import static org.firstinspires.ftc.teamcode.Main.HelperClasses.Stats.BaseOpMode.*;

import java.util.concurrent.TimeUnit;

public abstract class EncoderAuto extends OpMode {

    /**
     * Created by union on 18年9月28日.
     */
        protected DcMotor rightFront, leftFront, leftBack, rightBack;
        protected DcMotor extender;
        protected DcMotor rotator, rotator1;
        protected DcMotor grabber;

        protected Servo knocker;

        //static final double COUNTS_PER_MOTOR_REV = 1120;    // < DRIVE BASE Neverest 40:1
        //static final double COUNTS_PER_MOTOR_REV = 2240; // < ROTATOR REV MOTORS
        static final double COUNTS_PER_MOTOR_REV = 512;
        static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        static final double DRIVE_GEAR_REDUCTION2 = 4.0;     // This is < 1.0 if geared UP
        static final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
        static final double COUNTS_PER_ANGLE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION2) / (360);

        protected double lF, rF, lB, rB, maxVector;

        protected double lFE, rFE, lBE, rBE, maxVectorE;

        protected Thread singleThread = new Thread(new Runnable() {
            @Override
            public void run() {
                runAutonomous();
            }
        });

        private long maxTime;

        public void init(){
            rightFront = hardwareMap.dcMotor.get("rightFront");
            leftFront  = hardwareMap.dcMotor.get("leftFront");
            rightBack  = hardwareMap.dcMotor.get("rightBack");
            leftBack   = hardwareMap.dcMotor.get("leftBack");
        }

        @Override
        public void start(){
            singleThread.start();
        }

        @Override
        public void stop(){
            try{
                singleThread.interrupt();
                singleThread.join();
            }catch(InterruptedException e){
                telemetry.addData("ENCOUNTERED AN EXCEPTION", e);
            }

        }

        protected void resetEncoders() {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        public void mecanumEMove(double speed, double distance, boolean forward) {

            /** distance is in INCHES **/

            resetEncoders();

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
                newTargetLF = leftFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
                newTargetLB = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
                newTargetRF = rightFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
                newTargetRB = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            }

            leftFront.setTargetPosition(newTargetLF);
            leftBack.setTargetPosition(newTargetLB);
            rightFront.setTargetPosition(newTargetRF);
            rightBack.setTargetPosition(newTargetRB);


            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        protected void mecanumETurn(double speed, double degrees, boolean clockwise) {
            /** distance is in INCHES **/

            resetEncoders();

            degrees = ((26.5 * Math.PI) / 360) * degrees;

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

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public abstract void runAutonomous();

        protected void pause(long millis){
        maxTime = System.currentTimeMillis() + millis;
        while(System.currentTimeMillis() < maxTime && !singleThread.isInterrupted()) {}
        }


        public void alignLeft(){
            mecanumETurn(0.5, 135, true);
            //pause(1000);
            mecanumEMove(0.5, 12, true);
            //pause(200);
            mecanumEMove(1.0, 48, false);
            //pause(200);
            mecanumETurn(0.5, 15, true);
        }

        public void loop(){}

        public void AmyOpmodeRed(){
            mecanumEMove(0.5, 60, true);
            mecanumETurn(0.5, 90, true);
            mecanumEMove( 0.5,  84,  true);
            mecanumETurn(.5, 90,true);
            mecanumEMove(.5,60,true);
            mecanumETurn(.5, 90, false);
            mecanumEMove(.5,36,true);
        }

        public void ColtonOpMode(){
            mecanumEMove(.5,48, true);
            mecanumEMove(.5, 20, false);
            mecanumETurn(.5, 90, false);
            mecanumEMove(.5,76,true);
            mecanumETurn(.5, 90, true);
            mecanumEMove(.5, 16, true);
            mecanumEMove(.5,45.3, false);
            mecanumETurn(.5, 118, true );
            mecanumEMove(.5, 95.5, true);
            mecanumETurn(.5, 61.6, true);
            mecanumEMove(.5, 60.9, true);


        }

        public void TristanOpModeBlue(){
            mecanumEMove(.7,55,true);
            mecanumETurn(.6,150,false);
            mecanumEMove(.7,42,true);
            mecanumETurn(.6,90,true);
            mecanumEMove(.8,35, false);
            mecanumETurn(.6,135,true);
            mecanumEMove(.7,15,true);
            mecanumETurn(.6,90,true);
            mecanumEMove(.9,60,true);

        }

        public void MoveOneBlock(){
            mecanumEMove(.5, 24, true);
        }


        /**Copy and paste this segment to create an encoder OpMode**/
 /*

        package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;


        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Main.HelperClasses.Auto.EncoderAuto;

    @Autonomous(name = "NAME HERE")
    @Disabled
    public class EXAMPLEEncoderAuto extends EncoderAuto {

        public void runAutonomous(){
            try {

            }catch(Exception e){
                singleThread.interrupt();
                telemetry.addData("AUTONOMOUS WAS INTERRUPTED", e);
            }

        }
    } */

}
