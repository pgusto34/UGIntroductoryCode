package org.firstinspires.ftc.robotcontroller.GRsamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


/** Example of using multiple threads **/
@Disabled
@Autonomous(name = "MultiThread Auto", group = "test")
public class MultiThreadAuto extends OpMode {
    double COUNTS_PER_INCH = 512;

    DcMotor leftFront, leftBack, rightFront, rightBack;

    Thread moveThread = new Thread(new Runnable() {
        @Override
        public void run() {
          moveForward();
        }
    });

    Thread turnThread = new Thread(new Runnable() {
        @Override
        public void run() {
           turn360();
        }
    });


    @Override
    public void init(){
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront  = hardwareMap.dcMotor.get("leftFront");
        rightBack  = hardwareMap.dcMotor.get("rightBack");
        leftBack   = hardwareMap.dcMotor.get("leftBack");
    }

    public void loop(){}

    @Override
    public void start(){
        moveThread.start();
        turnThread.start();

    }

    public void stop(){
        try{
            moveThread.interrupt();
            turnThread.interrupt();
            moveThread.join();
            turnThread.join();
        }catch(InterruptedException e){
            telemetry.addData("ENCOUNTERED AN Closing the program", e);
        }
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

    protected void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveForward(){
        try{
        mecanumEMove(1, 48, true);
        }catch(Exception e){
            moveThread.interrupt();
            telemetry.addData("moveForward Interrupted", e);
        }
    }

    public void turn360(){
        try{
            mecanumETurn(.5, 360, true);
        }catch (Exception e){
            turnThread.interrupt();
            telemetry.addData("turn360 Interrupted", e);
        }
    }

}
