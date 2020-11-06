package org.firstinspires.ftc.teamcode.Testing.StrafeTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import java.lang.annotation.ElementType;
@Disabled
@Autonomous(name = "STRAAFEE")
public /*abstract*/ class StrafeOpMode extends TunableOpMode {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    Double lF, rF, lB, rB, maxVector;

    int Quadrant;
    Double motorPower;

    private ElapsedTime runtime = new ElapsedTime();

//    protected Thread autoThread = new Thread(new Runnable() {
//        @Override
//        public void run() {
//            runAutonomous();
//        }
//    });

    public void loop(){
        telemetry.addData("45", giveMeMotorPowers(45));
        telemetry.addData("140", giveMeMotorPowers((140)));
        telemetry.addData("200", giveMeMotorPowers(200));
        telemetry.addData("300", giveMeMotorPowers(300));

    }

    double[] motorPowers = new double[2];

    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void start() {


     strafe(123, 0.5, 750);



    }

    public void strafeDrive(double angle, double speed){
        angle = angleWrap(angle);

        giveMeAngleQuadrant(angle);

        switch(Quadrant) {
            case 1:
                motorPowers[0] = 1 * speed;
                motorPowers[1] = giveMeMotorPowers(angle) * speed;

                break;

            case 2:
                motorPowers[0] = giveMeMotorPowers(angle) * speed;
                motorPowers[1] = 1 * speed;

                break;

            case 3:
                motorPowers[0] = -1 * speed;
                motorPowers[1] = giveMeMotorPowers(angle) * speed;

                break;

            case 4:
                motorPowers[0] = giveMeMotorPowers(angle) * speed;
                motorPowers[1] = -1 * speed;

                break;

        }

    }

    public void strafe(double angle, double speed, double time) {

        angle = angleWrap(angle);

        giveMeAngleQuadrant(angle);

        switch(Quadrant) {
            case 1:
                motorPowers[0] = 1 * speed;
                motorPowers[1] = giveMeMotorPowers(angle) * speed;

                break;

            case 2:
                motorPowers[0] = giveMeMotorPowers(angle) * speed;
                motorPowers[1] = 1 * speed;

                break;

            case 3:
                motorPowers[0] = -1 * speed;
                motorPowers[1] = giveMeMotorPowers(angle) * speed;

                break;

            case 4:
                motorPowers[0] = giveMeMotorPowers(angle) * speed;
                motorPowers[1] = -1 * speed;

                break;

        }

        leftFront.setPower(motorPowers[0]);
        rightFront.setPower(motorPowers[1]);
        leftBack.setPower(motorPowers[1]);
        rightBack.setPower(motorPowers[0]);

        runtime.reset();
        while(runtime.milliseconds() < time){
            telemetry.addData("Milliseconds Remaining: ", (time - runtime.milliseconds()));
            telemetry.addData("Front Left: ", motorPowers[0]);
            telemetry.addData("Front Right: ", motorPowers[1]);
            telemetry.addData("Back Left: ", motorPowers[1]);
            telemetry.addData("Back Right: ", motorPowers[0]);

        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

//        runtime.reset();
//        while(runtime.milliseconds() < 500){
//            telemetry.addData("Paused to prevent slippage", "");
//        }

    }

    public double giveMeMotorPowers(double angle){

        angle = angleWrap(angle);

        giveMeAngleQuadrant(angle);

        double a =  0.00000251762;
        double b = -0.000340768;
        double c =  0.0323741;
        double d = -0.996608;

        if(Quadrant == 1){
            motorPower = ((a*Math.pow(angle, 3)) + (b*Math.pow(angle, 2)) + (c*angle + d));
        }else if(Quadrant == 2){
            motorPower = -(a*Math.pow((angle - 90), 3) + b*(Math.pow((angle - 90), 2)) + c*(angle - 90) + d);
        }else if(Quadrant == 3){
            motorPower = -(a*Math.pow((angle - 180), 3) + b*(Math.pow((angle - 180), 2)) + c*(angle - 180) + d);
        }else if(Quadrant == 4){
            motorPower = a*Math.pow((angle - 270), 3) + b*(Math.pow((angle - 270), 2)) + c*(angle - 270) + d;
        }


        return motorPower;
    }

    public double angleWrap(double angle){
        while (angle > 360) {
            angle -= 360;
        } while(angle < 0){
            angle += 360;
        }

        return angle;
    }

    public int giveMeAngleQuadrant(double angle){
        angleWrap(angle);

        if (angle <= 90) {
            Quadrant = 1;
        } else if (angle <= 180) {
            Quadrant = 2;
        } else if (angle <= 270) {
            Quadrant = 3;
        } else if (angle <= 360) {
            Quadrant = 4;
        }

        return Quadrant;
    }

  // public abstract void runAutonomous();
}
