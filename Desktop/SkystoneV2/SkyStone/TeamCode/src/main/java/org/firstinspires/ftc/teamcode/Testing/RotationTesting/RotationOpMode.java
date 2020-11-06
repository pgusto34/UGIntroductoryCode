package org.firstinspires.ftc.teamcode.Testing.RotationTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
@Disabled
@Autonomous(name = "Test Rotation", group = "test")
public class RotationOpMode extends RobotRotation {
    public void runAutonomous(){
        try {
            strafe45();

        }catch(Exception e){
            singleThread.interrupt();
            telemetry.addData("AUTONOMOUS WAS INTERRUPTED", e);
        }
    }

    public void rotate90(){
        mecanumETurn(1, 90, false);
    }

    public void rotate180(){
        mecanumETurn(.5, 180, true);
    }

    public void rotate270(){
        mecanumETurn(1, 270, false);
    }

    public void rotate360(){
        mecanumETurn(1, 360, true);
    }


        int Quadrant;
        Double motorPower;

        private ElapsedTime runtime = new ElapsedTime();

        protected Thread autoThread = new Thread(new Runnable() {
            @Override
            public void run() {
                runAutonomous();
            }
        });

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


        public void strafe45(){
            strafe(45, 1, 3);
        }


}
