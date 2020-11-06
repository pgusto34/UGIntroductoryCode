package org.firstinspires.ftc.teamcode.Testing.AutonomousSkyStonePaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous.AutonomousMovement;
import org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous.AutonomousMovementGyro;


@Autonomous(name = "GetSkystoneRight")
public class RightStone extends AutonomousMovementGyro {

    double speed = .5;

    public void runAutonomous(){
        try {

            mecanumEMove(speed, 8, false);
            gyroEStrafeRight(speed,10);
            mecanumEMove(speed, 18, false);
            mecanumEMove(speed, 8, true);
            gyroEStrafeLeft(speed, 115);
            mecanumEMove(speed, 8, false);
            mecanumEMove(speed, 8, true);
            gyroEStrafeRight(speed, 79);
            mecanumEMove(speed, 2, false);
            mecanumEMove(speed, 4, true);
            gyroEStrafeLeft(speed, 79);
            mecanumEMove(speed, 8, false);
            mecanumEMove(speed, 8, true);
            gyroEStrafeRight(speed, 87);
            mecanumEMove(speed, 2, false);
            mecanumEMove(speed, 4, true);
            gyroEStrafeLeft(speed, 87);
            mecanumEMove(speed, 8, false);
            mecanumEMove(speed, 8, true);
            mecanumEStrafeRight(speed, 50);















//            mecanumEMove(speed, 16, true);
//            controlArm(0);
//            controlGrips(true);
//            runIntake(true);
//            mecanumEStrafeRight(speed, 15);
//            mecanumEMove(.5, 8, true);
//            pause(500);
//            controlGrips(false);
//            mecanumEStrafeLeft(speed, 19);
//            mecanumEMove(speed, 70, false);
//            controlArm(6);
//            pause(1000);
//            controlGrips(true);
//            pause(500);
//            mecanumEStrafeRight(speed, 1);
//            mecanumEMove(speed, 26, true);

        }catch(Exception e){
            autoThread.interrupt();
            telemetry.addData("AUTONOMOUS WAS INTERRUPTED", e);
        }

    }
}
