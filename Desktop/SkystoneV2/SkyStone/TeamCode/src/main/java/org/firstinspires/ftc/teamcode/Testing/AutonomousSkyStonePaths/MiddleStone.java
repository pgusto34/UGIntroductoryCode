package org.firstinspires.ftc.teamcode.Testing.AutonomousSkyStonePaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous.AutonomousMovement;
@Disabled
@Autonomous(name = "GetSkystoneMiddle")
public class MiddleStone extends AutonomousMovement {

    double speed = .75;

    public void runAutonomous(){
        try {
            mecanumEMove(speed, 26, true);
            controlArm(0);
            controlGrips(true);
            runIntake(true);
            mecanumEStrafeRight(speed, 15);
            mecanumEMove(.5, 8, true);
            pause(500);
            controlGrips(false);
            mecanumEStrafeLeft(speed, 18);
            mecanumEMove(speed, 74, false);
            controlArm(6);
            pause(1000);
            mecanumEStrafeLeft(speed, 1);
            controlGrips(true);
            pause(500);
            mecanumEStrafeRight(speed, 1);
            mecanumEMove(speed, 26, true);

        }catch(Exception e){
            autoThread.interrupt();
            telemetry.addData("AUTONOMOUS WAS INTERRUPTED", e);
        }

    }
}
