package org.firstinspires.ftc.teamcode.Testing.AutonomousSkyStonePaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous.AutonomousMovement;
@Disabled
@Autonomous(name = "GetSkystoneLeft")
public class LeftStone extends AutonomousMovement {

    double speed = .75;

    public void runAutonomous(){
        try {
            mecanumEMove(speed, 36, true);
            controlArm(0);
            controlGrips(true);
            runIntake(true);
            mecanumEStrafeRight(speed, 15);
            mecanumEMove(.5, 6, true);
            pause(500);
            controlGrips(false);
            mecanumEStrafeLeft(speed, 18);
            mecanumEMove(speed, 74, false);
            controlArm(6);
            pause(1000);
            mecanumEStrafeLeft(speed, 4);
            controlGrips(true);
            pause(500);
            mecanumEStrafeRight(speed, 4);
            mecanumEMove(speed, 26, true);


        }catch(Exception e){
            autoThread.interrupt();
            telemetry.addData("AUTONOMOUS WAS INTERRUPTED", e);
        }

    }
}
