package org.firstinspires.ftc.teamcode.Testing.AutonomousSkyStonePaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous.AutonomousMovement;
@Disabled
@Autonomous(name = "SkystoneFoundation Auto")
public class SkystoneFoundationAuto extends AutonomousMovement {
    int skyStonePosition;

    double speed = .75;

    /**
     * Autonomous starts one tile away from the depot
     */

    public void runAutonomous(){
        try {
            //1 == Right Position, 2 == Middle Position, 3 == Left Position
            skyStonePosition = getSkyStonePositionRed();
            telemetry.addData("Skystone Position", getSkyStonePositionRed());

            controlGrips(true);
            controlArm(0);

            if(skyStonePosition == 1){
                telemetry.addData("Skystone Position: ", "Right");

                mecanumEMove(speed, 19, true);
                mecanumETurn(speed, 65, true);

                //Intake Skystone
                mecanumEStrafeLeft(speed, 12);
                runIntake(true);
                mecanumEMove(.5, 12, true);
                pause(750);
                controlGrips(false);
                mecanumEStrafeRight(speed, 12);
                mecanumETurn(speed, 205, true);

                //Return to end position
                mecanumEStrafeLeft(speed, 7);
                mecanumEMove(speed, 8, false);
            }
            else if(skyStonePosition == 2){
                telemetry.addData("Skystone Position: ", "Middle");

                mecanumEMove(speed, 16, true);
                mecanumETurn(speed, 90, false);
                mecanumEMove(speed, 8, false);

                //Intake Skystone
                mecanumEStrafeRight(speed, 21);
                runIntake(true);
                mecanumEMove(.5, 7, true);
                pause(750);
                controlGrips(false);
                mecanumEStrafeLeft(speed,16);
                mecanumEMove(speed, 12, false);

            }

            else if(skyStonePosition == 3){
                telemetry.addData("Skystone Position: ", "Left");

                mecanumEMove(speed, 16, true);
                mecanumETurn(speed, 90, false);
                mecanumEMove(speed, 4, false);

                //Intake Skystone
                mecanumEStrafeRight(speed, 21);
                runIntake(true);
                mecanumEMove(.5, 6, true);
                pause(750);
                controlGrips(false);
                mecanumEStrafeLeft(speed,17);
                mecanumEMove(speed, 19, false);
            }

            //Deposit the Skystone
            mecanumEMove(speed, 28, false);
            controlArm(1);
            pause(750);
            controlArm(6);
            mecanumEStrafeLeft(speed, 6);
            pause(1250);
            controlGrips(true);
            pause(500);
            controlArm(5);
            mecanumEStrafeRight(speed, 6);
            controlArm(0);
            pause(1000);

            mecanumEMove(speed, 18, true);


        }catch(Exception e){
            autoThread.interrupt();
            telemetry.addData("AUTONOMOUS WAS INTERRUPTED", e);
        }

    }
}
