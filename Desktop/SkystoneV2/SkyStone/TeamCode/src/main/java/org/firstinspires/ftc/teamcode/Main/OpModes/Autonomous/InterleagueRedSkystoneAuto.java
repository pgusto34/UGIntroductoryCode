package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED Skystone Auto")
public class InterleagueRedSkystoneAuto extends AutonomousMovement {
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

            if(skyStonePosition == 1){
                telemetry.log().add("Skystone Position: ", "Right");

                //controlArm(0);

                /** Intake Skystone 1**/
                controlArm(1);
                mecanumEMove(speed, 6, false);
                mecanumEStrafeLeft(speed,11);
                mecanumEMove(0.5, 17, false);
                pause(250);
                controlArm(2);
                pause(1000);
                controlArm(3);
                mecanumEMove(speed, 12, true);

                /**Deposit first Skystone**/
                mecanumEStrafeLeft(speed, 79);
                mecanumEMove(speed, 20, false);
                pause(500);
                controlGrips(true);
                mecanumEMove(speed, 10, true);

                /**Intake Second Skystone**/
                mecanumEStrafeLeft(speed, 32);
                controlArm(4);
                mecanumEStrafeRight(speed, 120);


            }
            else if(skyStonePosition == 2){
               telemetry.addData("Skystone Position: ", "Middle");

                controlArm(0);

                /** Intake Skystone 1**/
                controlArm(1);
                mecanumEMove(speed, 6, false);
                mecanumEStrafeRight(speed,1);
                mecanumEMove(0.5, 17, false);
                pause(250);
                controlArm(2);
                pause(1000);
                controlArm(3);
                mecanumEMove(speed, 10, true);

                /**Deposit first Skystone**/
                mecanumEStrafeLeft(speed, 82);
                mecanumEMove(speed, 20, false);
                pause(500);
                controlGrips(true);
                mecanumEMove(speed, 10, true);

                /**Intake Second Skystone**/
                mecanumEStrafeLeft(speed, 32);
                controlArm(4);
                mecanumEStrafeRight(speed, 120);



            }

            else if(skyStonePosition == 3){
                telemetry.addData("Skystone Position: ", "Left");

                controlArm(0);

                /** Intake Skystone 1**/
                controlArm(1);
                mecanumEMove(speed, 6, false);
                mecanumEStrafeRight(speed,5.5);
                mecanumEMove(0.5, 17, false);
                pause(250);
                controlArm(2);
                pause(1000);
                controlArm(3);
                mecanumEMove(speed, 12, true);

                /**Deposit first Skystone**/
                mecanumEStrafeLeft(speed, 89);
                mecanumEMove(speed, 20, false);
                pause(500);
                controlGrips(true);
                mecanumEMove(speed, 10, true);

                /**Intake Second Skystone**/
                mecanumEStrafeLeft(speed, 32);
                controlArm(4);
                mecanumEStrafeRight(speed, 120);


            }





        }catch(Exception e){
            autoThread.interrupt();
            telemetry.addData("AUTONOMOUS WAS INTERRUPTED", e);
        }

    }
}
