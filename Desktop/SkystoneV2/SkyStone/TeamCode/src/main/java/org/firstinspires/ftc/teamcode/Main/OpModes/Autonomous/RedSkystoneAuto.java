package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "RED Skystone Auto")
public class RedSkystoneAuto extends AutonomousMovement {
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
                telemetry.addData("Skystone Position: ", "Right");
                controlGrips(true);

                /**Spit the Cap Stone out**/
                mecanumEMove(speed, 4, true);
                mecanumETurn(1, 90, true);
                runIntake(false);
                pause(750);
                controlArm(0);
                mecanumEMove(speed, 10.5, false);
                mecanumEStrafeRight(0.4,10);

                /**Grab a Stone**/
                controlGrips(true);
                runIntake(true);
                mecanumEStrafeLeft(speed, 46.5);
                mecanumEMove(.3, 12, true);
                pause(500);
                controlGrips(false);
                mecanumEStrafeRight(speed, 15.5);

                /**Move to Foundation**/
            mecanumETurn(speed, 173, true);
                mecanumEMove(speed, 70, false);
                mecanumEStrafeLeft(.5, 3);
                mecanumETurn(speed, 90, false);
                mecanumEMove(.5, 3, false);
                controlArm(1);
                mecanumEMove(0.3, 14, false);

                /**Deposit stone and move foundation to build site**/
                movePicks(false);
                controlArm(2);
                pause(1000);
                mecanumETurn(speed, 11, true);
                controlArm(6);
                mecanumEMove(0.4, 49, true);
                mecanumETurn(speed, 15, true);
                controlGrips(true);
                movePicks(true);
                pause(250);
                controlArm(0);

                /**Park**/
                mecanumEStrafeRight(speed, 30);
                mecanumEMove(speed, 21, false);
                mecanumEStrafeRight(speed, 17);
            }
            else if(skyStonePosition == 2){

                /**Spit out Capstone**/
                controlGrips(true);
                mecanumEMove(speed, 2, true);
                mecanumETurn(1, 90, true);
                runIntake(false);
                pause(750);
                mecanumETurn(0.75, 180, false);
                controlArm(0);
                mecanumEStrafeLeft(0.4,10);
                mecanumEMove(speed, 8, false);

                /**Intake Stone **/
                controlGrips(true);
                runIntake(true);
                mecanumEStrafeRight(speed, 44);
                mecanumEMove(.5, 6, true);
                pause(500);
                controlGrips(false);
                mecanumEStrafeLeft(speed, 19);
                mecanumEMove(speed, 70, false);
                mecanumEStrafeLeft(.5, 3);
                mecanumETurn(speed, 90, false);
                mecanumEMove(.5, 3, false);
                controlArm(1);
                mecanumEMove(0.3, 12, false);
                movePicks(false);
                controlArm(2);
                pause(1000);
                controlArm(6);
                mecanumEMove(0.4, 49, true);
                controlGrips(true);
                movePicks(true);
                pause(250);
                controlArm(0);
                mecanumEStrafeRight(speed, 30);
                mecanumEMove(speed, 21, false);
                mecanumEStrafeRight(speed, 17);




            }

            else if(skyStonePosition == 3){

                controlGrips(true);

                mecanumEMove(speed, 4, true);
                mecanumETurn(1, 90, true);
                runIntake(false);
                pause(750);
                mecanumETurn(0.75, 180, false);
                controlArm(0);
                mecanumEStrafeLeft(0.4,10);

              //  mecanumEMove(speed, 2, false);
                controlGrips(true);
                runIntake(true);
                mecanumEStrafeRight(speed, 44);
                mecanumEMove(.5, 6, true);
                pause(500);
                controlGrips(false);
                mecanumEStrafeLeft(speed, 18);
                mecanumEMove(speed, 77, false);
                mecanumEStrafeLeft(.5, 3);
                mecanumETurn(speed, 90, false);
                mecanumEMove(.5, 3, false);
                controlArm(1);
                mecanumEMove(0.3, 12, false);
                movePicks(false);
                controlArm(2);
                pause(1000);
                controlArm(6);
                mecanumETurn(1, 15, true);
                mecanumEMove(0.4, 49, true);
                controlGrips(true);
                movePicks(true);
                pause(250);
                controlArm(0);
                mecanumEStrafeRight(speed, 30);
                mecanumEMove(speed, 21, false);
                mecanumEStrafeRight(speed, 17);

            }





        }catch(Exception e){
            autoThread.interrupt();
            telemetry.addData("AUTONOMOUS WAS INTERRUPTED", e);
        }

    }
}
