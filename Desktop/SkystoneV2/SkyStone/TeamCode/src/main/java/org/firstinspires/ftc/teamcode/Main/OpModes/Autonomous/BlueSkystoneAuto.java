package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE Skystone Auto")
public class BlueSkystoneAuto extends AutonomousMovement {
    int skyStonePosition;

    double speed = .75;

    /**
     * Autonomous starts one tile away from the depot
     */

    public void runAutonomous(){
        try {
            //1 == Right Position, 2 == Middle Position, 3 == Left Position
            skyStonePosition = getSkyStonePositionBlue();
            telemetry.addData("Skystone Position", getSkyStonePositionBlue());

            if(skyStonePosition == 3){
                telemetry.addData("Skystone Position: ", "Right");

                controlGrips(true);

                mecanumEMove(speed, 6, true);
                mecanumETurn(1, 90, false);
                runIntake(false);
                pause(750);
                controlArm(0);
//                mecanumEStrafeRight(0.4,10);
                //mecanumEMove(speed, 1, false);
                mecanumEStrafeLeft(0.4,10);

                controlGrips(true);
                runIntake(true);
                mecanumEStrafeRight(speed, 46.5);
                mecanumEMove(.3, 12, true);
                pause(500);
                controlGrips(false);
                mecanumEStrafeLeft(speed, 19.5);

                mecanumETurn(speed, 173, false);
                mecanumEMove(speed, 70, false);
                mecanumEStrafeRight(.5, 3);
                mecanumETurn(speed, 90, true);
                mecanumEMove(.5, 3, false);
                controlArm(1);
                mecanumEMove(0.3, 14, false);
                movePicks(false);
                controlArm(2);
                pause(1000);
                mecanumETurn(speed, 11, false);
                controlArm(6);
                mecanumEMove(0.4, 49, true);
                mecanumETurn(speed, 15, false);
                controlGrips(true);
                movePicks(true);
                pause(250);
                controlArm(0);
                mecanumEStrafeLeft(speed, 30);
                mecanumEMove(speed, 21, false);
                mecanumEStrafeLeft(speed, 17);
            }
            else if(skyStonePosition == 2){
                controlGrips(true);

                mecanumEMove(speed, 2, true);
                mecanumETurn(1, 90, false);
                runIntake(false);
                pause(750);
                mecanumETurn(0.75, 180, true);
                controlArm(0);
                mecanumEMove(speed, 18, false);

                mecanumEStrafeRight(0.4,10);
                controlGrips(true);
                runIntake(true);
                mecanumEStrafeLeft(speed, 44);
                mecanumEMove(.5, 6, true);
                pause(500);
                controlGrips(false);
                mecanumEStrafeRight(speed, 19);
                mecanumEMove(speed, 70, false);
                mecanumEStrafeRight(.5, 3);
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
                mecanumEStrafeLeft(speed, 30);
                mecanumEMove(speed, 21, false);
                mecanumEStrafeLeft(speed, 17);


            }

            else if(skyStonePosition == 1){
                controlGrips(true);

                mecanumEMove(speed, 6, true);
                mecanumETurn(1, 90, false);
                runIntake(false);
                pause(750);
                mecanumETurn(0.75, 180, true);
                controlArm(0);
                mecanumEMove(speed, 10, false);
                mecanumEStrafeRight(0.4,11);

//                mecanumEMove(speed, 2, false);
                controlGrips(true);
                runIntake(true);
                mecanumEStrafeLeft(speed, 44);
                mecanumEMove(.5, 10, true);
                pause(500);
                controlGrips(false);
                mecanumEStrafeRight(speed, 16);
                mecanumEMove(speed, 77, false);
                mecanumEStrafeRight(.5, 3);
                mecanumETurn(speed, 90, true);
                mecanumEMove(.5, 3, false);
                controlArm(1);
                mecanumEMove(0.3, 12, false);
                movePicks(false);
                controlArm(2);
                pause(1000);
                controlArm(6);
                mecanumETurn(1, 15, false);
                mecanumEMove(0.4, 49, true);
                controlGrips(true);
                movePicks(true);
                pause(250);
                controlArm(0);
                mecanumEStrafeLeft(speed, 30);
                mecanumEMove(speed, 21, false);
                mecanumEStrafeLeft(speed, 17);

            }

//            //Deposit the Skystone
//            mecanumEMove(speed, 28, false);
//            controlArm(1);
//            pause(750);
//            controlArm(6);
//            mecanumEStrafeLeft(speed, 6);
//            pause(1250);
//            controlGrips(true);
//            pause(500);
//            controlArm(5);
//            mecanumEStrafeRight(speed, 6);
//            controlArm(0);
//            pause(1000);

//            //Get 2nd SkyStone
//            if(skyStonePosition == 1){
//                mecanumEMove(speed, 41, true);
//                controlGrips(true);
//                runIntake(true);
//                mecanumEStrafeRight(speed, 15);
//                mecanumEMove(.5, 8, true);
//                pause(500);
//                controlGrips(false);
//                mecanumEStrafeLeft(speed, 19);
//                mecanumEMove(speed, 70, false);
//                controlArm(6);
//                pause(1000);
//                controlGrips(true);
//                pause(500);
//                mecanumEStrafeRight(speed, 1);
//                mecanumEMove(speed, 26, true);
//            }
//            else if(skyStonePosition == 2){
//                mecanumEMove(speed, 51, true);
//                controlGrips(true);
//                runIntake(true);
//                mecanumEStrafeRight(speed, 15);
//                mecanumEMove(.5, 8, true);
//                pause(500);
//                controlGrips(false);
//                mecanumEStrafeLeft(speed, 18);
//                mecanumEMove(speed, 74, false);
//                controlArm(6);
//                pause(1000);
//                mecanumEStrafeLeft(speed, 1);
//                controlGrips(true);
//                pause(500);
//                mecanumEStrafeRight(speed, 1);
//                mecanumEMove(speed, 26, true);
//
//            }
//
//            else if(skyStonePosition == 3){
////                controlGrips(true);
////                runIntake(true);
////                mecanumEStrafeRight(speed, 33);
////                mecanumEMove(.5, 6, true);
////                pause(500);
////                controlGrips(false);
////                mecanumEStrafeLeft(speed, 19);
////                mecanumEMove(speed, 74, false);
////                controlArm(6);
////                pause(1000);
////                mecanumEStrafeLeft(speed, 4);
////                controlGrips(true);
////                pause(500);
////                controlArm(0);
////                mecanumEStrafeRight(speed, 4);
////                mecanumEMove(speed, 20, true);
//            }



        }catch(Exception e){
            autoThread.interrupt();
            telemetry.addData("AUTONOMOUS WAS INTERRUPTED", e);
        }

    }
}
