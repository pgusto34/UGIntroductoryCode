package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Test")
public class EXAMPLEEncoderAuto extends AutonomousMovementGyro {

    double speed = 0.3;

    public void runAutonomous(){
       try {
           //gyroEStrafeLeft(.4, 90);
            gyroEStrafeRight(speed, 80);

       }catch(Exception e){
           autoThread.interrupt();
           telemetry.addData("AUTONOMOUS WAS INTERRUPTED", e);
       }

    }
}

//        public void AmyOpmodeRed(){
//            mecanumEMove(0.5, 60, true);
//            mecanumETurn(0.5, 90, true);
//            mecanumEMove( 0.5,  84,  true);
//            mecanumETurn(.5, 90,true);
//            mecanumEMove(.5,60,true);
//            mecanumETurn(.5, 90, false);
//            mecanumEMove(.5,36,true);
//        }
//
//        public void ColtonOpMode(){
//            mecanumEMove(.5,48, true);
//            mecanumEMove(.5, 20, false);
//            mecanumETurn(.5, 90, false);
//            mecanumEMove(.5,76,true);
//            mecanumETurn(.5, 90, true);
//            mecanumEMove(.5, 16, true);
//            mecanumEMove(.5,45.3, false);
//            mecanumETurn(.5, 118, true );
//            mecanumEMove(.5, 95.5, true);
//            mecanumETurn(.5, 61.6, true);
//            mecanumEMove(.5, 60.9, true);
//
//
//        }
//
//        public void TristanOpModeBlue(){
//            mecanumEMove(.7,55,true);
//            mecanumETurn(.6,150,false);
//            mecanumEMove(.7,42,true);
//            mecanumETurn(.6,90,true);
//            mecanumEMove(.8,35, false);
//            mecanumETurn(.6,135,true);
//            mecanumEMove(.7,15,true);
//            mecanumETurn(.6,90,true);
//            mecanumEMove(.9,60,true);
//
//        }