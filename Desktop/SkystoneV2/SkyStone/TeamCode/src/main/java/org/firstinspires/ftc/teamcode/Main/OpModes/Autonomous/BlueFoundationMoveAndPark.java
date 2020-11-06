package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE FOUNDATION")
public class BlueFoundationMoveAndPark extends AutonomousMovement {

    double speed = 1;

    public void runAutonomous(){
       try {
           mecanumEMove(.5, 34, false);
           pause(1000);
           movePicks(false);
           pause(1000);
           mecanumEMove(.5, 42, true);
           pause(1000);
           mecanumETurn(.5, 10, false);
           pause(500);
           movePicks(true);
           pause(1000);
           mecanumETurn(.5, 10, true);
           pause(500);
           mecanumEStrafeLeft(.5, 56);
          // pause(1000);
           //   mecanumEMove(.5, 24, false);




       }catch(Exception e){
           autoThread.interrupt();
           telemetry.addData("AUTONOMOUS WAS INTERRUPTED", e);
       }

    }
}
