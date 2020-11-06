package org.firstinspires.ftc.teamcode.Outreach;
/**
 * Created by union on 18年10月27日.
 */

import android.os.Environment;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Outreach.OpModeMecanum;
import org.firstinspires.ftc.teamcode.Outreach.*;
import java.io.File;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

@Autonomous(name="Left Timer Autonomous", group="Competition")
@Disabled
public class AutoTimer extends OpModeMecanum {

    protected File dir = Environment.getExternalStorageDirectory();
    protected TimerTask land;
    protected TimerTask driveForward, driveBackward, strafeRightWaffle, strafeLeft, stopMotors, hitWaffle, returnFromWaffle;
    protected TimerTask knockFlag, retractFlag, knockFlag1;
    protected TimerTask stopMotors1, stopMotors2, stopMotors3, stopMotors4, stopMotors5, stopMotors6, stopMotors7;
    protected TimerTask testForWaffle1, testForWaffle2;
    protected TimerTask getInPosition, reAlign;
    protected TimerTask turn45, moveToWall, moveToDepot;
    protected TimerTask moveToPark, turnSmall, park;

    protected Timer time;

    protected Rev2mDistanceSensor[] revSensors = new Rev2mDistanceSensor[2];

    protected boolean waffle = false;
    protected boolean isWaffle = false;
    protected boolean left = true;

    protected long maxTime = 0;

    protected int waffleSpot = 2;
    protected int tB = 8000;

    protected double speed = 0.75;

    protected Servo knocker;

    private void declareTimerTasks(){

        land = new TimerTask(){
            public void run(){
                maxTime = System.currentTimeMillis() + 3250;
                while(System.currentTimeMillis() < maxTime) {
                    extender.setPower(1);
                }
                extender.setPower(0);
                maxTime = System.currentTimeMillis() + 500;
                while(System.currentTimeMillis() < maxTime) {
                    extender.setPower(0);
                    mecanumMove(0, 0, 0, false);
                }
                maxTime = System.currentTimeMillis() + 250;
                while(System.currentTimeMillis() < maxTime){
                    mecanumMove(-speed, 0, 0, false);
                }
                mecanumMove(0, 0, 0, false);
                maxTime = System.currentTimeMillis() + 500;
                while(System.currentTimeMillis() < maxTime) {
                    extender.setPower(0);
                    mecanumMove(0, 0, 0, false);
                }
                maxTime = System.currentTimeMillis() + 3100;
                while(System.currentTimeMillis() < maxTime) {
                    extender.setPower(-1);
                }
                extender.setPower(0);
                maxTime = System.currentTimeMillis() + 500;
                while(System.currentTimeMillis() < maxTime) {
                    extender.setPower(0);
                    mecanumMove(0, 0, 0, false);
                }
                maxTime = System.currentTimeMillis() + 200;
                while(System.currentTimeMillis() < maxTime){
                    mecanumMove(speed, 0, 0, false);
                }
                mecanumMove(0, 0, 0, false);
            }
        };

        park = new TimerTask(){
            public void run(){
                maxTime = System.currentTimeMillis() + 350;
                while(System.currentTimeMillis() < maxTime) {
                    mecanumMove(0, -speed, 0, false);
                }
                mecanumMove(0, 0, 0, false);
            }
        };

        driveForward = new TimerTask() {
            public void run() {
                mecanumMove(0,speed,0, false);
            }
        };

        driveBackward = new TimerTask() {
            public void run() {
                mecanumMove(0,-speed,0, false);
            }
        };

        strafeRightWaffle = new TimerTask() {
            public void run() {
                if(!waffle)
                    mecanumMove(speed, 0,0, false);
            }
        };

        strafeLeft = new TimerTask() {
            public void run() {
                mecanumMove(-speed, 0,0, false);
            }
        };

        stopMotors = new TimerTask() {
            public void run() {
                mecanumMove(0, 0, 0, false);
            }
        };

        hitWaffle = new TimerTask() {
            public void run() {
                if(waffle) mecanumMove(0,speed,0, false);
            }
        };

        returnFromWaffle = new TimerTask() {
            public void run() {
                if(waffle) mecanumMove(0,-speed,0, false);
            }
        };

        turn45 = new TimerTask() {
            public void run() {
                mecanumMove(0,0, speed, false);
            }
        };

        turnSmall = new TimerTask() {
            public void run() {
                maxTime = System.currentTimeMillis() + 750;
                while(System.currentTimeMillis() < maxTime) {
                    mecanumMove(0, 0, speed, false);
                }
                mecanumMove(0, 0, 0, false);
            }
        };

        stopMotors1 = new TimerTask() {
            public void run() {
                mecanumMove(0, 0, 0, false);
            }
        };

        stopMotors2 = new TimerTask() {
            public void run() {
                mecanumMove(0, 0, 0, false);
            }
        };

        stopMotors3 = new TimerTask() {
            public void run() {
                mecanumMove(0, 0, 0, false);
            }
        };

        stopMotors4 = new TimerTask() {
            public void run() {
                mecanumMove(0, 0, 0, false);
            }
        };

        stopMotors5 = new TimerTask() {
            public void run() {
                mecanumMove(0, 0, 0, false);
            }
        };

        stopMotors6 = new TimerTask() {
            public void run() {
                mecanumMove(0, 0, 0, false);
            }
        };

        stopMotors7 = new TimerTask() {
            public void run() {
                mecanumMove(0, 0, 0, false);
            }
        };

        knockFlag = new TimerTask() {
            public void run() {
                knocker.setPosition(0.5);
            }
        };

        knockFlag1 = new TimerTask() {
            public void run() {
                knocker.setPosition(0.5);
            }
        };

        retractFlag = new TimerTask() {
            public void run() {
                knocker.setPosition(0);
            }
        };

        testForWaffle1 = new TimerTask() {
            public void run() {
                maxTime = System.currentTimeMillis() + 600;
                while(!waffle && System.currentTimeMillis() < maxTime) {
                    isWaffle();
                    telemetry.addData("Has there been waffle", waffle);
                    telemetry.addData("Waffle now?", isWaffle);
                }
                if(waffle) {
                    telemetry.addData("Waffle", "Found");
                    waffleSpot = 1;
                } else {
                    telemetry.addData("Waffle", "NOT Found");
                }
            }
        };

        testForWaffle2 = new TimerTask() {
            public void run() {
                // need to figure out how to exit after certain amount of time
                maxTime = System.currentTimeMillis() + 600;
                while(!waffle && System.currentTimeMillis() < maxTime) {
                    isWaffle();
                    telemetry.addData("Has there been waffle", waffle);
                    telemetry.addData("Waffle now?", isWaffle);
                }
                if(waffle) {
                    if(waffleSpot != 1 && waffleSpot == 2){
                        waffleSpot = 0;
                    }
                    telemetry.addData("Waffle", "Found");
                } else {
                    waffleSpot = 2;
                    telemetry.addData("Waffle", "NOT Found");
                }
            }
        };

        getInPosition = new TimerTask() {
            public void run() {
                if(waffleSpot == 2) {
                    maxTime = System.currentTimeMillis() + 1150;
                    while (System.currentTimeMillis() < maxTime) {
                        mecanumMove(-speed, 0, 0, false);
                    }
                }
                mecanumMove(0, 0, 0, false);
            }
        };

        reAlign = new TimerTask() {
            public void run() {
                if(waffleSpot == 0) {
                    maxTime = System.currentTimeMillis() + 1200;
                    while (System.currentTimeMillis() < maxTime) {
                        mecanumMove(-speed, 0, 0, false);
                    }
                } else if(waffleSpot == 1){
                    maxTime = System.currentTimeMillis() + 600;
                    while (System.currentTimeMillis() < maxTime) {
                        mecanumMove(-speed, 0, 0, false);
                    }
                }
                mecanumMove(0, 0, 0, false);
            }
        };

        moveToWall = new TimerTask() {
            public void run() {
                mecanumMove(-speed, 0,0, false);
            }
        };

        moveToDepot = new TimerTask() {
            public void run() {
                mecanumMove(speed, 0,0, false);
            }
        };

        moveToPark = new TimerTask() {
            public void run() {
                mecanumMove(-speed, 0,0, false);
            }
        };
    }


    protected void timerScheduleAuto(){
        if(left) {
            // TODO land
            time.schedule(land, 0);

            // sample
            // robot strafes at 2 ft / sec at voltage 14.00+
            time.schedule(driveForward, 1050 + tB);
            time.schedule(stopMotors, 1355 + tB);
            time.schedule(testForWaffle1, 2390 + tB);
            time.schedule(strafeRightWaffle, 3390 + tB);
            time.schedule(stopMotors1, 3980 + tB);
            time.schedule(testForWaffle2, 5100 + tB);
            time.schedule(getInPosition, 6100 + tB);
            time.schedule(hitWaffle, 7510 + tB);
            time.schedule(stopMotors2, 7700 + tB);
            time.schedule(returnFromWaffle, 8200 + tB);
            time.schedule(stopMotors3, 8350 + tB);
            time.schedule(reAlign, 9000 + tB);

            // deposit
            time.schedule(moveToWall, 10500 + tB);
            time.schedule(turn45, 12000 + tB);
            time.schedule(stopMotors4, 13070 + tB);
            time.schedule(moveToDepot, 13500 + tB);
            time.schedule(stopMotors5, 14500 + tB);
            time.schedule(knockFlag, 16000 + tB);
            time.schedule(retractFlag, 17000 + tB);
            time.schedule(knockFlag1, 18000 + tB);

            // park
            time.schedule(moveToPark, 19000 + tB);
            time.schedule(stopMotors6, 21500 + tB);
            time.schedule(turnSmall, 22000 + tB);
            time.schedule(park, 23000 + tB);

        } else {
            // TODO land

            // sample
            time.schedule(driveForward, 910);
            time.schedule(stopMotors, 1375);
            time.schedule(testForWaffle1, 1390);
            time.schedule(strafeRightWaffle, 2050);
            time.schedule(stopMotors1, 2450);
            time.schedule(testForWaffle2, 2490);
            time.schedule(getInPosition, 3500);
            time.schedule(hitWaffle, 4470);
            time.schedule(stopMotors2, 4670);
            time.schedule(returnFromWaffle, 4690);
            time.schedule(stopMotors3, 4890);
            time.schedule(reAlign, 5000);

            // deposit
            time.schedule(moveToWall, 6000);
            time.schedule(turn45, 6450);
            time.schedule(stopMotors4, 6950);
            // TODO finish deposit

            // park
        }
    }

/*    protected void loadConfig () {
        try {
            FileReader fileReader = new FileReader(dir + "/robotSaves/config.cfg");
            BufferedReader reader = new BufferedReader(fileReader);
            String line;
            int i = 0;
            while((line = reader.readLine()) != null) {
                if (i == 0) left = (line.equals("1"));
                if (i == 1) speed = Double.parseDouble(line);
                i++;
                if(i == 2) voltage = Double.parseDouble(line);
            }
        }
        catch (FileNotFoundException e)
        {
            System.out.println(e.getStackTrace());
            telemetry.addData("FILE NOT FOUND", 0);
        }
        catch (IOException e)
        {
            System.out.println(e.getStackTrace());
            telemetry.addData("IO EXCEPTION", 0);
        }
    }*/

    @Override
    protected void timerLoop(){

    }

    @Override
    public void init(){
        time = new Timer();
        initMotors();
        knocker = hardwareMap.servo.get("knocker");
        for (int i = 0; i < 2; i++) {
            revSensors[i] = hardwareMap.get(Rev2mDistanceSensor.class, "d" + i);
        }
        declareTimerTasks();
    }

    @Override
    public void start() {
        timerScheduleAuto();
    }





    private double[] pollDistance() {
        double[] distances = new double[2];
        for (int i = 0; i < 2; i++) {
            distances[i] = revSensors[i].getDistance(DistanceUnit.MM);
        }

        return distances;
    }



    private boolean isWaffle() {
        double averageTop = 0;
        double averageBot = 0;
        boolean isWaffle = false;
        ArrayList<double[]> list = new ArrayList<double[]>();
        for(int i=0; i<5 ; i++) {
            list.add(pollDistance());
        }
        for(int i=0; i<list.size(); i++) {
            averageTop += list.get(i)[0];
            averageBot += list.get(i)[1];
        }

        averageTop /= list.size();
        averageBot /= list.size();

        double difference = Math.abs(averageTop - averageBot);

        if (difference > 300 && averageBot < 200) {
            waffle = true;
            isWaffle = true;
        } else {
            isWaffle = false;
        }
        telemetry.addData("Top", averageTop);
        telemetry.addData("Bottom", averageBot);
        telemetry.addData("Dif", difference);

        return isWaffle;
    }



    private void nonFunc() {
//     protected boolean busy = false;
//        protected int actionState = 0;
//        protected int count;
//
//
// @Override
//    protected void timerLoop(){
//        try {
//            if(!busy){
//                busy = true;
//                switch(actionState){
//                    case 0:
//
//                        count++;
//                        if(count < 100)
//                            break;
//                    case 1:
//                        break;
//                    case 2:
//                        break;
//                    default:
//                        break;
//                }
//
//            } else {
//                // check status of action IE (is the motor busy)
//                // if current action is done
//                // set busy = false and increment action state.
//                actionState++;
//            }
//
//
//            Thread.sleep(1);
//        } catch (InterruptedException e){
//
//        }
//    }
//}
        //        driveForward = new TimerTask() {
//            public void run() {
//                mecanumMove(0,speed,0, false);
//            }
//        };
//
//        driveBackward = new TimerTask() {
//            public void run() {
//                mecanumMove(0,-speed,0, false);
//            }
//        };
//
//        strafeRight = new TimerTask() {
//            public void run() {
//                mecanumMove(speed, 0,0, false);
//            }
//        };
//
//        strafeLeft = new TimerTask() {
//            public void run() {
//                mecanumMove(-speed, 0,0, false);
//            }
//        };
//
//        stopMotors = new TimerTask() {
//            public void run() {
//                mecanumMove(0, 0, 0, false);
//            }
//        };
//
//        waffleStrafe = new TimerTask() {
//            public void run() {
//                if(!waffle){
//                    double distances[] = pollDistance();
//                    double difference = Math.abs(distances[0] - distances[1]);
//                    if(difference > 100 && distances[1] < 1000) {
//                        waffle = true;
//                        if(count > 9500) waffleSpot = 2;
//                        else if(count > 10750) waffleSpot = 1;
//                        else if(count > 12000) waffleSpot = 0;
//                    } else {
//                        mecanumMove(-speed, 0, 0, false);
//                    }
//                } else {
//                    mecanumMove(0, 0, 0, false);
//                }
//            }
//        };
//
//        hitWaffle = new TimerTask() {
//            public void run() {
//                if(waffle) mecanumMove(0,speed,0, false);
//            }
//        };
//
//        returnFromWaffle = new TimerTask() {
//            public void run() {
//                if(waffle) mecanumMove(0,-speed,0, false);
//            }
//        };
//
//        countTime = new TimerTask() {
//            public void run() {
//                count++;
//                telemetry.addData("time", count);
//            }
//        };
//
//        moveToWall = new TimerTask() {
//            public void run() {
//                if(waffleSpot == 2) {
//                    mecanumMove(-speed, 0, 0, false);
//                } else if(waffleSpot == 1){
//                    if(count < 21450) {
//                        mecanumMove(-speed, 0, 0, false);
//                    }
//                } else {
//                    if(count < 19420) {
//                        mecanumMove(-speed, 0, 0, false);
//                    }
//                }
//            }
//        };
//
//        turn45 = new TimerTask() {
//            public void run() {
//                mecanumMove(0,0, speed, false);
//            }
//        };
    }
}


