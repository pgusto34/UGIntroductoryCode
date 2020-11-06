package org.firstinspires.ftc.teamcode.Main.OpModes.TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class ArmUpdate implements Runnable{
    boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;
    volatile int armState;
    volatile double armServoPosition, arm2ServoPosition, rotatorServoPosition, rotatorGripPosition, leftGripPosition, rightGripPosition;

    private enum ArmStates{
        INTAKE, ROTATE, MAX_HEIGHT, UPPER45, MAX_REACH, DOWN
    }

    /**
     *
     * @param state
     * @param threadSleepDelay in milliseconds
     */
    public ArmUpdate(int state, int threadSleepDelay){
        armState = state;

        sleepTime = threadSleepDelay;

    }

    /**
     * Updates the Heading (YAW Angle)
     */
//    private double updateServoPosition(){
//        switch(armState){
//            casee
//
//
//
//        }
//
//    }

    /**
     * Returns the robot's global heading
     * @return robot heading
     */



    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }




    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            //updateServoPosition()
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
