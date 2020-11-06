package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class GyroUpdate implements Runnable{
    boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;
    volatile double heading;

    ModernRoboticsI2cGyro gyro;

    public GyroUpdate(ModernRoboticsI2cGyro robotGyro, int threadSleepDelay){
        this.gyro = robotGyro;

        sleepTime = threadSleepDelay;

    }

    /**
     * Updates the Heading (YAW Angle)
     */
    private double UpdateHeading(){
        heading = this.gyro.getHeading();

        return heading;

    }

    /**
     * Returns the robot's global heading
     * @return robot heading
     */
    public double getHeading(){ return heading; }



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
            synchronized(this){
                if(this.gyro != null) {
                    heading = UpdateHeading();
                }
            }
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
