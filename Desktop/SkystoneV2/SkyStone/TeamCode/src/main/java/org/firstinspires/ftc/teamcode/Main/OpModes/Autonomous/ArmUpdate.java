//package org.firstinspires.ftc.teamcode.Main.OpModes.Autonomous;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ReadWriteFile;
//
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//
//import java.io.File;
//
///**
// * Created by Sarthak on 6/1/2019.
// */
//public class ArmUpdate implements Runnable{
//
//    //Sleep time interval (milliseconds) for the position update thread
//    private int sleepTime;
//
//
//    public ArmUpdate(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double COUNTS_PER_INCH, int threadSleepDelay){
//        this.verticalEncoderLeft = verticalEncoderLeft;
//        this.verticalEncoderRight = verticalEncoderRight;
//        this.horizontalEncoder = horizontalEncoder;
//        sleepTime = threadSleepDelay;
//
//        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
//        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
//
//    }
//
//    /**
//     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
//     */
//    private void globalCoordinatePositionUpdate(){
//        //Get Current Positions
//        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
//        verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);
//
//        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
//        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;
//
//        //Calculate Angle
//        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
//        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));
//
//        //Get the components of the motion
//        normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);
//        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
//        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);
//
//        double p = ((rightChange + leftChange) / 2);
//        double n = horizontalChange;
//
//        //Calculate and update the position values
//        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
//        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));
//
//        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
//        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
//        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
//    }
//
//    /**
//     * Returns the robot's global x coordinate
//     * @return global x coordinate
//     */
//    public double returnXCoordinate(){ return robotGlobalXCoordinatePosition; }
//
//    /**
//     * Returns the robot's global y coordinate
//     * @return global y coordinate
//     */
//    public double returnYCoordinate(){ return robotGlobalYCoordinatePosition; }
//
//    /**
//     * Returns the robot's global orientation
//     * @return global orientation, in degrees
//     */
//    public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }
//
//    /**
//     * Stops the position update thread
//     */
//    public void stop(){ isRunning = false; }
//
//    public void reverseLeftEncoder(){
//        if(verticalLeftEncoderPositionMultiplier == 1){
//            verticalLeftEncoderPositionMultiplier = -1;
//        }else{
//            verticalLeftEncoderPositionMultiplier = 1;
//        }
//    }
//
//    public void reverseRightEncoder(){
//        if(verticalRightEncoderPositionMultiplier == 1){
//            verticalRightEncoderPositionMultiplier = -1;
//        }else{
//            verticalRightEncoderPositionMultiplier = 1;
//        }
//    }
//
//    public void reverseNormalEncoder(){
//        if(normalEncoderPositionMultiplier == 1){
//            normalEncoderPositionMultiplier = -1;
//        }else{
//            normalEncoderPositionMultiplier = 1;
//        }
//    }
//
//    /**
//     * Runs the thread
//     */
//    @Override
//    public void run() {
//        while(isRunning) {
//            globalCoordinatePositionUpdate();
//            try {
//                Thread.sleep(sleepTime);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//    }
//}
