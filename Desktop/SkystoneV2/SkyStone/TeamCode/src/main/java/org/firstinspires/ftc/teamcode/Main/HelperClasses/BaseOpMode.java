package org.firstinspires.ftc.teamcode.Main.HelperClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import java.util.concurrent.Callable;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadPoolExecutor;

/**
 * The base OpMode class for all other OpModes
 * This extends the TunableOpMode from the frogbots, so we can 'tune' our values
 * Add all of the init code here
 * Define all hardware in this class
 */

public abstract class BaseOpMode extends TunableOpMode {

    private DcMotor rightFront, rightBack, leftFront, leftBack;

//    protected ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(10);
//
//    Runnable test = new Runnable() {
//        @Override
//        public void run() {
//            initMotors();
//        }
//    };

//    Callable<Double> runLongCalculation = new Callable<Double>() {
//        @Override
//        public Double call() throws Exception {
//            return Math.cos(1);
//        }
//    };



    public abstract void init();
//        Future<Double> abacus = executor.submit(runLongCalculation);
//        executor.execute(test);



//    public void initMotors(){
//        //Init Motors
//        rightFront = hardwareMap.dcMotor.get("rightFront");
//        leftFront  = hardwareMap.dcMotor.get("leftFront");
//        rightBack  = hardwareMap.dcMotor.get("rightBack");
//        leftBack   = hardwareMap.dcMotor.get("leftBack");
//    }

    public void start(){}

    public void loop(){}
}
