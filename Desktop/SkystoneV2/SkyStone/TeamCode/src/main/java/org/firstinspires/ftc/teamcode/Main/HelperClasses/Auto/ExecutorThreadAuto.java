//package org.firstinspires.ftc.teamcode.Main.HelperClasses.Auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//import java.util.concurrent.Callable;
//
//public class ExecutorThreadAuto extends OpMode {
//
//
//
//    DcMotor motor;
//    Servo servo;
//
//    Double Position;
//
//    Runnable runMotor = new Runnable() {
//        @Override
//        public void run() {
//            motor.setPower(1);
//        }
//    };
//
//    Runnable controlServo = new Runnable() {
//        @Override
//        public void run() {
//            servo.setPosition(.3);
//        }
//    };
//
//    Callable<Double> calculatePosition = new Callable<Double>() {
//        @Override
//        public Double call() throws Exception {
//            return Position;
//        }
//    };
//
//    @Override
//    public void init(){
//        motor = hardwareMap.dcMotor.get("motor");
//        servo = hardwareMap.servo.get("servo");
//    }
//
//    @Override
//    public void start(){
//        executor.execute(runMotor);
//        executor.execute(controlServo);
//        executor.submit(calculatePosition);
//    }
//
//    public void stop() {
//        executor.shutdown();
//    }
//}
