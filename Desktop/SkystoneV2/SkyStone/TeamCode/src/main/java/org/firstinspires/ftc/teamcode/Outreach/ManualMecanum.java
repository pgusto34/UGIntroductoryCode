package org.firstinspires.ftc.teamcode.Outreach;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by union on 18年9月28日.
 */
@Disabled
@TeleOp(name = "Stadarken", group = "Outreach")
public class ManualMecanum extends OpMode {

    private boolean reverse, reg, slomo, superSlomo, fastRotator, lastA, lastA2, lastX, lastB, lastY;

    private DcMotor rightFront, rightBack, leftFront, leftBack;
    private DcMotor extender;
    private DcMotor rotator, rotator1;
    private DcMotor grabber;

    private int grabPower = 0;

    private double lF, rF, lB, rB, maxVector;


    @Override
    public void loop() {

        // Wheelbase
        controlMecanum();

        // Arm
        controlExtender();
        controlRotator();
        controlGrabber();
    }

    private void controlMecanum() {
        if(gamepad1.a & !lastA) reverse = !reverse;
        if(gamepad1.a) lastA = true;
        else lastA = false;

        if(gamepad1.x & !lastX) reg = !reg;
        if(gamepad1.x) lastX = true;
        else lastX = false;

        if(gamepad1.y & !lastY) slomo = !slomo;
        if(gamepad1.y) lastY = true;
        else lastY = false;

        if(gamepad1.b & !lastB) superSlomo = !superSlomo;
        if(gamepad1.b) lastB = true;
        else lastB = false;

        if(reg) {
            mecanumMove(gamepad1.left_stick_x/1, -gamepad1.left_stick_y/1, gamepad1.right_stick_x/1, reverse);
        }   else{
            mecanumMove(gamepad1.left_stick_x/1, -gamepad1.left_stick_y/1, gamepad1.right_stick_x/1, reverse);
        }

        if (slomo) {
            mecanumMove(gamepad1.left_stick_x/4, -gamepad1.left_stick_y/4, gamepad1.right_stick_x/4, reverse);
        } else {
            mecanumMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, reverse);
        }

        if(superSlomo) {
            mecanumMove(gamepad1.left_stick_x/8, -gamepad1.left_stick_y/8, gamepad1.right_stick_x/8, reverse);
        } else {
            mecanumMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, reverse);
        }

        }

    private void controlExtender() {
        extender.setPower(gamepad2.right_stick_y);
    }

    private void controlRotator() {
        if(gamepad2.a &!lastA2) fastRotator = !fastRotator;
        if(gamepad2.a) lastA2 = true;
        else lastA2 = false;

        if(fastRotator) {
            rotator.setPower(-gamepad2.left_stick_y);
            rotator1.setPower(-gamepad2.left_stick_y);
        }else {
            if(gamepad2.left_stick_y < 0) {
                rotator.setPower(-gamepad2.left_stick_y / 2);
                rotator1.setPower(-gamepad2.left_stick_y / 2);
            } else {
                rotator.setPower(-gamepad2.left_stick_y / 4);
                rotator1.setPower(-gamepad2.left_stick_y / 4);
            }
        }
    }

    private void controlGrabber() {
        if((gamepad2.right_trigger > 0.3) && (gamepad2.left_trigger < 0.2)){
            grabber.setPower(gamepad2.right_trigger);
        } else if((gamepad2.left_trigger > 0.3) && (gamepad2.right_trigger < 0.2)){
            grabber.setPower(-gamepad2.left_trigger);
        } else {
            grabber.setPower(0);
        }
    }

    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftF");
        leftBack = hardwareMap.dcMotor.get("leftB");
        rightFront = hardwareMap.dcMotor.get("rightF");
        rightBack = hardwareMap.dcMotor.get("rightB");

        extender = hardwareMap.dcMotor.get("extender");

        rotator = hardwareMap.dcMotor.get("rotator");
        rotator1 = hardwareMap.dcMotor.get("rotator1");

        grabber = hardwareMap.dcMotor.get("grabber");
    }


    protected void mecanumMove(double leftX, double leftY, double rightX, boolean negated){
        if(negated){
            lF = leftX + leftY - rightX;
            rF = leftX - leftY - rightX;
            lB = -leftX + leftY - rightX;
            rB = -leftX - leftY - rightX;
        }else{
            lF = -(leftX + leftY + rightX);
            rF = -(leftX - leftY + rightX);
            lB = -(-leftX + leftY + rightX);
            rB = -(-leftX - leftY + rightX);
        }

        maxVector = Math.max(Math.max(Math.abs(lF), Math.abs(rF)),
                Math.max(Math.abs(lB), Math.abs(rB)));

        maxVector = maxVector > 1 ? maxVector : 1;

        leftFront.setPower(lF / maxVector);
        rightFront.setPower(rF / maxVector);
        leftBack.setPower(lB / maxVector);
        rightBack.setPower(rB / maxVector);
    }

}