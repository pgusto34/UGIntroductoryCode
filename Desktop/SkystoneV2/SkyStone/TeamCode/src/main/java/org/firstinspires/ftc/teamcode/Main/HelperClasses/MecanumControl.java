//package Hardware;
//
//
//public class MecanumControl {
//
//    private void controlMecanum() {
//        if(gamepad1.a & !lastA) reverse = !reverse;
//        if(gamepad1.a) lastA = true;
//        else lastA = false;
//
//        if(gamepad1.x & !lastX) reg = !reg;
//        if(gamepad1.x) lastX = true;
//        else lastX = false;
//
//        if(gamepad1.y & !lastY) slomo = !slomo;
//        if(gamepad1.y) lastY = true;
//        else lastY = false;
//
//        if(gamepad1.b & !lastB) superSlomo = !superSlomo;
//        if(gamepad1.b) lastB = true;
//        else lastB = false;
//
//        if(reg) {
//            mecanumMove(gamepad1.left_stick_x/1, -gamepad1.left_stick_y/1, gamepad1.right_stick_x/1, reverse);
//        }   else{
//            mecanumMove(gamepad1.left_stick_x/1, -gamepad1.left_stick_y/1, gamepad1.right_stick_x/1, reverse);
//        }
//
//        if (slomo) {
//            mecanumMove(gamepad1.left_stick_x/4, -gamepad1.left_stick_y/4, gamepad1.right_stick_x/4, reverse);
//        } else {
//            mecanumMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, reverse);
//        }
//
//        if(superSlomo) {
//            mecanumMove(gamepad1.left_stick_x/8, -gamepad1.left_stick_y/8, gamepad1.right_stick_x/8, reverse);
//        } else {
//            mecanumMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, reverse);
//        }
//
//    }
//
//
//    protected void mecanumMove(double leftX, double leftY, double rightX, boolean negated){
//        if(negated){
//            lF = leftX + leftY - rightX;
//            rF = leftX - leftY - rightX;
//            lB = -leftX + leftY - rightX;
//            rB = -leftX - leftY - rightX;
//        }else{
//            lF = -(leftX + leftY + rightX);
//            rF = -(leftX - leftY + rightX);
//            lB = -(-leftX + leftY + rightX);
//            rB = -(-leftX - leftY + rightX);
//        }
//
//        maxVector = Math.max(Math.max(Math.abs(lF),Math.abs(rF)),
//                Math.max(Math.abs(lB),Math.abs(rB)));
//
//        maxVector = maxVector > 1 ? maxVector : 1;
//
//        leftFront.setPower(lF / maxVector);
//        rightFront.setPower(rF / maxVector);
//        leftBack.setPower(lB / maxVector);
//        rightBack.setPower(rB / maxVector);
//    }
//
//}
