package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class MechanumDriveTrain {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    public void initMechanumDrivetrainTeleOp(HardwareMap hwMap) {
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void mechanumDrive(double x, double y, double turn) {
        leftFront.setPower(x - y + turn);
        rightFront.setPower(-x - y - turn);
        leftBack.setPower(-x - y + turn);
        rightBack.setPower(x - y - turn);
    }

    public double forwardMultiplier(double y) {
        if (y < 0) {
            return (-.35 * y * y + .025 * y - .025);
        } else if (y > 0) {
            return (.35 * y * y  + .025 * y + .025);
        } else return (0);
    }

    public double strafeMultiplier(double x) {
        if (x < 0) {
            return (-.98 * x * x -.17 * x - .09);
        } else if (x > 0) {
            return (.98 * x * x - .17 * x + .09);
        } else return (0);
    }

    public double turnMultiplier(double turn) {
        if (turn < 0) {
            return (-.386 * turn * turn + .159 * turn - .023);
        } else if (turn > 0) {
            return (.386 * turn * turn + .159 * turn + .023);
        } else return (0);


    }


    public void autoDrive (int x, int y, int turn, double power) {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        int lf = (x - y + turn);
        int rf =(-x - y - turn);
        int lb = (-x - y + turn);
        int rb =(x - y - turn);

        rightFront.setTargetPosition(rf);
        leftFront.setTargetPosition(lf);
        rightBack.setTargetPosition(rb);
        leftBack.setTargetPosition(lb);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setPower(power);

        // while (Math.abs(rf - right_front_drive_motor.getCurrentPosition())>100  || Math.abs(lf - left_front_drive_motor.getCurrentPosition())>100){

        // }



        ;//water_vapor//;

    }
    public void autostop(){
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
    }

}