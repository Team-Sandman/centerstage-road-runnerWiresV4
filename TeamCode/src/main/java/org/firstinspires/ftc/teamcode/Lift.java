package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor lift_motor;

    public void initLiftAuto( HardwareMap hwMap) {
        lift_motor = hwMap.get(DcMotor.class, "lift_motor");
       //
         lift_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //changed to DcmotorEx from DcMotor
        lift_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void autoLift (int position){
        lift_motor.setTargetPosition(position);
        lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_motor.setPower(.25);
    }

    public void initLift( HardwareMap hwMap) {
        lift_motor = hwMap.get(DcMotor.class, "lift_motor");
        //lift_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    public void lift(double speed){
        lift_motor.setPower(speed);
    }

    public int liftPosition(){
        return lift_motor.getCurrentPosition();
    }
    /*public class LiftyProgrammy {
    private DcMotor lifter_motor;
    private Servo claw;
    public void initLifterAuto( HardwareMap hwMap) {
        lifter_motor = hwMap.get(DcMotor.class, "lifter_motor");
        lifter_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = hwMap.get(Servo.class,"claw");
    }
    public void initLifterTeleOp( HardwareMap hwMap) {
        lifter_motor = hwMap.get(DcMotor.class, "lifter_motor");
        lifter_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = hwMap.get(Servo.class,"claw");
    }
    public void moveLifter (double up, double down) {
        if ( lifter_motor.getCurrentPosition() < 1400) lifter_motor.setPower(up - down );
        else lifter_motor.setPower(-down);
    }
    public void manualLift (){
        lifter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void autoLift (int position){
        lifter_motor.setTargetPosition(position);
        lifter_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter_motor.setPower(.25);
    }
    public void calibrateEncoder(){
        lifter_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void  setServoPosition(double position){
        claw.setPosition(position);
    }
}
*/


}
