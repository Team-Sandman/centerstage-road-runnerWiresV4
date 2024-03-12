package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.Light;
        import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
        import com.qualcomm.robotcore.hardware.NormalizedRGBA;
        import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


        import org.firstinspires.ftc.teamcode.MechanumDriveTrain;
        import org.firstinspires.ftc.teamcode.PerryThePlatypusIntake;
//import org.firstinspires.ftc.teamcode.AirplaneLauncher;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class PlaystationRemoteControl extends OpMode {

    MechanumDriveTrain drive = new MechanumDriveTrain();
    PerryThePlatypusIntake  intake = new PerryThePlatypusIntake();
    //AirplaneLauncher launch = new AirplaneLauncher();

    PixelDropper pixelDrop = new PixelDropper();

    Lights light = new Lights();

    Lift lift = new Lift();

    hang hang= new hang();

    private NormalizedColorSensor scanny;

    boolean PixelDropperOpen = false;

    float Red;
    float Green;
    float Blue;



    //BEGIN Timer variables and constants
    long startTime = 0;
    long elapsedTime = 0;
    static long FINAL_WARN = 110000;     //110 Seconds
    static long CRITICAL_WARN = 100000;       //100 Seconds
    static long PRE_CRITICAL_WARN = 90000;     //90 Seconds
    static long WARNING_WARN = 75000;     //75 Seconds
    static long PIXEL_DISPLAY_TIME = 1000;  //  1 second was 2500 (2.5 Seconds)
    boolean timerOverride = false;      //Flag to override timer.
    long timerOverrideStart = 0;
    //END Timer variables and constants

    @Override
    public void init () {
        drive.initMechanumDrivetrainTeleOp(hardwareMap);
        intake.initIntake(hardwareMap);
        //launch.initLaunchMotor(hardwareMap);
        lift.initLift(hardwareMap);
        hang.inithang(hardwareMap);
        pixelDrop.initPixel_Servo(hardwareMap);
        scanny = hardwareMap.get(NormalizedColorSensor.class,"color_sensor");
        light.initLights(hardwareMap);

        //Start timer as last step before handing control over after initilization.
        //Add 500ms (0.5 Sec) for human processing delay from hearing start to pressing start.
        startTime = System.currentTimeMillis(); // + 500 (took out to see if timer is closer)
    }

    public void setTimerOverride()
    {
        timerOverride = true;
        timerOverrideStart = System.currentTimeMillis();
    }
    @Override
    public void loop() {
        //BEGIN Timer code, evaluates time since start to determine color/pattern
        //Check for active override, and if timer expired, disable override
        if (timerOverride == true)
        {
            if ((System.currentTimeMillis()-timerOverrideStart) > PIXEL_DISPLAY_TIME)
            {
                timerOverride = false;
            }
        }

        if (timerOverride == false) {
            elapsedTime = System.currentTimeMillis()-startTime;
            if (elapsedTime > CRITICAL_WARN)       //End game lighting schema
            {
                light.Red();
                if (elapsedTime > FINAL_WARN)     //End game/match over lighting schema
                {
                    if ((elapsedTime % 1000) > 500) //0.5 second blink.
                    {
                        light.Red();
                    } else {
                        light.Black();
                    }
                }
            } else if ((elapsedTime > PRE_CRITICAL_WARN) && (elapsedTime < CRITICAL_WARN))  //Between mid-match & end-game lighting schema
            {
                light.SkyBlue();
            } else if ((elapsedTime > WARNING_WARN) && (elapsedTime < PRE_CRITICAL_WARN))  //Between mid-match & end-game lighting schema
            {
                light.Blue();
            } else    //First half lighting schema
            {
                light.Black(); //was light.White
            }
        }
        //END Timer code

        if (gamepad1.left_bumper) {
            drive.mechanumDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
        } else {
            drive.mechanumDrive(drive.strafeMultiplier(gamepad1.right_stick_x), drive.forwardMultiplier(gamepad1.right_stick_y) + drive.forwardMultiplier(gamepad1.left_trigger) - drive.forwardMultiplier(gamepad1.right_trigger), drive.turnMultiplier(gamepad1.left_stick_x));
        }
        telemetry.addData("Strafe", gamepad1.right_stick_x);
        telemetry.addData("Forward", gamepad1.right_stick_y);
        telemetry.addData("Turn", gamepad1.left_stick_x);

        intake.intake(gamepad2.right_trigger - gamepad2.left_trigger);

        NormalizedRGBA colors = scanny.getNormalizedColors();

        Red = colors.red;
        Green = colors.green;
        Blue = colors.blue;

        //intake.ColorPixel(colors.red,colors.green, colors.blue);


        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f",colors.green);
        telemetry.addData("Blue","%.3f", colors.blue);
        telemetry.addData("pixel",intake.ColorPixel(colors.red,colors.green,colors.blue) );
        telemetry.addData("Lift", lift.liftPosition());
        if (intake.ColorPixel(colors.red,colors.green, colors.blue)=="purple" ){
            light.Purple();
            setTimerOverride();
        }
        else if (intake.ColorPixel(colors.red,colors.green, colors.blue)=="yellow" ){
            light.Yellow();
            setTimerOverride();
        }
        else if (intake.ColorPixel(colors.red,colors.green, colors.blue)=="green" ){
            light.Green();
            setTimerOverride();
        }
        else if (intake.ColorPixel(colors.red,colors.green, colors.blue)=="white" ){
            light.White();
            setTimerOverride();
            
        };

        //else {light.Black();}


        //launch.launch(gamepad2.left_stick_y);

        //lift.lift(.5 * gamepad2.right_stick_y);
        if (lift.liftPosition() <-2000 ){
            if (gamepad2.right_stick_y < 0){
                lift.lift(0);
            }
            else{
                lift.lift(.5 * gamepad2.right_stick_y);
            }

        }
        else if (lift.liftPosition() >0 ){
            if (gamepad2.right_stick_y > 0){
                lift.lift(0);
            }
            else{
                lift.lift(.5 * gamepad2.right_stick_y);
            }

        }
        else {
            lift.lift(.5 *gamepad2.right_stick_y);
        }


        hang.hang(.75 * gamepad2.left_stick_x);

        if (gamepad2.dpad_up){
            hang.hang(.5); //was .3
        }
        else if (gamepad2.dpad_down){
            hang.hang(0);
        }

        if (gamepad2.right_bumper) {
            pixelDrop.pixelDrop();

        }
        else {
            pixelDrop.pixelDropClose();
        }

        if (gamepad2.left_bumper) {
            pixelDrop.linkageForward();

        }
        else {
            pixelDrop.linkageBackward();
        }

      /* if (gamepad2.a){
           if (PixelDropperOpen==false) {

               pixelDrop.pixelDrop();

               PixelDropperOpen=true;

           }
            else {
               PixelDropperOpen = false;
               pixelDrop.pixelDropClose();

           }
       } */



    }

}

