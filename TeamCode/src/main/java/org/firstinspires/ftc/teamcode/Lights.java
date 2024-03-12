package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Lights {
  private   RevBlinkinLedDriver blinkinLedDriver;

    public void initLights(HardwareMap hwMap) { //Run this in Int to map the class items
      blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "LED");
   }
//servo 0 control hub

    public void Purple (){
       blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
  }
    public void SkyBlue (){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
    }
    public void Yellow (){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);}

    public void Red (){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);}
    public void Blue (){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);}
    public void Green (){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void White (){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public void Black (){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }





}