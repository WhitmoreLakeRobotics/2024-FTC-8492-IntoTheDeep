package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.CommonLogic;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Claw extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    //private ColorRangeSensor IntakeSensor;
    //private DistanceSensor RearLeftSensor
    private ColorRangeSensor NTKCRS1; // Color-Range Sensor

    private Servo NTKS01;   //Claw/Intake servo
    private Servo WRIST01;   //Wrist servo
    private boolean cmdComplete = true;
    private Mode CurrentMode = Mode.STOP;
    private final double StopPos = 0.5;
    private final double InPos = 1.0;
    private final double OutPos = 0.0;

    private final double WristUpPos = 1;
    private final double WristDownPos = 0;
    private double wristToGroundPosition = 0.65;
    private double wristToSubPosition = 0.57;


    private Color SignalColor;
    private double NTKdistance; //in cm
    private double PickupDistance = 6.0; //in cm
    private double PickupDistanceTol = 1.25; //in cm

    private int SensorBlue;
    private int SensorRed;
    private int SensorGreen;
    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class

    public Target currentTarget = Target.UNKNOWNT;

    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    /*public Swing_Arm_And_Lift() {

    }*/

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init(){
        NTKS01 = hardwareMap.get(Servo.class,"NTKS01"); // Intake Servo
        WRIST01 = hardwareMap.get(Servo.class, "WRIST01"); // Wrist Servo
        NTKCRS1 = hardwareMap.get(ColorRangeSensor.class, "NTKCRS1"); // Color-Range Sensor
        CurrentMode = Mode.STOP;
    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {
        telemetry.addData("NTKCRS1 ", NTKCRS1.getDistance(DistanceUnit.CM));
        //telemetry.addData("NTKCRS1-red ", NTKCRS1.red());
        //telemetry.addData("NTKCRS1-blue ", NTKCRS1.blue());
        //telemetry.addData("NTKCRS1-green ", NTKCRS1.green());
    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){

    }

    public void setCurrentMode(Mode newMode) {
        CurrentMode = newMode;
    }
    public Mode getCurrentMode() {
        return CurrentMode;
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){
        switch(CurrentMode){
            case STOP:
                doStop();
                break;
            case CLAW_OPEN:
                ClawOpen();
                CurrentMode = Mode.IDLE;
                break;
            case CLAW_CLOSE:
                ClawClose();
                CurrentMode = Mode.IDLE;
                //AutoStopIntake();
                break;
            case WRIST_UP:
                WristUp();
                CurrentMode = Mode.IDLE;
                break;
            case WRIST_DOWN:
                WristDown();
                CurrentMode = Mode.IDLE;
                break;
            case WRIST_TO_GROUND:
                WristToGroundPosition();
                CurrentMode = Mode.IDLE;
                break;
            case WRIST_TO_SIDE_SUBMERSIBLE:
                WristToSideSubmersiblePosition();
                CurrentMode = Mode.IDLE;
                break;
            case IDLE:

                break;
            default:
                CurrentMode = Mode.STOP;
            //make case for each option
        }

    }


    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    void stop(){

    }

    private void getSenDist() {
        NTKdistance = NTKCRS1.getDistance(DistanceUnit.CM);
    }
    private void updateColorSensor() {

        SensorBlue = NTKCRS1.blue();
        SensorRed = NTKCRS1.red();
        SensorGreen = NTKCRS1.green();

    }

    public void CheckForTarget(){
        updateColorSensor();

        if ((Target.REDT.red <= SensorRed) &&
                (Target.REDT.blue >= SensorBlue) &&
                (Target.REDT.green >= SensorGreen)){
            currentTarget = Target.REDT;
        }
        else if ((Target.BLUET.red >= SensorRed) &&
                (Target.BLUET.blue <= SensorBlue) &&
                (Target.BLUET.green >= SensorGreen)) {
            currentTarget = Target.BLUET;
        }
        else if ((Target.YELLOWT.red <= SensorRed) &&
                (Target.YELLOWT.blue >= SensorBlue) &&
                (Target.YELLOWT.green <= SensorGreen)) {
            currentTarget = Target.YELLOWT;
        }
        else {currentTarget = Target.UNKNOWNT;

        }

    }

    public void ClawClose(){
        //CurrentMode = Mode.IN;
        NTKS01.setPosition(InPos);
        cmdComplete = true;
    }
    public void ClawOpen(){
        //CurrentMode = Mode.OUT;
        NTKS01.setPosition(OutPos);
        cmdComplete = true;
    }
    public void WristUp(){
        WRIST01.setPosition(WristUpPos);
        cmdComplete = true;
    }

    public void WristDown(){
        WRIST01.setPosition(WristDownPos);
        cmdComplete = true;
    }

    public void WristToPosition( double setPos ){
        WRIST01.setPosition(setPos);
        cmdComplete = true;
    }

    public void WristToGroundPosition(){
        WRIST01.setPosition(wristToGroundPosition);
        cmdComplete = true;
    }
    public void WristToSideSubmersiblePosition(){
        WRIST01.setPosition(wristToSubPosition);
        cmdComplete = true;
    }

    public double getWristToGroundPosition(){
        return wristToGroundPosition;
    }
    public double getWristToSideSubmersiblePosition(){
        return wristToSubPosition;
    }

    public void doStop(){
        //CurrentMode = Mode.STOP;
        //NTKS01.setPosition(StopPos);
        cmdComplete = true;
    }
    private void  AutoStopIntake() {
        getSenDist();
        if(CommonLogic.inRange(NTKdistance,PickupDistance,PickupDistanceTol)){
            CurrentMode = Mode.STOP;
        }
    }

    public enum Mode {
        CLAW_CLOSE,
        CLAW_OPEN,
        WRIST_UP,
        WRIST_DOWN,
        WRIST_TO_GROUND,
        WRIST_TO_SIDE_SUBMERSIBLE,
        IDLE,
        STOP;


    }

    public enum Target {
        REDT(200,0,0),
        BLUET(0,200,0),
        YELLOWT(200,0,200),
        UNKNOWNT( 0,0,0);

        private int red;
        private int blue;
        private int green;

        Target(int red, int blue, int green) {
            this.red = red;
            this.blue = blue;
            this.green = green;
        }



    }



}

