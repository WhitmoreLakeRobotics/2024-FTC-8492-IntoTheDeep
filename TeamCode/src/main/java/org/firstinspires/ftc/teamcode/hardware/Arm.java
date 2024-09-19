package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.CommonLogic;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Arm extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    //private ColorRangeSensor IntakeSensor;
    //private DistanceSensor RearLeftSensor



    private boolean cmdComplete = true;
    private Mode CurrentMode = Mode.STOP;
    private DcMotor AM1;
    private DcMotor EM1;

    private static final double ARMSPEED = 0.40;
    private double ARMHOLDPOWER =0.00;
    private static final int minArmPos = 0;
    private static final int maxArmPos = 1000;
    private int armPValue = 50;
    private int armTargetPos = 0;

    private static final double EXTSPEED = 0.40;
    private double EXTHOLDPOWER =0.00;
    private static final int minExtPos = 0;
    private static final int maxExtPos = 1000;
    private int extPValue = 50;
    private int extTargetPos = 0;



    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class


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
        AM1 = hardwareMap.dcMotor.get("AM1");
        AM1.setDirection(DcMotor.Direction.REVERSE);
        AM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        EM1 = hardwareMap.dcMotor.get("EM1");
        EM1.setDirection(DcMotor.Direction.REVERSE);
        EM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
     public void init_loop() {
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

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){

        AM1.setPower(CommonLogic.CapValue(
                CommonLogic.PIDcalc(armPValue,ARMHOLDPOWER,AM1.getCurrentPosition(),armTargetPos)
                ,-ARMSPEED,ARMSPEED));

        EM1.setPower(CommonLogic.CapValue(
                CommonLogic.PIDcalc(extPValue,EXTHOLDPOWER,EM1.getCurrentPosition(),extTargetPos)
                ,-EXTSPEED,EXTSPEED));

        switch(CurrentMode){
            case START:
                armTargetPos = CommonLogic.CapValueint(Mode.START.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.START.ArmP;
                ARMHOLDPOWER = Mode.START.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.START.ExtPos, Mode.START.ExtPos,Mode.START.ExtMax);
                extPValue = Mode.START.ExtP;
                EXTHOLDPOWER = Mode.START.ExtF;

                break;
            case CLIMB:
                armTargetPos = CommonLogic.CapValueint(Mode.CLIMB.Pos, minArmPos,maxArmPos);

                break;
            case PICKUP_TANK:
                armTargetPos = CommonLogic.CapValueint(Mode.PICKUP_TANK.Pos, minArmPos,maxArmPos);

                break;
            case PICKUP_WALL:
                armTargetPos = CommonLogic.CapValueint(Mode.PICKUP_WALL.Pos, minArmPos,maxArmPos);

                break;
            case PICKUP_GROUND:
                armTargetPos = CommonLogic.CapValueint(Mode.PICKUP_GROUND.Pos, minArmPos,maxArmPos);

                break;
            case DELIVER_TO_LOW_BASKET:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_LOW_BASKET.Pos, minArmPos,maxArmPos);

                break;
            case DELIVER_TO_HIGH_BASKET:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_HIGH_BASKET.Pos, minArmPos,maxArmPos);

                break;
            case DELIVER_TO_LOW_CHAMBER:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_LOW_CHAMBER.Pos, minArmPos,maxArmPos);

                break;
            case DELIVER_TO_OBSERVATION:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_OBSERVATION.Pos, minArmPos,maxArmPos);

                break;
            case DELIVER_TO_HIGH_CHAMBER:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_HIGH_CHAMBER.Pos, minArmPos,maxArmPos);

                break;
            default:
        }


    }

    public void doStop(){
        CurrentMode = Mode.STOP;

        cmdComplete = true;
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

public void setCurrentMode(){
        //update to recieve and set mode
}

private enum Mode{
   START(0,50,0,0,50,0,5),
    PICKUP_TANK(5,50,0,0,50,0,5),
    PICKUP_GROUND(10,50,0,0,50,0,5),
    PICKUP_WALL(15,50,0,0,50,0,5),
    DELIVER_TO_OBSERVATION(20,50,0,0,50,0,5),
    DELIVER_TO_LOW_CHAMBER(25,50,0,0,50,0,5),
    DELIVER_TO_HIGH_CHAMBER(30,50,0,0,50,0,5),
    DELIVER_TO_LOW_BASKET(35,50,0,0,50,0,5),
    DELIVER_TO_HIGH_BASKET(40,50,0,0,50,0,5),
    CLIMB(45,50,0,0,50,0,5),
    STOP(0,1000000,0,0,10000000,0,5);

   private int ArmPos;
   private int ArmP;
   private int ArmF;
   private int ExtPos;
   private int ExtP;
   private int ExtF;
   private int ExtMax;

 private Mode(int ArmPos,int ArmP, int ArmF,int ExtPos, int ExtP,int ExtF, int ExtMax){
     this.ArmPos = ArmPos;
     this.ArmP = ArmP;
     this.ArmF = ArmF;
     this.ExtPos = ExtPos;
     this.ExtP = ExtP;
     this.ExtF = ExtF;
     this.ExtMax = ExtMax;
 }



}





}

