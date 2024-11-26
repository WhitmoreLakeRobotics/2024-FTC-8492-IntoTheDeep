package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.CommonLogic;
import org.firstinspires.ftc.teamcode.common.BasicPID;

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

    BasicPID EM1pid = new BasicPID(0.0006,0.00005,0, 0);


    private final boolean calEncoderFlag = true;
    private boolean cmdComplete = false;
    private Mode CurrentMode = Mode.STOP;
    private DcMotor AM1;
    private DcMotor EM1;

    private static final double ARMSPEED = 0.900;
    private double ARMHOLDPOWER =0.00;
    private int minArmPos = 0;
    private int maxArmPos = 4370;
    private int armPValue = 50;
    private int armTargetPos = 0;

    private static final double EXTSPEED = 0.90;
    private double EXTHOLDPOWER =0.00;
    //private static final int minExtPos = 0;
    //private static final int maxExtPos = 1180;
    private int extPValue = 50;
    private int extTargetPos = 0;


    private double extStepSize = 20.0;
    private double armStepSize = 10.0;

    // Calibration for the different micro adjustment arm movement in different modes
    private final int PICKUP_SUBMERSIBLE_ARM_POS_MIN = 0;
    private final int PICKUP_SUBMERSIBLE_ARM_POS_MAX = 0;
    private final int DELIVER_TO_HIGH_BASKET_ARM_POS_MIN = 0;
    private final int DELIVER_TO_HIGH_BASKET_ARM_POS_MAX = 0;

    private final int DELIVER_TO_HIGH_CHAMBER_ARM_POS_MIN = 0;
    private final int DELIVER_TO_HIGH_CHAMBER_ARM_POS_MAX = 0;

    private final int DEFAULT_ARM_POS_MIN = 0;
    private final int DEFAULT_ARM_POS_MAX = 4370;



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
        AM1.setDirection(DcMotor.Direction.FORWARD);
        AM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (calEncoderFlag){
            AM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else {
            AM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


        EM1 = hardwareMap.dcMotor.get("EM1");
        EM1.setDirection(DcMotor.Direction.FORWARD);
        EM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (calEncoderFlag){
            EM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else {
            EM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        // Configure the PID limits
        EM1pid.setOutputLimits(-EXTSPEED, EXTSPEED);

        // Init the default ARM min, max position
        setArmMinMaxPositions( DEFAULT_ARM_POS_MIN, DEFAULT_ARM_POS_MAX );

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
     public void init_loop() {

         telemetry.addData("AM1 ", AM1.getCurrentPosition());
         telemetry.addData("EM1 ", EM1.getCurrentPosition());

         CurrentMode = Mode.START;
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

        telemetry.addData("AM1 ", AM1.getCurrentPosition());
        telemetry.addData("EM1 ", EM1.getCurrentPosition());
        telemetry.addData("EM1 power ", CommonLogic.CapValue(
                CommonLogic.PIDcalc(extPValue,EXTHOLDPOWER,EM1.getCurrentPosition(),extTargetPos)
                ,-EXTSPEED,EXTSPEED));
        telemetry.addData("AM1 power ", CommonLogic.CapValue(
                CommonLogic.PIDcalc(armPValue,ARMHOLDPOWER,AM1.getCurrentPosition(),armTargetPos)
                ,-ARMSPEED,ARMSPEED));
        telemetry.addData("Arm Mode ",CurrentMode.toString());
        telemetry.addData("AM1 Target ", armTargetPos);
        telemetry.addData("EXT target ", extTargetPos);

        AM1.setPower(CommonLogic.CapValue(
                CommonLogic.PIDcalc(armPValue,ARMHOLDPOWER,AM1.getCurrentPosition(),armTargetPos)
                ,-ARMSPEED,ARMSPEED));

        EM1.setPower(CommonLogic.CapValue(
                CommonLogic.PIDcalc(extPValue,EXTHOLDPOWER,EM1.getCurrentPosition(),extTargetPos)
                ,-EXTSPEED,EXTSPEED));
/*
        EM1.setPower( CommonLogic.CapValue( EM1pid.getOutput( EM1.getCurrentPosition(), extTargetPos ),
                -EXTSPEED, EXTSPEED ) );
*/

        switch(CurrentMode){
            case START:
                armTargetPos = CommonLogic.CapValueint(Mode.START.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.START.ArmP;
                ARMHOLDPOWER = Mode.START.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.START.ExtPos, Mode.START.ExtPos,Mode.START.ExtMax);
                extPValue = Mode.START.ExtP;
                EXTHOLDPOWER = Mode.START.ExtF;

                break;

            case RETRACT_TO_NEUTRAL_POS:
                extTargetPos = CommonLogic.CapValueint(Mode.RETRACT_TO_NEUTRAL_POS.ExtPos, Mode.RETRACT_TO_NEUTRAL_POS.ExtPos,Mode.RETRACT_TO_NEUTRAL_POS.ExtMax);
                extPValue = Mode.RETRACT_TO_NEUTRAL_POS.ExtP;
                EXTHOLDPOWER = Mode.RETRACT_TO_NEUTRAL_POS.ExtF;


                if (CommonLogic.inRange( EM1.getCurrentPosition(), extTargetPos, 100)){
                    CurrentMode = Mode.NEUTRAL_POS;

                }
                break;

            case NEUTRAL_POS:
                armTargetPos = CommonLogic.CapValueint(Mode.NEUTRAL_POS.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.NEUTRAL_POS.ArmP;
                ARMHOLDPOWER = Mode.NEUTRAL_POS.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.NEUTRAL_POS.ExtPos, Mode.NEUTRAL_POS.ExtPos,Mode.NEUTRAL_POS.ExtMax);
                extPValue = Mode.NEUTRAL_POS.ExtP;
                EXTHOLDPOWER = Mode.NEUTRAL_POS.ExtF;

                break;


            case CLIMB:
                armTargetPos = CommonLogic.CapValueint(Mode.CLIMB.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.CLIMB.ArmP;
                ARMHOLDPOWER = Mode.CLIMB.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.CLIMB.ExtPos, Mode.CLIMB.ExtPos,Mode.CLIMB.ExtMax);
                extPValue = Mode.CLIMB.ExtP;
                EXTHOLDPOWER = Mode.CLIMB.ExtF;

                break;

            case PICKUP_WALL:
                armTargetPos = CommonLogic.CapValueint(Mode.PICKUP_WALL.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.PICKUP_WALL.ArmP;
                ARMHOLDPOWER = Mode.PICKUP_WALL.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.PICKUP_WALL.ExtPos, Mode.PICKUP_WALL.ExtPos,Mode.PICKUP_WALL.ExtMax);
                extPValue = Mode.PICKUP_WALL.ExtP;
                EXTHOLDPOWER = Mode.PICKUP_WALL.ExtF;

                break;
            case PICKUP_GROUND:
                armTargetPos = CommonLogic.CapValueint(Mode.PICKUP_GROUND.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.PICKUP_GROUND.ArmP;
                ARMHOLDPOWER = Mode.PICKUP_GROUND.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.PICKUP_GROUND.ExtPos, Mode.PICKUP_GROUND.ExtPos,Mode.PICKUP_GROUND.ExtMax);
                extPValue = Mode.PICKUP_GROUND.ExtP;
                EXTHOLDPOWER = Mode.PICKUP_GROUND.ExtF;

                break;
            case PICKUP_SUBMERSIBLE:
                armTargetPos = CommonLogic.CapValueint(Mode.PICKUP_SUBMERSIBLE.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.PICKUP_SUBMERSIBLE.ArmP;
                ARMHOLDPOWER = Mode.PICKUP_SUBMERSIBLE.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.PICKUP_SUBMERSIBLE.ExtPos, Mode.PICKUP_SUBMERSIBLE.ExtPos,Mode.PICKUP_SUBMERSIBLE.ExtMax);
                extPValue = Mode.PICKUP_SUBMERSIBLE.ExtP;
                EXTHOLDPOWER = Mode.PICKUP_SUBMERSIBLE.ExtF;
                CurrentMode = Mode.PICKUP_SUBMERSIBLE_IDLE;

                break;
            case PICKUP_SUBMERSIBLE_IDLE:
                break;
            case DELIVER_TO_HIGH_BASKET_IDLE:

                break;

            case DELIVER_TO_HIGH_CHAMBER_IDLE:

                break;

            case DELIVER_TO_LOW_BASKET:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_LOW_BASKET.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.DELIVER_TO_LOW_BASKET.ArmP;
                ARMHOLDPOWER = Mode.DELIVER_TO_LOW_BASKET.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_LOW_BASKET.ExtPos, Mode.DELIVER_TO_LOW_BASKET.ExtPos,Mode.DELIVER_TO_LOW_BASKET.ExtMax);
                extPValue = Mode.DELIVER_TO_LOW_BASKET.ExtP;
                EXTHOLDPOWER = Mode.DELIVER_TO_LOW_BASKET.ExtF;

                break;
            case DELIVER_TO_HIGH_BASKET_ARM_ONLY:
                cmdComplete = false;
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_HIGH_BASKET_ARM_ONLY.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.DELIVER_TO_HIGH_BASKET_ARM_ONLY.ArmP;
                ARMHOLDPOWER = Mode.DELIVER_TO_HIGH_BASKET_ARM_ONLY.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_HIGH_BASKET_ARM_ONLY.ExtPos, Mode.DELIVER_TO_HIGH_BASKET_ARM_ONLY.ExtPos,Mode.DELIVER_TO_HIGH_BASKET_ARM_ONLY.ExtMax);
                extPValue = Mode.DELIVER_TO_HIGH_BASKET_ARM_ONLY.ExtP;
                EXTHOLDPOWER = Mode.DELIVER_TO_HIGH_BASKET_ARM_ONLY.ExtF;


                if (CommonLogic.inRange( AM1.getCurrentPosition(), armTargetPos, 100)){
                    CurrentMode = Mode.DELIVER_TO_HIGH_BASKET_EXT_ONLY;

                }
                break;

            case DELIVER_TO_HIGH_BASKET_EXT_ONLY:
                extTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_HIGH_BASKET_EXT_ONLY.ExtPos, Mode.DELIVER_TO_HIGH_BASKET_EXT_ONLY.ExtPos,Mode.DELIVER_TO_HIGH_BASKET_EXT_ONLY.ExtMax);
                extPValue = Mode.DELIVER_TO_HIGH_BASKET_EXT_ONLY.ExtP;
                EXTHOLDPOWER = Mode.DELIVER_TO_HIGH_BASKET_EXT_ONLY.ExtF;


                if( CommonLogic.inRange( EM1.getCurrentPosition(), extTargetPos, 100) ){
                    cmdComplete = true;
                    CurrentMode = Mode.DELIVER_TO_HIGH_BASKET_IDLE;
                }

                break;
            case DELIVER_TO_LOW_CHAMBER:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_LOW_CHAMBER.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.DELIVER_TO_LOW_CHAMBER.ArmP;
                ARMHOLDPOWER = Mode.DELIVER_TO_LOW_CHAMBER.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_LOW_CHAMBER.ExtPos, Mode.DELIVER_TO_LOW_CHAMBER.ExtPos,Mode.DELIVER_TO_LOW_CHAMBER.ExtMax);
                extPValue = Mode.DELIVER_TO_LOW_CHAMBER.ExtP;
                EXTHOLDPOWER = Mode.DELIVER_TO_LOW_CHAMBER.ExtF;

                break;
            case DELIVER_TO_OBSERVATION:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_OBSERVATION.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.DELIVER_TO_OBSERVATION.ArmP;
                ARMHOLDPOWER = Mode.DELIVER_TO_OBSERVATION.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_OBSERVATION.ExtPos, Mode.DELIVER_TO_OBSERVATION.ExtPos,Mode.DELIVER_TO_OBSERVATION.ExtMax);
                extPValue = Mode.DELIVER_TO_OBSERVATION.ExtP;
                EXTHOLDPOWER = Mode.DELIVER_TO_OBSERVATION.ExtF;

                break;
            case DELIVER_TO_HIGH_CHAMBER:
                armTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_HIGH_CHAMBER.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.DELIVER_TO_HIGH_CHAMBER.ArmP;
                ARMHOLDPOWER = Mode.DELIVER_TO_HIGH_CHAMBER.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.DELIVER_TO_HIGH_CHAMBER.ExtPos, Mode.DELIVER_TO_HIGH_CHAMBER.ExtPos,Mode.DELIVER_TO_HIGH_CHAMBER.ExtMax);
                extPValue = Mode.DELIVER_TO_HIGH_CHAMBER.ExtP;
                EXTHOLDPOWER = Mode.DELIVER_TO_HIGH_CHAMBER.ExtF;
                CurrentMode = Mode.DELIVER_TO_HIGH_BASKET_IDLE;

                break;
            case RETRACT_FROM_HIGH_CHAMBER:
                armTargetPos = CommonLogic.CapValueint(Mode.RETRACT_FROM_HIGH_CHAMBER.ArmPos, minArmPos,maxArmPos);
                armPValue = Mode.RETRACT_FROM_HIGH_CHAMBER.ArmP;
                ARMHOLDPOWER = Mode.RETRACT_FROM_HIGH_CHAMBER.ArmF;

                extTargetPos = CommonLogic.CapValueint(Mode.RETRACT_FROM_HIGH_CHAMBER.ExtPos, Mode.RETRACT_FROM_HIGH_CHAMBER.ExtPos,Mode.RETRACT_FROM_HIGH_CHAMBER.ExtMax);
                extPValue = Mode.RETRACT_FROM_HIGH_CHAMBER.ExtP;
                EXTHOLDPOWER = Mode.RETRACT_FROM_HIGH_CHAMBER.ExtF;

                if( CommonLogic.inRange( EM1.getCurrentPosition(), extTargetPos, 100) ){
                    CurrentMode = Mode.NO_OP;
                    cmdComplete = true;
                }

                break;
            case NO_OP:
                break;
            case STOP:
                break;
            default:
                break;
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

public void setCurrentMode(Mode otherMode ){
        //update to recieve and set mode
CurrentMode = otherMode;
}

public Mode getCurrentMode(){
    //update to recieve and set mode
    return CurrentMode;
}

public void updateExtension(double updateTarget){
        //multiply update target by some amount then add to target pos.
        //wrap set value in cap
    int nTarget = (int) (extTargetPos + (updateTarget * extStepSize));
    extTargetPos = CommonLogic.CapValueint(nTarget,CurrentMode.ExtPos,CurrentMode.ExtMax);
}

public void updateArm(double updateTarget){
    //multiply update targed by some amount then add to target pos.
    //wrap set value in cap
    int nTarget = (int) (armTargetPos + (updateTarget * armStepSize));
    armTargetPos = CommonLogic.CapValueint(nTarget,minArmPos,maxArmPos);
}

public void resetEncoders(){

    AM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    AM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    EM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    EM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

private void setArmMinMaxPositions( int minPos, int maxPos) {
    minArmPos = minPos;
    maxArmPos = maxPos;
}

public enum Mode{
/*
   START(0,100,0,0,100,0,5),
    PICKUP_TANK(5,100,0,0,100,0,5),
    PICKUP_GROUND(160,100,0,205,100,0,210),
    PICKUP_WALL(15,100,0,0,100,0,5),
    DELIVER_TO_OBSERVATION(20,100,0,0,100,0,695),
    DELIVER_TO_LOW_CHAMBER(25,100,0,0,100,0,5),
    DELIVER_TO_HIGH_CHAMBER(30,100,0,0,100,0,5),
    DELIVER_TO_LOW_BASKET(2400,100,0,780,50,0,1000),
    DELIVER_TO_HIGH_BASKET_ARM_ONLY(3200,100,0,0,100,0,1120),
    DELIVER_TO_HIGH_BASKET_EXT_ONLY(3200,100,0,1110,30,0,1120),
    CLIMB(3800,100,0,0,100,0,5),
    NEUTRAL_POS(1380,100,0,0,100,0,5),
    STOP(0,2100000000,0,0,2100000000,0,5);
 */
    START(                          0,  150,0,0,    150,0,5),
    PICKUP_GROUND(                  280,100,0,200,  100,1,400),
    PICKUP_WALL(                    60, 100,0,90,    100,0,95),
    PICKUP_SUBMERSIBLE(             620, 100, 0, 950, 120, 0, 680),
    PICKUP_SUBMERSIBLE_IDLE(        650, 120,0,  950, 120, 0,680),
    DELIVER_TO_OBSERVATION(         20, 100,0,0,    100,0,695),
    DELIVER_TO_LOW_CHAMBER(         25, 100,0,0,    100,0,5),
    DELIVER_TO_HIGH_CHAMBER(        1920,120,0,540,   200,1,600),
    DELIVER_TO_HIGH_CHAMBER_IDLE(   1920,120,0,540,   200,1,600),
    RETRACT_FROM_HIGH_CHAMBER(      1920,120,0,0,   100,0,600),
    DELIVER_TO_LOW_BASKET(          2400,100,0,1350,100,0,1500),
    DELIVER_TO_HIGH_BASKET_ARM_ONLY(3150,100,0,0,   100,0,1120),
    DELIVER_TO_HIGH_BASKET_EXT_ONLY(3150,100,0,1450,75,0,1650),
    DELIVER_TO_HIGH_BASKET_IDLE(    3150,100,0,1450,75,0,1650),
    CLIMB(                          3950,100,0,0,   100,0,5),
    NEUTRAL_POS(                    1480,150,0,0,   150,0,5),
    RETRACT_TO_NEUTRAL_POS(         1480,100,0,0,   100,0,5),
    STOP(0,2100000000,0,0,2100000000,0,5),
    NO_OP(0, 0, 0, 0, 0, 0, 0);


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

public boolean getCmdComlete(){
    return cmdComplete;
}




}

