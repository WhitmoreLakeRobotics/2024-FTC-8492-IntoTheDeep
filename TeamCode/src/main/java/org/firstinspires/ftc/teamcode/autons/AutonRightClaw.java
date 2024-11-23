package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Settings;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Robot;

//@Disabled
@Autonomous(name = "AutonRightClaw", group = "Auton")
// @Autonomous(...) is the other common choice

public class AutonRightClaw extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();




    private stage currentStage = stage._unknown;
    // declare auton power variables
    //private double AUTO_DRIVE_TURBO_SPEED = DriveTrain.DRIVETRAIN_TURBOSPEED;
    //private double AUTO_DRIVE_SLOW_SPEED = DriveTrain.DRIVETRAIN_SLOWSPEED;
   // private double AUTO_DRIVE_NORMAL_SPEED = DriveTrain.DRIVETRAIN_NORMALSPEED;
   // private double AUTO_TURN_SPEED = DriveTrain.DRIVETRAIN_TURNSPEED;

    private String RTAG = "8492-Auton";

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        msStuckDetectInit = Settings.msStuckDetectInit;
        msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        msStuckDetectStart = Settings.msStuckDetectStart;
        msStuckDetectLoop = Settings.msStuckDetectLoop;
        msStuckDetectStop = Settings.msStuckDetectStop;

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        telemetry.addData("Test AutonRightClaw", "Initialized");

        //Initialize Gyro
        robot.driveTrain.ResetGyro();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // initialize robot
        robot.init_loop();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // start robot
        runtime.reset();
        robot.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.loop();

        switch (currentStage){
            case  _unknown:
                currentStage = stage._00_preStart;
                break;
            case _00_preStart:
                robot.claw.setCurrentMode(Claw.Mode.CLAW_CLOSE);
                currentStage = stage._10_Drive_Out;
                break;
            case _10_Drive_Out:
                robot.driveTrain.CmdDrive(5,-50,0.35,0);
                robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_CHAMBER);
                currentStage = stage._30_Drive_Forward;
                break;
            /*case _20_Strafe_Left:
                if(robot.driveTrain.getCmdComplete()){
                    robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_CHAMBER);
                    robot.driveTrain.CmdDrive(2,-90,0.35,0);
                    currentStage = stage._30_Drive_Forward;
                }
                break;*/
            case _30_Drive_Forward:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(24,0,0.20,0);  // hang first specimen
                    currentStage = stage._35_Retract_Arm;
                }
                break;
            case _35_Retract_Arm:
                if(robot.driveTrain.getCmdComplete()){
                    robot.claw.setCurrentMode(Claw.Mode.CLAW_OPEN);
                    robot.arm.setCurrentMode(Arm.Mode.RETRACT_FROM_HIGH_CHAMBER);
                    runtime.reset();
                    currentStage = stage._40_Drive_Back;
                }
                break;
            case _40_Drive_Back:
                if(robot.arm.getCmdComlete() || (runtime.milliseconds() > 750)){
                    robot.driveTrain.CmdDrive(1,-179,0.35,0);
                    currentStage = stage._45_Turn_Around;
                }
                break;
            case _45_Turn_Around:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdTurn(90, 0.20);
                    currentStage = stage._46_Drive_To_Side_Wall;
                }
                break;
            case _46_Drive_To_Side_Wall:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(26.5,70,0.40,90);
                    currentStage = stage._47_Drive_Past_Sample;
                }
                break;
            case _47_Drive_Past_Sample:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.arm.setCurrentMode(Arm.Mode.START);
                    robot.claw.setCurrentMode(Claw.Mode.WRIST_UP);
                    robot.driveTrain.CmdDrive(18,0,0.40,90);
                    currentStage = stage._48_Drive_To_Side_Wall;
                }
                break;
            case _48_Drive_To_Side_Wall:
                if (robot.driveTrain.getCmdComplete()) {
                    //robot.arm.setCurrentMode(Arm.Mode.PICKUP_WALL);
                    robot.driveTrain.CmdDrive(4.2,90,0.35,90);
                    currentStage = stage._49_Sweep_Specimen;
                }
                break;
            case _49_Sweep_Specimen:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.arm.setCurrentMode(Arm.Mode.START);
                    robot.driveTrain.CmdDrive(53,170,0.55,90);  // sweep first specimen to wall
                    currentStage = stage._55_Pick_Up_Specimen;
                }
                break;
                /*case _50_Strafe_Right:
                if(robot.driveTrain.getCmdComplete()){
                    robot.arm.setCurrentMode(Arm.Mode.NEUTRAL_POS);
                    robot.driveTrain.CmdDrive(44,90,0.35,90);
                    currentStage = stage._52_Drive_To_Wall;
                }
                break;
            case _52_Drive_To_Wall:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.arm.setCurrentMode(Arm.Mode.PICKUP_WALL);
                    robot.driveTrain.CmdDrive(10,-179,0.35,90);
                    currentStage = stage._55_Pick_Up_Specimen;
                }
                break;*/
            case _55_Pick_Up_Specimen:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(11,90,0.28,90);
                    currentStage = stage._57_Close_Claw;
                }
                break;
            case _57_Close_Claw:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.claw.setCurrentMode(Claw.Mode.CLAW_CLOSE);  // pick up specimen 2
                    runtime.reset();
                    currentStage = stage._60_Lift_Arm;
                }
                break;
            case _60_Lift_Arm:
                if (runtime.milliseconds() > 500)     {
                    robot.arm.setCurrentMode(Arm.Mode.NEUTRAL_POS);
                    runtime.reset();
                    currentStage = stage._70_Go_Back;
                }
                break;
            case _70_Go_Back:
                if (runtime.milliseconds() > 500) {
                    robot.driveTrain.CmdDrive(31,-89,0.55,90);
                    currentStage = stage._80_Turn;
                }
                break;
            case _80_Turn:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdTurn(0, 0.20);
                    currentStage = stage._85_Extend_Arm;
                }
                break;
            case _85_Extend_Arm:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_CHAMBER);
                    //robot.driveTrain.CmdDrive(20,0,0.35,0);
                    currentStage = stage._90_Place_Specimen;
                }
                break;
            case _90_Place_Specimen:
                if (robot.driveTrain.getCmdComplete()) {

                    robot.driveTrain.CmdDrive(25,0,0.35,0);  // hang specimen 2
                    currentStage = stage._100_Retract_Arm;
                }
                break;
            case _100_Retract_Arm:
                if(robot.driveTrain.getCmdComplete()){
                   robot.claw.setCurrentMode(Claw.Mode.CLAW_OPEN);
                    robot.arm.setCurrentMode(Arm.Mode.RETRACT_FROM_HIGH_CHAMBER);
                    runtime.reset();
                    currentStage = stage._120_Turn;
                }
                break;
            case _120_Turn:
                if(robot.arm.getCmdComlete() || (runtime.milliseconds() > 750)){
                    robot.claw.setCurrentMode(Claw.Mode.WRIST_UP);
                    robot.driveTrain.CmdDrive(1,-179,0.35,0);
                    currentStage = stage._130_Drive_To_Wall;
                }
                break;
            case _130_Drive_To_Wall:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdTurn(90, 0.20);
                    currentStage = stage._135_Pickup_Speciman;
                }
                break;

            case _135_Pickup_Speciman:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.arm.setCurrentMode(Arm.Mode.START);
                    robot.driveTrain.CmdDrive(49,135,0.55,90);
                    currentStage = stage._137_Drive_To_Wall;
                }
                break;
            case _137_Drive_To_Wall:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(14,90,0.35,90);
                    currentStage = stage._138_Close_Claw;
                }
                break;
            case _138_Close_Claw:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.claw.setCurrentMode(Claw.Mode.CLAW_CLOSE);
                    runtime.reset();
                    currentStage = stage._139_Arm;
                }
                break;
            case _139_Arm:
                if (runtime.milliseconds() > 500)     {
                    robot.arm.setCurrentMode(Arm.Mode.NEUTRAL_POS);
                    runtime.reset();
                    currentStage = stage._140_Back_Up;
                }
                break;
            case _140_Back_Up:
                if (runtime.milliseconds() > 500)     {
                    robot.driveTrain.CmdDrive(38,-86,0.55,90);
                    currentStage = stage._150_Turn;
                }
                break;
            case _150_Turn:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_CHAMBER);
                    robot.driveTrain.cmdTurn(0, 0.20);
                    currentStage = stage._160_Place;
                }
                break;
            case _160_Place:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(22,0,0.35,0);
                    currentStage = stage._165_Retract_Arm;
                }
                break;
            case _165_Retract_Arm:
                if(robot.driveTrain.getCmdComplete()){
                    robot.claw.setCurrentMode(Claw.Mode.CLAW_OPEN);
                    robot.arm.setCurrentMode(Arm.Mode.RETRACT_FROM_HIGH_CHAMBER);
                    runtime.reset();
                    currentStage = stage._170_Backup;
                }
                break;
            case _170_Backup:
                if(robot.arm.getCmdComlete() || (runtime.milliseconds() > 750)){
                    //robot.arm.setCurrentMode(Arm.Mode.START);
                    robot.driveTrain.CmdDrive(1,-179,0.35,0);
                    currentStage = stage._190_Park;
                }
                break;
            case _190_Park:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.claw.setCurrentMode(Claw.Mode.WRIST_UP);
                    robot.arm.setCurrentMode(Arm.Mode.START);
                    robot.driveTrain.CmdDrive(60,115,0.55,0);
                    currentStage = stage._200_End;
                }
                break;
            case _200_End:
                if(robot.driveTrain.getCmdComplete()){
                    robot.stop();


                }
                break;
        }



    }  //  loop

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _10_Drive_Out,
        _20_Strafe_Left,
        _30_Drive_Forward,
        _35_Retract_Arm,
        _40_Drive_Back,
        _45_Turn_Around,
        _46_Drive_To_Side_Wall,
        _47_Drive_Past_Sample,
        _48_Drive_To_Side_Wall,
        _49_Sweep_Specimen,
        _50_Strafe_Right,
        _52_Drive_To_Wall,
        _55_Pick_Up_Specimen,
        _57_Close_Claw,
        _60_Lift_Arm,
        _70_Go_Back,
        _80_Turn,
        _85_Extend_Arm,
        _90_Place_Specimen,
        _100_Retract_Arm,
        _110_Backup,
        _120_Turn,
        _130_Drive_To_Wall,
        _135_Pickup_Speciman,
        _137_Drive_To_Wall,
        _138_Close_Claw,
        _139_Arm,
        _140_Back_Up,
        _150_Turn,
        _160_Place,
        _165_Retract_Arm,
        _170_Backup,
        _180_Arm,
        _190_Park,
        _200_End


    }
}