package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Settings;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Disabled
@Autonomous(name = "AutonRightMultiSpecimen", group = "Auton")
// @Autonomous(...) is the other common choice

public class AutonRightMultiSpecimen extends OpMode {

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
        telemetry.addData("Test AutonRightMultiSpecimen", "Initialized");

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
                currentStage = stage._10_Drive_Out;
                break;
            case _10_Drive_Out:
                robot.driveTrain.CmdDrive(3,0,0.35,0);
                robot.arm.setCurrentMode(Arm.Mode.NEUTRAL_POS);
                currentStage = stage._20_Strafe_Left;
                break;
            case _20_Strafe_Left:
                if(robot.driveTrain.getCmdComplete()){
                    robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_CHAMBER);
                    robot.driveTrain.CmdDrive(5,-90,0.35,0);
                    currentStage = stage._30_Drive_Forward;
                }
                break;
            case _30_Drive_Forward:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(22,0,0.16,0);
                    currentStage = stage._35_Retract_Arm;
                }
                break;
            case _35_Retract_Arm:
                if(robot.driveTrain.getCmdComplete()){
                    robot.arm.setCurrentMode(Arm.Mode.RETRACT_FROM_HIGH_CHAMBER);
                    runtime.reset();
                    currentStage = stage._40_Drive_Back;
                }
                break;
            case _40_Drive_Back:
                if(robot.arm.getCmdComlete() || (runtime.milliseconds() > 750)){
                    robot.driveTrain.CmdDrive(5,-179,0.35,0);
                    currentStage = stage._45_Turn_Around;
                }
                break;
            case _45_Turn_Around:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdTurn(90, 0.20);
                    currentStage = stage._50_Strafe_Right;
                }
                break;
            /*case _48_Drive_To_Wall:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(10,-179,0.35,90);
                    currentStage = stage._49_Drive_Out;
                }
                break;
            case _49_Drive_Out:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(1,0,0.35,90);
                    currentStage = stage._50_Strafe_Right;
                }
                break;*/
            case _50_Strafe_Right:
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
                break;
            case _55_Pick_Up_Specimen:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(10,90,0.35,90);
                    currentStage = stage._60_Lift_Arm;
                }
                break;
            case _60_Lift_Arm:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.arm.setCurrentMode(Arm.Mode.NEUTRAL_POS);
                    runtime.reset();
                    currentStage = stage._70_Go_Back;
                }
                break;
            case _70_Go_Back:
                if (runtime.milliseconds() > 1000) {
                    robot.driveTrain.CmdDrive(42,-85,0.35,90);
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

                    robot.driveTrain.CmdDrive(17,0,0.35,0);
                    currentStage = stage._100_End;
                }
                break;
            case _100_End:
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
        _48_Drive_To_Wall,
        _49_Drive_Out,
        _50_Strafe_Right,
        _52_Drive_To_Wall,
        _55_Pick_Up_Specimen,
        _60_Lift_Arm,
        _70_Go_Back,
        _80_Turn,
        _85_Extend_Arm,
        _90_Place_Specimen,
        _100_End


    }
}