package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Settings;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

//@Disabled
@Autonomous(name = "AutonLeftBasket", group = "Auton")
// @Autonomous(...) is the other common choice

public class AutonLeftBasket extends OpMode {

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
        telemetry.addData("Test AutonLeftBasket", "Initialized");

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
                robot.arm.setCurrentMode(Arm.Mode.BACKDROP_HIGH_BASKET_ARM_ONLY);
                //robot.claw.setCurrentMode(Claw.Mode.WRIST_UP);
                currentStage = stage._10_Strafe_To_Wall;
                break;
            case _10_Strafe_To_Wall:
                robot.driveTrain.CmdDrive(20, -62, 0.35, 30);
                runtime.reset();
                currentStage = stage._15_Back_Up;
                break;
            case _15_Back_Up:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(1, -179, 0.35, 30);
                    currentStage = stage._20_Drop;
                }
                break;
            case _20_Drop:
                if(runtime.milliseconds() > 3800){
                    robot.claw.setCurrentMode(Claw.Mode.WRIST_UP);
                }
                if(runtime.milliseconds() > 4200){
                    robot.claw.setCurrentMode(Claw.Mode.CLAW_OPEN);
                    runtime.reset();
                    currentStage = stage._30_Drive_To_Block1;
                }
                break;
            case _30_Drive_To_Block1:
                if(runtime.milliseconds() > 500){
                    robot.arm.setCurrentMode(Arm.Mode.PICKUP_GROUND);
                    robot.claw.setCurrentMode(Claw.Mode.WRIST_TO_GROUND);
                    robot.driveTrain.CmdDrive(9,80,0.35,0);
                    runtime.reset();
                    currentStage = stage._40_Pick_Up;
                }
                break;
            case _40_Pick_Up:
                if(runtime.milliseconds() > 2700){
                    robot.claw.setCurrentMode(Claw.Mode.CLAW_CLOSE);
                    runtime.reset();
                    currentStage = stage._50_Drive_Back;
                }
                break;
            case _50_Drive_Back:
                if(runtime.milliseconds() > 700){
                    robot.arm.setCurrentMode(Arm.Mode.BACKDROP_HIGH_BASKET_ARM_ONLY);
                    robot.driveTrain.CmdDrive(7,-160,0.35,30);
                    runtime.reset();
                    currentStage = stage._60_Drop;
                }
                break;
            /*case _55_Back_Up:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(2,-179,0.35,30);
                    currentStage = stage._60_Drop;
                }
                break;*/
            case _60_Drop:
                if(runtime.milliseconds() > 3000){
                    robot.claw.setCurrentMode(Claw.Mode.WRIST_UP);
                }
                if(runtime.milliseconds() > 3500){
                    robot.claw.setCurrentMode(Claw.Mode.CLAW_OPEN);
                    runtime.reset();
                    currentStage = stage._70_Drive_To_Block2;
                }
                break;
            case _70_Drive_To_Block2:
                if(runtime.milliseconds() > 500){
                    robot.arm.setCurrentMode(Arm.Mode.PICKUP_GROUND);
                    robot.claw.setCurrentMode(Claw.Mode.WRIST_TO_GROUND);
                    robot.driveTrain.CmdDrive(12,-30,0.35,0);
                    runtime.reset();
                    currentStage = stage._80_Pick_Up2;
                }
                break;
            case _80_Pick_Up2:
                if(runtime.milliseconds() > 2500){
                    robot.claw.setCurrentMode(Claw.Mode.CLAW_CLOSE);
                    runtime.reset();
                    currentStage = stage._90_Drive_Back;
                }
                break;
            case _90_Drive_Back:
                if(runtime.milliseconds() > 700){
                    robot.arm.setCurrentMode(Arm.Mode.BACKDROP_HIGH_BASKET_ARM_ONLY);
                    robot.driveTrain.CmdDrive(7.5,-175,0.35,30);
                    runtime.reset();
                    currentStage = stage._100_Drop;
                }
                break;
            case _100_Drop:
                if(runtime.milliseconds() > 3000){
                    robot.claw.setCurrentMode(Claw.Mode.WRIST_UP);
                }
                if(runtime.milliseconds() > 3500){
                    robot.claw.setCurrentMode(Claw.Mode.CLAW_OPEN);
                    runtime.reset();
                    currentStage = stage._110_Drive_To_Block3;
                }
                break;
            case _110_Drive_To_Block3:
                if (runtime.milliseconds() > 500) {
                    robot.arm.setCurrentMode(Arm.Mode.START);
                    robot.claw.setCurrentMode(Claw.Mode.WRIST_UP);
                    robot.driveTrain.CmdDrive(39,40,0.35,0);
                    runtime.reset();
                    currentStage = stage._120_Pick_Up;
                }
                break;
            case _120_Pick_Up:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(28, 90, 0.35, 0);
                    currentStage = stage._200_End;
                }
                break;
            case _130_Drive_Back:
                break;
            case _140_Drop3:
                break;
            case _150_Drive_Forward:
                break;
            case _160_Park:
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
        _10_Strafe_To_Wall,
        _15_Back_Up,
        _20_Drop,
        _30_Drive_To_Block1,
        _40_Pick_Up,
        _50_Drive_Back,
        _55_Back_Up,
        _60_Drop,
        _70_Drive_To_Block2,
        _80_Pick_Up2,
        _90_Drive_Back,
        _100_Drop,
        _110_Drive_To_Block3,
        _120_Pick_Up,
        _130_Drive_Back,
        _140_Drop3,
        _150_Drive_Forward,
        _160_Park,
        _200_End


    }
}