// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.config.HopperConfig;
import frc.robot.config.IntakeConfig;
import frc.robot.config.TunerConstants;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  private Flywheel flywheel = new Flywheel();
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private Hopper hopper = new Hopper();
  private Intake intake = new Intake();
  private Kicker kicker = new Kicker();
  private CommandXboxController controller = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;
  public static boolean isExtended = false;

  private Command shootGroup =
      Commands.parallel(
          new KickerCommand(kicker, KickerCommand.Position.INTAKE),
          new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE));

  private Command hopperShoot() {
    return Commands.repeatingSequence(
        Commands.runEnd(
                () -> hopper.slowMove(HopperConfig.HOPPER_RETRACT_ROTATION),
                () -> hopper.stop(),
                hopper)
            .withDeadline(Commands.waitSeconds(.75)),
        Commands.runEnd(
                () -> hopper.slowMove(HopperConfig.HOPPER_EXTEND_ROTATION),
                () -> hopper.stop(),
                hopper)
            .withDeadline(Commands.waitSeconds(.75)));
  }

  public RobotContainer() {

    NamedCommands.registerCommand(
        "idle",
        Commands.runOnce(() -> hopper.stop(), hopper)
            .alongWith(Commands.runOnce(() -> intake.stop(), intake))
            .alongWith(Commands.runOnce(() -> kicker.stop(), kicker)));

    NamedCommands.registerCommand(
        "hopper deploy",
        Commands.runEnd(
                () -> hopper.move(HopperConfig.HOPPER_EXTEND_ROTATION), () -> hopper.stop(), hopper)
            .alongWith(Commands.run(() -> System.out.println("Hopper Deployed")))
            .withDeadline(Commands.waitSeconds(0.75)));

    NamedCommands.registerCommand(
        "hopper retract",
        Commands.runEnd(
                () -> hopper.move(HopperConfig.HOPPER_RETRACT_ROTATION),
                () -> hopper.stop(),
                hopper)
            .alongWith(Commands.run(() -> System.out.println("Hopper Deployed")))
            .withDeadline(Commands.waitSeconds(0.75)));

    NamedCommands.registerCommand(
        "intake",
        Commands.run(() -> intake.intake(IntakeConfig.INTAKE_INTAKE_SPEED), intake)
            .alongWith(Commands.run(() -> System.out.println("Intaking"))));

    NamedCommands.registerCommand(
        "rev shooter",
        new FlywheelCommand(flywheel, FlywheelCommand.Position.TRENCH_SHOT, drivetrain)
            .withDeadline(Commands.waitSeconds(3)));

    NamedCommands.registerCommand(
        "shoot long",
        Commands.parallel(
                new KickerCommand(kicker, KickerCommand.Position.INTAKE),
                new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE),
                new FlywheelCommand(flywheel, FlywheelCommand.Position.TRENCH_SHOT, drivetrain),
                hopperShoot())
            .withDeadline(Commands.waitSeconds(5)));

    NamedCommands.registerCommand(
        "shoot depot",
        Commands.parallel(
                new KickerCommand(kicker, KickerCommand.Position.INTAKE),
                new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE),
                new FlywheelCommand(flywheel, FlywheelCommand.Position.DEPOT_SHOT, drivetrain))
            .withDeadline(Commands.waitSeconds(5)));

    NamedCommands.registerCommand(
        "short shoot",
        Commands.parallel(
                new KickerCommand(kicker, KickerCommand.Position.INTAKE),
                new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE),
                new FlywheelCommand(flywheel, FlywheelCommand.Position.TRENCH_SHOT, drivetrain))
            .withDeadline(Commands.waitSeconds(3)));

    NamedCommands.registerCommand(
        "hopper down",
        Commands.runEnd(
                () -> hopper.move(HopperConfig.HOPPER_EXTEND_ROTATION), () -> hopper.stop(), hopper)
            .withDeadline(Commands.waitSeconds(0.5)));

    NamedCommands.registerCommand(
        "hub shot",
        Commands.parallel(
                new KickerCommand(kicker, KickerCommand.Position.INTAKE),
                new IntakeCommand(intake, IntakeCommand.Position.SLOW_INTAKE),
                new FlywheelCommand(flywheel, FlywheelCommand.Position.HUB_SHOT, drivetrain))
            .withDeadline(Commands.waitSeconds(10)));

    autoChooser = AutoBuilder.buildAutoChooser("Test");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  private void configureBindings() {
    /* Default commands */
    // Drive
    drivetrain.setDefaultCommand(
        new DrivetrainCommand(
            drivetrain,
            DrivetrainCommand.Position.TELEOP,
            controller::getLeftX,
            controller::getLeftY,
            controller::getRightX));

    /* Controls */
    // Left Trigger = Flywheel On and Off
    controller
        .leftTrigger()
        .toggleOnTrue(
            new FlywheelCommand(flywheel, FlywheelCommand.Position.TRENCH_SHOT, drivetrain));
    // Right Trigger = Shoot On and Off
    controller
        .rightTrigger()
        .toggleOnTrue(
            (shootGroup.alongWith(hopperShoot()))
                .finallyDo(
                    interrupt ->
                        CommandScheduler.getInstance()
                            .schedule(
                                Commands.runEnd(
                                        () -> hopper.move(HopperConfig.HOPPER_EXTEND_ROTATION),
                                        () -> hopper.stop(),
                                        hopper)
                                    .withDeadline(Commands.waitSeconds(1)))));

    // Left Back Button = Shoot Reverse
    controller.x().whileTrue(new KickerCommand(kicker, KickerCommand.Position.OUTTAKE));

    // Left Bumper = Hopper deploy/stow toggle
    controller
        .leftBumper()
        .onTrue(
            Commands.either(
                // IF isExtended is TRUE: Flip state to false, then retract
                Commands.runOnce(() -> isExtended = false)
                    .andThen(
                        Commands.runEnd(
                                () -> hopper.move(HopperConfig.HOPPER_RETRACT_ROTATION),
                                () -> hopper.stop(),
                                hopper)
                            .withDeadline(Commands.waitSeconds(.5))),

                // IF isExtended is FALSE: Flip state to true, then extend
                Commands.runOnce(() -> isExtended = true)
                    .andThen(
                        Commands.runEnd(
                                () -> hopper.move(HopperConfig.HOPPER_EXTEND_ROTATION),
                                () -> hopper.stop(),
                                hopper)
                            .withDeadline(Commands.waitSeconds(.5))),

                // Condition to check
                () -> isExtended));

    // Right Bumper = Intake ON/OFF toggle
    controller.rightBumper().toggleOnTrue(new IntakeCommand(intake, IntakeCommand.Position.INTAKE));

    // Back Right (y) = Reverse intake
    controller.y().whileTrue(new IntakeCommand(intake, IntakeCommand.Position.OUTTAKE));

    // Back button = Zero Pigeon
    controller.back().onTrue(Commands.runOnce(drivetrain::zeroPigeon));

    // B = Zero Hopper
    controller.b().onTrue((Commands.runOnce(() -> hopper.zero(), hopper)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }
}
