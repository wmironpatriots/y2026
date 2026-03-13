// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystem.shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc6423.lib.io.ServoIO;
import org.frc6423.lib.io.ServoIONone;
import org.frc6423.lib.io.ServoIOTalonFx;
import org.frc6423.lib.io.ServoIOTalonFxFlywheelSim;
import org.frc6423.lib.io.ServoIOTalonFxPivotSim;
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Constants.Matrix;
import org.frc6423.robot.Robot;
import org.frc6423.robot.subsystem.RobotState;

/** {@link SubsystemBase} Manager class for the shooter */
public class ShooterSubsystem extends SubsystemBase {
  /**
   * Static Factory for automatically configuring and creating a {@link ShooterSubsystem}
   *
   * @return {@link ShooterSubsystem}
   */
  public static ShooterSubsystem create() {
    return (Robot.isReal())
        ? new ShooterSubsystem(
            new ServoIOTalonFx(
                "Pivot", MotorType.KrakenX44, kCanBus, kPivotCanDeviceId, kPivotConfig),
            new ServoIOTalonFx(
                "Left", MotorType.KrakenX60, kCanBus, kFlywheelLeftCanDeviceId, kFlywheelConfig),
            new ServoIOTalonFx(
                "Right", MotorType.KrakenX60, kCanBus, kFlywheelRightCanDeviceId, kFlywheelConfig))
        : new ShooterSubsystem(
            new ServoIOTalonFxPivotSim(
                "Pivot",
                MotorType.KrakenX44,
                kCanBus,
                kPivotCanDeviceId,
                kPivotConfig,
                kPivotRotationalInertiaKgSquaredMeters,
                kPivotArmLengthMeters,
                Units.rotationsToDegrees(kMinAngleRevs),
                Units.rotationsToDegrees(kMaxAngleRevs),
                Units.rotationsToDegrees(kMinAngleRevs),
                true,
                DCMotor.getKrakenX60Foc(1),
                kPivotGearRatio),
            new ServoIOTalonFxFlywheelSim(
                "Left",
                MotorType.KrakenX60,
                kCanBus,
                kFlywheelLeftCanDeviceId,
                kPivotConfig,
                kFlywheelRotationalInertiaKgSquaredMeters,
                DCMotor.getKrakenX60Foc(2),
                kFlywheelGearRatio),
            new ServoIONone("Right")); // No need for this one me thinks
  }

  // * ~~~~~~~~ CONSTANTS ~~~~~~~~

  /** {@link CANBus} CAN bus devices are on */
  public static final CANBus kCanBus = Matrix.kSubsystemCanBus;

  /** {@link Integer} Unique CAN device identifier for the pivot servo */
  public static final int kPivotCanDeviceId = Matrix.kHoodId;

  /** {@link Integer} Unique CAN device identifier for left flywheel servo */
  public static final int kFlywheelLeftCanDeviceId = Matrix.kFlywheelLeftId;

  /** {@link Integer} Unique CAN device identifier for right flywheel servo */
  public static final int kFlywheelRightCanDeviceId = Matrix.kFlywheelRightId;

  /** {@link Double} Lower limit on pivot angular position */
  public static final double kMinAngleRevs = Units.degreesToRotations(14.703759);

  /** {@link Double} Upper limit on pivot angular position */
  public static final double kMaxAngleRevs = Units.degreesToRotations(43);

  /** {@link Double} Gear ratio between pivot servo rotor and the mech output */
  public static final double kPivotGearRatio = 2.57142857143 * 10.83;

  /** {@link Double} Gear ratio between flywheel servos to mechanism */
  public static final double kFlywheelGearRatio = (24.0 / 20.0);

  /** {@link TalonFXConfiguration} Hardware config of pivot servo */
  public static final TalonFXConfiguration kPivotConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(40.0)
                  .withStatorCurrentLimitEnable(true))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kPivotGearRatio));

  /** {@link TalonFXConfiguration} Hardware config of flywheel servos */
  public static final TalonFXConfiguration kFlywheelConfig =
      new TalonFXConfiguration()
          .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(70.0)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(40.0)
                  .withSupplyCurrentLimitEnable(true))
          .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(100.0))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kFlywheelGearRatio));

  /** {@link Double} Min filtered stator current value to be considered homed */
  public static final double kPivotCurrentZeroThreshold = 10.0;

  /** {@link Double} Moment of Inertia of pivot system */
  public static final double kPivotRotationalInertiaKgSquaredMeters = 402.290096 * 0.0002926397;

  /** {@link Double} Length of pivot arm in meters */
  public static final double kPivotArmLengthMeters = 0.5;

  /** {@link Double} Moment of Inertia of flywheel */
  public static final double kFlywheelRotationalInertiaKgSquaredMeters = 10.491008 * 0.0002926397;

  /** {@link Double} Radius of flywheel */
  public static final double kFlywheelRadiusMeters = Units.inchesToMeters(2);

  /**
   * {@link Transform2d} Transform from center of robot on floor to approximated projectile exit
   * location
   */
  public static final Transform3d kRobotToShooter =
      new Transform3d(
          Units.inchesToMeters(-8.3), 0.0, Units.inchesToMeters(24.6), Rotation3d.kZero);

  public static final InterpolatingProjectileParametersTree kHubShotMap =
      new InterpolatingProjectileParametersTree();

  public static final InterpolatingProjectileParametersTree kGroundShotMap =
      new InterpolatingProjectileParametersTree();

  static {
    // Hub Shot Map (Least Velocity Points)
    kHubShotMap.addSample(
        0.901563,
        new ProjectileParameters(1.2771938761619444, 5.264705389924421, 0.6206088677805742));
    kHubShotMap.addSample(
        1.401563,
        new ProjectileParameters(1.1358791999216864, 5.579475449166665, 0.627560397528039));
    kHubShotMap.addSample(
        1.901563,
        new ProjectileParameters(1.0006847908587537, 6.008474088255557, 0.619557923092387));
    kHubShotMap.addSample(
        2.401562,
        new ProjectileParameters(1.01083236466515, 6.3487178981822865, 0.7579363533409061));
    kHubShotMap.addSample(
        2.901562,
        new ProjectileParameters(1.0108323648882775, 6.753361797855045, 0.8670598674104143));
    kHubShotMap.addSample(
        3.401562,
        new ProjectileParameters(0.9462509437974365, 7.121430398521264, 0.8798182289527228));
    kHubShotMap.addSample(
        3.901562,
        new ProjectileParameters(0.946250944149243, 7.502604987141412, 0.9645542962636728));
    kHubShotMap.addSample(
        4.401562,
        new ProjectileParameters(0.9074351348150194, 7.865648092275598, 0.9909486303293451));
    kHubShotMap.addSample(
        4.901562,
        new ProjectileParameters(0.9074351353097425, 8.228548440909341, 1.0621028945410949));
    kHubShotMap.addSample(
        5.401562,
        new ProjectileParameters(0.8814159193754088, 8.579815964033871, 1.0929262758482317));
    kHubShotMap.addSample(
        5.901562,
        new ProjectileParameters(0.8814159199659383, 8.928074317294639, 1.1553502334578851));
    kHubShotMap.addSample(
        6.401562,
        new ProjectileParameters(0.881415920265334, 9.272319737632214, 1.2149800937106772));
    kHubShotMap.addSample(
        6.776562,
        new ProjectileParameters(0.8567412757904819, 9.52036706622863, 1.2212615359110726));
    kHubShotMap.addSample(
        7.276562,
        new ProjectileParameters(0.8567412766468879, 9.853178654062324, 1.275686707608683));
    kHubShotMap.addSample(
        7.776562,
        new ProjectileParameters(0.8567412770123607, 10.182609393714808, 1.3282310625945843));
    kHubShotMap.addSample(
        8.276563,
        new ProjectileParameters(0.856741277257644, 10.508893479539905, 1.3791079622775493));
    kHubShotMap.addSample(
        8.776563,
        new ProjectileParameters(0.85674127743473, 10.83229443393307, 1.4284939003591837));
    kHubShotMap.addSample(
        9.276563,
        new ProjectileParameters(0.8281159586028316, 11.14164955317965, 1.4285808177123847));
    kHubShotMap.addSample(
        9.776563,
        new ProjectileParameters(0.8281159595725861, 11.456281513897284, 1.4741680116253943));
    kHubShotMap.addSample(
        10.276563,
        new ProjectileParameters(0.8281159599822888, 11.76891585415174, 1.5186618975680894));
    kHubShotMap.addSample(
        10.776563,
        new ProjectileParameters(0.8281159602720778, 12.079781908944472, 1.562154058854181));
    kHubShotMap.addSample(
        11.276563,
        new ProjectileParameters(0.8126419973303874, 12.38542060700912, 1.5768771688967491));
    kHubShotMap.addSample(
        11.776563,
        new ProjectileParameters(0.812641998355039, 12.691721387866053, 1.6179792869430203));
    kHubShotMap.addSample(
        12.276563,
        new ProjectileParameters(0.8126419988147486, 12.996898117395801, 1.6582972339730058));
    kHubShotMap.addSample(
        12.776563,
        new ProjectileParameters(0.8126419991232069, 13.301134496893718, 1.6978848761605236));
    kHubShotMap.addSample(
        13.276563,
        new ProjectileParameters(0.8061326757140431, 13.602676925461624, 1.7242066233299556));
    kHubShotMap.addSample(
        13.776563,
        new ProjectileParameters(0.8061326759329653, 13.90490870358001, 1.7622279991279937));
    kHubShotMap.addSample(
        14.276563,
        new ProjectileParameters(0.8061326759329572, 14.206696629908983, 1.7996505127895444));
    kHubShotMap.addSample(
        14.500000,
        new ProjectileParameters(0.8061326759329395, 14.341453965847178, 1.8161893018317163));

    // Ground Shot Map (Least Velocity Points)
    kGroundShotMap.addSample(
        0.500000,
        new ProjectileParameters(0.8061326758573387, 1.5014777613384982, 0.49740117291738195));
    kGroundShotMap.addSample(
        1.000000,
        new ProjectileParameters(0.8061326763413506, 2.5000647846631634, 0.6009549214504335));
    kGroundShotMap.addSample(
        1.500000,
        new ProjectileParameters(0.8061326763412355, 3.287598994786462, 0.6897828674599029));
    kGroundShotMap.addSample(
        2.000000,
        new ProjectileParameters(0.8061326763409928, 3.9570778982213755, 0.76899240886175));
    kGroundShotMap.addSample(
        2.500000,
        new ProjectileParameters(0.8061326763407864, 4.55043647424551, 0.8412987789459383));
    kGroundShotMap.addSample(
        3.000000,
        new ProjectileParameters(0.8061326763401685, 5.090348726931933, 0.9083482950519163));
    kGroundShotMap.addSample(
        3.500000,
        new ProjectileParameters(0.8061326763395368, 5.590552739822466, 0.9712275575117884));
    kGroundShotMap.addSample(
        4.000000,
        new ProjectileParameters(0.8061326764814627, 6.060052242744736, 1.0306978308410377));
    kGroundShotMap.addSample(
        4.500000,
        new ProjectileParameters(0.8061326766115153, 6.505097676026233, 1.0873166410955286));
    kGroundShotMap.addSample(
        5.000000,
        new ProjectileParameters(0.8061326767312846, 6.930223494141052, 1.1415065590060443));
    kGroundShotMap.addSample(
        5.500000,
        new ProjectileParameters(0.8061326770087112, 7.33883583203188, 1.193596750694068));
    kGroundShotMap.addSample(
        6.000000,
        new ProjectileParameters(0.8061326772657605, 7.733566571178354, 1.2438494115918257));
    kGroundShotMap.addSample(
        6.500000,
        new ProjectileParameters(0.8061326775040609, 8.116497483198323, 1.2924773048761222));
    kGroundShotMap.addSample(
        7.000000,
        new ProjectileParameters(0.8061326777265486, 8.489308013272046, 1.3396558075554792));
    kGroundShotMap.addSample(
        7.500000,
        new ProjectileParameters(0.8061326779334276, 8.853376091204954, 1.3855314298330175));
    kGroundShotMap.addSample(
        7.781250,
        new ProjectileParameters(0.8061326781323375, 9.054760983420483, 1.4108111795695923));
    kGroundShotMap.addSample(
        8.281250,
        new ProjectileParameters(0.806132678317774, 9.40739647381529, 1.4548919824766078));
    kGroundShotMap.addSample(
        8.781250,
        new ProjectileParameters(0.8061326749990918, 9.75388379579816, 1.4979522608846767));
    kGroundShotMap.addSample(
        9.281250,
        new ProjectileParameters(0.8061326749935412, 10.094980872332522, 1.5400760344765734));
    kGroundShotMap.addSample(
        9.781250,
        new ProjectileParameters(0.8061326749882719, 10.431344149958079, 1.581336524794682));
    kGroundShotMap.addSample(
        10.281250,
        new ProjectileParameters(0.8061326749836809, 10.763547127700239, 1.621797978820751));
    kGroundShotMap.addSample(
        10.781250,
        new ProjectileParameters(0.8061326750838044, 11.092094786205088, 1.661517098657199));
    kGroundShotMap.addSample(
        11.281250,
        new ProjectileParameters(0.8061326751257867, 11.41743498535687, 1.7005441931264826));
    kGroundShotMap.addSample(
        11.781250,
        new ProjectileParameters(0.8061326751392676, 11.739967569534112, 1.7389241135910487));
    kGroundShotMap.addSample(
        12.281250,
        new ProjectileParameters(0.8061326751392665, 12.060051715515346, 1.776697019561604));
    kGroundShotMap.addSample(
        12.781250,
        new ProjectileParameters(0.806132675139262, 12.37801191967606, 1.8138990113678308));
    kGroundShotMap.addSample(
        13.281250,
        new ProjectileParameters(0.8061326749616289, 12.694142918365802, 1.8505626564267807));
    kGroundShotMap.addSample(
        13.781250,
        new ProjectileParameters(0.8061326750317377, 13.008713766143941, 1.8867174321444584));
    kGroundShotMap.addSample(
        14.281250,
        new ProjectileParameters(0.8061326750615371, 13.321971235645846, 1.9223900960763556));
    kGroundShotMap.addSample(
        14.781250,
        new ProjectileParameters(0.8061326750715261, 13.634142678661064, 1.9576050040279231));
    kGroundShotMap.addSample(
        15.281250,
        new ProjectileParameters(0.8061326750719175, 13.945438443383933, 1.9923843795136933));
    kGroundShotMap.addSample(
        15.781250,
        new ProjectileParameters(0.8061326750719169, 14.256053931251111, 2.026748545531309));
    kGroundShotMap.addSample(
        16.000000,
        new ProjectileParameters(0.8061326750719161, 14.391781281482015, 2.0416571188032537));
  }

  /** {@link String} Nt directory to store tunables in */
  public static final String kTunablesPrefix = "/Shooter";

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  /** {@link TunableNumber} Max acceptable angular position error (degrees) */
  public static TunableNumber kPivotToleranceDeg =
      new TunableNumber(kTunablesPrefix + "/Pivot/Epsilon (degrees)");

  /** {@link TunableNumber} Gain acting against static fricition */
  public static TunableNumber kPivotKs = new TunableNumber(kTunablesPrefix + "/Pivot/Ks");

  /** {@link TunableNumber} Gain inducing velocity */
  public static TunableNumber kPivotKv = new TunableNumber(kTunablesPrefix + "/Pivot/Kv");

  /** {@link TunableNumber} Gain inducing acceleration */
  public static TunableNumber kPivotKa = new TunableNumber(kTunablesPrefix + "/Pivot/Ka");

  /** {@link TunableNumber} Gain driving angular position error to zero */
  public static TunableNumber kPivotKp = new TunableNumber(kTunablesPrefix + "/Pivot/Kp");

  /** {@link TunableNumber} Gain driving the derivative of angular position error to zero */
  public static TunableNumber kPivotKd = new TunableNumber(kTunablesPrefix + "/Pivot/Kd");

  /** {@link TunableNumber} Angular velocity cap on pivot (revs per sec) */
  public static TunableNumber kPivotCruiseVelocityRevsPerSec =
      new TunableNumber(
          kTunablesPrefix + "/Pivot/Cruise Velocity (revolutions per second per second)");

  /** {@link TunaleNumber} Angular Acceleration cap on pivot (revs per sec per sec) */
  public static TunableNumber kPivotMaxAccelerationRevsPerSecPerSec =
      new TunableNumber(
          kTunablesPrefix + "/Pivot/Max Acceleratoin (revolutions per second per second)");

  /** {@link TunableNumber} Tunable for manually controlling the angular position of subsystem */
  public static TunableNumber kPivotTestingAngleDeg =
      new TunableNumber(kTunablesPrefix + "/Pivot/Testing Angle (degrees)");

  /** {@link TunableNumber} Max acceptable angular velocity error (degrees per second) */
  public static TunableNumber kFlywheelToleranceCmPerSec =
      new TunableNumber(kTunablesPrefix + "/Flywheel/Epsilon (centimeters per second)");

  /** {@link TunableNumber} Gain acting against static friciton */
  public static TunableNumber kFlywheelKs = new TunableNumber(kTunablesPrefix + "/Flywheel/Ks");

  /** {@link TunableNumber} Gain inducing velocity */
  public static TunableNumber kFlywheelKv = new TunableNumber(kTunablesPrefix + "/Flywheel/Kv");

  /** {@link TunableNumber} Gain inducing acceleration */
  public static TunableNumber kFlywheelKa = new TunableNumber(kTunablesPrefix + "/Flywheel/Ka");

  /** {@link TunableNumber} Gain driving angular position error to zero */
  public static TunableNumber kFlywheelKp = new TunableNumber(kTunablesPrefix + "/Flywheel/Kp");

  /** {@link TunableNumber} Gain driving the deriviative of angular position to zero */
  public static TunableNumber kFlywheelKd = new TunableNumber(kTunablesPrefix + "/Flywheel/Kd");

  /** {@link TunableNumber} Tunable for manually controlling the angular velocity of subsystem */
  public static TunableNumber kFlywheelTestingSpeedMetersPerSec =
      new TunableNumber(kTunablesPrefix + "/Flywheel/Testing Muzzle Velocity (meters per second)");

  public static TunableNumber kLatencyCompensationSec =
      new TunableNumber(kTunablesPrefix + "/LatencyCompensationSec");

  static {
    if (Robot.isReal()) {
      kPivotToleranceDeg.initDefault(1.0);
      kPivotKs.initDefault(0.0);
      kPivotKv.initDefault(0.0);
      kPivotKa.initDefault(0.0);
      kPivotKp.initDefault(4000.0);
      kPivotKd.initDefault(25.0);
      kPivotCruiseVelocityRevsPerSec.initDefault(2);
      kPivotMaxAccelerationRevsPerSecPerSec.initDefault(3);
      kPivotTestingAngleDeg.initDefault(Units.rotationsToDegrees(kMinAngleRevs));

      kFlywheelToleranceCmPerSec.initDefault(1.0);
      kFlywheelKs.initDefault(3.035);
      kFlywheelKv.initDefault(0.75631);
      kFlywheelKa.initDefault(7.4852);
      kFlywheelKp.initDefault(15.9);
      kFlywheelKd.initDefault(0.0);
      kFlywheelTestingSpeedMetersPerSec.initDefault(0.0);

      kLatencyCompensationSec.initDefault(0.3);
    } else {
      kPivotToleranceDeg.initDefault(1.0);
      kPivotKs.initDefault(0.0);
      kPivotKv.initDefault(0.0);
      kPivotKa.initDefault(0.0);
      kPivotKp.initDefault(0.0);
      kPivotKd.initDefault(0.0);
      kPivotTestingAngleDeg.initDefault(Units.rotationsToDegrees(kMinAngleRevs));

      kFlywheelToleranceCmPerSec.initDefault(1.0);
      kFlywheelKs.initDefault(4.8691);
      kFlywheelKv.initDefault(0.10515);
      kFlywheelKa.initDefault(1.729);
      kFlywheelKp.initDefault(0.26322);
      kFlywheelKd.initDefault(0.0);
      kFlywheelTestingSpeedMetersPerSec.initDefault(0.0);

      kLatencyCompensationSec.initDefault(0.3);
    }
  }

  /**
   * Convert a flywheel velocity (revolutions per second) to muzzle velocity (meters per second)
   *
   * @param flywheelVelocityRps {@link Double} Flywheel Velocity in revolutions per second
   * @return {@link Double}
   */
  public static double flywheelVelocityRpsToMuzzleVelocityMps(double flywheelVelocityRps) {
    return flywheelVelocityRps * Math.PI * 2 * kFlywheelRadiusMeters * 0.5;
  }

  /**
   * Convert a muzzle velocity (meters per second) to flywheel velocity (revolutions per second)
   *
   * @param muzzleVelocityMps {@link Double} Muzzle Velocity in meters per second
   * @return {@link Double}
   */
  public static double muzzleVelocityMpsToFlywheelVelocityRps(double muzzleVelocityMps) {
    return muzzleVelocityMps * 2 / kFlywheelRadiusMeters / (2 * Math.PI);
  }

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  /** {@link ServoIO} Hardware Interface for the hood pivot */
  @Logged private final ServoIO mPivot;

  /** {@link ServoIO} Hardware Interface for the left side flywheel motor */
  @Logged private final ServoIO mLeft;

  /** {@link ServoIO} Hardware Interface for the right side flywheel motor */
  @Logged private final ServoIO mRight;

  /** {@link SysIdRoutine} Characterization routines for the flywheel */
  private final SysIdRoutine mFlywheelCharacterization;

  /** {@link LinearFilter} Moving average filter for checking current spikes for the pivot motor */
  private final LinearFilter mPivotCurrentFilter = LinearFilter.movingAverage(10);

  @Logged(name = "Filtered Stator Current (amps)")
  private double mPivotCurrentFilterValue = 0.0;

  /** {@link Boolean} Latch for checking if subsystem has been homed */
  @Logged(name = "Is Hood Homed (bool)")
  private boolean mIsHomed = false;

  private Debouncer mIsHolding = new Debouncer(0.2);

  @Logged(name = "Target Muzzle Velocity (meters per second)")
  private double mTargetMuzzleVelocityMps = 0.0;

  @Logged(name = "Target Angle (Rotation2d)")
  private Rotation2d mTargetRotation2d = Rotation2d.fromRotations(kMinAngleRevs);

  protected ShooterSubsystem(ServoIO pivot, ServoIO left, ServoIO right) {
    mPivot = pivot;
    mLeft = left;
    mRight = right;

    mRight.setLeader(mLeft, true);

    mLeft.resetRelativeEncoder();

    mFlywheelCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(25).per(Second),
                Volts.of(15),
                null,
                (state) -> SmartDashboard.putString("FlywheelSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> mLeft.setTorqueCurrentOutput(voltage.in(Volts)), null, this));

    mPivot.resetRelativeEncoder(kMinAngleRevs);

    setDefaultCommand(stowAndDeaccel());
    // setDefaultCommand(
    //     runCurrentHoming()
    //         .unless(new Trigger(() -> isHomed()))
    //         .andThen(stowAndDeaccel())
    //         .repeatedly());
  }

  @Override
  public void periodic() {
    // Update all Hardware
    mPivot.periodic();
    mLeft.periodic();
    mRight.periodic();

    // Calculate filtered pivot current
    mPivotCurrentFilterValue = mPivotCurrentFilter.calculate(mPivot.getStatorCurrentAmps());

    // Update gains from tunables
    if (kPivotKs.hasChanged(hashCode())
        || kPivotKv.hasChanged(hashCode())
        || kPivotKa.hasChanged(hashCode())
        || kPivotKp.hasChanged(hashCode())
        || kPivotKd.hasChanged(hashCode())
        || kPivotCruiseVelocityRevsPerSec.hasChanged(hashCode())
        || kPivotMaxAccelerationRevsPerSecPerSec.hasChanged(hashCode())) {
      mPivot.setGains(
          kPivotKp.get(), kPivotKd.get(), kPivotKs.get(), 0.0, kPivotKv.get(), kPivotKa.get());
      mPivot.setProfilingConstraints(
          kPivotCruiseVelocityRevsPerSec.get(), kPivotMaxAccelerationRevsPerSecPerSec.get());
    }

    if (kFlywheelKs.hasChanged(hashCode())
        || kFlywheelKv.hasChanged(hashCode())
        || kFlywheelKa.hasChanged(hashCode())
        || kFlywheelKp.hasChanged(hashCode())
        || kFlywheelKd.hasChanged(hashCode())) {
      mLeft.setGains(
          kFlywheelKp.get(),
          kFlywheelKd.get(),
          kFlywheelKs.get(),
          0.0,
          kFlywheelKv.get(),
          kFlywheelKa.get());
      mRight.setGains(
          kFlywheelKp.get(),
          kFlywheelKd.get(),
          kFlywheelKs.get(),
          0.0,
          kFlywheelKv.get(),
          kFlywheelKa.get());
    }
  }

  // * ~~~~~~~~ GETTERS ~~~~~~~~

  public boolean isHomed() {
    return mIsHomed;
  }

  /**
   * Check if shooter is holding setpoint aka is ready to shoot
   *
   * @return {@link Boolean}
   */
  @Logged(name = "Is Holding Setpoint (bool)", importance = Importance.INFO)
  public boolean isHoldingSetpoint() {
    return mIsHolding.calculate(
        MathUtil.isNear(
                getTargetRotation2d().getDegrees(),
                getRotation2d().getDegrees(),
                kPivotToleranceDeg.getAsDouble())
            && MathUtil.isNear(
                getTargetMuzzleVelocityMetersPerSec(),
                getApproximatedMuzzleVelocityMetersPerSec(),
                kFlywheelToleranceCmPerSec.getAsDouble() / 100.0));
  }

  /**
   * Get hood angle
   *
   * @return {@link Rotation2d}
   */
  @Logged(name = "Angle (Rotation2d)", importance = Importance.INFO)
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(mLeft.getAngularPositionRevs());
  }

  /**
   * Get desired hood angle
   *
   * @return {@link Rotation2d}
   */
  @Logged(name = "Target Angle (Rotation2d)", importance = Importance.INFO)
  public Rotation2d getTargetRotation2d() {
    return mTargetRotation2d;
  }

  /**
   * Get expected projectile exit velocity assuming no slippage (meters per second)
   *
   * @return {@link Double}
   */
  @Logged(name = "Approximated Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public double getApproximatedMuzzleVelocityMetersPerSec() {
    return flywheelVelocityRpsToMuzzleVelocityMps(getFlywheelVelocityRevsPerSec());
  }

  /**
   * Get flywheel angular velocity (revolutions per second)
   *
   * @return {@link Double}
   */
  @Logged(name = "Flywheel Velocity (revolutions per second)", importance = Importance.INFO)
  public double getFlywheelVelocityRevsPerSec() {
    return mLeft.getAngularVelocityRevsPerSec();
  }

  /**
   * Get desired projectile exit velocity (meters per second)
   *
   * @return {@link Double}
   */
  @Logged(name = "Target Muzzle Velocity (meters per second)", importance = Importance.INFO)
  public double getTargetMuzzleVelocityMetersPerSec() {
    return mTargetMuzzleVelocityMps;
  }

  /**
   * Get desired flywheel angular velocity (revolutions per second)
   *
   * @return {@link Double}
   */
  @Logged(name = "Target Flywheel Velocity (revolutions per second)", importance = Importance.INFO)
  public double getTargetFlywheelVelocityRevsPerSec() {
    return muzzleVelocityMpsToFlywheelVelocityRps(getTargetMuzzleVelocityMetersPerSec());
  }

  // * ~~~~~~~~ COMMANDS ~~~~~~~~

  /**
   * Run flywheel System Identification (quasis forward, quasis reverse, dynamic forward, dynamic
   * reverse)
   *
   * @return {@link Command}
   */
  public Command runFlywheelCharacterizationSequence() {
    return Commands.sequence(
            Commands.print("Starting Quasistatic Forward"),
            mFlywheelCharacterization.quasistatic(Direction.kForward),
            Commands.print("Starting Quasistatic Reverse"),
            mFlywheelCharacterization.quasistatic(Direction.kReverse),
            Commands.print("Starting Dynamic Forward"),
            mFlywheelCharacterization.dynamic(Direction.kForward),
            Commands.print("Starting Dynamic Forward"),
            mFlywheelCharacterization.dynamic(Direction.kReverse))
        .beforeStarting(
            () -> {
              mTargetMuzzleVelocityMps = 0.0;
              mTargetRotation2d = Rotation2d.fromRotations(kMinAngleRevs);
            })
        .withName("Run Flywheel Characterization");
  }

  /**
   * Run homing sequence to reset hood pivot
   *
   * <p>Command will run a constant backwards voltage until pivot current spikes
   *
   * @return {@link Command}
   */
  public Command runCurrentHoming() {
    return this.run(() -> mPivot.setVoltageOutput(-4.5, true))
        .until(
            new Trigger(() -> Math.abs(mPivotCurrentFilterValue) > kPivotCurrentZeroThreshold)
                .debounce(0.25))
        .andThen(
            this.run(() -> mPivot.resetRelativeEncoder()).alongWith(Commands.print("Hood Homed")))
        .finallyDo((bool) -> mIsHomed = bool ? false : true)
        .withName("Run Current Homing");
  }

  /**
   * Run shooter at setpoints given by tunables {@link #kFlywheelTestingSpeedMetersPerSec} & {@link
   * #kPivotTestingAngleDeg}
   *
   * @return {@link Command}
   */
  public Command runTunablesSetpoint() {
    return runSetpoint(
            kFlywheelTestingSpeedMetersPerSec,
            () -> Rotation2d.fromDegrees(kPivotTestingAngleDeg.get()))
        .withName("Run Tunables Setpoint");
  }

  /**
   * Run shooter at specified setpoints
   *
   * @param desiredMuzzleVelMps {@link DoubleSupplier} Desired muzzle velocity (projectile exit
   *     velocity) in meters per second
   * @param desiredAngle {@link Supplier} of {@link Rotation2d} Desired hood angle
   * @return {@link Command}
   */
  public Command runSetpoint(
      DoubleSupplier desiredMuzzleVelMps, Supplier<Rotation2d> desiredAngle) {
    return this.run(
            () -> {
              // Clamp values
              mTargetMuzzleVelocityMps = desiredMuzzleVelMps.getAsDouble();
              mTargetRotation2d =
                  Rotation2d.fromRotations(
                      MathUtil.clamp(
                          desiredAngle.get().getRotations(), kMinAngleRevs, kMaxAngleRevs));

              mLeft.setProfiledVelocitySetpoint(getTargetFlywheelVelocityRevsPerSec());
              mPivot.setProfiledPositionSetpoint(mTargetRotation2d.getRotations());
            })
        .withName("Run Shooter Setpoint");
  }

  public Command runSetpoint(
      Supplier<InterpolatingProjectileParametersTree> tree, Supplier<Pose2d> target) {
    return runSetpoint(
        () -> calculateParameters(tree.get(), target.get()).velocity(),
        () ->
            Rotation2d.fromDegrees(90)
                .minus(
                    Rotation2d.fromRadians(calculateParameters(tree.get(), target.get()).pitch()))
                .times(-1.0));
  }

  /**
   * Stow hood & let flywheel coast
   *
   * @return {@link Command}
   */
  public Command stowAndCoast() {
    return this.run(
        () -> {
          mTargetMuzzleVelocityMps = getApproximatedMuzzleVelocityMetersPerSec();
          mTargetRotation2d = Rotation2d.fromRotations(kMinAngleRevs);

          mLeft.setNeutral();
          mPivot.setProfiledPositionSetpoint(mTargetRotation2d.getRotations());
        });
  }

  /**
   * Stow hood & apply flywheel brake
   *
   * @return {@link Command}
   */
  public Command stowAndDeaccel() {
    return this.run(
        () -> {
          mTargetMuzzleVelocityMps = 0.0;
          mTargetRotation2d = Rotation2d.fromRotations(kMinAngleRevs);

          mLeft.setVelocitySetpoint(getTargetFlywheelVelocityRevsPerSec());
          mPivot.setProfiledPositionSetpoint(mTargetRotation2d.getRotations());
        });
  }

  public ProjectileParameters calculateParameters(
      InterpolatingProjectileParametersTree tree, Pose2d target) {
    //   // Get current state
    //   var position = RobotState.getInstance().getEstimatedPosition();
    //   var fieldRelativeSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();
    //   var speeds = RobotState.getInstance().getChassisSpeeds();

    //   var target = Flags.getRobotAlliancePose2d(Rebuilt.kHubPose2d);

    //   // Compensate for predicted latency
    //   var predictedPosition =
    //       position.exp(
    //           new Twist2d(
    //               speeds.vxMetersPerSecond * kLatencyCompensationSec.get(),
    //               speeds.vyMetersPerSecond * kLatencyCompensationSec.get(),
    //               speeds.omegaRadiansPerSecond * kLatencyCompensationSec.get()));

    //   // Calculate for a standstill shot at position for a tof approximation
    //   var approximatedTof =
    //       kHubShotMap
    //           .get(predictedPosition.getTranslation().getDistance(target.getTranslation()))
    //           .timeOfFlightSec();

    //   // Calculate virtual target
    //   mVirtualTarget =
    //       target
    //           .getTranslation()
    //           .minus(
    //               new Translation2d(
    //                   fieldRelativeSpeeds.vxMetersPerSecond * approximatedTof,
    //                   fieldRelativeSpeeds.vyMetersPerSecond * approximatedTof));

    //   // Calculate final shot
    //   var parameters =
    //       kHubShotTree.get(predictedPosition.getTranslation().getDistance(mVirtualTarget));

    //   if (parameters != null) {
    //     return parameters;
    //   } else return new ProjectileParameters(0.0, 0.0, 0.0);

    var result =
        tree.calculateProjectileParameters(
            RobotState.getInstance().getEstimatedPosition().getTranslation(),
            target.getTranslation());

    if (result != null) return result;
    else return new ProjectileParameters(Units.rotationsToRadians(kMinAngleRevs), 5.0, 0.0);
  }
}
