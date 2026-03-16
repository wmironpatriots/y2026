// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.fcs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc6423.lib.util.TunableNumber;
import org.frc6423.robot.Constants.Field;
import org.frc6423.robot.subsystem.shooter.ShooterSubsystem;

public class FireControlSystem {
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

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  private static final TunableNumber kLatencyCompensationSec =
      new TunableNumber("Fire Control System/Latency Compensation (seconds)", 0.3);

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  private static Pose2d mVirtualTarget = Pose2d.kZero;

  public static Pose2d getVirtualTarget() {
    return mVirtualTarget;
  }

  public static ProjectileParameters calculateParameters(
      Pose2d robotPose, ChassisSpeeds speedsWrtField) {
    // Compensate for approximated latency
    var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedsWrtField, robotPose.getRotation());
    var predictedPose =
        robotPose.exp(
            new Twist2d(
                speeds.vxMetersPerSecond * kLatencyCompensationSec.get(),
                speeds.vyMetersPerSecond * kLatencyCompensationSec.get(),
                speeds.omegaRadiansPerSecond * kLatencyCompensationSec.get()));

    // Calculate current target
    var isScoring = Field.getAllianceZone().contains(robotPose.getTranslation());
    var target =
        isScoring
            ? Field.getHubPose2d().getTranslation()
            : new Pose2d(
                    Field.getAllianceZone().getCenter().getMeasureX(),
                    robotPose.getMeasureY(),
                    Rotation2d.kZero)
                .getTranslation();
    var tree = isScoring ? kHubShotMap : kGroundShotMap;

    // Calculate virtual target
    var virtualTarget =
        calculateVirtualTarget(predictedPose.getTranslation(), target, tree, speedsWrtField);
    mVirtualTarget = new Pose2d(virtualTarget, Rotation2d.kZero);

    // Calculate final parameters
    var parameters = tree.get(predictedPose.getTranslation().getDistance(virtualTarget));

    if (parameters != null) return parameters;
    else return new ProjectileParameters(ShooterSubsystem.kMinAngleRevs, 0.0, 0.0);
  }

  public static Translation2d calculateVirtualTarget(
      Translation2d position,
      Translation2d target,
      InterpolatingProjectileParametersTree tree,
      ChassisSpeeds speedsWrtField) {
    var tof = tree.get(position.getDistance(target)).timeOfFlight();

    return target.minus(
        new Translation2d(
            speedsWrtField.vxMetersPerSecond * tof, speedsWrtField.vyMetersPerSecond * tof));
  }
}
