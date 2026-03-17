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
        new ProjectileParameters(1.277193876161945, 5.264705389924398, 0.620608867780567));
    kHubShotMap.addSample(
        1.401563,
        new ProjectileParameters(1.1358791999308975, 5.57947544916603, 0.627560397540539));
    kHubShotMap.addSample(
        1.901563,
        new ProjectileParameters(1.000684790858748, 6.0084740882555785, 0.6195579230923745));
    kHubShotMap.addSample(
        2.401562,
        new ProjectileParameters(1.0108323646762731, 6.34871789818227, 0.7579363533546051));
    kHubShotMap.addSample(
        2.901562, new ProjectileParameters(1.010832364899025, 6.75336179786416, 0.867059867424476));
    kHubShotMap.addSample(
        3.401562,
        new ProjectileParameters(0.9462509438235296, 7.121430398521246, 0.8798182289853479));
    kHubShotMap.addSample(
        3.901562,
        new ProjectileParameters(0.9462509442078333, 7.50260498716379, 0.9645542963400501));
    kHubShotMap.addSample(
        4.401562,
        new ProjectileParameters(0.9074351348687948, 7.865648092275542, 0.9909486303995867));
    kHubShotMap.addSample(
        4.901562,
        new ProjectileParameters(0.9074351353503192, 8.22854844092392, 1.062102894596593));
    kHubShotMap.addSample(
        5.401562,
        new ProjectileParameters(0.907435135592692, 8.587658364406366, 1.1292554753469188));
    kHubShotMap.addSample(
        5.901562,
        new ProjectileParameters(0.9074351357561066, 8.942328781355119, 1.1930811036321753));
    kHubShotMap.addSample(
        6.401562,
        new ProjectileParameters(0.9074351358794791, 9.292455304680304, 1.254086498026572));
    kHubShotMap.addSample(
        6.901562,
        new ProjectileParameters(0.8549159499610518, 9.6038564746987, 1.2323521408393834));
    kHubShotMap.addSample(
        7.401562,
        new ProjectileParameters(0.8549159506818588, 9.93548815279047, 1.2861978017824631));
    kHubShotMap.addSample(
        7.901562,
        new ProjectileParameters(0.8549159510323836, 10.2638107921241, 1.3382214678664746));
    kHubShotMap.addSample(
        8.401563,
        new ProjectileParameters(0.8549159512684457, 10.589064303958782, 1.388626286269622));
    kHubShotMap.addSample(
        8.901563,
        new ProjectileParameters(0.85491595144494, 10.911512745058172, 1.4375808570926736));
    kHubShotMap.addSample(
        9.401563,
        new ProjectileParameters(0.8499753458949898, 11.227893438976837, 1.476800154281469));
    kHubShotMap.addSample(
        9.901563,
        new ProjectileParameters(0.8499753458603082, 11.544846697661715, 1.5230418140841522));
    kHubShotMap.addSample(
        10.401563,
        new ProjectileParameters(0.8499753460227361, 11.859801763479394, 1.5682014500021022));
    kHubShotMap.addSample(
        10.901563,
        new ProjectileParameters(0.8499753458191178, 12.173001472222888, 1.6123681208175438));
    kHubShotMap.addSample(
        11.401563,
        new ProjectileParameters(0.8499753458059532, 12.484676586869222, 1.6556193928856069));
    kHubShotMap.addSample(
        11.901563,
        new ProjectileParameters(0.8499753459125352, 12.795045470214399, 1.6980232676760407));
    kHubShotMap.addSample(
        12.401563,
        new ProjectileParameters(0.8499753457872351, 13.104314312383456, 1.7396397147741987));
    kHubShotMap.addSample(
        12.901563,
        new ProjectileParameters(0.8499753458720953, 13.412677667355927, 1.7805219041030973));
    kHubShotMap.addSample(
        13.401563,
        new ProjectileParameters(0.849975345903444, 13.72031914427781, 1.8207172005941197));
    kHubShotMap.addSample(
        13.901563,
        new ProjectileParameters(0.8499753459093182, 14.027412170074014, 1.8602679825989226));
    kHubShotMap.addSample(
        14.401563,
        new ProjectileParameters(0.8499753459093167, 14.334120763901865, 1.8992123152737268));
    kHubShotMap.addSample(
        14.500000,
        new ProjectileParameters(0.8499753459093156, 14.394472271920085, 1.9068110504103652));

    // Ground Shot Map (Least Velocity Points)
    kGroundShotMap.addSample(
        0.500000,
        new ProjectileParameters(0.8499753465206894, 1.5431060617965031, 0.5077188428227241));
    kGroundShotMap.addSample(
        1.000000,
        new ProjectileParameters(0.8499753469055713, 2.550568223243365, 0.6180456526577633));
    kGroundShotMap.addSample(
        1.500000,
        new ProjectileParameters(0.8499753469050491, 3.3414662376828783, 0.7121757452591001));
    kGroundShotMap.addSample(
        2.000000,
        new ProjectileParameters(0.8499753469048806, 4.012893895011929, 0.7958652549912226));
    kGroundShotMap.addSample(
        2.500000,
        new ProjectileParameters(0.8499753469047125, 4.607815086640692, 0.8721195016419622));
    kGroundShotMap.addSample(
        3.000000,
        new ProjectileParameters(0.8499753469040299, 5.149231330893658, 0.9427409474038516));
    kGroundShotMap.addSample(
        3.500000,
        new ProjectileParameters(0.849975347013053, 5.650998109495918, 1.0089105410735295));
    kGroundShotMap.addSample(
        4.000000,
        new ProjectileParameters(0.8499753471099555, 6.12216280327684, 1.0714512611501854));
    kGroundShotMap.addSample(
        4.500000,
        new ProjectileParameters(0.8499753471969359, 6.568991049542148, 1.130963405625446));
    kGroundShotMap.addSample(
        5.000000,
        new ProjectileParameters(0.8499753472755998, 6.996020995677199, 1.1879005235859543));
    kGroundShotMap.addSample(
        5.500000,
        new ProjectileParameters(0.8499753474549505, 7.406657841607501, 1.2426150084068859));
    kGroundShotMap.addSample(
        6.000000,
        new ProjectileParameters(0.8499753476188622, 7.803530840199975, 1.2953869642311862));
    kGroundShotMap.addSample(
        6.500000,
        new ProjectileParameters(0.8499753477692976, 8.188718721793068, 1.3464432827344424));
    kGroundShotMap.addSample(
        7.000000,
        new ProjectileParameters(0.8499753479078301, 8.563898030027698, 1.395970703056141));
    kGroundShotMap.addSample(
        7.500000,
        new ProjectileParameters(0.8499753480357699, 8.930444154232838, 1.4441250224830675));
    kGroundShotMap.addSample(
        8.000000,
        new ProjectileParameters(0.8499753481542495, 9.28950218416463, 1.4910377589064001));
    kGroundShotMap.addSample(
        8.500000,
        new ProjectileParameters(0.8499753482642243, 9.642037864380809, 1.5368210764966503));
    kGroundShotMap.addSample(
        9.000000,
        new ProjectileParameters(0.8499753483665181, 9.988875045180539, 1.5815714973888015));
    kGroundShotMap.addSample(
        9.500000,
        new ProjectileParameters(0.8499753484618475, 10.330723739745576, 1.625372745858069));
    kGroundShotMap.addSample(
        10.000000,
        new ProjectileParameters(0.8499753485508268, 10.668201501719308, 1.6682979603823618));
    kGroundShotMap.addSample(
        10.500000,
        new ProjectileParameters(0.8499753486340669, 11.001849960130242, 1.710411437064928));
    kGroundShotMap.addSample(
        11.000000,
        new ProjectileParameters(0.8499753487119527, 11.332147782064455, 1.7517700201834598));
    kGroundShotMap.addSample(
        11.500000,
        new ProjectileParameters(0.849975348784964, 11.659520958911534, 1.792424223299989));
    kGroundShotMap.addSample(
        12.000000,
        new ProjectileParameters(0.8499753488534867, 11.98435105899255, 1.8324191420184572));
    kGroundShotMap.addSample(
        12.500000,
        new ProjectileParameters(0.8499753489178874, 12.30698191517547, 1.8717952037659336));
    kGroundShotMap.addSample(
        13.000000,
        new ProjectileParameters(0.8499753489784544, 12.627725094025676, 1.910588788743099));
    kGroundShotMap.addSample(
        13.500000,
        new ProjectileParameters(0.8499753490354558, 12.946864406160898, 1.9488327480500878));
    kGroundShotMap.addSample(
        14.000000,
        new ProjectileParameters(0.8499753490891647, 13.264659654732426, 1.986556839011992));
    kGroundShotMap.addSample(
        14.500000,
        new ProjectileParameters(0.849975349139814, 13.581349773032018, 2.0237880932799643));
    kGroundShotMap.addSample(
        15.000000,
        new ProjectileParameters(0.8499753491876149, 13.897155468199202, 2.0605511299379344));
    kGroundShotMap.addSample(
        15.500000,
        new ProjectileParameters(0.8499753492327834, 14.21228146250609, 2.0968684233010007));
    kGroundShotMap.addSample(
        15.875000,
        new ProjectileParameters(0.8499753492760546, 14.448295073078791, 2.1238263085077476));
  }

  // * ~~~~~~~~ TUNABLES ~~~~~~~~

  private static final TunableNumber kLatencyCompensationSec =
      new TunableNumber("Fire Control System/Latency Compensation (seconds)", 0.15);

  // * ~~~~~~~~ MEMBERS ~~~~~~~~

  private static Translation2d mVirtualTarget = Translation2d.kZero;

  public static Translation2d getVirtualTarget() {
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
    mVirtualTarget =
        calculateVirtualTarget(predictedPose.getTranslation(), target, tree, speedsWrtField);

    // Calculate final parameters
    var parameters = tree.get(predictedPose.getTranslation().getDistance(mVirtualTarget));

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
