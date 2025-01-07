package frc.robot;

import static org.ironmaple.utils.FieldMirroringUtils.FIELD_HEIGHT;
import static org.ironmaple.utils.FieldMirroringUtils.toCurrentAllianceTranslation;

import org.dyn4j.geometry.Convex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class sigmasigmsboyArena extends SimulatedArena {
    /** the obstacles on the 2024 competition field */
    public static final class sigmasigmsboyArenaObstacleMap extends FieldMap {
        private static final double FIELD_WIDTH = 17.54;

        public sigmasigmsboyArenaObstacleMap() {
            super();
            // left wall
            // super.addBorderLine(new Translation2d(0, 1), new Translation2d(0, 4.51));
            // super.addBorderLine(new Translation2d(0, 4.51), new Translation2d(0.9, 5));

            // super.addBorderLine(new Translation2d(0.9, 5), new Translation2d(0.9, 6.05));

            // super.addBorderLine(new Translation2d(0.9, 6.05), new Translation2d(0, 6.5));
            super.addBorderLine(new Translation2d(0, 1.27), new Translation2d(0, 7.2));
            
            super.addBorderLine(new Translation2d(0, 1.27), new Translation2d(1.75, 0));

            super.addBorderLine(new Translation2d(0, 6.93), new Translation2d(1.75, 8.2));

            super.addBorderLine(new Translation2d(1.75, 8.2), new Translation2d(FIELD_WIDTH-1.75, 8.2));

            super.addBorderLine(new Translation2d(FIELD_WIDTH-1.75, 8.2), new Translation2d(FIELD_WIDTH, 6.93));

            super.addBorderLine(new Translation2d(FIELD_WIDTH, 6.93), new Translation2d(FIELD_WIDTH, 1.27));

            super.addBorderLine(new Translation2d(FIELD_WIDTH, 1.27), new Translation2d(FIELD_WIDTH-1.75, 0));

            super.addBorderLine(new Translation2d(FIELD_WIDTH-1.75, 0), new Translation2d(1.75, 0));


            super.addRectangularObstacle(Units.inchesToMeters(12), Units.inchesToMeters(12), new Pose2d(FIELD_WIDTH/2, FIELD_HEIGHT/2, new Rotation2d()));
           
            super.addBorderLine(new Translation2d(4+0.5, 3), new Translation2d(3.7, 3.5));
            super.addBorderLine(new Translation2d(3.7, 3.5), new Translation2d(3.7, 4.5));
            super.addBorderLine(new Translation2d(3.7, 4.5), new Translation2d(4.5, 5));
            super.addBorderLine(new Translation2d(4+0.5, 5), new Translation2d(5.3, 4.5));
            super.addBorderLine(new Translation2d(5.3, 4.5), new Translation2d(5.3, 3.5));
            super.addBorderLine(new Translation2d(5.3, 3.5), new Translation2d(4.5, 3));

           
            // // upper wall
            // super.addBorderLine(new Translation2d(0, 8.12), new Translation2d(FIELD_WIDTH, 8.12));

            // // righter wall
            // super.addBorderLine(new Translation2d(FIELD_WIDTH, 1), new Translation2d(FIELD_WIDTH, 4.51));
            // super.addBorderLine(new Translation2d(FIELD_WIDTH, 4.51), new Translation2d(FIELD_WIDTH - 0.9, 5));
            // super.addBorderLine(new Translation2d(FIELD_WIDTH - 0.9, 5), new Translation2d(FIELD_WIDTH - 0.9, 6.05));
            // super.addBorderLine(new Translation2d(FIELD_WIDTH - 0.9, 6.05), new Translation2d(FIELD_WIDTH, 6.5));
            // super.addBorderLine(new Translation2d(FIELD_WIDTH, 6.5), new Translation2d(FIELD_WIDTH, 8.2));

            // // lower wall
            // super.addBorderLine(new Translation2d(1.92, 0), new Translation2d(FIELD_WIDTH - 1.92, 0));

            // // red source wall
            // super.addBorderLine(new Translation2d(1.92, 0), new Translation2d(0, 1));

            // // blue source wall
            // super.addBorderLine(new Translation2d(FIELD_WIDTH - 1.92, 0), new Translation2d(FIELD_WIDTH, 1));

            // // blue state
            // super.addRectangularObstacle(0.35, 0.35, new Pose2d(3.4, 4.1, new Rotation2d()));
            // super.addRectangularObstacle(0.35, 0.35, new Pose2d(5.62, 4.1 - 1.28, Rotation2d.fromDegrees(30)));
            // super.addRectangularObstacle(0.35, 0.35, new Pose2d(5.62, 4.1 + 1.28, Rotation2d.fromDegrees(60)));

            // // red stage
            // super.addRectangularObstacle(0.35, 0.35, new Pose2d(FIELD_WIDTH - 3.4, 4.1, new Rotation2d()));
            // super.addRectangularObstacle(
            //         0.35, 0.35, new Pose2d(FIELD_WIDTH - 5.62, 4.1 - 1.28, Rotation2d.fromDegrees(60)));
            // super.addRectangularObstacle(
            //         0.35, 0.35, new Pose2d(FIELD_WIDTH - 5.62, 4.1 + 1.28, Rotation2d.fromDegrees(30)));
        }
    }

    private static final Translation2d[] NOTE_INITIAL_POSITIONS = new Translation2d[] {
        new Translation2d(2.9, 4.1),
        new Translation2d(2.9, 5.55),
        new Translation2d(2.9, 7),
        new Translation2d(8.27, 0.75),
        new Translation2d(8.27, 2.43),
        new Translation2d(8.27, 4.1),
        new Translation2d(8.27, 5.78),
        new Translation2d(8.27, 7.46),
        new Translation2d(13.64, 4.1),
        new Translation2d(13.64, 5.55),
        new Translation2d(13.64, 7),
    };

    public sigmasigmsboyArena() {
        super(new sigmasigmsboyArenaObstacleMap());
    }

    @Override
    public void placeGamePiecesOnField() {
        for (Translation2d notePosition : NOTE_INITIAL_POSITIONS)
            super.addGamePiece(new GamePiece(notePosition));
    }

    private static final Translation2d BLUE_SOURCE_POSITION = new Translation2d(15.6, 0.8);
    private double previousThrowTimeSeconds = 0;

    @Override
    public void competitionPeriodic() {
        if (!DriverStation.isTeleopEnabled()) return;

        if (Timer.getFPGATimestamp() - previousThrowTimeSeconds < 1) return;

        final Translation2d sourcePosition = toCurrentAllianceTranslation(BLUE_SOURCE_POSITION);
        /* if there is any game-piece 0.5 meters within the human player station, we don't throw a new note */
        for (GamePieceOnFieldSimulation gamePiece : super.gamePieces)
            if (gamePiece instanceof GamePiece
                    && gamePiece.getPoseOnField().getTranslation().getDistance(sourcePosition) < 1) return;

        /* otherwise, place a note */
        addGamePiece(new GamePiece(sourcePosition));
        previousThrowTimeSeconds = Timer.getFPGATimestamp();
    }
}
