package org.firstinspires.ftc.teamcode.Util.Vision;

import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.teamcode.Util.ArtifactType;
import java.util.ArrayList;

public class VisionUtil {
    public static void filterByArea(double minArea, double maxArea, ArrayList<VisionArtifact> artifacts) {

        ArrayList<VisionArtifact> toRemove = new ArrayList<>();

        for(VisionArtifact a : artifacts) {

            if (a.getContourArea() > maxArea || a.getContourArea() < minArea) {
                toRemove.add(a);
            }
        }

        artifacts.removeAll(toRemove);
    }

    public static void sortByArea(SortOrder sortOrder, ArrayList<VisionArtifact> artifacts) {

        artifacts.sort((c1, c2) -> {

            int tmp = (int)Math.signum(c2.getContourArea() - c1.getContourArea());

            if (sortOrder == SortOrder.ASCENDING) {
                tmp = -tmp;
            }

            return tmp;
        });
    }

    public static void filterByDensity(double minDensity, double maxDensity, ArrayList<VisionArtifact> artifacts) {

        ArrayList<VisionArtifact> toRemove = new ArrayList<>();

        for(VisionArtifact a : artifacts) {

            if (a.getDensity() > maxDensity || a.getDensity() < minDensity) {
                toRemove.add(a);
            }
        }

        artifacts.removeAll(toRemove);
    }

    public static void sortByDensity(SortOrder sortOrder, ArrayList<VisionArtifact> artifacts) {

        artifacts.sort((c1, c2) -> {
            int tmp = (int)Math.signum(c2.getDensity() - c1.getDensity());

            if (sortOrder == SortOrder.ASCENDING) {
                tmp = -tmp;
            }

            return tmp;
        });
    }

    public static void filterByAspectRatio(double minAspectRatio, double maxAspectRatio, ArrayList<VisionArtifact> artifacts) {

        ArrayList<VisionArtifact> toRemove = new ArrayList<>();

        for(VisionArtifact a : artifacts) {

            if (a.getAspectRatio() > maxAspectRatio || a.getAspectRatio() < minAspectRatio) {
                toRemove.add(a);
            }
        }

        artifacts.removeAll(toRemove);
    }

    public static void sortByAspectRatio(SortOrder sortOrder, ArrayList<VisionArtifact> artifacts) {

        artifacts.sort((c1, c2) -> {

            int tmp = (int)Math.signum(c2.getAspectRatio() - c1.getAspectRatio());

            if (sortOrder == SortOrder.ASCENDING) {
                tmp = -tmp;
            }

            return tmp;
        });
    }

    public static void filterByArtifactType(ArtifactType type, ArrayList<VisionArtifact> artifacts) {

        if (type == ArtifactType.EMPTY) {
            throw new IllegalArgumentException("ArtifactType: EMPTY is not allowed here! Please filter by either GREEN or PURPLE!");
        }

        ArrayList<VisionArtifact> toRemove = new ArrayList<>();

        for (VisionArtifact a : artifacts) {

            if (a.getArtifactType() != type) {
                toRemove.add(a);
            }
        }

        artifacts.removeAll(toRemove);
    }

    public static void sortByArtifactType(SortOrder sortOrder, ArrayList<VisionArtifact> artifacts) {

        artifacts.sort((c1, c2) -> {

            int tmp = (int)Math.signum(c2.getArtifactType().ordinal() - c1.getArtifactType().ordinal());

            if (sortOrder == SortOrder.ASCENDING) {
                tmp = -tmp;
            }

            return tmp;
        });
    }
}
