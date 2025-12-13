package org.firstinspires.ftc.teamcode.Util;

public class ArtifactPattern {

    ArtifactType artifact1;
    ArtifactType artifact2;
    ArtifactType artifact3;

    public ArtifactPattern(ArtifactType a1, ArtifactType a2, ArtifactType a3) {
        this.artifact1 = a1;
        this.artifact2 = a2;
        this.artifact3 = a3;
    }

    public ArtifactType getArtifact1() {
        return artifact1;
    }
    public ArtifactType getArtifact2() {
        return artifact2;
    }
    public ArtifactType getArtifact3() {
        return artifact3;
    }
}
