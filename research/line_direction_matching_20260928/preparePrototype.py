"""Create isolated solver copies to evaluate a line-direction residual."""
from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[2];dest=root/'output/line_direction_matching_20260928/prototype'
s=subprocess.check_output(['git','show','30f464725f8154b0f3e990db14ad563fe9bbe849:localization/registerSemanticProbabilityCloud.m'],text=True).replace('registerSemanticProbabilityCloud','experimentalDirectionRegistration').replace('prepareSemanticRegistrationGeometry','experimentalDirectionGeometry')
(dest/'experimentalDirectionRegistration.m').write_text(s)
s=subprocess.check_output(['git','show','30f464725f8154b0f3e990db14ad563fe9bbe849:localization/prepareSemanticRegistrationGeometry.m'],text=True).replace('prepareSemanticRegistrationGeometry','experimentalDirectionGeometry')
s=s.replace('    m.quality=quality(m);','''    m.quality=quality(m);
    [m.lineTangent,m.lineDirectionValid]=sourceLineDirections(m,cfg.lineDirection);
    m.lineDirectionSigma=cfg.lineDirection.standardDeviation;
''')
s=s.replace('    weights=m.quality(source);','''    tangent=m.lineTangent(source,:);normals=f.normal(target,:);
    directionValid=line & m.lineDirectionValid(source);
    directionScale=double(directionValid)/m.lineDirectionSigma;
    rotatedTangent=tangent*r.';
    directionResidual=sum(rotatedTangent.*normals,2).*directionScale;
    residual(3,:)=directionResidual.';
    jacobian(3,3,:)=sum((tangent*[r(:,2),-r(:,1)].').*normals,2).*directionScale*scale(3);
    weights=m.quality(source);''')
s=s.replace('rowWeights=repelem(weights.*robust,2,1);','rowWeights=repelem(weights.*robust,3,1);')
s=s.replace("'precision',precision(:,:,1:rows),'targetMean',f.mean(target,1:2),", "'precision',precision(:,:,1:rows),'targetMean',f.mean(target,1:2), ...\n        'directionNormal',normals,'directionScale',directionScale,")
s=s.replace('    q=rx.^2+ry.^2;','    direction=sum((m.lineTangent(system.pairs.source,:)*r.\').*system.directionNormal,2).*system.directionScale;\n    q=rx.^2+ry.^2+direction.^2;')
(dest/'experimentalDirectionGeometry.m').write_text(s)

# A second candidate also uses line direction in target assignment.
s=(dest/'experimentalDirectionRegistration.m').read_text().replace('experimentalDirectionRegistration','directionAssociationRegistration').replace('experimentalDirectionGeometry','directionAssociationGeometry')
(dest/'directionAssociationRegistration.m').write_text(s)
s=(dest/'experimentalDirectionGeometry.m').read_text().replace('experimentalDirectionGeometry','directionAssociationGeometry')
needle='                distance=dn.^2./variance+dt.^2./(f.majorVariance(targets)+cfg.maximumMatchDistance^2);'
s=s.replace(needle,needle+"\n                tangent=m.lineTangent(source,:)*r.';\n                angular=(normal*tangent.').^2.*double(m.lineDirectionValid(source)).'/m.lineDirectionSigma^2;\n                distance=distance+angular;")
(dest/'directionAssociationGeometry.m').write_text(s)
