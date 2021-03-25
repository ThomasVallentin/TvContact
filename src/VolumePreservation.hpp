#ifndef VOLUME_PRESERVATION_HPP
#define VOLUME_PRESERVATION_HPP

#include <maya/MPointArray.h>
#include <maya/MVector.h>
#include <maya/MIntArray.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MFnMesh.h>

double tetrahedralVolume(MPointArray& tetPoints);
double triangleAera(MPointArray& triPoints);
double triangleAera(const MPoint &a, const MPoint &b, const MPoint &c);
MPoint barycenter(MPointArray& points);
double polygonMeshVolume(MObject& inputMesh, const MMatrix& meshWorldMatrix);

#endif  // VOLUME_PRESERVATION_HPP