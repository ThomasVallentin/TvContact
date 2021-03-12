#ifndef VOLUME_PRESERVATION
#define VOLUME_PRESERVATION

#include <maya/MPointArray.h>
#include <maya/MVector.h>
#include <maya/MIntArray.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MFnMesh.h>

double tetrahedralVolume(MPointArray& tetPoints);
double triangleAera(MPointArray& triPoints);
MPoint barycenter(MPointArray& points);
//MIntArray objectIndexToFaceIndexVertices(MIntArray& faceIndexes, MIntArray& triangleIndexes);
double polygonMeshVolume(MObject& inputMesh, const MMatrix& meshWorldMatrix);

#endif