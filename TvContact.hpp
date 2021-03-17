//
// Created by root on 3/12/21.
//

#ifndef MAYADEV_TVCONTACT_HPP
#define MAYADEV_TVCONTACT_HPP

#include "Smooth.hpp"
#include "VolumePreservation.hpp"

#include <maya/MPxDeformerNode.h>
#include <maya/MMeshIntersector.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MItGeometry.h>
#include <maya/MPoint.h>
#include <maya/MFloatPointArray.h>
#include <maya/MGlobal.h>
#include <maya/MMatrix.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MItMeshVertex.h>

#include <vector>


class TvContact : public MPxDeformerNode
{
public:
    static  void*   creator();
    static  MStatus initialize();

    // Deformation function
    //
    MStatus deform(MDataBlock&    block,
                   MItGeometry&   iter,
                   const MMatrix& mat,
                   unsigned int multiIndex) override;
    MStatus compute(const MPlug& plug, MDataBlock &block) override;
    MStatus initializeCache(const MObject& refMesh);
    bool collides(const MPoint &point,
                  MMeshIntersector &intersector,
                  MVector &toColliderPoint,
                  MStatus &status);

    static const MTypeId id;

    static MObject aCollider;
    static MObject aColliderMesh;
    static MObject aColliderMatrix;

    static MObject aSmoothIterations;
    static MObject aSmoothStrength;

    static MObject aPreserveVolumeWeight;

    static MObject aCachedGeometry;



    MObject cachedMeshData;
};


#endif // MAYADEV_TVCONTACT_HPP
