#ifndef TV_CONTACT
#define TV_CONTACT


#include <maya/MPxDeformerNode.h>

#include <maya/MItGeometry.h>

#include <maya/MGlobal.h>
#include <maya/MStatus.h>
#include <maya/MIntArray.h>
#include <maya/MMatrix.h>
#include <maya/MPointArray.h>
#include <maya/MMeshIntersector.h>

#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMesh.h>

#include <maya/MIOStream.h>



class TvContact : public MPxDeformerNode
{
public:
	TvContact() = default;
	~TvContact() override = default;
	static void* creator();

	static MStatus initialize();
    MStatus deform(MDataBlock& data,
                   MItGeometry& itGeo,
                   const MMatrix& localToWorldMatrix,
                   unsigned int geoIndex) override;

	const static MTypeId id;
	
	static MObject aCollider;
	static MObject aColliderMesh;
	static MObject aColliderMatrix;
	static MObject aPreserveVolumeWeight;
};

#endif
