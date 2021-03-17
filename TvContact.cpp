//
// Created by root on 3/12/21.
//

#include "TvContact.hpp"

const MTypeId TvContact::id(0x00003684);

MObject TvContact::aCollider;
MObject TvContact::aColliderMesh;
MObject TvContact::aColliderMatrix;
MObject TvContact::aPreserveVolumeWeight;
MObject TvContact::aSmoothIterations;
MObject TvContact::aSmoothStrength;
MObject TvContact::aCachedGeometry;


void* TvContact::creator()
{
    return new TvContact;
}

MStatus TvContact::initialize()
{
    MStatus status;

    MFnNumericAttribute nAttr;
    MFnTypedAttribute tAttr;
    MFnMatrixAttribute mAttr;
    MFnCompoundAttribute cpAttr;

    aColliderMesh = tAttr.create("colliderMesh", "colliderMesh", MFnData::kMesh);
    addAttribute(aColliderMesh);
    attributeAffects(aColliderMesh, outputGeom);

    aColliderMatrix = mAttr.create("colliderMatrix", "colliderMatrix");
    addAttribute(aColliderMatrix);
    attributeAffects(aColliderMatrix, outputGeom);

    aCollider = cpAttr.create("collider", "collider");
    cpAttr.addChild(aColliderMesh);
    cpAttr.addChild(aColliderMatrix);
    attributeAffects(aCollider, outputGeom);

    addAttribute(aCollider);

    aSmoothIterations = nAttr.create("smoothIterations", "smoothIterations", MFnNumericData::kInt);
    nAttr.setMin(0);
    nAttr.setDefault(2);
    addAttribute(aSmoothIterations);
    attributeAffects(aSmoothIterations, outputGeom);

    aSmoothStrength = nAttr.create("smoothStrength", "smoothStrength", MFnNumericData::kFloat);
    nAttr.setMin(0);
    nAttr.setMax(1);
    nAttr.setDefault(0.5);
    addAttribute(aSmoothStrength);
    attributeAffects(aSmoothStrength, outputGeom);

    aPreserveVolumeWeight = nAttr.create("volumePreserve", "volumePreserve", MFnNumericData::kFloat);
    addAttribute(aPreserveVolumeWeight);
    attributeAffects(aPreserveVolumeWeight, outputGeom);

    aCachedGeometry = tAttr.create("cachedMeshData", "cachedMeshData", MFnData::kMesh);
    addAttribute(aCachedGeometry);
    attributeAffects(outputGeom, aCachedGeometry);
    attributeAffects(aColliderMesh, aCachedGeometry);
    attributeAffects(aColliderMatrix, aCachedGeometry);
    attributeAffects(aCollider, aCachedGeometry);
    attributeAffects(aSmoothIterations, aCachedGeometry);
    attributeAffects(aSmoothStrength, aCachedGeometry);
    attributeAffects(input, aCachedGeometry);
    attributeAffects(inputGeom, aCachedGeometry);


    return MS::kSuccess;
}

MStatus TvContact::deform(MDataBlock& block, MItGeometry& itGeo,
                          const MMatrix& localToWorldMatrix, unsigned int geoIndex) {
    MStatus status;

    // == Getting all attributes values from the block ===================================

    // Compute inverse world matrix to set the collider in input local space
    MMatrix worldToLocalMatrix = localToWorldMatrix.inverse();

    // Get collider transformed mesh
    MDataHandle hCollider = block.inputValue(aColliderMesh, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MObject oColliderMesh = hCollider.asMeshTransformed();

    if (oColliderMesh.isNull()) {
        std::cout << "Collider is empty" << std::endl;
        return MS::kSuccess;
    }

    // Getting collider world matrix to set the normals in world space
    MDataHandle hColliderMatrix = block.inputValue(aColliderMatrix, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MMatrix colliderLocalToWorldMatrix = hColliderMatrix.asMatrix();

    // Get numeric attributes values
    float env = block.inputValue(envelope).asFloat();
    float preserveVolume = block.inputValue(aPreserveVolumeWeight).asFloat();
    int smoothIterations = block.inputValue(aSmoothIterations).asInt();
    float smoothStrength = block.inputValue(aSmoothStrength).asFloat();

    // Getting input mesh by going through input -> input[i] -> inputGeom
    // We get the output value since the input has already been computed during the
    // compute method
    MArrayDataHandle hInput = block.outputArrayValue(input, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = hInput.jumpToElement(geoIndex);
    MDataHandle hInputElement = hInput.outputValue(&status);
    MObject oInputMesh = hInputElement.child(inputGeom).asMesh();

    // == Initializing all data structures ===============================================

    MFnMesh fnCollider(oColliderMesh);

    double preVolume = polygonMeshVolume(oInputMesh, localToWorldMatrix);

    // Ensure cachedMeshData contains a mesh
    if (cachedMeshData.isNull())
        initializeCache(oInputMesh);

    MFnMesh fnCachedMesh(cachedMeshData);

    // TODO: Getting all the points from the iterator at once
    //       because it's faster than iterating testedPoint per testedPoint

    // Intersector initialisation
    MMeshIntersector intersector;
    intersector.create(oColliderMesh);
    MPointOnMesh meshPoint;

//    std::vector<int> collidedIds;
    MPoint cachedPoint, origPoint;
    MVector cachedPointToOrig;
    
    // Getting original points and set it as the base for the output points
    MPointArray origPoints, cachedPoints, outPositions;
    MVectorArray deltas;
    itGeo.allPositions(outPositions);

    unsigned int pointCount = outPositions.length();
    deltas.setLength(pointCount);
    origPoints.setLength(pointCount);

    // Copying all positions to origPoints and transform them in world space
    for ( size_t i=0; i < pointCount ; i++ )
        origPoints[i] = outPositions[i] * localToWorldMatrix;

    // Get all the points from the cache in world space
    fnCachedMesh.getPoints(cachedPoints, MSpace::kWorld);
    for (auto &pt : cachedPoints)
        pt *= localToWorldMatrix;

    // == COLLISION DETECTION ============================================================

    for ( size_t i=0 ; i < pointCount ; i++ ) {
        cachedPoint = cachedPoints[i];
        origPoint = origPoints[i];
        MVector delta;
        if (collides(cachedPoint, intersector, delta, status)) {
            delta += cachedPoint - origPoint;
        }
        else {
            cachedPointToOrig = origPoint - cachedPoint;

            MFloatPoint hitPoint;
            if (fnCollider.closestIntersection(cachedPoint,
                                               cachedPointToOrig,
                                               nullptr,
                                               nullptr,
                                               false,
                                               MSpace::kWorld,
                                               1.0,
                                               false,
                                               nullptr,
                                               hitPoint,
                                               nullptr,
                                               nullptr,
                                               nullptr,
                                               nullptr,
                                               nullptr)) {
                delta = (MPoint(hitPoint) - origPoint);
            }
            else {
                delta = MVector(0, 0, 0);
            }
        }
        deltas[i] = delta * worldToLocalMatrix;
    }

    // == SMOOTH =========================================================================
    std::cout << deltas[pointCount-1] << std::endl;
    laplacianSmooth(deltas, oInputMesh, smoothStrength, smoothIterations);
    for ( size_t i=0 ; i < pointCount ; i++ ) {
        outPositions[i] += deltas[i] * env;
    }
    std::cout << deltas[pointCount-1] << std::endl;
    itGeo.setAllPositions(outPositions);

    // == VOLUME PRESERVATION ============================================================

//    double postVolume = polygonMeshVolume(oInputMesh, localToWorldMatrix);
//    double volumeDelta = std::max(0.0, preVolume - postVolume);
//
//    // Not enough volume loss, returning before preserving phase
//    if (volumeDelta < 1e-8 || preserveVolume < 1e-8)
//        return MS::kSuccess;
//
//    itGeo.reset();
//    double volumePerVertice = volumeDelta / (double)(itGeo.count() - collidedIds.size());
//    for ( ; !itGeo.isDone(); itGeo.next()) {
//        // If the testedPoint has not collided, skip the volume preservation
//        bool hasCollided = false;
//        for (const auto & colId : collidedIds)
//            if (itGeo.index() == colId) {
//                hasCollided = true;
//                break;
//            }
//        if (hasCollided)
//            continue;
//
//        itGeo.setPosition(itGeo.position() + itGeo.normal() * volumePerVertice * preserveVolume * env);
//    }
    // Get final position and sets it to the cache
    fnCachedMesh.copy(oInputMesh, cachedMeshData);

    return MS::kSuccess;
}

MStatus TvContact::compute(const MPlug &plug, MDataBlock &block) {
    if (plug == aCachedGeometry) {
        MDataHandle hCachedGeometry = block.outputValue(aCachedGeometry);
        hCachedGeometry.set(cachedMeshData);
        block.setClean(plug);

        return MS::kSuccess;
    }
    else {
        return MPxNode::compute(plug, block);
    }
}

MStatus TvContact::initializeCache(const MObject &refMesh) {
    MStatus status;

    MFnMeshData cachedGeometryMeshData;
    cachedMeshData = cachedGeometryMeshData.create(&status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MFnMesh fnCachedGeometry(cachedMeshData);
    fnCachedGeometry.copy(refMesh, cachedMeshData);

    return status;
}

bool TvContact::collides(const MPoint &point,
                         MMeshIntersector &intersector,
                         MVector &toColliderPoint,
                         MStatus &status) {
    MPointOnMesh pointOnMesh;
    // Get closest testedPoint on collider
    status = intersector.getClosestPoint(point, pointOnMesh);
    CHECK_MSTATUS_AND_RETURN_IT(status)

    // Computing if the testedPoint is inside the collider
    toColliderPoint  = MPoint(pointOnMesh.getPoint()) - point;

    // Getting normal in world space
    MVector closestNormal = pointOnMesh.getNormal();
    return (closestNormal * toColliderPoint) > 0;
}
