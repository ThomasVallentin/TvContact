//
// Created by root on 3/12/21.
//

#include "TvContact.hpp"


const MTypeId TvContact::id(0x00003684);

MObject TvContact::aCollider;
MObject TvContact::aColliderMesh;
MObject TvContact::aColliderMatrix;
MObject TvContact::aPreserveVolumeWeight;
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

    aCachedGeometry = tAttr.create("cachedGeometry", "cachedGeometry", MFnData::kMesh);
    addAttribute(aCachedGeometry);
    attributeAffects(outputGeom, aCachedGeometry);
    attributeAffects(aColliderMesh, aCachedGeometry);
    attributeAffects(aColliderMatrix, aCachedGeometry);
    attributeAffects(aCollider, aCachedGeometry);
    attributeAffects(input, aCachedGeometry);
    attributeAffects(inputGeom, aCachedGeometry);

    aPreserveVolumeWeight = nAttr.create("volumePreserve", "volumePreserve", MFnNumericData::kFloat);
    addAttribute(aPreserveVolumeWeight);
    attributeAffects(aPreserveVolumeWeight, outputGeom);

//    MGlobal::executeCommand("makePaintable -attrType multiFloat -shapeMode deformer tvContact weights");

    return MS::kSuccess;
}

MStatus TvContact::deform(MDataBlock& data, MItGeometry& itGeo, const MMatrix& localToWorldMatrix, unsigned int geoIndex) {
    MStatus status;

    MMatrix worldToLocalMatrix = localToWorldMatrix.inverse();

    MDataHandle hCollider = data.inputValue(aColliderMesh, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MObject oColliderMesh = hCollider.asMeshTransformed();

    if (oColliderMesh.isNull()) {
        std::cout << "Collider is empty" << std::endl;
        return MS::kSuccess;
    }

    MDataHandle hColliderMatrix = data.inputValue(aColliderMatrix, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MMatrix colliderLocalToWorldMatrix = hColliderMatrix.asMatrix();

    float env = data.inputValue(envelope).asFloat();
    float preserveVolume = data.inputValue(aPreserveVolumeWeight).asFloat();

    // Getting input mesh by going through input -> input[i] -> inputGeom
    // We get the output value since the input has already been computed during the
    // compute method
    MArrayDataHandle hInput = data.outputArrayValue(input, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = hInput.jumpToElement(geoIndex);
    MDataHandle hInputElement = hInput.outputValue(&status);
    MObject oInputMesh = hInputElement.child(inputGeom).asMesh();

    MFnMesh fnCollider(oColliderMesh);

    // If there is no cached geometry, copy the original one
    if (cachedGeometry.isNull()) {
        std::cout << "cache is null" << std::endl;

        MFnMeshData cachedGeometryMeshData;
        cachedGeometry = cachedGeometryMeshData.create(&status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }
    MFnMesh fnCachedGeometry(cachedGeometry);
    fnCachedGeometry.copy(oInputMesh, cachedGeometry);

    MFnMesh fnInputMesh(oInputMesh);
    MPoint printPoint;
    MVector printNormal;

    MPoint printPoint2;
    MVector printNormal2;

    double preVolume = polygonMeshVolume(oInputMesh, localToWorldMatrix);

    double dot;

    // TODO: Getting all the points from the iterator at once
    //       because it's faster than iterating point per point
    MPoint vertex, closestPoint;
    MVector toColliderPoint, closestNormal;

    // Intersector initialisation
    MMeshIntersector intersector;
    intersector.create(oColliderMesh);
    MPointOnMesh meshPoint;

    std::vector<int> collidedIds;

    MPoint point;
    int i = 0;
    for (; !itGeo.isDone(); itGeo.next()) {
        point = itGeo.position();
        point *= localToWorldMatrix;

        // Get closest point on collider
        status = intersector.getClosestPoint(point, meshPoint);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        // Computing if the point is inside the collider
        closestPoint = meshPoint.getPoint();
        toColliderPoint = MVector(closestPoint - point);

        // Getting normal in world space
        closestNormal = meshPoint.getNormal();
        closestNormal *= colliderLocalToWorldMatrix;
        dot = closestNormal * toColliderPoint;

        if (dot > 0) {
            collidedIds.push_back(itGeo.index());
            point += toColliderPoint * env;
            point *= worldToLocalMatrix;
            itGeo.setPosition(point);
        }
        i++;
    }

    // == VOLUME PRESERVATION ============================================================

    double postVolume = polygonMeshVolume(oInputMesh, localToWorldMatrix);
    double volumeDelta = std::max(0.0, preVolume - postVolume);
    std::cout << "volumeDelta : " << volumeDelta << std::endl;

//    // Not enough volume loss, returning before preserving phase
//    if (volumeDelta < 1e-8 || preserveVolume < 1e-8)
//        return MS::kSuccess;
//
//    itGeo.reset();
//    double volumePerVertice = volumeDelta / (double)(itGeo.count() - collidedIds.size());
//    for ( ; !itGeo.isDone(); itGeo.next()) {
//        // If the point has not collided, skip the volume preservation
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
    MPointArray pntsToCache;
    itGeo.reset();
    itGeo.allPositions(pntsToCache);
    fnCachedGeometry.setPoints(pntsToCache);

    fnCachedGeometry.setObject(cachedGeometry);
    return MS::kSuccess;
}

MStatus TvContact::compute(const MPlug &plug, MDataBlock &block) {
    std::cout << "COMPUTE " << plug.name() << std::endl;
    if (plug == aCachedGeometry) {
        std::cout << "aCachedGeometry" << std::endl;

        MDataHandle hCachedGeometry = block.outputValue(aCachedGeometry);
        hCachedGeometry.set(cachedGeometry);
        block.setClean(plug);

        return MS::kSuccess;
    }
    else {
        return MPxNode::compute(plug, block);
    }
}
