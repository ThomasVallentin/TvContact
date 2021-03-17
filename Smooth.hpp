#ifndef MAYADEV_SMOOTH_HPP
#define MAYADEV_SMOOTH_HPP

#include <maya/MItMeshVertex.h>
#include <maya/MFnMesh.h>

#include <maya/MIntArray.h>
#include <maya/MPointArray.h>
#include <maya/MObject.h>


inline MStatus laplacianSmooth(MPointArray &positions,
                               MObject &inputMeshTopology,
                               const float &strength=0.2f,
                               const unsigned int &iterations=1) {
    MStatus status;
    MItMeshVertex itVertices(inputMeshTopology, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status)

    MIntArray connectedVertices;
    for ( size_t i=0 ; i < iterations ; i++ ) {
        MPointArray newPositions;

        for (itVertices.reset(); !itVertices.isDone(); itVertices.next()) {
            itVertices.getConnectedVertices(connectedVertices);

            MVector delta(0, 0, 0);
            for (int &id : connectedVertices) {
                delta += positions[id];
            }
            newPositions.append(positions[itVertices.index()] + delta / connectedVertices.length() * strength);
        }
        // Copy from one array to the other to avoid querying the mesh,
        // since we already know the value of each points
        positions.copy(newPositions);
    }

    return MS::kSuccess;
}


inline MStatus laplacianSmooth(MVectorArray &vectors,
                               MObject &inputMeshTopology,
                               const float &strength=0.2f,
                               const unsigned int &iterations=1) {
    MStatus status;
    MItMeshVertex itVertices(inputMeshTopology, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status)

    MVector vector;
    MIntArray connectedVertices;
    MVectorArray newVectors(itVertices.count());
    for ( size_t i=0 ; i < iterations ; i++ ) {
        for ( ; !itVertices.isDone(); itVertices.next() ) {
            int vIndex = itVertices.index();
            itVertices.getConnectedVertices(connectedVertices);
            vector = vectors[vIndex];

            MVector delta(0, 0, 0);
            for (int &id : connectedVertices) {
                delta += vectors[id];
            }
            delta /= connectedVertices.length();
            newVectors[vIndex] = (vector + (delta - vector) * strength);
        }
        // Copy from one array to the other to avoid querying the mesh,
        // since we already know the value of each points
        vectors.copy(newVectors);
        itVertices.reset();
    }

    return MS::kSuccess;
}

#endif // MAYADEV_SMOOTH_HPP
