#include "volumePreservation.h"


double tetrahedralVolume(MPointArray& tetPoints)
{
	// Volume of a tetraherdon ABCD is 1/6 * |det(AB, AC, AD)|
	// Lets call the vectors a,  b,  c. It gives :
	//   1/6 * |det(a, b, c)|
	// =  ''   |xa(yb*zc-yc*zb) + ya(xb*zc - xc*zb) + za(xb*yc - xc*yb)|

	MVector v1, v2, v3;
	v1 = tetPoints[1] - tetPoints[0]; // B - A
	v2 = tetPoints[2] - tetPoints[0]; // C - A
	v3 = tetPoints[3] - tetPoints[0]; // D - A

	double det =  v1.x * (v2.y * v3.z - v3.y * v2.z);
	       det -= v1.y * (v2.x * v3.z - v3.x * v2.z);
		   det += v1.z * (v2.x * v3.y - v3.x * v2.y);
	
	return abs(det) / 6;
}

double triangleAera(MPointArray& triPoints)
{
	// Aera of a triangle ABC is |AB x AC| / 2
	MVector v1(triPoints[1] - triPoints[0]);
	MVector v2(triPoints[2] - triPoints[0]);
	
	MVector cross(v1 ^ v2);

	return cross.length() / 2.0f;
}

MPoint barycenter(MPointArray& points)
{
	MPoint point;
	for (const auto & i : points)
	{
		point += i;
	}

	return point / points.length();
}


//MIntArray objectIndexToFaceIndexVertices(MIntArray& faceIndices, MIntArray& triangleIndices) {
//	/// Return the face-local indices of the triangle from the object-base face and triangle indices;
//	MIntArray localIndexes;
//	unsigned int iFace, iTriangle;
//
//	// for each vertice index of the triangle
//	for (iTriangle = 0; iTriangle < triangleIndices.length(); iTriangle++)
//	{
//		// for each vertice index of the face
//		for (iFace = 0; iFace < faceIndices.length(); iFace++)
//		{
//			// if the index from the face and triangle matches, add the loop index to the return list
//			if (faceIndices[iFace] == triangleIndices[iTriangle])
//			{
//				localIndexes.append(iFace);
//				break;
//			}
//		}
//	}
//	return localIndexes;
//}


double polygonMeshVolume(MObject& inputMesh, const MMatrix& meshWorldMatrix)
{
	MItMeshPolygon itMeshPoly(inputMesh);

	MPointArray trianglePoints;
	MIntArray polygonVertices, triangleVertices;
	MVector normal, toBarycenter;
	double dot;
	double volume(0);
	int triCount;
	// for each mesh polygon
	for ( ; !itMeshPoly.isDone(); itMeshPoly.next()) {
		// get the number of triangle of the current polygon
		itMeshPoly.numTriangles(triCount);
		
		// get the normal of the face
		itMeshPoly.getNormal(normal);
		normal *= meshWorldMatrix;
		
		// for each triangle of the current polygon
		for (int i = 0; i < triCount; i++)
		{
			// getting the info relative to the triangle
			// WARNING : trianglePoints may not include the tweaks informations
			itMeshPoly.getTriangle(i, trianglePoints, triangleVertices);

			// building a vector from the origin to the barycenter of the triangle
			toBarycenter = MVector(barycenter(trianglePoints));
			dot = normal * toBarycenter.normal();

			// computing the volume of each tetrahedron ( each triangle + P(0, 0, 0) )
			// subtract the value if the triangle normal face the origin, add it if it doesn't
			trianglePoints.append(MPoint(0, 0, 0));
			if (dot >= 0) 
			{
				volume += tetrahedralVolume(trianglePoints);
			}
			else
			{
				volume -= tetrahedralVolume(trianglePoints);
			}
		}
	}

	return volume;
}



//MPoint closestPointOnTriangle(MPoint& point, MPointArray& triPoints)
//{
//	MVector edgeA(triPoints[1] - triPoints[0]);
//	MVector edgeB(triPoints[2] - triPoints[0]);
//	MVector toSummit(point - triPoints[0]);
//
//	// computing the normal from the 2 points of the triangle
//	MVector normal = (edgeA ^ edgeB).normal();
//
//	// computing distance point -> plan abs(dot(normal, toSummit)) (if normal is a unit vector)
//	// were not applying the abs to keep the information of direction used later to get the point on plan
//	double toPlan(normal * toSummit);
//
//	// closest point is point + distance * normal.(toSummit.normalize)
//}

