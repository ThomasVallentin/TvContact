#include "tvContact.h"

#include <maya/MFnPlugin.h>

MStatus initializePlugin( MObject obj ) 
{
	MStatus status;

	MFnPlugin fnPlugin(obj, "Thomas Vallentin", "0.0.1", "Any");

	fnPlugin.registerNode("tvContact",
		TvContact::id,
		TvContact::creator,
		TvContact::initialize,
		MPxNode::kDeformerNode);

	return MS::kSuccess;
}

MStatus uninitializePlugin( MObject obj ) 
{
	MStatus status;

	MFnPlugin fnPlugin(obj);

	fnPlugin.deregisterNode(TvContact::id);

	return MS::kSuccess;
}