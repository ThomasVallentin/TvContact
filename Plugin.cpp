//
// Created by root on 3/12/21.
//

#include "TvContact.hpp"

#include <maya/MFnPlugin.h>

MStatus initializePlugin( MObject obj )
{
    MStatus result;

    MFnPlugin plugin( obj, "ThomasVallentin", "0.0.0", "Any");
    result = plugin.registerNode(
            "tvContact" ,
            TvContact::id ,
            &TvContact::creator ,
            &TvContact::initialize ,
            MPxNode::kGeometryFilter
    );

    return result;
}

MStatus uninitializePlugin( MObject obj )
{
    MStatus result;

    MFnPlugin plugin( obj );
    result = plugin.deregisterNode( TvContact::id );

    return result;
}
