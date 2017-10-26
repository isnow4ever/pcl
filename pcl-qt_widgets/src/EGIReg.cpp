#include "EGIReg.h"

#define ICO_X .525731112119133606
#define ICO_Z .850650808352039932

EGIReg::EGIReg(PointCloudT::Ptr cloud_model, PointCloudT::Ptr cloud_data)
	:model(cloud_model), data(cloud_data)
{

}


EGIReg::~EGIReg()
{
}

