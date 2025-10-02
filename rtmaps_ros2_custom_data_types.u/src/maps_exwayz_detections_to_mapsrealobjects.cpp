#include <cmath>

#include "maps_exwayz_detections_to_mapsrealobjects.h"

MAPS_BEGIN_INPUTS_DEFINITION(MAPSExwayzDetectionsToMAPSRealObjects)
    MAPS_INPUT("detections", MAPS::FilterUnsignedInteger8, MAPS::LastOrNextReader)
MAPS_END_INPUTS_DEFINITION

MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSExwayzDetectionsToMAPSRealObjects)
    MAPS_OUTPUT("realobjs", MAPS::RealObject, NULL,NULL,MAX_OBJECTS)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSExwayzDetectionsToMAPSRealObjects)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSExwayzDetectionsToMAPSRealObjects)
    //MAPS_ACTION("aName",MAPSmy_data_types_component::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (my_data_types_component) behaviour
MAPS_COMPONENT_DEFINITION(MAPSExwayzDetectionsToMAPSRealObjects,"exwayz_detections_to_realobjs","1.0",128,
			  MAPS::Sequential,MAPS::Sequential,
			  -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
			  -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions

void MAPSExwayzDetectionsToMAPSRealObjects::Birth()
{

}

void MAPSExwayzDetectionsToMAPSRealObjects::Core() 
{
  MAPSIOElt* ioelt = StartReading(Input(0));
  if (ioelt == nullptr) { return; }

  int num_detections = ioelt->VectorSize() / sizeof(Detection);
  if (num_detections >= MAX_OBJECTS)
  {
    num_detections = MAX_OBJECTS;
  }
    
  MAPSIOElt* ioeltout = StartWriting(Output(0));
  MAPSRealObject* realobj_out = &ioeltout->RealObject();
  Detection* detection_in = (Detection*)ioelt->Data();

  MAPS::Memset(realobj_out,0, num_detections * sizeof(MAPSRealObject));
  for (int i=0 ; i< num_detections; i++)
  {
    (*realobj_out).kind = MAPSRealObject::Vehicle;
    (*realobj_out).id = (*detection_in).objectid_;
    (*realobj_out).x = (*detection_in).box_center_[0];
    (*realobj_out).y = (*detection_in).box_center_[1];
    (*realobj_out).z = (*detection_in).box_center_[2];
    (*realobj_out).color = MAPS_RGB(255,0,0);
    (*realobj_out).vehicle.theta = (*detection_in).box_angle_;
    (*realobj_out).vehicle.speed = sqrt((*detection_in).velocity_[0] * (*detection_in).velocity_[0] + (*detection_in).velocity_[1] * (*detection_in).velocity_[1] + (*detection_in).velocity_[2] * (*detection_in).velocity_[2]);
    (*realobj_out).vehicle.kind = MAPSVehicle::Car;
    (*realobj_out).vehicle.width = (*detection_in).box_size_[0];
    (*realobj_out).vehicle.length = (*detection_in).box_size_[1];
    (*realobj_out).vehicle.height = (*detection_in).box_size_[2];
    (*realobj_out).misc1 = (*detection_in).num_pts_;
    (*realobj_out).misc2 = (*detection_in).age_;

    realobj_out++;
    detection_in++;
  }
  ioeltout->Timestamp() = ioelt->Timestamp();
  ioeltout->VectorSize() = num_detections;
  StopWriting(ioeltout);
}

void MAPSExwayzDetectionsToMAPSRealObjects::Death() 
{
}