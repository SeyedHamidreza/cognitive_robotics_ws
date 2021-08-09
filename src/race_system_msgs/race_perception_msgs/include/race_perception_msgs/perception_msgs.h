#ifndef _PERCEPTION_MSGS_H_
#define _PERCEPTION_MSGS_H_


/* _________________________________
   |                                 |
   |  RACE perception msg header     |
   |_________________________________| */

#include <race_perception_msgs/PCTOV.h>
#include <race_perception_msgs/SITOV.h>
#include <race_perception_msgs/RTOV.h>
#include <race_perception_msgs/ObjectCategory.h>
#include <race_perception_msgs/NOCD.h>
#include <race_perception_msgs/RRTOV.h>
#include <race_perception_msgs/ObjectCategoryBayesModel.h>
#include <race_perception_msgs/TOVI.h>
#include <race_perception_msgs/GTOV.h>
namespace race_perception_msgs
{

/////////////////////////////////////////////
//      the key shold not be redundant and 
//      needs to end with underscore '_'
/////////////////////////////////////////////
namespace key
{

    const std::string PC = "PCTOV_";
    const std::string SI = "SITOV_";
    const std::string RV = "RTOV_";
    const std::string OC = "OC_";
    const std::string RR = "RRTOV_";
    const std::string EL = "ELEM_";
    const std::string CL = "CLST_";
    const std::string OR = "OBJREP_";
    const std::string OBM = "OBJBAY_";
    const std::string TOVI = "TOVI_";
    const std::string GTOV = "GTOV_";
} //namespace of race_perception_msgs::key

namespace topic
{
    const std::string PC = "pctov_";
    const std::string SI = "sitov_";
    const std::string RV = "rtov_";
    const std::string RR = "rrtov_";
} //namespace of race_perception_msgs::topic

namespace c_state
{
    const uint32_t NOC = 0;
    const uint32_t ADD = 1;
    const uint32_t DEL = 2;
    const uint32_t UPD = 4;
} //namespace of race_perception_msgs::c_state

namespace feature_type
{
    const uint32_t SPIM = 0;        //spin image
    const uint32_t SIZE = 1;        //size
    const uint32_t COLO = 2;        //color
    const uint32_t SIFT = 4;        //SIFT
    const uint32_t SURF = 8;        //SURF
} //namespace of race_perception_msgs::feature_type


    const std::string Separator = "_";
    const std::string SEP = "_";


} //namespace of race_perception_msgs

#endif
