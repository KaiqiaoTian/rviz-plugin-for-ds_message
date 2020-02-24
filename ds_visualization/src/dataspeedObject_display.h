#pragma once
#ifndef Q_MOC_RUN
#include "dataspeedObject_display_common.h"
#include <ds_av_msgs/DataspeedObjectArray.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/arrow.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#endif

namespace ds_visualization 
{

  class DataspeedObjectDisplay: public DataspeedObjectDisplayCommon<ds_av_msgs::DataspeedObject>
  {
  Q_OBJECT
  public:
    DataspeedObjectDisplay();
    virtual ~DataspeedObjectDisplay(); 
  protected:
    void onInitialize();
    virtual void reset();

    bool only_edge_;
    bool show_coords_;
    bool show_speed_;
    bool show_text_;
    // Properties
    rviz::EnumProperty* coloring_property_;
    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::BoolProperty* only_edge_property_;
    rviz::FloatProperty* line_width_property_;
    rviz::FloatProperty* word_size_property_;    
    rviz::BoolProperty* show_coords_property_;
    rviz::BoolProperty* show_speed_property_;
     rviz::BoolProperty* show_text_property_;

    ds_av_msgs::DataspeedObject::ConstPtr latest_msg_;
  protected Q_SLOTS:
    void updateColor();
    void updateAlpha();
    void updateOnlyEdge();
    void updateColoring();
    void updateLineWidth();
    void updateShowCoords();
    void updateShowSpeed();
    void updateShowText();
    void updateWordSize();
  private:
    void processMessage(const ds_av_msgs::DataspeedObject::ConstPtr& msg);      
  };

}  // namespace ds_visualization 
