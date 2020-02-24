#include "dataspeedObject_display_common.h"
#include "dataspeedObject_display.h"

namespace ds_visualization 
{

  DataspeedObjectDisplay::DataspeedObjectDisplay()
  {
    coloring_property_ = new rviz::EnumProperty(
      "coloring", "Flat color",
      "coloring method",
      this, SLOT(updateColoring()));
    coloring_property_->addOption("Flat color", 0);
    coloring_property_->addOption("Message", 1);

    color_property_ = new rviz::ColorProperty(
      "color", QColor(25, 255, 0),
      "color to draw the bounding boxes",
      this, SLOT(updateColor()));
    alpha_property_ = new rviz::FloatProperty(
      "alpha", 0.8,
      "alpha value to draw the bounding boxes",
      this, SLOT(updateAlpha()));
    only_edge_property_ = new rviz::BoolProperty(
      "only edge", true,
      "show only the edges of the boxes",
      this, SLOT(updateOnlyEdge()));
    line_width_property_ = new rviz::FloatProperty(
      "line width", 0.05,
      "line width of the edges",
      this, SLOT(updateLineWidth()));
    show_coords_property_ = new rviz::BoolProperty(
      "show coords", false,
      "show coordinate of bounding box",
      this, SLOT(updateShowCoords()));
    show_speed_property_ = new rviz::BoolProperty(
      "show speed", true,
      "show speed of bounding box",
      this, SLOT(updateShowSpeed()));
    show_text_property_ = new rviz::BoolProperty( 
     "show speed", true,
      "show object speed",
      this, SLOT(updateShowText()));
    word_size_property_ = new rviz::FloatProperty(
      "word size", 2,
      "word size of label and id",
      this, SLOT(updateWordSize()));
  }

  DataspeedObjectDisplay::~DataspeedObjectDisplay()
  {
    delete color_property_;
    delete alpha_property_;
    delete only_edge_property_;
    delete coloring_property_;
    delete show_coords_property_;
    delete show_speed_property_;
    delete show_text_property_;
  }

  void DataspeedObjectDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

    updateColor();
    updateAlpha();
    updateOnlyEdge();
    updateColoring();
    updateLineWidth();
    updateShowCoords();
    updateShowSpeed();
    updateShowText();
    updateWordSize();
  }

  void DataspeedObjectDisplay::updateLineWidth()
  {
    line_width_ = line_width_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

    void DataspeedObjectDisplay::updateWordSize()
  {
    word_size_ = word_size_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectDisplay::updateColor()
  {
    color_ = color_property_->getColor();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectDisplay::updateOnlyEdge()
  {
    only_edge_ = only_edge_property_->getBool();
    if (only_edge_) {
      line_width_property_->show();
    }
    else {
      line_width_property_->hide();;
    }
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectDisplay::updateColoring()
  {
    
    if (coloring_property_->getOptionInt() == 0) {
      coloring_method_ = "flat";
      color_property_->show();
    }
    else if (coloring_property_->getOptionInt() == 1) {
      coloring_method_ = "message"; //label
      color_property_->hide();
    }

    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectDisplay::updateShowCoords()
  {
    show_coords_ = show_coords_property_->getBool();
    // Immediately apply show_coords attribute
    if (!show_coords_) {
      hideCoords();
    }
    else if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectDisplay::updateShowSpeed()
  {
    show_speed_ = show_speed_property_->getBool();
    // Immediately apply show_coords attribute
    if (!show_speed_) {
      hideSpeed();
    }
    else if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectDisplay::updateShowText()
  {
    show_text_ = show_text_property_->getBool();//   getStdString();
   
    if (show_text_) {
      word_size_property_->show();
    }
    else {
      word_size_property_->hide();
      }
    // Immediately apply show_coords attribute
        if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectDisplay::reset()
  {
    MFDClass::reset();
    shapes_.clear();
    edges_.clear();
    coords_nodes_.clear();
    coords_objects_.clear();
    speed_objects_.clear();
    speed_nodes_.clear();
    latest_msg_.reset();
  }

  void DataspeedObjectDisplay::processMessage(const ds_av_msgs::DataspeedObject::ConstPtr& msg)
  {
    // Store latest message
    latest_msg_ = msg;

    // Convert bbox to bbox_array to show it
    ds_av_msgs::DataspeedObjectArrayPtr bbox_array_msg(new ds_av_msgs::DataspeedObjectArray);
    bbox_array_msg->header = msg->header;
    std::vector<ds_av_msgs::DataspeedObject> boxes;
    boxes.push_back(*msg);
    bbox_array_msg->objects = boxes;
    if (!only_edge_) {
      showBoxes(bbox_array_msg);
    } else {
      showEdges(bbox_array_msg);
    }

    if (show_coords_) {
      showCoords(bbox_array_msg);
    } else {
      hideCoords();
    }

    if (show_speed_) {
      showSpeed(bbox_array_msg);
    } else {
      hideSpeed();
    }

    if (show_text_) {
      showText(bbox_array_msg);
    } else {
      hideText();
    }
  }

}  // namespace ds_visualization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ds_visualization::DataspeedObjectDisplay, rviz::Display)