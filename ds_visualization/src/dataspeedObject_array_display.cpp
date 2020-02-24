#include "dataspeedObject_display_common.h"
#include "dataspeedObject_array_display.h"

namespace ds_visualization 
{

  DataspeedObjectArrayDisplay::DataspeedObjectArrayDisplay()
  {
    coloring_property_ = new rviz::EnumProperty(
      "coloring", "Flat color", //Auto
      "coloring method",
      this, SLOT(updateColoring()));
    coloring_property_->addOption("Flat color", 1);
    coloring_property_->addOption("Message", 2);

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
      "show object speed",
      this, SLOT(updateShowSpeed()));
    show_text_property_ = new rviz::BoolProperty( 
      "show text", true,
      "show object label and id",
      this, SLOT(updateShowText()));
    word_size_property_ = new rviz::FloatProperty(
      "word size", 2,
      "word size of label and id",
      this, SLOT(updateWordSize()));
  }

  DataspeedObjectArrayDisplay::~DataspeedObjectArrayDisplay()
  {
    delete color_property_;
    delete alpha_property_;
    delete only_edge_property_;
    delete coloring_property_;
    delete show_coords_property_;
    delete show_speed_property_;
    delete show_text_property_;
  }

  void DataspeedObjectArrayDisplay::onInitialize()  
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

  void DataspeedObjectArrayDisplay::updateLineWidth()
  {
    line_width_ = line_width_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectArrayDisplay::updateWordSize()
  {
    word_size_ = word_size_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }


  void DataspeedObjectArrayDisplay::updateColor()
  {
    color_ = color_property_->getColor();
    if (latest_msg_) {
      
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectArrayDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectArrayDisplay::updateOnlyEdge()
  {
    only_edge_ = only_edge_property_->getBool();
    if (only_edge_) {
      line_width_property_->show();
    }
    else {
      line_width_property_->hide();
    }
    // Imediately apply attribute
    if (latest_msg_) {
      if (only_edge_) {
        showEdges(latest_msg_);
      }
      else {
        showBoxes(latest_msg_);
      }
    }
  }

  void DataspeedObjectArrayDisplay::updateColoring()
  {
    
    if (coloring_property_->getOptionInt() == 0) {
      coloring_method_ = "message";
      color_property_->hide();
    }
    else if (coloring_property_->getOptionInt() == 1) {
      coloring_method_ = "flat";
      color_property_->show();
    }
    else if (coloring_property_->getOptionInt() == 2) {
      coloring_method_ = "message";
      color_property_->hide();
    }

    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void DataspeedObjectArrayDisplay::updateShowCoords()
  {
    show_coords_ = show_coords_property_->getBool();
    // Immediately apply show_coords attribute
    if (!show_coords_) {
      hideCoords();

    }
    else if (show_coords_ && latest_msg_) {
      showCoords(latest_msg_);
    //  showarrow_text(latest_msg_);
    }
  }

  void DataspeedObjectArrayDisplay::updateShowSpeed()
  {
    show_speed_ = show_speed_property_-> getBool();
    // Immediately apply show_coords attribute
    if (!show_speed_) {
      hideSpeed();

    }
    else if (show_speed_ && latest_msg_) {
      showSpeed(latest_msg_);
    }
  }

  void DataspeedObjectArrayDisplay::updateShowText()
  {
    show_text_ = show_text_property_->getBool();//   getStdString();
    // Immediately apply show_coords attribute
    if (show_text_) {
      word_size_property_->show();
    }
    else {
      word_size_property_->hide();
    }
    // Imediately apply attribute

    if (!show_text_) {
      hideText();

    }
    else if (show_text_ && latest_msg_) {
      showText(latest_msg_);
    }
  }

  void DataspeedObjectArrayDisplay::reset()
  {
    MFDClass::reset();
    shapes_.clear();
    edges_.clear();
    coords_nodes_.clear();
    coords_objects_.clear();
    speed_nodes_.clear();
    speed_objects_.clear();
    text_nodes_.clear();
    text_objects_.clear();
    latest_msg_.reset();
  }

  void DataspeedObjectArrayDisplay::processMessage(const ds_av_msgs::DataspeedObjectArray::ConstPtr& msg)
  {
    // Store latest message
    latest_msg_ = msg;

    if (!only_edge_) {
      showBoxes(msg);
    } else {
      showEdges(msg);
    }

    if (show_speed_) {
      showSpeed(msg);
    } else {
      hideSpeed();
    }
    
    if (show_coords_) {
      showCoords(msg);
    } else {
      hideCoords();
    }

    if (show_text_) {
      showText(msg);
    } else {
      hideText();
    }
  }
}  // namespace ds_visualization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ds_visualization::DataspeedObjectArrayDisplay, rviz::Display)
