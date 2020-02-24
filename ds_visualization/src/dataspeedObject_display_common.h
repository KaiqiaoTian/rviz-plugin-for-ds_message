#pragma once
#ifndef Q_MOC_RUN
#include "dataspeedObject_display_common.h"  
#include <ds_av_msgs/DataspeedObjectArray.h>  
#include <rviz/properties/color_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTexture.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#endif

namespace ds_visualization   
{

  template <class MessageType>
  class DataspeedObjectDisplayCommon: public rviz::MessageFilterDisplay<MessageType>  
  {
    public:
    DataspeedObjectDisplayCommon() {};
    ~DataspeedObjectDisplayCommon() {};
#if ROS_VERSION_MINIMUM(1,12,0)
    typedef std::shared_ptr<rviz::Shape> ShapePtr;
    typedef std::shared_ptr<rviz::BillboardLine> BillboardLinePtr;
    typedef std::shared_ptr<rviz::Arrow> ArrowPtr;
    typedef std::shared_ptr<rviz::MovableText> TextPtr;
#else
    typedef boost::shared_ptr<rviz::Shape> ShapePtr;
    typedef boost::shared_ptr<rviz::BillboardLine> BillboardLinePtr;
    typedef boost::shared_ptr<rviz::Arrow> ArrowPtr;
    typedef boost::shared_ptr<rviz::MovableText> TextPtr;
#endif

    protected:
    QColor color_;
    std::string coloring_method_;
    double alpha_;
    double line_width_;
    double word_size_;
    visualization_msgs::MarkerArray marker_msg_;

    std::vector<ArrowPtr> speed_objects_;
    std::vector<std::vector<ArrowPtr> > coords_objects_;
    std::vector<TextPtr> text_objects_;  

    std::vector<Ogre::SceneNode*> coords_nodes_;
    std::vector<Ogre::SceneNode*> speed_nodes_;
    std::vector<Ogre::SceneNode*> text_nodes_;

  
    std::vector<BillboardLinePtr> edges_;
    std::vector<ShapePtr> shapes_;


    QColor getColor(size_t index, const ds_av_msgs::DataspeedObject& box)
    {
      if (coloring_method_ == "message") { 
       
        return QColor(box.color.r,
                      box.color.g,
                      box.color.b,
                      box.color.a);
      }
      else if (coloring_method_ == "flat") {
        return color_;
      }
    }
   
    bool isValidBoundingBox(const ds_av_msgs::DataspeedObject box_msg)
    {
      return !(box_msg.bounding_box_axes.x < 1.0e-9 ||
          box_msg.bounding_box_axes.y < 1.0e-9 ||
          box_msg.bounding_box_axes.z < 1.0e-9 ||
          std::isnan(box_msg.bounding_box_axes.x) ||
          std::isnan(box_msg.bounding_box_axes.y) ||
          std::isnan(box_msg.bounding_box_axes.z));
    }

    void allocateShapes(int num)
    {
      if (num > shapes_.size()) {
        for (size_t i = shapes_.size(); i < num; i++) {
          ShapePtr shape (new rviz::Shape(
                            rviz::Shape::Cube, this->context_->getSceneManager(),
                            this->scene_node_));
          shapes_.push_back(shape);
        }
      } else if (num < shapes_.size()) {
        shapes_.resize(num);
      }
    }

    void allocateBillboardLines(int num)
    {
      if (num > edges_.size()) {
        for (size_t i = edges_.size(); i < num; i++) {
          BillboardLinePtr line(new rviz::BillboardLine(
                                  this->context_->getSceneManager(), this->scene_node_));
          edges_.push_back(line);
        }
      } else if (num < edges_.size()) {
          edges_.resize(num);
      }
    }

    void allocateCoords(int num)
    {
      if (num > coords_objects_.size()) {
        for (size_t i = coords_objects_.size(); i < num; i++) {
          Ogre::SceneNode* scene_node = this->scene_node_->createChildSceneNode();
          std::vector<ArrowPtr> coord;
          for (int i = 0; i < 3; i++) {
            ArrowPtr arrow (new rviz::Arrow(this->scene_manager_, scene_node));
            coord.push_back(arrow);
          }
          coords_nodes_.push_back(scene_node);
          coords_objects_.push_back(coord);
        }
      } else if (num < coords_objects_.size()) {
        for (size_t i = num; i < coords_objects_.size(); i++) {
          coords_nodes_[i]->setVisible(false);
        }
        coords_objects_.resize(num);
        coords_nodes_.resize(num);
      }
    }

    void allocateCoords_speed(int num)
    {
      if (num > speed_objects_.size()) {
        for (size_t i = speed_objects_.size(); i < num; i++) {
          Ogre::SceneNode* scene_node = this->scene_node_->createChildSceneNode();
          speed_nodes_.push_back(scene_node);
          speed_objects_.push_back(ArrowPtr(new rviz::Arrow(this->scene_manager_, scene_node)));
        }
      } else if (num < speed_objects_.size()) {
        for (size_t i = num; i < speed_objects_.size(); i++) {
          speed_nodes_[i]->setVisible(false);
        }
        speed_objects_.resize(num);
        speed_nodes_.resize(num);
      }
    }

    void allocateText(int num)
    {
      if (num > text_objects_.size()) {
        for (size_t i = text_objects_.size(); i < num; i++) {
          Ogre::SceneNode* scene_node = this->scene_node_->createChildSceneNode();
          text_nodes_.push_back(scene_node);
          TextPtr text_lable( new rviz::MovableText("kkkkkkkkkkkkkk","Liberation Sans", 2.0, Ogre::ColourValue::White));
          scene_node->attachObject(text_lable.get());
          text_lable->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_ABOVE);
          text_objects_.push_back(text_lable);
        }
      } else if (num < text_objects_.size()) {
        for (size_t i = num; i < text_objects_.size(); i++) {
          text_nodes_[i]->setVisible(false);
        }
        text_objects_.resize(num);
        text_nodes_.resize(num);
      }
    }
 
    void showText(const ds_av_msgs::DataspeedObjectArray::ConstPtr& msg)
    {
      allocateText(msg->objects.size());

      for (size_t i = 0; i < msg->objects.size(); i++) {
        std::string text_recv;
        if (msg->objects[i].label.length() > 0) {
          std::stringstream ss;
          ss <<"label:"<< msg->objects[i].label << " (id:" << msg->objects[i].id << ")";
          text_recv = ss.str();
        } else {
          text_recv = std::to_string(msg->objects[i].id);
        }

        ds_av_msgs::DataspeedObject box = msg->objects[i]; 
        Ogre::Vector3 position;
        float a = 1;
        Ogre::Quaternion orientation;
        if(!this->context_->getFrameManager()->getTransform(
            box.header, position, orientation)) {
          ROS_ERROR("Error transforming from frame '%s' to frame '%s'",
                    box.header.frame_id.c_str(), qPrintable(this->fixed_frame_));
          return;
        }
        position[0] = box.pose.position.x + box.bounding_box_offset.x;
        position[1] = box.pose.position.y + box.bounding_box_offset.y;
        position[2] = box.pose.position.z + box.bounding_box_offset.z;

        Ogre::SceneNode* scene_node = text_nodes_[i];
        scene_node->setVisible(true);
        scene_node->setPosition(position);
        text_objects_[i]->setGlobalTranslation(Ogre::Vector3(0, 0, 0.6 * box.bounding_box_axes.z));
        text_objects_[i]->setCaption(text_recv);
        text_objects_[i]->setVisible(true);
        text_objects_[i]->setCharacterHeight(word_size_);
      }
    }
    
    void showBoxes(const ds_av_msgs::DataspeedObjectArray::ConstPtr& msg)
    {
      edges_.clear();
      allocateShapes(msg->objects.size());
      
      for (size_t i = 0; i < msg->objects.size(); i++) {
        ds_av_msgs::DataspeedObject box = msg->objects[i];        
        if (!isValidBoundingBox(box)) {
          ROS_WARN_THROTTLE(10, "Invalid size of bounding box is included and skipped: [%f, %f, %f]",
            box.bounding_box_axes.x, box.bounding_box_axes.y, box.bounding_box_axes.z);
          continue;
        }
        
        ShapePtr shape = shapes_[i];
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        geometry_msgs::Pose offset_pose = box.pose;
        offset_pose.position.x += box.bounding_box_offset.x;
        offset_pose.position.y += box.bounding_box_offset.y;
        offset_pose.position.z += box.bounding_box_offset.z;
        if (!this->context_->getFrameManager()->transform(box.header, offset_pose, position, orientation)) {
          std::ostringstream oss;
          oss << "Error transforming pose";
          oss << " from frame '" << box.header.frame_id << "'";
          oss << " to frame '" << qPrintable(this->fixed_frame_) << "'";
          ROS_ERROR_STREAM(oss.str());
          this->setStatus(rviz::StatusProperty::Error, "Transform", QString::fromStdString(oss.str()));
          return;
        }
        
        shape->setPosition(position);
        shape->setOrientation(orientation);
        Ogre::Vector3 dimensions;
        dimensions[0] = box.bounding_box_axes.x;
        dimensions[1] = box.bounding_box_axes.y;
        dimensions[2] = box.bounding_box_axes.z;
        shape->setScale(dimensions);
        QColor color = getColor(i, box);
        shape->setColor(color.red() / 255.0,
                        color.green() / 255.0,
                        color.blue() / 255.0,
                        alpha_);
      }
    }

    void showEdges(const ds_av_msgs::DataspeedObjectArray::ConstPtr& msg)      
    {
      shapes_.clear();
      allocateBillboardLines(msg->objects.size());      
      for (size_t i = 0; i < msg->objects.size(); i++) {
        ds_av_msgs::DataspeedObject box = msg->objects[i];        
        if (!isValidBoundingBox(box)) {
          ROS_WARN_THROTTLE(10, "Invalid size of bounding box is included and skipped: [%f, %f, %f]",
            box.bounding_box_axes.x, box.bounding_box_axes.y, box.bounding_box_axes.z);
          continue;
        }

        geometry_msgs::Vector3 dimensions = box.bounding_box_axes;

        BillboardLinePtr edge = edges_[i];
        edge->clear();
        Ogre::Vector3 position;
        Ogre::Quaternion quaternion;
        geometry_msgs::Pose offset_pose = box.pose;
        offset_pose.position.x += box.bounding_box_offset.x;
        offset_pose.position.y += box.bounding_box_offset.y;
        offset_pose.position.z += box.bounding_box_offset.z;
        if (!this->context_->getFrameManager()->transform(box.header, offset_pose, position, quaternion)) {
          std::ostringstream oss;
          oss << "Error transforming pose";
          oss << " from frame '" << box.header.frame_id << "'";
          oss << " to frame '" << qPrintable(this->fixed_frame_) << "'";
          ROS_ERROR_STREAM(oss.str());
          this->setStatus(rviz::StatusProperty::Error, "Transform", QString::fromStdString(oss.str()));
          return;
        }
        edge->setPosition(position);
        edge->setOrientation(quaternion);

        edge->setMaxPointsPerLine(2);
        edge->setNumLines(12);
        edge->setLineWidth(line_width_);
        
        QColor color = getColor(i, box);
        edge->setColor(color.red() / 255.0,
                        color.green() / 255.0,
                        color.blue() / 255.0,
                        alpha_);

        Ogre::Vector3 A, B, C, D, E, F, G, H;
        A[0] = dimensions.x / 2.0;
        A[1] = dimensions.y / 2.0;
        A[2] = dimensions.z / 2.0;
        B[0] = - dimensions.x / 2.0;
        B[1] = dimensions.y / 2.0;
        B[2] = dimensions.z / 2.0;
        C[0] = - dimensions.x / 2.0;
        C[1] = - dimensions.y / 2.0;
        C[2] = dimensions.z / 2.0;
        D[0] = dimensions.x / 2.0;
        D[1] = - dimensions.y / 2.0;
        D[2] = dimensions.z / 2.0;

        E[0] = dimensions.x / 2.0;
        E[1] = dimensions.y / 2.0;
        E[2] = - dimensions.z / 2.0;
        F[0] = - dimensions.x / 2.0;
        F[1] = dimensions.y / 2.0;
        F[2] = - dimensions.z / 2.0;
        G[0] = - dimensions.x / 2.0;
        G[1] = - dimensions.y / 2.0;
        G[2] = - dimensions.z / 2.0;
        H[0] = dimensions.x / 2.0;
        H[1] = - dimensions.y / 2.0;
        H[2] = - dimensions.z / 2.0;

        edge->addPoint(A); edge->addPoint(B); edge->newLine();
        edge->addPoint(B); edge->addPoint(C); edge->newLine();
        edge->addPoint(C); edge->addPoint(D); edge->newLine();
        edge->addPoint(D); edge->addPoint(A); edge->newLine();
        edge->addPoint(E); edge->addPoint(F); edge->newLine();
        edge->addPoint(F); edge->addPoint(G); edge->newLine();
        edge->addPoint(G); edge->addPoint(H); edge->newLine();
        edge->addPoint(H); edge->addPoint(E); edge->newLine();
        edge->addPoint(A); edge->addPoint(E); edge->newLine();
        edge->addPoint(B); edge->addPoint(F); edge->newLine();
        edge->addPoint(C); edge->addPoint(G); edge->newLine();
        edge->addPoint(D); edge->addPoint(H);
      }
    }

    void showCoords(const ds_av_msgs::DataspeedObjectArray::ConstPtr& msg)
    {
      allocateCoords(msg->objects.size());
      for (size_t i = 0; i < msg->objects.size(); i++) {
        ds_av_msgs::DataspeedObject box = msg->objects[i];
        std::vector<ArrowPtr> coord = coords_objects_[i];

        Ogre::SceneNode* scene_node = coords_nodes_[i];
        scene_node->setVisible(true);
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!this->context_->getFrameManager()->getTransform(box.header, position, orientation)) {
          ROS_WARN("Error transforming from frame '%s' to frame '%s'",
                    box.header.frame_id.c_str(), qPrintable(this->fixed_frame_));
          return;
        }
        scene_node->setPosition(position);
        scene_node->setOrientation(orientation); // scene node is at frame pose

        Ogre::Vector3 pos(box.pose.position.x,
                          box.pose.position.y,
                          box.pose.position.z);
        Ogre::Quaternion qua(box.pose.orientation.w,
                            box.pose.orientation.x,
                            box.pose.orientation.y,
                            box.pose.orientation.z);

        for (int j = 0; j < 3; j++) {
          Ogre::Vector3 scale;
          Ogre::Vector3 direction;
          Ogre::ColourValue rgba;
          switch (j) {
            case 0:
              scale = Ogre::Vector3(0.5 * box.bounding_box_axes.x, 1.0, 1.0);
              direction = qua * Ogre::Vector3(1, 0, 0);
              rgba = Ogre::ColourValue(1.0, 0.0, 0.0, 1.0);
            break;
            case 1:
              scale = Ogre::Vector3(0.5 * box.bounding_box_axes.y, 1.0, 1.0);
              direction = qua * Ogre::Vector3(0, 1, 0);
              rgba = Ogre::ColourValue(0.0, 1.0, 0.0, 1.0);
            break;
            case 2:
              scale = Ogre::Vector3(0.5 * box.bounding_box_axes.z, 1.0, 1.0);
              direction = qua * Ogre::Vector3(0, 0, 1);
              rgba = Ogre::ColourValue(0.0, 0.0, 1.0, 1.0);
            break;
          }

          coords_objects_[i][j]->setPosition(pos);
          coords_objects_[i][j]->setDirection(direction);
          coords_objects_[i][j]->setScale(scale);
          coords_objects_[i][j]->setColor(rgba);
        }
      }
    }

    void showSpeed(const ds_av_msgs::DataspeedObjectArray::ConstPtr& msg)
    {
      allocateCoords_speed(msg->objects.size());
      for (size_t i = 0; i < msg->objects.size(); i++) {
        ds_av_msgs::DataspeedObject box = msg->objects[i];
        Ogre::SceneNode* scene_node = speed_nodes_[i];
        scene_node->setVisible(true);
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if(!this->context_->getFrameManager()->getTransform(box.header, position, orientation)) {
          ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                    box.header.frame_id.c_str(), qPrintable(this->fixed_frame_));
          return;
        }
        scene_node->setPosition(position);
        scene_node->setOrientation(orientation); // scene node is at frame pose

        Ogre::Vector3 velocity(box.velocity.linear.x, box.velocity.linear.y, box.velocity.linear.z);
        speed_objects_[i]->setPosition(Ogre::Vector3(box.pose.position.x, box.pose.position.y, box.pose.position.z));
        speed_objects_[i]->setDirection(velocity.normalisedCopy());
        speed_objects_[i]->setScale(Ogre::Vector3(velocity.length(), 2.0, 2.0));
        speed_objects_[i]->setColor(Ogre::ColourValue(1, 0, 0, 1));
      }
    }

    void hideCoords()
    {
      for (size_t i = 0; i < coords_nodes_.size(); i++) {
        coords_nodes_[i]->setVisible(false);
      }
    }

    void hideSpeed()
    {
      for (size_t i = 0; i < speed_nodes_.size(); i++) {
        speed_nodes_[i]->setVisible(false);
      }
    }

    void hideText()
    {
      for (size_t i = 0; i < text_nodes_.size(); i++) {
        text_nodes_[i]->setVisible(false);
      }
    }

  };  // class dataspeedObject_display_comman  

}  // namespace ds_visualization
