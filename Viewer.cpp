// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>
#include <igl\exact_geodesic.cpp>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
namespace opengl
{
namespace glfw
{

  IGL_INLINE void Viewer::init()
  {
   

  }

  //IGL_INLINE void Viewer::init_plugins()
  //{
  //  // Init all plugins
  //  for (unsigned int i = 0; i<plugins.size(); ++i)
  //  {
  //    plugins[i]->init(this);
  //  }
  //}

  //IGL_INLINE void Viewer::shutdown_plugins()
  //{
  //  for (unsigned int i = 0; i<plugins.size(); ++i)
  //  {
  //    plugins[i]->shutdown();
  //  }
  //}

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1)
  {
    data_list.front().id = 0;

  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }
  IGL_INLINE void Viewer::load_meshes_from_file(const std::string& mesh_file_name_string) {

      std::ifstream file(mesh_file_name_string);
      if (file.is_open()) {
          std::string line;
          int times = 1;
          while (getline(file, line))
          {
              
              load_mesh_from_file(line);

          }
          file.close();
      }
      else {
          std::cerr << "-- configuration file not found --" << std::endl;
      }

  }
  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      data().set_mesh(V,F);
      data().set_uv(UV_V,UV_F);

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }
    

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;

    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
  void Viewer::initEdges()
  {
      int index = 0;
      std::vector<ViewerData>::iterator ptr;
      //snake_Links.resize(data_list.size() - 1);
      for (ptr = data_list.begin(); ptr != data_list.end(); ++ptr) {
          if (index < 10)
          {
              if (index == 0) {
                  ptr->prev = nullptr;
              }
              else
              {
                  ptr->prev = &data_list[index - 1];    
              }
              Eigen::Vector3d m = ptr->V.colwise().minCoeff();
              Eigen::Vector3d M = ptr->V.colwise().maxCoeff();
              double centerX = (M(0) + m(0)) / 2;
              double centerZ = (M(2) + m(2)) / 2;
              double link_length = M(1) - m(1);
              ptr->SetCenterOfRotation(Eigen::Vector3d(data_list[index].V.colwise().mean()[0], data_list[index].V.colwise().minCoeff()[1], data_list[index].V.colwise().mean()[0]));
            
              ptr->MyTranslate(Eigen::Vector3f(0, link_length, 0));

              if (index == 9) {
                  snake_head = &data_list[index];
                  ptr->tree.init(ptr->V, ptr->F);
                  
            }
          }
          else if(index ==10) {
              ptr->stage = 0;
              switch (game_level)
              {
              case 1:
                  ptr->MyTranslate(Eigen::Vector3f(11, 16, 0));
                  ptr->_x = -0.3;
                  ptr->_y = 0.0;
                  ptr->_z = 0.0;
                  break;

              case 2:
                  ptr->MyTranslate(Eigen::Vector3f(-5, 5, 0));
                  ptr->_x = 0.1;
                  ptr->_y = 0.0;
                  ptr->_z = 0.0;
                  break;

              case 3:
                  ptr->MyTranslate(Eigen::Vector3f(-8,0, 0));
                  ptr->_x = 0.15;
                  ptr->_y = 0.0;
                  ptr->_z = 0.0;
                  break;
              
              case 4:
                  ptr->MyTranslate(Eigen::Vector3f(11, 5, 0));
                  ptr->_x = -0.2;
                  ptr->_y = 2;
                  ptr->_z = 0.0;
                  break;

              case 5:
                  ptr->MyTranslate(Eigen::Vector3f(11, 3, 0));
                  ptr->_x = -1.5;
                  ptr->_y = -3.6;
                  ptr->_z = 0.0;
                  break;

              case 6:
                  ptr->MyTranslate(Eigen::Vector3f(0, 3, -30));
                  ptr->_x = -0.2;
                  ptr->_y = 0;
                  ptr->_z = 0.2;
                  break;

              case 7:
                  ptr->MyTranslate(Eigen::Vector3f(0, 3, 30));
                  ptr->_x = -0.2;
                  ptr->_y = 0;
                  ptr->_z = -0.1;
                  break;

              default:
                  break;
              }
              ptr->tree.init(ptr->V, ptr->F);
          }
          ptr->set_colors(Eigen::RowVector3d(135. / 255., 255. / 255., 255. / 255.));
          index++;
      }
  


  }
  Eigen::Matrix4f Viewer::CalcParentsTrans(ViewerData *first) {
      if (first == nullptr) {
         
          return  Eigen::Matrix4f::Identity();
      }
     
      return CalcParentsTrans(first->prev) * first->MakeTrans();
  }
  void Viewer::drawBox(AABB<Eigen::MatrixXd, 3> node, int index, Eigen::RowVector3d edgesColor) {
      Eigen::MatrixXd V_box(8, 3);
      Eigen::Vector3d blf = node.m_box.corner(node.m_box.BottomLeftFloor);
      Eigen::Vector3d brf = node.m_box.corner(node.m_box.BottomRightFloor);
      Eigen::Vector3d tlf = node.m_box.corner(node.m_box.TopLeftFloor);
      Eigen::Vector3d trf = node.m_box.corner(node.m_box.TopRightFloor);
      Eigen::Vector3d blc = node.m_box.corner(node.m_box.BottomLeftCeil);
      Eigen::Vector3d brc = node.m_box.corner(node.m_box.BottomRightCeil);
      Eigen::Vector3d tlc = node.m_box.corner(node.m_box.TopLeftCeil);
      Eigen::Vector3d trc = node.m_box.corner(node.m_box.TopRightCeil);
     // data_list[index].old_points_size = data_list[index].points.rows();
      //data_list[index].old_lines_size = data_list[index].lines.rows();
      V_box << blf[0], blf[1], blf[2],//0
          brf[0], brf[1], brf[2],//1
          tlf[0], tlf[1], tlf[2],//2
          trf[0], trf[1], trf[2],//3
          blc[0], blc[1], blc[2],//4
          brc[0], brc[1], brc[2],//5	
          tlc[0], tlc[1], tlc[2],//6
          trc[0], trc[1], trc[2];//7

      Eigen::MatrixXi E_box(12, 2);
      E_box <<
          0, 1,//BottomLeftFloor,BottomRightFloor
          2, 3,//TopLeftFloor,TopRightFloor
          4, 5,//BottomLeftCeil,BottomRightCeil
          6, 7,//TopLeftCeil,TopRightCeil
          0, 2,//BottomLeftFloor,TopLeftFloor
          1, 3,//BottomRightFloor,TopRightFloor
          4, 6,//BottomLeftCeil,TopLeftCeil
          5, 7,//BottomRightCeil,TopRightCeil
          2, 6,//TopLeftFloor,TopLeftCeil
          0, 4,//BottomLeftFloor,BottomLeftCeil
          3, 7,// TopRightFloor, TopRightCeil
          1, 5;//BottomRightFloor,BottomRightCeil

      //data_list[index].add_points(V_box, edgesColor);

      for (int i = 0; i < 12; i++) {
          data_list[index].add_edges(V_box.row(E_box(i, 0)), V_box.row(E_box(i, 1)), edgesColor);
      }

  }
  Eigen::Matrix3f Viewer::CalcParentsInverse(ViewerData* first) {
      if (first==nullptr) {

          return Eigen::Matrix3f::Identity();
      }

      return  first->GetRotation().inverse() * CalcParentsInverse(first->prev);
  }
  Eigen::Matrix3f Viewer::CalParentsRotationMatrixes(ViewerData* first) {
      if (first == nullptr)
          return Eigen::Matrix3f::Identity();
      else {
          return CalParentsRotationMatrixes(first->prev) * first->GetRotation().matrix();
      }

  }
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core()core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;

    this->load_mesh_from_file(fname.c_str());
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible)
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

 

} // end namespace
} // end namespace
}
