// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"
#include <igl/circulation.h>
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
#include <igl\edge_flaps.h>
#include <igl\shortest_edge_and_midpoint.h>
#include<igl/vertex_triangle_adjacency.h>
#include <igl\edge_collapse_is_valid.h>

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

			IGL_INLINE Viewer::Viewer() :
				data_list(1),
				selected_data_index(0),
				next_data_id(1),
				data_list2(1)
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
				std::cout << usage << std::endl;
#endif
			}
			/* 
			<<<<<<<<<<<<<<<<<<<<<<<new Function>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			*/
			void Viewer::initEdges()
			{
				vector<ViewerData>::iterator ptr;
				data_list2.resize(data_list.size());
				int index = 0;
				for (ptr = data_list.begin(); ptr != data_list.end(); ++ptr) {
					
			
					edge_flaps(ptr->F, data_list2[index].E, data_list2[index].EMAP, data_list2[index].EF, data_list2[index].EI);
					data_list2[index].Qs.resize(ptr->V.rows());

					
					data_list2[index].Qit.resize(data_list2[index].E.rows());
					data_list2[index].C.resize(data_list2[index].E.rows(), ptr->V.cols());
					data_list2[index].Q.clear();
					for (int e = 0; e < data_list2[index].E.rows(); e++)
					{
						double cost = e;
						Eigen::RowVectorXd p(1, 3);
						shortest_edge_and_midpoint(e, ptr->V, ptr->F, data_list2[index].E, data_list2[index].EMAP, data_list2[index].EF, data_list2[index].EI, cost, p);
						data_list2[index].C.row(e) = p;
						data_list2[index].Qit[e] = data_list2[index].Q.insert(std::pair<double, int>(cost, e)).first;
					}
					
					++index;
				}
			}
			/*void Viewer::initEdges2()
			{
				int index = 0;
				vector<ViewerData>::iterator ptr;
				data_list2.resize(data_list.size());
				for (ptr = data_list.begin(); ptr != data_list.end(); ++ptr) {


					if (ptr == data_list.begin()) {


						ptr->MyTranslate(Eigen::Vector3f(5, 0, 0));

					}
					else {
						Eigen::Vector3d m = ptr->V.colwise().minCoeff();
						Eigen::Vector3d M = ptr->V.colwise().maxCoeff();

						Eigen::MatrixXd V_box(7, 3);
						double 
							
							X = (M(0) + m(0)) / 2;
						double centerX = (M(0) + m(0)) / 2;
						double centerZ = (M(2) + m(2)) / 2;
						double link_length = M(1) - m(1);

						V_box <<
							centerX, m(1), centerZ, //0-lower y 
							centerX, M(1), centerZ, //1-center of axis's
							centerX, M(1) + link_length, centerZ,//2-higher y 
							centerX + link_length, M(1), centerZ, //3- higher x
							centerX - link_length, M(1), centerZ,//4- lower x
							centerX, M(1), centerZ + link_length, //5-higher z
							centerX, M(1), centerZ - link_length; //6-lower z


						Eigen::MatrixXi E_box(3, 2);
						E_box <<
							0, 2,
							3, 4,
							5, 6;



						ptr->add_points(V_box, Eigen::RowVector3d(0, 1, 0));
						for (unsigned i = 0; i < E_box.rows(); ++i)
							ptr->add_edges
							(
								V_box.row(E_box(i, 0)),
								V_box.row(E_box(i, 1)),
								Eigen::RowVector3d(1, 0, 0)
							);
						ptr->MyTranslate(Eigen::Vector3f(0, (link_length / 2) * (index - 1), 0));
						ptr->show_overlay_depth = false;
						ptr->point_size = 5;
						ptr->line_width = 1;

					}
				
					edge_flaps(ptr->F, data_list2[index].E, data_list2[index].EMAP, data_list2[index].EF, data_list2[index].EI);
					vertex_triangle_adjacency(ptr->V, ptr->F, data_list2[index].VF, data_list2[index].VI);
					data_list2[index].Qit.resize(data_list2[index].E.rows());
					data_list2[index].C.resize(data_list2[index].E.rows(), ptr->V.cols());
					data_list2[index].Qs.resize(ptr->V.rows());
					
					for (auto& v : data_list2[index].Qs) {
						v = Eigen::Matrix4d::Zero();
					}
					data_list2[index].Q.clear();
					for (int f = 0; f < ptr->F.rows();++f) {
						Eigen::Vector3d normal(ptr->F_normals.row(f).normalized());
						Eigen::RowVector3d v(ptr->V.row(ptr->F.row(f)[0]));
						double d =  (-v) * normal;
						Eigen::Matrix4d rp(initRp(d,normal));
						for (int i = 0; i < 3; ++i) {
							data_list2[index].Qs[ptr->F.row(f)[i]] += rp;
						}

					}
					for (int e = 0; e < data_list2[index].E.rows(); e++)
					{
						double cost = e;
						Eigen::RowVectorXd p(1, 3);
						int v_index = data_list2[index].E.row(e)[0];
						Eigen::Vector3d v1 = ptr->V.row(v_index);
						Eigen::Matrix4d q1 = data_list2[index].Qs[v_index];
						v_index = data_list2[index].E.row(e)[1];
						Eigen::Vector3d v2 = ptr->V.row(v_index);
						Eigen::Matrix4d q2 = data_list2[index].Qs[v_index];
						//v1 += v2;
						//q1 += q2;
						
						minErrorcost(cost, p, v1, v2, q1, q2);
						
						data_list2[index].C.row(e) = p;
						data_list2[index].Qit[e] = data_list2[index].Q.insert(std::pair<double, int>(cost, e)).first;

					}
					++index;
				}
			}*/
			//will update data_list_indices list
			void Viewer::initEdges3()
			{
				int index = 0;
				vector<ViewerData>::iterator ptr;
				data_list_indices.resize(data_list.size() - 1);
				for (ptr = data_list.begin(); ptr != data_list.end(); ++ptr) {

					ptr->set_colors(Eigen::RowVector3d(135. / 255., 255. / 255., 255. / 255.));
					if (ptr == data_list.begin()) {
						ptr->MyTranslate(Eigen::Vector3f(5, 0, 0));
					}
					else {
						Eigen::Vector3d m = ptr->V.colwise().minCoeff();
						Eigen::Vector3d M = ptr->V.colwise().maxCoeff();

						Eigen::MatrixXd V_box(7, 3);
						double centerX = (M(0) + m(0)) / 2;
						double centerZ = (M(2) + m(2)) / 2;
						double link_length = M(1) - m(1);
						ptr->SetCenterOfRotation(Eigen::Vector3d(data_list[index].V.colwise().mean()[0], data_list[index].V.colwise().minCoeff()[1], data_list[index].V.colwise().mean()[0]));
						if(index==1)ptr->MyTranslate(Eigen::Vector3f(0, 0, 0));
						else
						{
							ptr->MyTranslate(Eigen::Vector3f(0, link_length, 0));
						}
						V_box <<
							centerX, m(1), centerZ, //0-lower y 
							centerX, M(1), centerZ, //1-center of axis's
							centerX, M(1) + link_length, centerZ,//2-higher y 
							centerX + link_length, M(1), centerZ, //3- higher x
							centerX - link_length, M(1), centerZ,//4- lower x
							centerX, M(1), centerZ + link_length, //5-higher z
							centerX, M(1), centerZ - link_length; //6-lower z


						Eigen::MatrixXi E_box(3, 2);
						E_box <<
							0, 2,
							3, 4,
							5, 6;



						ptr->add_points(V_box, Eigen::RowVector3d(0, 1, 0));
						ptr->add_edges(V_box.row(E_box(0, 0)), V_box.row(E_box(0, 1)), Eigen::RowVector3d(0, 1, 0));//y axis
						ptr->add_edges(V_box.row(E_box(1, 0)), V_box.row(E_box(1, 1)), Eigen::RowVector3d(1, 0, 0));//x axis
						ptr->add_edges(V_box.row(E_box(2, 0)), V_box.row(E_box(2, 1)), Eigen::RowVector3d(0, 0, 1));//z axis
						/*for (unsigned i = 0; i < E_box.rows(); ++i)
							ptr->add_edges
							(
								V_box.row(E_box(i, 0)),
								V_box.row(E_box(i, 1)),
								Eigen::RowVector3d(1, 0, 0)
							);*/
	//					if(index == 1)
	//						ptr->MyTranslate(Eigen::Vector3f(0, 0, 0));
	//					else
						
						ptr->show_overlay_depth = false;
						ptr->point_size = 5;
						ptr->line_width = 1;
						Eigen::Vector4f center_rot;
						data_list_indices[index - 1].prev = index - 1;//update father of each link
						data_list_indices[index - 1].cylinder_length = link_length;
						//data_list_indices[index - 1].topCylinder << centerX, M(1)*index, centerZ;
						Eigen::Vector4f vec(0, 0, 0, 1);
						
						//
					
					}
					index++;
				}
				data_list_indices[0].prev = -1;//the first link has no father
				
				
			}


			Eigen::Matrix4f Viewer::CalcParentsTrans(int index) {
				if (index < 2) {
					
					return Eigen::Matrix4f::Identity();
				}
				
				return CalcParentsTrans(index-1) * data_list[index-1].MakeTrans();
			}
			Eigen::Matrix3f Viewer::CalcParentsInverse(int index) {
				if (index < 2) {
					
					return Eigen::Matrix3f::Identity();
				}
				
				return  data_list[index - 1].GetRotation().inverse() * CalcParentsInverse(index - 1);
			}
			double Viewer::Scale(int index) {
				if (index == 0)
					return 0;
				return data_list_indices[index - 1].cylinder_length;
			}
			Eigen::Matrix4d Viewer::initRp(double d,Eigen::Vector3d normal)
			{
				Eigen::Matrix4d ans;
				ans<<normal[0] * normal[0], normal[0] * normal[1], normal[0] * normal[2], normal[0] * d,
					normal[1] * normal[0], normal[1] * normal[1], normal[1] * normal[2], normal[1] * d,
					normal[2] * normal[0], normal[2] * normal[1], normal[2] * normal[2], normal[2] * d,
					d * normal[0], d * normal[1], d * normal[2], d * d;
				return 
					ans;
			}
			
			////////<<<<<<<<<<<<<<<<New Cost Function>>>>>>>>>>>>>>>>
			void Viewer::minErrorcost(double& cost, Eigen::RowVectorXd& p, const Eigen::Vector3d v1, const Eigen::Vector3d v2, const Eigen::Matrix4d q1, const Eigen::Matrix4d q2)
			{
				//Eigen::Vector4d _v;
				//_v << v1 + v2, 1;
				Eigen::Matrix4d _qtag(q1 + q2);
				Eigen::Matrix4d _q(q1 + q2);
				//Eigen::Matrix4d minq;
				
				_qtag.row(3) << 0, 0, 0, 1;
				Eigen::Vector4d zeroX31;
				zeroX31 << 0, 0, 0, 1;
				//Eigen::Vector4d minV(_qtag.inverse() * zeroX31);
				Eigen::Vector4d minV(0,0,0,0);
				minV << (v1 + v2) / 2,0;
				cost = minV.transpose() * _q * minV;
				//p << minV[0], minV[1], minV[2];
				p = (v1 + v2) / 2;
				//cout << "p (" << minV[0] << "," << minV[1] << "," << minV[2] << ") ,cost = " << cost << endl;
			}
			IGL_INLINE Viewer::~Viewer()
			{
			}
			IGL_INLINE void Viewer::load_meshes_from_file(const std::string& mesh_file_name_string) {

				std::ifstream file(mesh_file_name_string);
				if (file.is_open()) {
					std::string line;
					int times = 1;
					while (getline(file,line))
					{
						for(int i = 0;i<times;++i )
							load_mesh_from_file(line);
						times = 4;
					}
					file.close();
				}
				else {
					std::cerr << "-- configuration file not found --" << std::endl;
				}
				
			}
			IGL_INLINE bool Viewer::load_mesh_from_file(const std::string& mesh_file_name_string)
			{
				
				// Create new data slot and set to selected
				if (!(data().F.rows() == 0 && data().V.rows() == 0))
				{
					append_mesh();
				}
				data().clear();

				size_t last_dot = mesh_file_name_string.rfind('.');
				if (last_dot == std::string::npos)
				{
					std::cerr << "Error: No file extension found in " <<
						mesh_file_name_string << std::endl;
					return false;
				}

				std::string extension = mesh_file_name_string.substr(last_dot + 1);

				if (extension == "off" || extension == "OFF")
				{
					Eigen::MatrixXd V;
					Eigen::MatrixXi F;
					if (!igl::readOFF(mesh_file_name_string, V, F))
						return false;
					data().set_mesh(V, F);


					//igl::edge_flaps(F,uE,EMAP,EF,EI);
				}
				else if (extension == "obj" || extension == "OBJ")
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

					data().set_mesh(V, F);
					data().set_uv(UV_V, UV_F);

				}
				else
				{
					// unrecognized file type
					//printf("Error: %s is not a recognized file type.\n", extension.c_str());
					return false;
				}

				data().compute_normals();
				data().uniform_colors(Eigen::Vector3d(51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0),
					Eigen::Vector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0),
					Eigen::Vector3d(255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0));

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
				const std::string& mesh_file_name_string)
			{
				// first try to load it with a plugin
				//for (unsigned int i = 0; i<plugins.size(); ++i)
				//  if (plugins[i]->save(mesh_file_name_string))
				//    return true;

				size_t last_dot = mesh_file_name_string.rfind('.');
				if (last_dot == std::string::npos)
				{
					// No file type determined
					std::cerr << "Error: No file extension found in " <<
						mesh_file_name_string << std::endl;
					return false;
				}
				std::string extension = mesh_file_name_string.substr(last_dot + 1);
				if (extension == "off" || extension == "OFF")
				{
					return igl::writeOFF(
						mesh_file_name_string, data().V, data().F);
				}
				else if (extension == "obj" || extension == "OBJ")
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
					printf("Error: %s is not a recognized file type.\n", extension.c_str());
					return false;
				}
				return true;
			}

			IGL_INLINE bool Viewer::load_scene()
			{
				std::string fname = igl::file_dialog_open();
				if (fname.length() == 0)
					return false;
				return load_scene(fname);
			}

			IGL_INLINE bool Viewer::load_scene(std::string fname)
			{
				// igl::deserialize(core(),"Core",fname.c_str());
				igl::deserialize(data(), "Data", fname.c_str());
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
				//igl::serialize(core(),"Core",fname.c_str(),true);
				igl::serialize(data(), "Data", fname.c_str());

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

				if (fname.length() == 0)
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
			// <<<<<<<<<<<<<<<<<<<<<<<new Function>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			IGL_INLINE Viewer::InfoStruct& Viewer::data2(int mesh_id /*= -1*/)
			{
				assert(!data_list2.empty() && "data_list should never be empty");
				int index;
				if (mesh_id == -1)
					index = selected_data_index;
				else
					index = mesh_index(mesh_id);

				assert((index >= 0 && index < data_list.size()) &&
					"selected_data_index or mesh_id should be in bounds");
				return data_list2[index];
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
				selected_data_index = data_list.size() - 1;
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
				if (data_list.size() == 1)
				{
					// Cannot remove last mesh
					return false;
				}
				data_list[index].meshgl.free();
				data_list.erase(data_list.begin() + index);
				if (selected_data_index >= index && selected_data_index > 0)
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
			using namespace Eigen;
			bool Viewer::new_collapse_edge(Eigen::MatrixXd& V,
				Eigen::MatrixXi& F,
				Eigen::MatrixXi& E,
				Eigen::VectorXi& EMAP,
				Eigen::MatrixXi& EF,
				Eigen::MatrixXi& EI, vector<vector<int>>& VF,
				vector<vector<int>>& VI,
				Eigen::MatrixXd& C,
				std::set<std::pair<double, int> >& Q,
				vector<Eigen::Matrix4d>& Qs,
				std::vector<std::set<std::pair<double, int> >::iterator >& Qit)
			{

				if (Q.empty())
				{
					// no edges to collapse
					return false;
				}
				std::pair<double, int> p = *(Q.begin());
				if (p.first == std::numeric_limits<double>::infinity())
				{
					// min cost edge is infinite cost
					return false;
				}
				
				auto& kill_edge = [&E, &EI, &EF](int e_index)
				{

					E(e_index, 0) = IGL_COLLAPSE_EDGE_NULL;
					E(e_index, 1) = IGL_COLLAPSE_EDGE_NULL;
					EF(e_index, 0) = IGL_COLLAPSE_EDGE_NULL;
					EF(e_index, 1) = IGL_COLLAPSE_EDGE_NULL;
					EI(e_index, 0) = IGL_COLLAPSE_EDGE_NULL;
					EI(e_index, 1) = IGL_COLLAPSE_EDGE_NULL;
				};
				Q.erase(Q.begin());
				int e = p.second;
				double e_cost = p.first;
				Qit[e] = Q.end();
				vector<int> N = circulation(e, true, EMAP, EF, EI);
				vector<int> Nd = circulation(e, false, EMAP, EF, EI);
				int v1 = E.row(e)[0];
				int v2 = E.row(e)[1];
				
				N.insert(N.begin(), Nd.begin(), Nd.end());
				for (auto& f : VF[v1]) {
					Vector3d normal(data().F_normals.row(f).normalized());
					RowVector3d v(V.row(F.row(f)[0]));
					double d = (-v) * normal;
					Matrix4d rp(initRp(d, normal));
					for (int i = 0; i < 3; ++i) {
						Qs[F.row(f)[i]] -= rp;
					}
				}
				
				
				const int eflip = E(e, 0) > E(e, 1);
				// source and destination
				const int s = eflip ? E(e, 1) : E(e, 0);
				const int d = eflip ? E(e, 0) : E(e, 1);
				if (!edge_collapse_is_valid(e, F, E, EMAP, EF, EI))
				{
					p.first = std::numeric_limits<double>::infinity();
					Qit[e] = Q.insert(p).first;
					return false;
				}

				for (auto& f : VF[v2]) {
					if (f != EF.row(e)[0] && f != EF.row(e)[1]) {
						Vector3d normal(data().F_normals.row(f).normalized());
						RowVector3d v(V.row(F.row(f)[0]));
						double d = (-v) * normal;
						Matrix4d rp(initRp(d, normal));
						for (int i = 0; i < 3; ++i) {
							Qs[F.row(f)[i]] -= rp;
						}
					}
				}
				
				const vector<int> nV2Fd = circulation(e, !eflip, EMAP, EF, EI);
				V.row(s) = C.row(e);
				V.row(d) = C.row(e);
				const int m = F.rows();
				int a_e1, a_e2, a_f1, a_f2;
				
				for (int side = 0; side < 2; side++)
				{
					const int f = EF(e, side);
					const int v = EI(e, side);
					const int sign = (eflip == 0 ? 1 : -1) * (1 - 2 * side);

					const int e1 = EMAP(f + m * ((v + sign * 1 + 3) % 3));

					const int e2 = EMAP(f + m * ((v + sign * 2 + 3) % 3));
					const bool flip1 = EF(e1, 1) == f;
					const int f1 = flip1 ? EF(e1, 0) : EF(e1, 1);
					const int v1 = flip1 ? EI(e1, 0) : EI(e1, 1);

					kill_edge(e1);
					for (int v_in_f = 0; v_in_f < 3; ++v_in_f) {
						int vertex = F.row(f)[v_in_f];
						VF[vertex].erase(
							remove(VF[vertex].begin(),
								VF[vertex].end(), f),
							VF[vertex].end());
					}

					F(f, 0) = IGL_COLLAPSE_EDGE_NULL;
					F(f, 1) = IGL_COLLAPSE_EDGE_NULL;
					F(f, 2) = IGL_COLLAPSE_EDGE_NULL;
					EMAP(f1 + m * v1) = e2;
					//EMAP.row(f)[0] = IGL_COLLAPSE_EDGE_NULL;
					//EMAP.row(f)[1] = IGL_COLLAPSE_EDGE_NULL;
					//EMAP.row(f)[2] = IGL_COLLAPSE_EDGE_NULL;
					const int opp2 = (EF(e2, 0) == f ? 0 : 1);
					EF(e2, opp2) = f1;
					EI(e2, opp2) = v1;

					E(e2, 0) = E(e2, 0) == d ? s : E(e2, 0);
					E(e2, 1) = E(e2, 1) == d ? s : E(e2, 1);
					if (side == 0)
					{
						a_e1 = e1;
						a_f1 = f;
					}
					else
					{
						a_e2 = e1;
						a_f2 = f;
					}
				}
				
				for (auto f : nV2Fd)
				{
					for (int v = 0; v < 3; v++)
					{
						if (F(f, v) == d)
						{
							const int flip1 = (EF(EMAP(f + m * ((v + 1) % 3)), 0) == f) ? 1 : 0;
							const int flip2 = (EF(EMAP(f + m * ((v + 2) % 3)), 0) == f) ? 0 : 1;
							E(EMAP(f + m * ((v + 2) % 3)), flip2) = s;
							F(f, v) = s;
							break;
						}
					}
				}
				
				kill_edge(e);
				VF[v1].insert(VF[v1].end(),
					VF[v2].begin(),
					VF[v2].end());
				
				for (auto& newf : VF[v1]) {
					Vector3d normal(data().F_normals.row(newf).normalized());
					RowVector3d v(V.row(v1));
					double d = (-v) * normal;
					Matrix4d rp(initRp(d, normal));
					for (int i = 0; i < 3; ++i) {
						if (F.row(newf)[i] == v2) {
							F.row(newf)[i] = v1;
						}
						Qs[F.row(newf)[i]] += rp;
					}
				}
				
				Q.erase(Qit[a_e1]);
				Qit[a_e1] = Q.end();
				Q.erase(Qit[a_e2]);
				Qit[a_e2] = Q.end();
				// update local neighbors
				// loop over original face neighbors
				for (auto n : N)
				{
					if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
						F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
						F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
					{
						for (int v = 0; v < 3; v++)
						{

							const int ei = EMAP(v * F.rows() + n);

							Q.erase(Qit[ei]);

							double cost =0;
							Eigen::RowVectorXd place(1, 3);
							int vi1 = E.row(ei)[0];
							int vi2 = E.row(ei)[1];
							minErrorcost(cost, place,
								V.row(vi1),
								V.row(vi2), Qs[vi1], Qs[vi2]);

							Qit[ei] = Q.insert(std::pair<double, int>(cost, ei)).first;
							C.row(ei) = place;
							
						}
					}
				}
				
				cout << "edge " << e << ",cost = " << e_cost << "new v position (" << V.row(v1)[0] << "," << V.row(v1)[1] << "," << V.row(v1)[2] << ")" << endl;
				return true;


			}



		} // end namespace
	} // end namespace
}