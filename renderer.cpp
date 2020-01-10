#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include <Eigen/Dense>
Renderer::Renderer() : selected_core_index(0),
next_core_id(2)
{
	core_list.emplace_back(igl::opengl::ViewerCore());
	core_list.front().id = 1;
	// C-style callbacks
	callback_init = nullptr;
	callback_pre_draw = nullptr;
	callback_post_draw = nullptr;
	callback_mouse_down = nullptr;
	callback_mouse_up = nullptr;
	callback_mouse_move = nullptr;
	callback_mouse_scroll = nullptr;
	callback_key_down = nullptr;
	callback_key_up = nullptr;

	callback_init_data = nullptr;
	callback_pre_draw_data = nullptr;
	callback_post_draw_data = nullptr;
	callback_mouse_down_data = nullptr;
	callback_mouse_up_data = nullptr;
	callback_mouse_move_data = nullptr;
	callback_mouse_scroll_data = nullptr;
	callback_key_down_data = nullptr;
	callback_key_up_data = nullptr;
	highdpi = 1;

	xold = 0;
	yold = 0;

}

IGL_INLINE void Renderer::draw(GLFWwindow* window)
{
	using namespace std;
	using namespace Eigen;

	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);

	auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

	if (fabs(highdpi_tmp - highdpi) > 1e-8)
	{
		post_resize(window, width, height);
		highdpi = highdpi_tmp;
	}

	for (auto& core : core_list)
	{
		core.clear_framebuffers();
	}


	for (auto& core : core_list)
	{

		for (auto& mesh : scn->data_list)
		{
			if (mesh.is_visible & core.id)
			{
			core.draw(scn->MakeTrans(), mesh);

			}

		}



		
	}
}



void Renderer::SetScene(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
	core().init(); 

	core().align_camera_center(scn->data().V, scn->data().F);
}

void Renderer::UpdatePosition(double xpos, double ypos)
{
	xrel = xold - xpos;
	yrel = yold - ypos;
	xold = xpos;
	yold = ypos;
}

void Renderer::MouseProcessing(int button)
{
	
	if (button == 1)
	{
		if (scn->selected_data_index ==0|| scn->selected_data_index == 1) {
			scn->data().MyTranslate(Eigen::Vector3f(-xrel / 1000.0f, 0, 0));
			scn->data().MyTranslate(Eigen::Vector3f(0, yrel / 1000.0f, 0));
		}
		else {
			scn->MyTranslate(Eigen::Vector3f(-xrel / 1000.0f, 0, 0));
			scn->MyTranslate(Eigen::Vector3f(0, yrel / 1000.0f, 0));
		}
		
	}
	else
	{
		if (scn->selected_data_index == -1) {
			scn->MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
			scn->MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
		
		}
		else {
			scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
			scn->data().MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
		}
	}
	
}

Renderer::~Renderer()
{
	//if (scn)
	//	delete scn;
}

double Renderer::Picking2(double newx, double newy)
{
	int fid;
	//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
	Eigen::Vector3f bc;
	double x = newx;
	double y = core().viewport(3) - newy;
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
	igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
	view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
		* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTrans() *scn->CalcParentsTrans(scn->selected_data_index) *scn->data().MakeTrans() ;
	if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
		core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc))
		{
			int p1, p2, p3;
			p1 = scn->data().F.row(fid)[0];
			p2 = scn->data().F.row(fid)[1];
			p3 = scn->data().F.row(fid)[2];
			Eigen::Vector3f v1(scn->data().V.row(p1)[0], scn->data().V.row(p1)[1], scn->data().V.row(p1)[2]);
			Eigen::Vector3f v2(scn->data().V.row(p2)[0], scn->data().V.row(p2)[1], scn->data().V.row(p2)[2]);
			Eigen::Vector3f v3(scn->data().V.row(p3)[0], scn->data().V.row(p3)[1], scn->data().V.row(p3)[2]);
			Eigen::Vector3f b(v1[0]*bc[0]+v2[0]*bc[1]+v3[0]*bc[2], v1[1] * bc[0] + v2[1] * bc[1] + v3[1] * bc[2], v1[2] * bc[0] + v2[2] * bc[1] + v3[2] * bc[2]);
			Eigen::Vector4f a(4);
			a << b, 1;
			Eigen::Vector4f ans4f(4);
			ans4f = view * a;
			return ans4f.norm();
		}
		return INFINITY;
}
void Renderer::line_less() {
	for (int i = 0; i < scn->data_list.size(); ++i)
		core().toggle(scn->data_list[i].show_lines);
}
//previous function
bool Renderer::Picking(double newx, double newy)
{
	int fid;
	//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
	Eigen::Vector3f bc;
	double x = newx;
	double y = core().viewport(3) - newy;
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
	igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
	view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
		* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTrans() * scn->data().MakeTrans();
	if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
		core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc))
	{
		return true;
	}
	return false;

}

IGL_INLINE void Renderer::resize(GLFWwindow* window,int w, int h)
	{
		if (window) {
			glfwSetWindowSize(window, w / highdpi, h / highdpi);
		}
		post_resize(window,w, h);
	}

	IGL_INLINE void Renderer::post_resize(GLFWwindow* window, int w, int h)
	{
		if (core_list.size() == 1)
		{
			core().viewport = Eigen::Vector4f(0, 0, w, h);
		}
		else
		{
			// It is up to the user to define the behavior of the post_resize() function
			// when there are multiple viewports (through the `callback_post_resize` callback)
		}
		//for (unsigned int i = 0; i < plugins.size(); ++i)
		//{
		//	plugins[i]->post_resize(w, h);
		//}
		if (callback_post_resize)
		{
			callback_post_resize(window, w, h);
		}
	}

	IGL_INLINE igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/)
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE const igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/) const
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE bool Renderer::erase_core(const size_t index)
	{
		assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
		//assert(data_list.size() >= 1);
		if (core_list.size() == 1)
		{
			// Cannot remove last viewport
			return false;
		}
		core_list[index].shut(); // does nothing
		core_list.erase(core_list.begin() + index);
		if (selected_core_index >= index && selected_core_index > 0)
		{
			selected_core_index--;
		}
		return true;
	}

	IGL_INLINE size_t Renderer::core_index(const int id) const {
		for (size_t i = 0; i < core_list.size(); ++i)
		{
			if (core_list[i].id == id)
				return i;
		}
		return 0;
	}

	IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
	{
		core_list.push_back(core()); // copies the previous active core and only changes the viewport
		core_list.back().viewport = viewport;
		core_list.back().id = next_core_id;
		next_core_id <<= 1;
		if (!append_empty)
		{
			for (auto& data : scn->data_list)
			{
				data.set_visible(true, core_list.back().id);
				//data.copy_options(core(), core_list.back());
			}
		}
		selected_core_index = core_list.size() - 1;
		return core_list.back().id;
	}
	void Renderer::animate(GLFWwindow* window) {
		
	}
	Eigen::Vector3f Renderer::Calc_E() {
		int index = scn->data_list.size() - 1;
		Eigen::Matrix4f parentsTransform = scn->CalcParentsTrans(scn->data_list.size());
		Eigen::Vector4f tt = parentsTransform * Eigen::Vector4f(scn->data_list[index].V.colwise().mean()[0],
			scn->data_list[index].V.colwise().maxCoeff()[1], scn->data_list[index].V.colwise().mean()[2], 1);
		return tt.head(3);
	}
	Eigen::Vector3f Renderer::Calc_R(int index) {
		
		Eigen::Matrix4f parentsTransform = scn->CalcParentsTrans(index+1);
		Eigen::Vector4f tt = parentsTransform * Eigen::Vector4f(scn->data_list[index].V.colwise().mean()[0],
			scn->data_list[index].V.colwise().minCoeff()[1], scn->data_list[index].V.colwise().mean()[2], 1);
		return tt.head(3);
	}
	void Renderer::ik_fixer() {
		for (int i = 1; i < scn->data_list.size(); ++i)
		{
			Eigen::Matrix3f rot = scn->data_list[i].GetRotation();
			float y1 = 0;
			if(rot.row(1)[1] < 1&& rot.row(1)[1] > -1){
			
				y1 = atan2f(rot.row(1)[0], -rot.row(1)[2]);
			}
			scn->data_list[i].MyRotate(Eigen::Vector3f(0, 1, 0), -y1);
			if (scn->data_list.size() > i+1) {
				scn->data_list[i+1].RotInSys(Eigen::Vector3f(0, 1, 0), y1);
			}
			
		}
	}
	
	void Renderer::ik_solver() {
		
		
		Eigen::Vector3f D = scn->data_list[0].MakeTrans().col(3).head(3);
		Eigen::Vector3f Base = Calc_R(1);
		Eigen::Vector3f E = Calc_E();
		double EDdistance = (D - E).norm();
		double BaseDdistance = (D-Base).norm();
		double max_Arm_length = 1.6 * scn->data_list_indices.size();
		if (BaseDdistance > max_Arm_length) {
			cout << "cannot reach" << endl;
			scn->ik_flag = false;
			
			return;
		}
		if(EDdistance<0.1){
			ik_fixer();
			scn->ik_flag = false;
			return;
		}


			for (int i = scn->data_list.size() - 1; i > 0; i--) {
				
				 
				Eigen::Vector3f R = Calc_R(i);
				
				Eigen::Vector3f RD = (D - R).normalized();
				Eigen::Vector3f RE = (E - R).normalized();
				float pre_angle = RE.dot(RD);
				if (pre_angle > 1) {
					pre_angle = 1;
				}
				else if (pre_angle < -1)
					pre_angle = -1;

				float angle = acosf(pre_angle);

				Eigen::Vector3f cross = RE.cross(RD).normalized();
				Eigen::Vector3f crossInverse = scn->CalcParentsInverse(scn->data_list.size()) * cross;
				if (EDdistance > 0.5)
					angle = angle / 10;
				scn->data_list[i].MyRotate(crossInverse, angle);
				E = Calc_E();
				if (i > 1) {

					Eigen::Vector3f R_tag = Calc_R(i-1);
					Eigen::Vector3f R_tagR = (R_tag - R).normalized();
					Eigen::Vector3f RE = (E - R).normalized();
					pre_angle = RE.dot(R_tagR);
					if (pre_angle > 1) {
						pre_angle = 1;
					}
					else if (pre_angle < -1)
						pre_angle = -1;

					float angle2 = acosf(pre_angle);
					if (angle2 < (M_PI / 6)) {
						scn->data_list[i].MyRotate(crossInverse, -angle);
						angle = (M_PI / 6) - angle2;
						scn->data_list[i].MyRotate(crossInverse, angle);
					}
				
				}
			}
		

	}

	bool Renderer::checkCollision(igl::AABB<Eigen::MatrixXd, 3> bunnyA, igl::AABB<Eigen::MatrixXd, 3> bunnyB) {
		Eigen::Vector3f bunnyAcenter = bunnyA.m_box.center().cast<float>();
		Eigen::Vector3f bunnyBcenter = bunnyB.m_box.center().cast<float>();
		Eigen::Vector3f centerC0 = (scn->data_list[0].MakeTrans() * Eigen::Vector4f(bunnyAcenter[0],
			bunnyAcenter[1], bunnyAcenter[2], 1)).head(3);
		Eigen::Vector3f centerC1 = (scn->data_list[1].MakeTrans() * Eigen::Vector4f(bunnyBcenter[0],
			bunnyBcenter[1], bunnyBcenter[2], 1)).head(3);
		Eigen::Vector3f D = centerC1 - centerC0;
		Eigen::Vector3f A0 = (scn->data_list[0].GetRotation() * Eigen::Vector3f(1, 0, 0));//Xaxis
		Eigen::Vector3f A1 = (scn->data_list[0].GetRotation() * Eigen::Vector3f(0, 1, 0));//Yaxis
		Eigen::Vector3f A2 = (scn->data_list[0].GetRotation() * Eigen::Vector3f(0, 0, 1));//Zaxis
		Eigen::Vector3f B0 = (scn->data_list[1].GetRotation() * Eigen::Vector3f(1, 0, 0));//Xaxis
		Eigen::Vector3f B1 = (scn->data_list[1].GetRotation() * Eigen::Vector3f(0, 1, 0));//Yaxis
		Eigen::Vector3f B2 = (scn->data_list[1].GetRotation() * Eigen::Vector3f(0, 0, 1));//Zaxis
		Eigen::Vector3f randomPointA = bunnyA.m_box.corner(bunnyA.m_box.BottomLeftFloor).cast<float>();
		Eigen::Vector3f randomPointB = bunnyB.m_box.corner(bunnyB.m_box.BottomLeftFloor).cast<float>();
		float a0 = bunnyA.m_box.sizes().cast<float>()[0] / 2;//sqrt(pow(bunnyAcenter[0] - randomPointA[0], 2.));
		float a1 = bunnyA.m_box.sizes().cast<float>()[1] / 2;//sqrt(pow(bunnyAcenter[1] - randomPointA[1], 2.));
		float a2 = bunnyA.m_box.sizes().cast<float>()[2] / 2;//sqrt(pow(bunnyAcenter[2] - randomPointA[2], 2.));
		float b0 = bunnyB.m_box.sizes().cast<float>()[0] / 2;//sqrt(pow(bunnyBcenter[0] - randomPointA[0], 2.));
		float b1 = bunnyB.m_box.sizes().cast<float>()[1] / 2;//sqrt(pow(bunnyBcenter[1] - randomPointA[1], 2.));
		float b2 = bunnyB.m_box.sizes().cast<float>()[2] / 2;//sqrt(pow(bunnyBcenter[2] - randomPointA[2], 2.));
		Eigen::Matrix3f C = scn->data_list[0].GetRotation().inverse() * scn->data_list[1].GetRotation();
		bool tableCalcAns = tableCalc(A0, A1, A2, a0, a1, a2, B0, B1, B2, b0, b1, b2, C, D);
		if (!tableCalcAns)
		{
			if (bunnyA.is_leaf())
			{
				if (bunnyB.is_leaf())
				{
					scn->drawBox(bunnyA, 0, Eigen::RowVector3d(0,1,0));
					scn->drawBox(bunnyB, 1, Eigen::RowVector3d(0,1,0));
					scn->go_flag = false;
					scn->collision_happend = true;
					return true;
				}
				else
				{
					return checkCollision(bunnyA, *bunnyB.m_left) || checkCollision(bunnyA, *bunnyB.m_right);
				}
			}
			else if (bunnyB.is_leaf())
			{
					return checkCollision(*bunnyA.m_left, bunnyB) || checkCollision(*bunnyA.m_right, bunnyB);
			}
			else
			{
				return checkCollision(*bunnyA.m_left, *bunnyB.m_left) || checkCollision(*bunnyA.m_left, *bunnyB.m_right)|| checkCollision(*bunnyA.m_right, *bunnyB.m_left) || checkCollision(*bunnyA.m_right, *bunnyB.m_right);
			}
			
		}
		else//no Collision
		{
			return false;
		}
	}

	bool Renderer::tableCalc(Eigen::Vector3f A0, Eigen::Vector3f A1, Eigen::Vector3f A2, float a0, float a1, float a2, Eigen::Vector3f B0, Eigen::Vector3f B1, Eigen::Vector3f B2, float b0, float b1, float b2, Eigen::Matrix3f C, Eigen::Vector3f D) {
		return (a0 + (b0 * abs(C.row(0)[0]) + b1 * abs(C.row(0)[1]) + b2 * abs(C.row(0)[2])) < abs(A0.dot(D))) ||
			(a1 + (b0 * abs(C.row(1)[0]) + b1 * abs(C.row(1)[1]) + b2 * abs(C.row(1)[2])) < abs(A1.dot(D))) ||
			(a2 + (b0 * abs(C.row(2)[0]) + b1 * abs(C.row(2)[1]) + b2 * abs(C.row(2)[2])) < abs(A2.dot(D))) ||
			(b0 + (a0 * abs(C.row(0)[0]) + a1 * abs(C.row(1)[0]) + a2 * abs(C.row(2)[0])) < abs(B0.dot(D))) ||
			(b1 + (a0 * abs(C.row(0)[1]) + a1 * abs(C.row(1)[1]) + a2 * abs(C.row(2)[1])) < abs(B1.dot(D))) ||
			(b2 + (a0 * abs(C.row(0)[2]) + a1 * abs(C.row(1)[2]) + a2 * abs(C.row(2)[2])) < abs(B2.dot(D))) ||
			(((a1 * abs(C.row(2)[0]) + a2 * abs(C.row(1)[0])) + (b1 * abs(C.row(0)[2]) + b2 * abs(C.row(0)[1]))) < abs(C.row(1)[0] * A2.dot(D) - C.row(2)[0] * A1.dot(D))) ||
			(((a1 * abs(C.row(2)[1]) + a2 * abs(C.row(1)[1])) + (b0 * abs(C.row(0)[2]) + b2 * abs(C.row(0)[0]))) < abs(C.row(1)[1] * A2.dot(D) - C.row(2)[1] * A1.dot(D))) ||
			(((a1 * abs(C.row(2)[2]) + a2 * abs(C.row(1)[2])) + (b0 * abs(C.row(0)[1]) + b1 * abs(C.row(0)[0]))) < abs(C.row(1)[2] * A2.dot(D) - C.row(2)[2] * A1.dot(D))) ||

			(((a0 * abs(C.row(2)[0]) + a2 * abs(C.row(0)[0])) + (b1 * abs(C.row(1)[2]) + b2 * abs(C.row(1)[1]))) < abs(C.row(2)[0] * A0.dot(D) - C.row(0)[0] * A2.dot(D))) ||
			(((a0 * abs(C.row(2)[1]) + a2 * abs(C.row(0)[1])) + (b0 * abs(C.row(1)[2]) + b2 * abs(C.row(1)[0]))) < abs(C.row(2)[1] * A0.dot(D) - C.row(0)[1] * A2.dot(D))) ||
			(((a0 * abs(C.row(2)[2]) + a2 * abs(C.row(0)[2])) + (b0 * abs(C.row(1)[1]) + b1 * abs(C.row(1)[0]))) < abs(C.row(2)[2] * A0.dot(D) - C.row(0)[2] * A2.dot(D))) ||

			(((a0 * abs(C.row(1)[0]) + a1 * abs(C.row(0)[0])) + (b1 * abs(C.row(2)[2]) + b2 * abs(C.row(2)[1]))) < abs(C.row(0)[0] * A1.dot(D) - C.row(1)[0] * A0.dot(D))) ||
			(((a0 * abs(C.row(1)[1]) + a2 * abs(C.row(0)[1])) + (b0 * abs(C.row(2)[2]) + b2 * abs(C.row(2)[0]))) < abs(C.row(0)[1] * A1.dot(D) - C.row(1)[1] * A0.dot(D))) ||
			(((a0 * abs(C.row(1)[2]) + a2 * abs(C.row(0)[2])) + (b0 * abs(C.row(2)[1]) + b1 * abs(C.row(2)[0]))) < abs(C.row(0)[2] * A1.dot(D) - C.row(1)[2] * A0.dot(D)));   
	}

	void Renderer::go_Bunny() {
		switch (arrow)
		{
		case 0: //left
			scn->data_list[scn->last_selected_data_index].MyTranslate(Eigen::Vector3f(-0.01, 0, 0));
			break;
		case 1:	//right
			scn->data_list[scn->last_selected_data_index].MyTranslate(Eigen::Vector3f(0.01, 0, 0));
			break;
		case 2:	//up
			scn->data_list[scn->last_selected_data_index].MyTranslate(Eigen::Vector3f(0, 0.01, 0));
			break;
		case 3:	//down
			scn->data_list[scn->last_selected_data_index].MyTranslate(Eigen::Vector3f(0, -0.01, 0));
			break;
		default:
			break;
		}
		checkCollision(scn->data_list[0].tree, scn->data_list[1].tree);
		

	}

	//IGL_INLINE void Viewer::select_hovered_core()
	//{
	//	int width_window, height_window = 800;
	//   glfwGetFramebufferSize(window, &width_window, &height_window);
	//	for (int i = 0; i < core_list.size(); i++)
	//	{
	//		Eigen::Vector4f viewport = core_list[i].viewport;

	//		if ((current_mouse_x > viewport[0]) &&
	//			(current_mouse_x < viewport[0] + viewport[2]) &&
	//			((height_window - current_mouse_y) > viewport[1]) &&
	//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
	//		{
	//			selected_core_index = i;
	//			break;
	//		}
	//	}
	//}